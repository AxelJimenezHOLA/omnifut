// ── Librerías ─────────────────────────────────────────────────────────────────
#include <Adafruit_NeoPixel.h>
#include <Adafruit_BNO08x.h>

// ── Pines de hardware ─────────────────────────────────────────────────────────
#define PIN_PUSH       45
#define PIN_BATTERY     9
#define PIN_LED         3

// ── Pines I2C del IMU BNO08x ──────────────────────────────────────────────────
#define BNO08X_SDA     41
#define BNO08X_SCL     40
#define BNO08X_INT     -1
#define BNO08X_RST     -1

// ── Configuración PWM ─────────────────────────────────────────────────────────
#define PWM_MAX        255
#define PWM_FREQUENCY 4000
#define PWM_RESOLUTION   8

// ── Parámetros cinemáticos y de encoder ───────────────────────────────────────
#define RPM_MAX         1000.0f
#define PPR              200
#define PID_INTERVAL_MS   20

// ── Velocidad de traslación manual (fracción de RPM_MAX, rango 0–1) ───────────
#define SPEED_MANUAL    0.40f

// ── Parámetros del controlador de orientación (yaw) ───────────────────────────
// Ganancia proporcional del corrector angular
#define YAW_KP          0.005f
// Umbral mínimo de error (°) para activar corrección; evita oscilaciones en reposo
#define YAW_DEADBAND    2.0f
// Saturación máxima del término de corrección rotacional (fracción de RPM_MAX)
#define YAW_W_MAX       0.30f
// Intervalo de actualización del IMU en ms
#define IMU_INTERVAL_MS   10

// ── Colores LED ───────────────────────────────────────────────────────────────
#define COLOR_IDLE     pixels.Color( 50,  50,  50)
#define COLOR_READY    pixels.Color(  0, 150,   0)
#define COLOR_MOVING   pixels.Color(  0,   0, 150)

// ── Número de motores ─────────────────────────────────────────────────────────
#define MOTOR_COUNT 4

// ── Estructura PID por motor ──────────────────────────────────────────────────
struct MotorPID {
    float targetRPM;
    float currentRPM;
    float integral;
    float previousError;
    int   pwmOutput;
    volatile long encoderCount;
    long  lastEncoderCount;
    unsigned long lastUpdateTime;
    float kp;
    float ki;
    float kd;
};

// ── Estructura de motor: pines hardware + estado PID ──────────────────────────
struct Motor {
    uint8_t pwmP;
    uint8_t pwmN;
    uint8_t encoderA;
    uint8_t encoderB;
    MotorPID pid;
};

// Índices de motor
enum MotorIndex { FRONT_LEFT = 0, FRONT_RIGHT, BACK_RIGHT, BACK_LEFT };

// ── Definición centralizada de motores ────────────────────────────────────────
static Motor motors[MOTOR_COUNT] = {
  // pwmP  pwmN  encA  encB  pid (kp, ki, kd)
  { 2, 1, 4, 5, { 0, 0, 0, 0, 0, 0, 0, 0, 1.2f, 0.05f, 0.02f } },      // FRONT_LEFT
  { 7, 6, 19, 21, { 0, 0, 0, 0, 0, 0, 0, 0, 1.2f, 0.05f, 0.02f } },    // FRONT_RIGHT
  { 18, 17, 22, 23, { 0, 0, 0, 0, 0, 0, 0, 0, 1.2f, 0.05f, 0.02f } },  // BACK_RIGHT
  { 38, 37, 25, 26, { 0, 0, 0, 0, 0, 0, 0, 0, 1.2f, 0.05f, 0.02f } },  // BACK_LEFT
};

// ── ISR de encoders ───────────────────────────────────────────────────────────
static void IRAM_ATTR isr_enc0() { motors[0].pid.encoderCount++; }
static void IRAM_ATTR isr_enc1() { motors[1].pid.encoderCount++; }
static void IRAM_ATTR isr_enc2() { motors[2].pid.encoderCount++; }
static void IRAM_ATTR isr_enc3() { motors[3].pid.encoderCount++; }

static void (*const ENCODER_ISRS[MOTOR_COUNT])() = {
    isr_enc0, isr_enc1, isr_enc2, isr_enc3
};

// ── Periféricos ───────────────────────────────────────────────────────────────
Adafruit_NeoPixel pixels(1, PIN_LED, NEO_GRB + NEO_KHZ800);
Adafruit_BNO08x   imu(BNO08X_RST);
sh2_SensorValue_t imuSensorValue;

// ── RemoteXY (BLE) ────────────────────────────────────────────────────────────
#define REMOTEXY_MODE__ESP32CORE_BLE
#include <BLEDevice.h>
#define REMOTEXY_BLUETOOTH_NAME "RobofutOreo"
#include <RemoteXY.h>

#pragma pack(push, 1)

uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] = {
  255,5,0,0,0,89,0,19,0,0,0,82,111,98,111,102,117,116,79,114,
  101,111,0,29,1,106,200,1,1,5,0,10,5,152,97,41,49,4,26,31,
  79,78,0,31,79,70,70,0,1,41,59,24,24,1,192,31,226,134,145,0,
  1,15,86,24,24,1,192,31,226,134,144,0,1,67,86,24,24,1,192,31,
  226,134,146,0,1,41,112,24,24,1,192,31,226,134,147,0
};

struct {uint8_t button_toggle, button_up, button_left, button_right, button_down, connect_flag} RemoteXY;
#pragma pack(pop)

// ── Variables globales de orientación ─────────────────────────────────────────
// Offset de calibración: yaw del IMU en el momento del último reset de referencia
static float yawOffset         = 0.0f;
// Yaw global actual del robot, normalizado a [-180°, 180°]
static float globalYaw         = 0.0f;
// Referencia de orientación que el robot debe mantener, normalizada a [-180°, 180°]
static float globalYawReference = 0.0f;
// Indica si el IMU fue inicializado correctamente
static bool  imuReady          = false;

// ── Control de flanco del push button ────────────────────────────────────────
// Detectar flanco de bajada (presión) del pulsador sin delay()
static bool  lastButtonState   = HIGH;

// =============================================================================
//  Normalizar un ángulo al rango [-180°, 180°]
// =============================================================================
float normalizeAngle(float angle) {
    while (angle >  180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

// =============================================================================
//  Calcular el error angular mínimo entre target y current en [-180°, 180°]
// =============================================================================
float calculateYawError(float target, float current) {
    return normalizeAngle(target - current);
}

// =============================================================================
//  Convertir cuaternión a ángulo yaw en grados
// =============================================================================
static float quaternionToYaw(float qr, float qi, float qj, float qk) {
    // Yaw = atan2(2*(qr*qk + qi*qj), 1 - 2*(qj*qj + qk*qk))
    float sinYaw = 2.0f * (qr * qk + qi * qj);
    float cosYaw = 1.0f - 2.0f * (qj * qj + qk * qk);
    return degrees(atan2f(sinYaw, cosYaw));
}

// =============================================================================
//  Inicializar el IMU BNO08x y habilitar el reporte de rotación
// =============================================================================
static void initIMU() {
    Wire.begin(BNO08X_SDA, BNO08X_SCL);

    if (!imu.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire)) {
        Serial.println("IMU BNO08x no encontrado. Verificar conexiones.");
        imuReady = false;
        return;
    }

    // Habilitar reporte de rotación (cuaternión de orientación absoluta)
    if (!imu.enableReport(SH2_ARVR_STABILIZED_RV, 10000)) {   // 10 ms = 100 Hz
        Serial.println("Error al habilitar reporte IMU.");
        imuReady = false;
        return;
    }

    imuReady = true;
    Serial.println("IMU BNO08x inicializado correctamente.");
}

// =============================================================================
//  Actualizar la orientación global desde el IMU BNO08x
//  Debe llamarse periódicamente desde el loop principal
// =============================================================================
void updateIMU() {
    if (!imuReady) return;

    static unsigned long lastIMURun = 0;
    if (millis() - lastIMURun < IMU_INTERVAL_MS) return;
    lastIMURun = millis();

    // Leer el sensor solo si hay dato nuevo disponible
    if (!imu.getSensorEvent(&imuSensorValue)) return;
    if (imuSensorValue.sensorId != SH2_ARVR_STABILIZED_RV) return;

    // Extraer cuaternión del reporte
    float qr = imuSensorValue.un.arvrStabilizedRV.real;
    float qi = imuSensorValue.un.arvrStabilizedRV.i;
    float qj = imuSensorValue.un.arvrStabilizedRV.j;
    float qk = imuSensorValue.un.arvrStabilizedRV.k;

    // Convertir a yaw en grados y aplicar offset de calibración
    float rawYaw = quaternionToYaw(qr, qi, qj, qk);
    globalYaw    = normalizeAngle(rawYaw - yawOffset);
}

// =============================================================================
//  Recalibrar la referencia global: el yaw actual pasa a ser el nuevo 0°
// =============================================================================
void recalibrateGlobalReference() {
    // El offset absorbe el yaw físico actual del IMU, haciéndolo 0° global
    float qr = imuSensorValue.un.arvrStabilizedRV.real;
    float qi = imuSensorValue.un.arvrStabilizedRV.i;
    float qj = imuSensorValue.un.arvrStabilizedRV.j;
    float qk = imuSensorValue.un.arvrStabilizedRV.k;

    yawOffset          = quaternionToYaw(qr, qi, qj, qk);
    globalYaw          = 0.0f;
    globalYawReference = 0.0f;

    Serial.print("Referencia recalibrada. Nuevo yawOffset: ");
    Serial.println(yawOffset);
}

// =============================================================================
//  Inicializar motores y encoders
// =============================================================================
static void initMotors() {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        Motor& m = motors[i];

        ledcAttach(m.pwmP, PWM_FREQUENCY, PWM_RESOLUTION);
        ledcAttach(m.pwmN, PWM_FREQUENCY, PWM_RESOLUTION);
        ledcWrite(m.pwmP, 0);
        ledcWrite(m.pwmN, 0);

        pinMode(m.encoderA, INPUT_PULLUP);
        pinMode(m.encoderB, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(m.encoderA), ENCODER_ISRS[i], RISING);

        m.pid.encoderCount     = 0;
        m.pid.lastEncoderCount = 0;
        m.pid.lastUpdateTime   = millis();
    }
}

// =============================================================================
//  Escribir PWM a un motor (admite valores negativos = reversa)
// =============================================================================
static void applyMotorPWM(Motor& m, int pwm) {
    pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
    ledcWrite(m.pwmP, pwm > 0 ? pwm : 0);
    ledcWrite(m.pwmN, pwm < 0 ? -pwm : 0);
}

// =============================================================================
//  Actualizar PID de un motor individual; llamar cada PID_INTERVAL_MS
// =============================================================================
static void updateMotorPID(Motor& m) {
    unsigned long now = millis();
    float dt          = (now - m.pid.lastUpdateTime) / 1000.0f;
    if (dt <= 0.0f) return;

    long encNow;
    noInterrupts();
    encNow = m.pid.encoderCount;
    interrupts();

    long deltaTicks        = encNow - m.pid.lastEncoderCount;
    m.pid.lastEncoderCount = encNow;
    m.pid.lastUpdateTime   = now;

    m.pid.currentRPM = (deltaTicks / (float)PPR) / (dt / 60.0f);

    if (m.pid.targetRPM == 0.0f && fabsf(m.pid.currentRPM) < 0.1f) {
        m.pid.integral      = 0.0f;
        m.pid.previousError = 0.0f;
        m.pid.pwmOutput     = 0;
        applyMotorPWM(m, 0);
        return;
    }

    float error = m.pid.targetRPM - m.pid.currentRPM;

    m.pid.integral += error * dt;
    m.pid.integral  = constrain(m.pid.integral, -RPM_MAX, RPM_MAX);

    float derivative    = (error - m.pid.previousError) / dt;
    m.pid.previousError = error;

    float pidOut = m.pid.kp * error
                 + m.pid.ki * m.pid.integral
                 + m.pid.kd * derivative;

    m.pid.pwmOutput = (int)(pidOut * (PWM_MAX / RPM_MAX));
    m.pid.pwmOutput = constrain(m.pid.pwmOutput, -PWM_MAX, PWM_MAX);

    applyMotorPWM(m, m.pid.pwmOutput);
}

// Actualizar todos los motores si ha pasado el intervalo
static void updateAllPID() {
    static unsigned long lastRun = 0;
    if (millis() - lastRun < PID_INTERVAL_MS) return;
    lastRun = millis();
    for (int i = 0; i < MOTOR_COUNT; i++) updateMotorPID(motors[i]);
}

// =============================================================================
//  Cinemática omni-drive → asigna targetRPM a cada motor
//  vx, vy, w en rango [-1, 1] (fracción de velocidad máxima)
// =============================================================================
static void omniDrive(float vx, float vy, float w) {
    float raw[MOTOR_COUNT] = {
         vx - vy - w,   // FRONT_LEFT
         vx + vy + w,   // FRONT_RIGHT
         vx - vy + w,   // BACK_RIGHT
         vx + vy - w,   // BACK_LEFT
    };

    float maxVal = 0.0f;
    for (int i = 0; i < MOTOR_COUNT; i++) maxVal = max(maxVal, fabsf(raw[i]));
    if (maxVal > 1.0f)
        for (int i = 0; i < MOTOR_COUNT; i++) raw[i] /= maxVal;

    for (int i = 0; i < MOTOR_COUNT; i++) {
        motors[i].pid.targetRPM = raw[i] * RPM_MAX;
    }
}

static void stopMotors() { omniDrive(0.0f, 0.0f, 0.0f); }

// =============================================================================
//  Calcular la corrección rotacional proporcional al error de yaw
//  Solo actúa si hay traslación activa para no rotar el robot cuando está detenido
// =============================================================================
static float computeYawCorrection(bool isMoving) {
    if (!imuReady || !isMoving) return 0.0f;

    float yawError = calculateYawError(globalYawReference, globalYaw);

    // Aplicar zona muerta para suprimir correcciones por ruido del sensor
    if (fabsf(yawError) < YAW_DEADBAND) return 0.0f;

    float wCorrection = YAW_KP * yawError;
    return constrain(wCorrection, -YAW_W_MAX, YAW_W_MAX);
}

// =============================================================================
//  Detectar flanco de bajada del push button sin delay()
// =============================================================================
static bool buttonPressed() {
    bool currentState = digitalRead(PIN_PUSH);
    bool pressed      = (lastButtonState == HIGH && currentState == LOW);
    lastButtonState   = currentState;
    return pressed;
}

// =============================================================================
//  Establecer el color del LED NeoPixel
// =============================================================================
static void setLed(uint32_t color) {
    pixels.setPixelColor(0, color);
    pixels.show();
}

// =============================================================================
//  Leer botones de dirección → vx, vy
// =============================================================================
static void readDirectionButtons(float& vx, float& vy) {
    vx = 0.0f;
    vy = 0.0f;
    if (RemoteXY.button_up)    vx += SPEED_MANUAL;
    if (RemoteXY.button_down)  vx -= SPEED_MANUAL;
    if (RemoteXY.button_right) vy -= SPEED_MANUAL;
    if (RemoteXY.button_left)  vy += SPEED_MANUAL;
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
    Serial.begin(115200);
    RemoteXY_Init();
    delay(500);

    pinMode(PIN_PUSH, INPUT_PULLUP);
    initMotors();
    initIMU();

    // Tomar la primera lectura del IMU y definirla como 0° global inicial
    // Se intenta durante un segundo para asegurar que el sensor esté listo
    unsigned long t0 = millis();
    while (millis() - t0 < 1000) {
        if (imuReady && imu.getSensorEvent(&imuSensorValue)) {
            if (imuSensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
                recalibrateGlobalReference();
                break;
            }
        }
    }

    pixels.begin();
    pixels.clear();
    setLed(COLOR_IDLE);
}

// =============================================================================
//  LOOP
// =============================================================================
void loop() {
    RemoteXYEngine.handler();

    // Actualizar la orientación global desde el IMU
    updateIMU();

    // Detectar pulsación del push button para recalibrar referencia
    if (buttonPressed()) {
        recalibrateGlobalReference();
    }

    float vx = 0.0f, vy = 0.0f;

    if (RemoteXY.connect_flag && RemoteXY.button_toggle) {
        // Leer botones de dirección
        readDirectionButtons(vx, vy);

        bool moving = (vx != 0.0f || vy != 0.0f);
        setLed(moving ? COLOR_MOVING : COLOR_READY);

    } else {
        // Toggle OFF o BLE desconectado: detener
        setLed(COLOR_IDLE);
    }

    // Calcular corrección angular solo si el robot está en movimiento
    bool isMoving      = (vx != 0.0f || vy != 0.0f);
    float wCorrection  = computeYawCorrection(isMoving);

    // Inyectar la corrección angular en la cinemática omniwheel
    omniDrive(vx, vy, wCorrection);
    updateAllPID();
}
