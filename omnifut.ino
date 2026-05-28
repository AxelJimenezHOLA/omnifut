#include <Adafruit_NeoPixel.h>

// =============================================================================
//  Pines de hardware
// =============================================================================
#define PIN_PUSH      45
#define PIN_BATTERY    9
#define PIN_LED        3
#define PIN_STM_RX    15
#define PIN_STM_TX    16

// =============================================================================
//  Configuración PWM
// =============================================================================
#define PWM_MAX        200     // Límite superior de PWM (0–255 a 8 bits)
#define PWM_FREQUENCY 4000
#define PWM_RESOLUTION   8

// =============================================================================
//  Parámetros cinemáticos y de encoder — ajustar según hardware real
// =============================================================================
#define RPM_MAX          300.0f   // RPM máximas del motor a PWM_MAX
#define PPR               20      // Pulsos por revolución del encoder (solo canal A)
#define PID_INTERVAL_MS   20      // Período de actualización PID en ms

// =============================================================================
//  Tiempos y velocidades de la máquina de estados
// =============================================================================
#define TIME_ADVANCE_MS   2000
#define TIME_REVERSE_MS   1000

#define SPEED_ADVANCE     0.40f
#define SPEED_REVERSE     0.40f
#define SPEED_SEARCH      0.18f

// =============================================================================
//  Colores LED
// =============================================================================
#define COLOR_SEARCH   pixels.Color(  0,   0, 150)
#define COLOR_ADVANCE  pixels.Color(  0, 150,   0)
#define COLOR_REVERSE  pixels.Color(150,   0,   0)
#define COLOR_IDLE     pixels.Color( 50,  50,  50)
#define COLOR_READY    pixels.Color(  0, 150,   0)

// =============================================================================
//  Número de motores
// =============================================================================
#define MOTOR_COUNT 4

// =============================================================================
//  Estructura PID por motor
// =============================================================================
struct MotorPID {
    float targetRPM;
    float currentRPM;
    float integral;
    float previousError;
    int   pwmOutput;
    volatile long encoderCount;   // modificado en ISR
    long  lastEncoderCount;
    unsigned long lastUpdateTime;
    float kp;
    float ki;
    float kd;
};

// =============================================================================
//  Estructura de motor: pines hardware + estado PID
// =============================================================================
struct Motor {
    uint8_t pwmP;
    uint8_t pwmN;
    uint8_t encoderA;   // canal A (interrupción)
    uint8_t encoderB;   // canal B (dirección, opcional)
    bool    invertDir;  // true = invertir signo de velocidad objetivo
    MotorPID pid;
};

// Índices de motor
enum MotorIndex { FRONT_LEFT = 0, FRONT_RIGHT, BACK_RIGHT, BACK_LEFT };

// =============================================================================
//  Definición centralizada de motores
//  Ajustar encoderA/B según los pines físicos del ESP32 conectados al encoder.
//  Los pines de encoder deben ser GPIO con capacidad de interrupción.
// =============================================================================
static Motor motors[MOTOR_COUNT] = {
    // pwmP  pwmN  encA  encB  invertDir  pid (inicializado abajo)
    {  1,    2,    4,    5,    false,  {0,0,0,0,0,0,0,0, 1.2f, 0.05f, 0.02f} }, // FRONT_LEFT
    {  6,    7,    19,   21,   true,   {0,0,0,0,0,0,0,0, 1.2f, 0.05f, 0.02f} }, // FRONT_RIGHT
    { 17,   18,    22,   23,   true,   {0,0,0,0,0,0,0,0, 1.2f, 0.05f, 0.02f} }, // BACK_RIGHT
    { 37,   38,    25,   26,   false,  {0,0,0,0,0,0,0,0, 1.2f, 0.05f, 0.02f} }, // BACK_LEFT
};

// =============================================================================
//  ISR de encoders (una por motor, referencia al contador del arreglo)
// =============================================================================
static void IRAM_ATTR isr_enc0() { motors[0].pid.encoderCount++; }
static void IRAM_ATTR isr_enc1() { motors[1].pid.encoderCount++; }
static void IRAM_ATTR isr_enc2() { motors[2].pid.encoderCount++; }
static void IRAM_ATTR isr_enc3() { motors[3].pid.encoderCount++; }

static void (*const ENCODER_ISRS[MOTOR_COUNT])() = {
    isr_enc0, isr_enc1, isr_enc2, isr_enc3
};

// =============================================================================
//  Datos recibidos del STM32
// =============================================================================
struct StmData {
    bool    ballFound = false;
    int16_t ballCx    = 0;
    int16_t ballCy    = 0;
    bool    sideFound = false;
    uint8_t sideId    = 0;
    int16_t sideCx    = 0;
    int16_t sideCy    = 0;
};
static StmData stm;

// =============================================================================
//  Máquina de estados del robot
// =============================================================================
enum RobotState { STATE_SEARCH, STATE_ADVANCE, STATE_REVERSE };
static RobotState    currentState = STATE_SEARCH;
static unsigned long stateTimer   = 0;
static bool          robotRunning = false;

// =============================================================================
//  Periféricos
// =============================================================================
HardwareSerial stmSerial(1);
Adafruit_NeoPixel pixels(1, PIN_LED, NEO_GRB + NEO_KHZ800);

// ── RemoteXY (BLE) ────────────────────────────────────────────────────────────
#define REMOTEXY_MODE__ESP32CORE_BLE
#include <BLEDevice.h>
#define REMOTEXY_BLUETOOTH_NAME "RobofutOreo"
#include <RemoteXY.h>

#pragma pack(push, 1)
uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] = {
    255,1,0,0,0,41,0,19,0,0,0,82,111,98,111,102,117,116,79,114,
    101,111,0,31,1,106,200,1,1,1,0,10,5,76,96,93,49,4,26,31,
    79,78,0,31,79,70,70,0
};
struct { uint8_t button_01; uint8_t connect_flag; } RemoteXY;
#pragma pack(pop)

// =============================================================================
//  Inicialización de motores + encoders
// =============================================================================
static void initMotors() {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        Motor& m = motors[i];

        // Canales PWM
        ledcAttach(m.pwmP, PWM_FREQUENCY, PWM_RESOLUTION);
        ledcAttach(m.pwmN, PWM_FREQUENCY, PWM_RESOLUTION);
        ledcWrite(m.pwmP, 0);
        ledcWrite(m.pwmN, 0);

        // Encoders con interrupción en flanco de subida del canal A
        pinMode(m.encoderA, INPUT_PULLUP);
        pinMode(m.encoderB, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(m.encoderA), ENCODER_ISRS[i], RISING);

        // Estado PID inicial
        m.pid.encoderCount    = 0;
        m.pid.lastEncoderCount = 0;
        m.pid.lastUpdateTime  = millis();
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
//  Actualización PID de un motor individual
//  Llamar cada PID_INTERVAL_MS.
// =============================================================================
static void updateMotorPID(Motor& m) {
    unsigned long now      = millis();
    float dt               = (now - m.pid.lastUpdateTime) / 1000.0f;
    if (dt <= 0.0f) return;

    // Captura atómica del contador (evitar glitch con ISR activa)
    long encNow;
    noInterrupts();
    encNow = m.pid.encoderCount;
    interrupts();

    long deltaTicks  = encNow - m.pid.lastEncoderCount;
    m.pid.lastEncoderCount = encNow;
    m.pid.lastUpdateTime   = now;

    // RPM real = (ticks / PPR) / tiempo_min
    m.pid.currentRPM = (deltaTicks / (float)PPR) / (dt / 60.0f);

    // Error (en RPM)
    float error = m.pid.targetRPM - m.pid.currentRPM;

    // Término integral con anti-windup por saturación
    m.pid.integral += error * dt;
    m.pid.integral  = constrain(m.pid.integral, -RPM_MAX, RPM_MAX);

    // Término derivativo
    float derivative = (error - m.pid.previousError) / dt;
    m.pid.previousError = error;

    // Salida PID → escalar a rango PWM
    float pidOut = m.pid.kp * error
                 + m.pid.ki * m.pid.integral
                 + m.pid.kd * derivative;

    // Mapear RPM → PWM (pendiente lineal)
    m.pid.pwmOutput = (int)(pidOut * (PWM_MAX / RPM_MAX));
    m.pid.pwmOutput = constrain(m.pid.pwmOutput, -PWM_MAX, PWM_MAX);

    applyMotorPWM(m, m.pid.pwmOutput);
}

// Actualiza todos los motores si ha pasado el intervalo
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
    // Mezcla cinemática (4 ruedas a 45°)
    float raw[MOTOR_COUNT] = {
         vx - vy - w,  // FRONT_LEFT
         vx + vy + w,  // FRONT_RIGHT
         vx - vy + w,  // BACK_RIGHT
         vx + vy - w,  // BACK_LEFT
    };

    // Normalización para respetar ±1
    float maxVal = 0.0f;
    for (int i = 0; i < MOTOR_COUNT; i++) maxVal = max(maxVal, fabsf(raw[i]));
    if (maxVal > 1.0f)
        for (int i = 0; i < MOTOR_COUNT; i++) raw[i] /= maxVal;

    // Asignar targetRPM a cada motor (invertir signo según montaje físico)
    for (int i = 0; i < MOTOR_COUNT; i++) {
        float dir = motors[i].invertDir ? -1.0f : 1.0f;
        motors[i].pid.targetRPM = dir * raw[i] * RPM_MAX;
    }
}

static void stopMotors() { omniDrive(0.0f, 0.0f, 0.0f); }

// =============================================================================
//  Lectura del STM32 (protocolo: 0xAA 0xBB + 12 bytes + checksum)
// =============================================================================
static void readStm32() {
    if (stmSerial.available() < 14) return;

    uint8_t buf[14];
    stmSerial.readBytes(buf, 14);

    if (buf[0] != 0xAA || buf[1] != 0xBB) { stmSerial.read(); return; }

    uint8_t checksum = 0;
    for (int i = 0; i < 13; i++) checksum += buf[i];
    if (checksum != buf[13]) { Serial.println("ERROR: checksum STM32"); return; }

    stm.ballFound = buf[2];
    stm.ballCx    = (int16_t)((buf[3]  << 8) | buf[4]);
    stm.ballCy    = (int16_t)((buf[5]  << 8) | buf[6]);
    stm.sideId    = buf[7];
    stm.sideFound = buf[8];
    stm.sideCx    = (int16_t)((buf[9]  << 8) | buf[10]);
    stm.sideCy    = (int16_t)((buf[11] << 8) | buf[12]);
}

// =============================================================================
//  Helpers de LED
// =============================================================================
static void setLed(uint32_t color) {
    pixels.setPixelColor(0, color);
    pixels.show();
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
    Serial.begin(115200);
    stmSerial.begin(115200, SERIAL_8N1, PIN_STM_RX, PIN_STM_TX);
    RemoteXY_Init();
    delay(500);

    pinMode(PIN_PUSH, INPUT_PULLUP);
    initMotors();

    pixels.begin();
    pixels.clear();
    setLed(COLOR_READY);
}

// =============================================================================
//  LOOP
// =============================================================================
void loop() {
    readStm32();
    RemoteXYEngine.handler();

    float vx = 0.0f, vy = 0.0f, w = 0.0f;

    // ── Botón BLE: toggle ─────────────────────────────────────────────────────
    bool wasRunning = robotRunning;
    robotRunning = RemoteXY.connect_flag && (RemoteXY.button_01 == 1);
    if (wasRunning && !robotRunning) currentState = STATE_SEARCH;

    // ── Máquina de estados ────────────────────────────────────────────────────
    if (robotRunning) {
        switch (currentState) {

            case STATE_SEARCH:
                w = SPEED_SEARCH;
                if (stm.ballFound) {
                    currentState = STATE_ADVANCE;
                    stateTimer   = millis();
                    Serial.println("Pelota detectada → AVANZAR 2s");
                }
                break;

            case STATE_ADVANCE:
                vx = SPEED_ADVANCE;
                if (millis() - stateTimer >= TIME_ADVANCE_MS) {
                    currentState = STATE_REVERSE;
                    stateTimer   = millis();
                    Serial.println("2s cumplidos → RETROCEDER 1s");
                }
                break;

            case STATE_REVERSE:
                vx = -SPEED_REVERSE;
                if (millis() - stateTimer >= TIME_REVERSE_MS) {
                    currentState = STATE_SEARCH;
                    Serial.println("1s cumplido → vuelve a BUSCAR");
                }
                break;
        }

        const uint32_t STATE_COLORS[] = { COLOR_SEARCH, COLOR_ADVANCE, COLOR_REVERSE };
        setLed(STATE_COLORS[currentState]);

    } else {
        setLed(COLOR_IDLE);
    }

    // omniDrive asigna targetRPM; updateAllPID cierra el lazo periódicamente
    omniDrive(vx, vy, w);
    updateAllPID();
}
