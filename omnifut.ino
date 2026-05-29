// =============================================================================
//  RobofutOreo — Control manual BLE mediante RemoteXY + PID por encoder (ESP32)
//  Fuente de comandos: botones RemoteXY  → omniDrive() → PID → PWM
// =============================================================================

#include <Adafruit_NeoPixel.h>

// =============================================================================
//  Pines de hardware
// =============================================================================
#define PIN_PUSH      45
#define PIN_BATTERY    9
#define PIN_LED        3

// =============================================================================
//  Configuración PWM
// =============================================================================
#define PWM_MAX        255
#define PWM_FREQUENCY 4000
#define PWM_RESOLUTION   8

// =============================================================================
//  Parámetros cinemáticos y de encoder — ajustar según hardware real
// =============================================================================
#define RPM_MAX         1000.0f   // RPM máximas del motor a PWM_MAX
#define PPR              200      // Pulsos por revolución del encoder (canal A)
#define PID_INTERVAL_MS   20     // Período de actualización PID en ms

// =============================================================================
//  Velocidad de traslación manual (fracción de RPM_MAX, rango 0–1)
// =============================================================================
#define SPEED_MANUAL    0.60f

// =============================================================================
//  Colores LED
// =============================================================================
#define COLOR_IDLE     pixels.Color( 50,  50,  50)   // blanco tenue — toggle OFF
#define COLOR_READY    pixels.Color(  0, 150,   0)   // verde — toggle ON, quieto
#define COLOR_MOVING   pixels.Color(  0,   0, 150)   // azul — en movimiento

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
    volatile long encoderCount;    // modificado en ISR
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
    uint8_t encoderA;    // canal A (interrupción)
    uint8_t encoderB;    // canal B (dirección, opcional)
    bool    invertDir;   // true = invertir signo de velocidad objetivo
    MotorPID pid;
};

// Índices de motor
enum MotorIndex { FRONT_LEFT = 0, FRONT_RIGHT, BACK_RIGHT, BACK_LEFT };

// =============================================================================
//  Definición centralizada de motores
//  Ajustar encoderA/B según pines físicos del ESP32 con capacidad de interrupción
// =============================================================================
static Motor motors[MOTOR_COUNT] = {
    // pwmP  pwmN  encA  encB  invertDir  pid (kp, ki, kd)
    {  1,    2,    4,    5,    false,  {0,0,0,0,0,0,0,0, 1.2f, 0.05f, 0.02f} }, // FRONT_LEFT
    {  6,    7,    19,   21,   true,   {0,0,0,0,0,0,0,0, 1.2f, 0.05f, 0.02f} }, // FRONT_RIGHT
    { 17,   18,    22,   23,   true,   {0,0,0,0,0,0,0,0, 1.2f, 0.05f, 0.02f} }, // BACK_RIGHT
    { 37,   38,    25,   26,   false,  {0,0,0,0,0,0,0,0, 1.2f, 0.05f, 0.02f} }, // BACK_LEFT
};

// =============================================================================
//  ISR de encoders — una por motor, solo incrementa el contador
// =============================================================================
static void IRAM_ATTR isr_enc0() { motors[0].pid.encoderCount++; }
static void IRAM_ATTR isr_enc1() { motors[1].pid.encoderCount++; }
static void IRAM_ATTR isr_enc2() { motors[2].pid.encoderCount++; }
static void IRAM_ATTR isr_enc3() { motors[3].pid.encoderCount++; }

static void (*const ENCODER_ISRS[MOTOR_COUNT])() = {
    isr_enc0, isr_enc1, isr_enc2, isr_enc3
};

// =============================================================================
//  Periféricos
// =============================================================================
Adafruit_NeoPixel pixels(1, PIN_LED, NEO_GRB + NEO_KHZ800);

// ── RemoteXY (BLE) ────────────────────────────────────────────────────────────
#define REMOTEXY_MODE__ESP32CORE_BLE
#include <BLEDevice.h>
#define REMOTEXY_BLUETOOTH_NAME "RobofutOreo"
#include <RemoteXY.h>

#pragma pack(push, 1)
uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] =   // 96 bytes V19
  { 255,5,0,0,0,89,0,19,0,0,0,82,111,98,111,102,117,116,79,114,
  101,111,0,29,1,106,200,1,1,5,0,10,5,152,97,41,49,4,26,31,
  79,78,0,31,79,70,70,0,1,41,59,24,24,1,192,31,226,134,145,0,
  1,15,86,24,24,1,192,31,226,134,144,0,1,67,86,24,24,1,192,31,
  226,134,146,0,1,41,112,24,24,1,192,31,226,134,147,0 };

struct {
    uint8_t button_toggle;  // =1 si está ON, =0 si OFF
    uint8_t button_up;      // =1 mientras se mantiene pulsado
    uint8_t button_left;
    uint8_t button_right;
    uint8_t button_down;
    uint8_t connect_flag;   // =1 si BLE conectado
} RemoteXY;
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
//  Actualización PID de un motor individual
//  Llamar cada PID_INTERVAL_MS
// =============================================================================
static void updateMotorPID(Motor& m) {
    unsigned long now = millis();
    float dt          = (now - m.pid.lastUpdateTime) / 1000.0f;
    if (dt <= 0.0f) return;

    // Captura atómica del contador (evitar glitch con ISR activa)
    long encNow;
    noInterrupts();
    encNow = m.pid.encoderCount;
    interrupts();

    long deltaTicks        = encNow - m.pid.lastEncoderCount;
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
    float derivative    = (error - m.pid.previousError) / dt;
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
//  Helpers de LED
// =============================================================================
static void setLed(uint32_t color) {
    pixels.setPixelColor(0, color);
    pixels.show();
}

// =============================================================================
//  Leer botones de dirección → vx, vy
//  Los botones son momentáneos: se activan mientras se mantienen pulsados.
//  Múltiples botones simultáneos se combinan (ej. UP+RIGHT = diagonal).
// =============================================================================
static void readDirectionButtons(float& vx, float& vy) {
    vx = 0.0f;
    vy = 0.0f;
    if (RemoteXY.button_up)    vx += SPEED_MANUAL;
    if (RemoteXY.button_down)  vx -= SPEED_MANUAL;
    if (RemoteXY.button_right) vy += SPEED_MANUAL;
    if (RemoteXY.button_left)  vy -= SPEED_MANUAL;
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

    pixels.begin();
    pixels.clear();
    setLed(COLOR_IDLE);
}

// =============================================================================
//  LOOP
// =============================================================================
void loop() {
    RemoteXYEngine.Handler();

    float vx = 0.0f, vy = 0.0f;

    if (RemoteXY.connect_flag && RemoteXY.button_toggle) {
        // ── Toggle ON: leer botones y mover ──────────────────────────────────
        readDirectionButtons(vx, vy);

        bool moving = (vx != 0.0f || vy != 0.0f);
        setLed(moving ? COLOR_MOVING : COLOR_READY);

    } else {
        // ── Toggle OFF o BLE desconectado: detener ────────────────────────────
        setLed(COLOR_IDLE);
    }

    // omniDrive asigna targetRPM; updateAllPID cierra el lazo periódicamente
    omniDrive(vx, vy, 0.0f);
    updateAllPID();
}
