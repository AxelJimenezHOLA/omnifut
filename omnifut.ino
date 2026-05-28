#include <Adafruit_NeoPixel.h>

// ── Pines de hardware ─────────────────────────────────────────────────────────
#define PIN_PUSH    45
#define PIN_BATTERY  9
#define PIN_LED      3

#define PIN_STM_RX  15
#define PIN_STM_TX  16

// ── PWM ───────────────────────────────────────────────────────────────────────
#define PWM_MAX        120
#define PWM_FREQUENCY 4000
#define PWM_RESOLUTION   8

// ── Tiempos y velocidades de la máquina de estados ───────────────────────────
#define TIME_ADVANCE_MS    2000
#define TIME_REVERSE_MS    1000

#define SPEED_ADVANCE   0.40f
#define SPEED_REVERSE   0.40f
#define SPEED_SEARCH    0.18f

// ── Colores LED ───────────────────────────────────────────────────────────────
#define COLOR_SEARCH   pixels.Color(  0,   0, 150)   // azul
#define COLOR_ADVANCE  pixels.Color(  0, 150,   0)   // verde
#define COLOR_REVERSE  pixels.Color(150,   0,   0)   // rojo
#define COLOR_IDLE     pixels.Color( 50,  50,  50)   // blanco tenue
#define COLOR_READY    pixels.Color(  0, 150,   0)   // verde espera
#define COLOR_BLINK    pixels.Color(150, 150,   0)   // amarillo parpadeo

// =============================================================================
//  Estructura de motor: pines PWM+ y PWM-
// =============================================================================
struct Motor {
  uint8_t pwmP;
  uint8_t pwmN;
};

// Índices de motor
enum MotorIndex { FRONT_LEFT = 0, FRONT_RIGHT, BACK_RIGHT, BACK_LEFT, MOTOR_COUNT };

// Factores de corrección por motor (ajuste empírico de tracción)
static const float MOTOR_TRIM[MOTOR_COUNT] = { 1.00f, 1.25f, 1.25f, 1.00f };

// Definición centralizada de motores
static const Motor MOTORS[MOTOR_COUNT] = {
  { 1,  2 },   // FRONT_LEFT  — Front Left  PWM_P=1,  PWM_N=2
  { 6,  7 },   // FRONT_RIGHT — Front Right PWM_P=6,  PWM_N=7
  { 17, 18 },  // BACK_RIGHT  —  Back Right PWM_P=17, PWM_N=18
  { 37, 38 },  // BACK_LEFT   —  Back Left  PWM_P=37, PWM_N=38
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
static RobotState currentState  = STATE_SEARCH;
static unsigned long stateTimer = 0;
static bool robotRunning = false;

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
//  Inicialización de motores
// =============================================================================
static void initMotors() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    ledcAttach(MOTORS[i].pwmP, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(MOTORS[i].pwmN, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcWrite(MOTORS[i].pwmP, 0);
    ledcWrite(MOTORS[i].pwmN, 0);
  }
}

// =============================================================================
//  Control de un motor individual
// =============================================================================
static void driveMotor(const Motor& m, int speed) {
  speed = constrain(speed, -PWM_MAX, PWM_MAX);
  ledcWrite(m.pwmP, speed > 0 ? speed : 0);
  ledcWrite(m.pwmN, speed < 0 ? -speed : 0);
}

// =============================================================================
//  Cinemática omni-drive → PWM en los 4 motores
// =============================================================================
static void omniDrive(float vx, float vy, float w) {
  // Mezcla cinemática omni (4 ruedas a 45°)
  float raw[MOTOR_COUNT] = {
    vx - vy - w,   // FRONT_LEFT
    vx + vy + w,   // FRONT_RIGHT
    vx - vy + w,   // BACK_RIGHT
    vx + vy - w,   // BACK_LEFT
  };

  // Normalización para no superar ±1
  float maxVal = 0.0f;
  for (int i = 0; i < MOTOR_COUNT; i++) maxVal = max(maxVal, abs(raw[i]));
  if (maxVal > 1.0f)
    for (int i = 0; i < MOTOR_COUNT; i++) raw[i] /= maxVal;

  // Aplicar al hardware (señal negada + trim)
  for (int i = 0; i < MOTOR_COUNT; i++)
    driveMotor(MOTORS[i], -(int)(raw[i] * PWM_MAX * MOTOR_TRIM[i]));
}

static void stopMotors() { omniDrive(0, 0, 0); }

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
  stm.ballCx    = (int16_t)((buf[3] << 8) | buf[4]);
  stm.ballCy    = (int16_t)((buf[5] << 8) | buf[6]);
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

  // ── Botón BLE: toggle (1 = ON, 0 = OFF) ──────────────────────────────────
  bool wasRunning = robotRunning;
  robotRunning = RemoteXY.connect_flag && (RemoteXY.button_01 == 1);
  if (wasRunning && !robotRunning) currentState = STATE_SEARCH;

  // ── Máquina de estados ─────────────────────────────────────────────────────
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

    // LED según estado activo
    const uint32_t STATE_COLORS[] = { COLOR_SEARCH, COLOR_ADVANCE, COLOR_REVERSE };
    setLed(STATE_COLORS[currentState]);

  } else {
    setLed(COLOR_IDLE);
  }

  omniDrive(vx, vy, w);
}
