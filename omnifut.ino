#include <Adafruit_NeoPixel.h>
 
#define PUSH     45
#define BATTERY   9
#define LED       3
 
#define STM_RX  15
#define STM_TX  16
 
#define FREQUENCY_PWM  4000
#define RESOLUTION_PWM    8
#define MAX_PWM_SPIN    120
 
#define MOTOR_COUNT      4
#define RPM_MAX       1000.0f
#define PPR            200
#define PID_INTERVAL_MS 20
 
#define VEL_AVANCE      0.30f
#define VEL_BUSQUEDA    0.20f
 
#define CAM_CENTER_X    0
#define TIEMPO_PORTERIA 1200

#define VEL_GOL          0.45f
#define KP_PORTERIA      0.003f
#define KP_BALON         0.003f
#define MAX_CORRECCION   0.10f

int16_t ultimaPorteriaX = 0;
bool porteriaVistaAlgunaVez = false;
 
bool    stm_found   = false;
int16_t stm_cx      = 0;
int16_t stm_cy      = 0;
bool    stm_s_found = false;
uint8_t stm_s_id    = 0;
int16_t stm_s_cx    = 0;
int16_t stm_s_cy    = 0;
 
float        spinDir              = VEL_BUSQUEDA;
float        spinDirPorteria      = VEL_BUSQUEDA;
unsigned long tiempoAvanzarNaranja = 0;
 
enum RobotState { BUSCAR_NARANJA, AVANZAR_NARANJA, BUSCAR_PORTERIA, AVANZAR_PORTERIA };
RobotState estadoActual = BUSCAR_NARANJA;
 
struct MotorPID {
    float        targetRPM;
    float        currentRPM;
    float        integral;
    float        previousError;
    int          pwmOutput;
    volatile long encoderCount;
    long         lastEncoderCount;
    unsigned long lastUpdateTime;
    float        kp;
    float        ki;
    float        kd;
    float        lastTargetSign;
};
 
struct Motor {
    uint8_t  pwmP;
    uint8_t  pwmN;
    uint8_t  encoderA;
    uint8_t  encoderB;
    float    bias;
    MotorPID pid;
};
 
enum MotorIndex { FRONT_LEFT = 0, FRONT_RIGHT, BACK_RIGHT, BACK_LEFT };
 
static Motor motors[MOTOR_COUNT] = {
    { 2,  1,  4,  5,  0.0f, { 0,0,0,0,0,0,0,0, 1.0f, 0.05f, 0.02f, 0.0f } },
    { 7,  6, 19, 21,  0.0f, { 0,0,0,0,0,0,0,0, 1.0f, 0.05f, 0.02f, 0.0f } },
    { 18,17, 22, 23,  0.0f, { 0,0,0,0,0,0,0,0, 1.0f, 0.05f, 0.02f, 0.0f } },
    { 38,37, 25, 26,  0.0f, { 0,0,0,0,0,0,0,0, 1.0f, 0.05f, 0.02f, 0.0f } },
};
 
static void IRAM_ATTR isr_enc0() { motors[0].pid.encoderCount += (digitalRead(motors[0].encoderB) == HIGH) ? 1 : -1; }
static void IRAM_ATTR isr_enc1() { motors[1].pid.encoderCount += (digitalRead(motors[1].encoderB) == HIGH) ? 1 : -1; }
static void IRAM_ATTR isr_enc2() { motors[2].pid.encoderCount += (digitalRead(motors[2].encoderB) == HIGH) ? 1 : -1; }
static void IRAM_ATTR isr_enc3() { motors[3].pid.encoderCount += (digitalRead(motors[3].encoderB) == HIGH) ? 1 : -1; }
 
static void (*const ENCODER_ISRS[MOTOR_COUNT])() = { isr_enc0, isr_enc1, isr_enc2, isr_enc3 };
 
HardwareSerial stm(1);
Adafruit_NeoPixel pixels(1, LED, NEO_GRB + NEO_KHZ800);
 
#define REMOTEXY_MODE__ESP32CORE_BLE
#include <BLEDevice.h>
#define REMOTEXY_BLUETOOTH_NAME "RobofutLuis"
#include <RemoteXY.h>
 
#pragma pack(push, 1)
uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] =
  { 255,1,0,0,0,33,0,19,0,0,0,82,111,98,111,102,117,116,76,117,
    105,115,0,31,1,106,200,1,1,1,0,1,40,139,24,24,0,2,31,0 };
 
struct {
    uint8_t button_01;
    uint8_t connect_flag;
} RemoteXY;
#pragma pack(pop)
 
bool    robotRunning    = false;
uint8_t lastButtonState = 0;
 
static void applyMotorPWM(Motor& m, int pwm) {
    pwm = constrain(pwm, -255, 255);
    ledcWrite(m.pwmP, pwm > 0 ? (uint32_t)pwm   : 0);
    ledcWrite(m.pwmN, pwm < 0 ? (uint32_t)(-pwm) : 0);
}
 
static void initMotors() {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        Motor& m = motors[i];
        ledcAttach(m.pwmP, FREQUENCY_PWM, RESOLUTION_PWM);
        ledcAttach(m.pwmN, FREQUENCY_PWM, RESOLUTION_PWM);
        ledcWrite(m.pwmP, 0);
        ledcWrite(m.pwmN, 0);
        pinMode(m.encoderA, INPUT_PULLUP);
        pinMode(m.encoderB, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(m.encoderA), ENCODER_ISRS[i], RISING);
        m.pid.encoderCount     = 0;
        m.pid.lastEncoderCount = 0;
        m.pid.lastUpdateTime   = millis();
        m.pid.integral         = 0.0f;
        m.pid.previousError    = 0.0f;
        m.pid.lastTargetSign   = 0.0f;
    }
}
 
static void stopAll() {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motors[i].pid.targetRPM      = 0.0f;
        motors[i].pid.integral       = 0.0f;
        motors[i].pid.previousError  = 0.0f;
        applyMotorPWM(motors[i], 0);
    }
}
 
static void resetPIDState() {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motors[i].pid.integral         = 0.0f;
        motors[i].pid.previousError    = 0.0f;
        motors[i].pid.lastTargetSign   = 0.0f;
        motors[i].pid.lastUpdateTime   = millis();
        noInterrupts();
        motors[i].pid.lastEncoderCount = motors[i].pid.encoderCount;
        interrupts();
    }
}
 
static void updateMotorPID(Motor& m) {
    unsigned long now = millis();
    float dt = (now - m.pid.lastUpdateTime) / 1000.0f;
    if (dt <= 0.0f) return;
 
    long encNow;
    noInterrupts();
    encNow = m.pid.encoderCount;
    interrupts();
 
    long deltaTicks        = encNow - m.pid.lastEncoderCount;
    m.pid.lastEncoderCount = encNow;
    m.pid.lastUpdateTime   = now;
 
    m.pid.currentRPM = (deltaTicks / (float)PPR) / (dt / 60.0f);
 
    float effectiveTarget = m.pid.targetRPM + m.bias;
 
    if (fabsf(effectiveTarget) < 0.5f && fabsf(m.pid.currentRPM) < 5.0f) {
        m.pid.integral      = 0.0f;
        m.pid.previousError = 0.0f;
        m.pid.pwmOutput     = 0;
        applyMotorPWM(m, 0);
        return;
    }
 
    float currentSign = (effectiveTarget > 0.0f) ? 1.0f : ((effectiveTarget < 0.0f) ? -1.0f : 0.0f);
    if (currentSign != 0.0f && currentSign != m.pid.lastTargetSign) {
        m.pid.integral       = 0.0f;
        m.pid.previousError  = 0.0f;
        m.pid.lastTargetSign = currentSign;
    }
 
    float error = effectiveTarget - m.pid.currentRPM;
    float integralLimit = RPM_MAX * 0.5f;
    m.pid.integral += error * dt;
    m.pid.integral  = constrain(m.pid.integral, -integralLimit, integralLimit);
 
    float derivative    = (error - m.pid.previousError) / dt;
    m.pid.previousError = error;
 
    float pidOut = m.pid.kp * error + m.pid.ki * m.pid.integral + m.pid.kd * derivative;
    m.pid.pwmOutput = (int)(pidOut * (255.0f / RPM_MAX));
    m.pid.pwmOutput = constrain(m.pid.pwmOutput, -255, 255);
 
    applyMotorPWM(m, m.pid.pwmOutput);
}
 
static void updateAllPID() {
    static unsigned long lastRun = 0;
    if (millis() - lastRun < PID_INTERVAL_MS) return;
    lastRun = millis();
    for (int i = 0; i < MOTOR_COUNT; i++) updateMotorPID(motors[i]);
}
 
static void omniDrive(float vx, float vy, float w) {
    float raw[MOTOR_COUNT] = {
        vx - vy - w,
        vx + vy + w,
        vx - vy + w,
        vx + vy - w,
    };
    float maxVal = 0.0f;
    for (int i = 0; i < MOTOR_COUNT; i++) maxVal = max(maxVal, fabsf(raw[i]));
    if (maxVal > 1.0f)
        for (int i = 0; i < MOTOR_COUNT; i++) raw[i] /= maxVal;
    for (int i = 0; i < MOTOR_COUNT; i++)
        motors[i].pid.targetRPM = raw[i] * RPM_MAX;
}
 
static void spinDirect(float w) {
    int pwm = (int)(w * MAX_PWM_SPIN);
    applyMotorPWM(motors[FRONT_LEFT],  -pwm);
    applyMotorPWM(motors[FRONT_RIGHT],  pwm);
    applyMotorPWM(motors[BACK_RIGHT],   pwm);
    applyMotorPWM(motors[BACK_LEFT],   -pwm);
}
 
void leerSTM32() {
    if (stm.available() >= 14) {
        uint8_t buf[14];
        stm.readBytes(buf, 14);
        if (buf[0] == 0xAA && buf[1] == 0xBB) {
            uint8_t chk = 0;
            for (int i = 0; i < 13; i++) chk += buf[i];
            if (chk == buf[13]) {
                stm_found   = buf[2];
                stm_cx      = (int16_t)((buf[3]  << 8) | buf[4]);
                stm_cy      = (int16_t)((buf[5]  << 8) | buf[6]);
                stm_s_id    = buf[7];
                stm_s_found = buf[8];
                stm_s_cx    = (int16_t)((buf[9]  << 8) | buf[10]);
                stm_s_cy    = (int16_t)((buf[11] << 8) | buf[12]);
            } else {
                Serial.println("ERROR checksum (STM32)");
            }
        } else {
            stm.read();
        }
    }
    if (stm_s_found) {
        ultimaPorteriaX = stm_s_cx;
        porteriaVistaAlgunaVez = true;
    }
}
 
void setup() {
    Serial.begin(115200);
    stm.begin(115200, SERIAL_8N1, STM_RX, STM_TX);
    RemoteXY_Init();
    delay(500);
 
    pinMode(PUSH, INPUT_PULLUP);
    initMotors();
 
    pixels.begin();
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(0, 150, 0));
    pixels.show();
 
    while (digitalRead(PUSH));
    for (int k = 0; k < 4; k++) {
        pixels.setPixelColor(0, pixels.Color(150, 150, 0));
        pixels.show(); delay(500);
        pixels.setPixelColor(0, pixels.Color(0, 0, 0));
        pixels.show(); delay(500);
    }
}
 
void loop() {
    leerSTM32();
    RemoteXYEngine.handler();
 
    if (RemoteXY.connect_flag != 0) {
        if (RemoteXY.button_01 == 1 && lastButtonState == 0) {
            robotRunning = !robotRunning;
            if (!robotRunning) {
                estadoActual = BUSCAR_NARANJA;
                stopAll();
            }
        }
        lastButtonState = RemoteXY.button_01;
    } else {
        if (robotRunning) stopAll();
        robotRunning    = false;
        lastButtonState = 0;
    }
 
    if (robotRunning) {
        switch (estadoActual) {
 
            case BUSCAR_NARANJA:
                spinDirect(spinDir);
                if (stm_found) {
                    spinDir              = (stm_cx < CAM_CENTER_X) ? VEL_BUSQUEDA : -VEL_BUSQUEDA;
                    tiempoAvanzarNaranja = millis();
                    estadoActual         = AVANZAR_NARANJA;
                    porteriaVistaAlgunaVez = false;
                    resetPIDState();
                }
                break;
 
            case AVANZAR_NARANJA:
                if (!stm_found) {
                    estadoActual = BUSCAR_NARANJA;
                    stopAll();
                    break;
                }
                
                spinDir = (stm_cx < CAM_CENTER_X) ? VEL_BUSQUEDA : -VEL_BUSQUEDA;

                {
                    float errorBalon = (float)(CAM_CENTER_X - stm_cx);
                    float correccionBalon = constrain(errorBalon * KP_BALON, -MAX_CORRECCION, MAX_CORRECCION);
                    omniDrive(VEL_AVANCE, 0.0f, correccionBalon);
                }

                if (stm_s_found) {
                    ultimaPorteriaX = stm_s_cx;
                    porteriaVistaAlgunaVez = true;
                }

                if (millis() - tiempoAvanzarNaranja >= TIEMPO_PORTERIA) {
                    estadoActual = BUSCAR_PORTERIA;
                    stopAll();
                    break;
                }
                updateAllPID();
                break;
 
            case BUSCAR_PORTERIA:
                if (!stm_found) {
                    estadoActual = BUSCAR_NARANJA;
                    stopAll();
                    break;
                }

                if (stm_s_found) {
                    estadoActual = AVANZAR_PORTERIA;
                    resetPIDState();
                    break;
                }

                if (porteriaVistaAlgunaVez) {
                    float giro = (ultimaPorteriaX > CAM_CENTER_X) ? VEL_BUSQUEDA : -VEL_BUSQUEDA;
                    omniDrive(0.0f, 0.0f, giro);
                } 
                else {
                    omniDrive(0.0f, 0.0f, spinDirPorteria);
                }
                
                updateAllPID();
                break;
 
            case AVANZAR_PORTERIA:
                if (!stm_found) {
                    estadoActual = BUSCAR_NARANJA;
                    stopAll();
                    break;
                }
                
                if (!stm_s_found) {
                    omniDrive(VEL_GOL, 0.0f, 0.0f);
                } 
                else {
                    float errorPorteria = (float)(CAM_CENTER_X - stm_s_cx);
                    float correccion = errorPorteria * KP_PORTERIA;
                    correccion = constrain(correccion, -MAX_CORRECCION, MAX_CORRECCION);
                    
                    omniDrive(VEL_GOL, 0.0f, correccion); 
                }
                
                updateAllPID();
                break;
        }
 
        switch (estadoActual) {
            case BUSCAR_NARANJA:   pixels.setPixelColor(0, pixels.Color(0,   0, 150)); break;
            case AVANZAR_NARANJA:  pixels.setPixelColor(0, pixels.Color(0, 150,   0)); break;
            case BUSCAR_PORTERIA:  pixels.setPixelColor(0, pixels.Color(150, 0, 150)); break;
            case AVANZAR_PORTERIA: pixels.setPixelColor(0, pixels.Color(150,150,   0)); break;
        }
        pixels.show();
 
    } else {
        pixels.setPixelColor(0, pixels.Color(50, 50, 50));
        pixels.show();
    }
}
