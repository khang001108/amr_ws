/**
 * ============================================================
 *  AMR ESP32 Firmware — ROS 2 Jazzy Compatible
 * ============================================================
 *  Hardware:
 *    Motor Driver : L298N
 *    IMU          : BNO055 (I2C 0x29, SDA=GPIO21, SCL=GPIO22)
 *    Encoder L    : A=GPIO34, B=GPIO35
 *    Encoder R    : A=GPIO32, B=GPIO33
 *    Motor L (A)  : ENA=GPIO25, IN1=GPIO26, IN2=GPIO27
 *    Motor R (B)  : ENB=GPIO14, IN3=GPIO12, IN4=GPIO13
 *    Wheel radius : 43.5 mm (diameter 87 mm)
 *    Wheel base   : 200 mm
 *    Motor        : 12V, 110 RPM at full load
 *    Encoder CPR  : 4400 counts/rev (calibrated from measured travel)
 *
 *  Serial protocol (115200 baud):
 *    PC → ESP32 : "linear_x,angular_z\n"   (e.g. "0.3,0.1\n")
 *    ESP32 → PC : tagged lines (prefix everything so PC can parse easily)
 *      ODOM,encL,encR,rpmL,rpmR\n
 *      IMU,yaw\n
 *      DBG,...\n   (debug only, PC should ignore)
 *
 *  FIXES applied vs previous version:
 *    1. motorControl() right motor: correctedPWM now actually used
 *    2. updateRPM(): currentRPM_R negated to match physical direction
 *    3. sendOdom(): encoderRight negated to match
 *    4. PID gains reduced (Ki especially) to prevent integrator windup
 *    5. Anti-windup: pwm reset when inside deadband
 *    6. Sync correction gain reduced 0.3 → 0.1
 * ============================================================
 */

#include <Arduino.h>
#include <PID_v1.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ─── Debug switch ───────────────────────────────────────────
// Set 1 to enable DBG lines, 0 to suppress (cleaner for ROS)
#define DEBUG_ENABLE 0

#if DEBUG_ENABLE
  #define DBG(...) Serial.print("DBG,"); Serial.println(__VA_ARGS__)
#else
  #define DBG(...)
#endif

// ─── Pin definitions ────────────────────────────────────────
#define ENA     25
#define IN1     26
#define IN2     27

#define ENB     14
#define IN3     12
#define IN4     13

#define ENC_L_A 32
#define ENC_L_B 33

#define ENC_R_A 18
#define ENC_R_B 19

// ─── PWM config ─────────────────────────────────────────────
#define PWM_FREQ   20000
#define PWM_RES    8          // 8-bit → 0..255
#define PWM_CH_L   0
#define PWM_CH_R   1
#define MAX_PWM    220       // allow more headroom under load, especially while turning
#define MAX_PID_PWM 75       // PID is only a correction around feed-forward PWM

// ─── Robot geometry ─────────────────────────────────────────
const float WHEEL_RADIUS  = 0.0435f;   // metres
const float WHEEL_BASE    = 0.20f;     // metres

// Calibrated count scale: 10 cm physical travel previously appeared as 1.00 m.
const int   ENCODER_PPR   = 4400;

// Deadband: ignore RPM targets smaller than this (prevents jitter)
const float RPM_DEADBAND  = 4.0f;
const bool  SWAP_ENCODER_SIDES = false;

// L298N loses voltage under load, but too much feed-forward makes the robot jump.
// Tune MIN_MOTOR_PWM first, then PWM_PER_RPM if the robot is still weak.
const int   MIN_MOTOR_PWM = 68;
const int   START_BOOST_PWM = 92;
const float PWM_PER_RPM   = 0.42f;
// const int   TURN_MIN_PWM = 86;
// const int   TURN_START_BOOST_PWM = 112;
// const float TURN_PWM_PER_RPM = 0.58f;
const int TURN_MIN_PWM = 120;
const int TURN_START_BOOST_PWM = 160;
const float TURN_PWM_PER_RPM = 1.2f;
const float LINEAR_ACCEL_LIMIT  = 0.8f;   // m/s^2
const float ANGULAR_ACCEL_LIMIT = 4.0f;   // rad/s^2
const float ANGULAR_CMD_SIGN = 1.0f;      // invert if teleop left/right drives physically reversed
const float LEFT_PWM_TRIM  = 1.00f;
const float RIGHT_PWM_TRIM = 0.95f;
const double SYNC_K = 0.3;
const double MAX_SYNC_PWM = 10.0;

// ─── IMU ────────────────────────────────────────────────────
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
float currentYaw = 0.0f;
float targetYaw  = 0.0f;

// Heading-hold PD gains
// const float HEADING_KP = 0.014f;
// const float HEADING_KD = 0.004f;
const float HEADING_KP = 0.1f;
const float HEADING_KD = 0.006f;
const float HEADING_MAX_CORRECTION = 0.40f; // m/s wheel speed difference command
const unsigned long HEADING_LOCK_DELAY_MS = 80;

// ─── Encoders ───────────────────────────────────────────────
ESP32Encoder encoderLeft;
ESP32Encoder encoderRight;

// ─── PID variables ──────────────────────────────────────────
double currentRPM_L = 0, currentRPM_R = 0;
double targetRPM_L  = 0, targetRPM_R  = 0;
double pwm_L        = 0, pwm_R        = 0;

// Keep gains conservative so the robot does not surge or oscillate.
double Kp_L = 0.7, Ki_L = 0.01, Kd_L = 0.03;
double Kp_R = 0.7, Ki_R = 0.01, Kd_R = 0.03;

PID pidLeft (
    &currentRPM_L, &pwm_L, &targetRPM_L,
    Kp_L, Ki_L, Kd_L, DIRECT);

PID pidRight(
    &currentRPM_R, &pwm_R, &targetRPM_R,
    Kp_R, Ki_R, Kd_R, DIRECT);

// ─── Command ─────────────────────────────────────────────────
float linear_x  = 0.0f;
float angular_z = 0.0f;
float smooth_linear_x  = 0.0f;
float smooth_angular_z = 0.0f;

// ─── Timing ──────────────────────────────────────────────────
unsigned long lastCmdTime   = 0;
unsigned long prevRPMTime   = 0;
unsigned long lastOdomTime  = 0;
unsigned long lastIMUTime   = 0;

const unsigned long CMD_TIMEOUT_MS  = 300;   // stop if no cmd
const unsigned long RPM_PERIOD_MS   = 40;    // PID loop ~25 Hz, smoother encoder feedback
const unsigned long ODOM_PERIOD_MS  = 50;    // odometry publish 20 Hz
const unsigned long IMU_PERIOD_MS   = 50;    // IMU publish 20 Hz

// ─── Encoder state ───────────────────────────────────────────
long prevCountL = 0, prevCountR = 0;

// ─── Forward declarations ────────────────────────────────────
void    updateIMU();
void    readCmdVel();
void    calculateTargetRPM();
bool    updateRPM();
void    motorControl(double leftPWM, double rightPWM);
int     commandPWM(double pidPWM, double targetRPM, double currentRPM, float trim);
void    balanceStraightPWM(int &leftPWM, int &rightPWM);
float   rampValue(float current, float target, float maxStep);
void    stopMotors();
void    sendOdom();
void    sendIMU();
float   wrapAngle(float angle);

// ════════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════════
void setup()
{
    Serial.begin(115200);

    // ── PWM channels ──
    ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
    ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
    ledcAttachPin(ENA, PWM_CH_L);
    ledcAttachPin(ENB, PWM_CH_R);

    // ── Motor direction pins ──
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    stopMotors();

    // ── Encoders ──
    // NOTE: GPIO34/35 are input-only on ESP32 → no internal pull-up available
    //       Use external 10k pull-ups on encoder VCC line instead.
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoderLeft.attachFullQuad (ENC_L_A, ENC_L_B);
    encoderRight.attachFullQuad(ENC_R_A, ENC_R_B);
    encoderLeft.clearCount();
    encoderRight.clearCount();

    // ── PID ──
    pidLeft.SetMode(AUTOMATIC);
    pidRight.SetMode(AUTOMATIC);
    pidLeft.SetOutputLimits (-MAX_PID_PWM, MAX_PID_PWM);
    pidRight.SetOutputLimits(-MAX_PID_PWM, MAX_PID_PWM);
    pidLeft.SetSampleTime (RPM_PERIOD_MS);
    pidRight.SetSampleTime(RPM_PERIOD_MS);

    // ── IMU ──
    Wire.begin(21, 22);
    if (!bno.begin()) {
        pinMode(2, OUTPUT);
        while (1) { digitalWrite(2, !digitalRead(2)); delay(200); }
    }
    delay(1000);
    bno.setExtCrystalUse(true);

    // ── Init timers ──
    unsigned long now = millis();
    lastCmdTime  = now;
    prevRPMTime  = now;
    lastOdomTime = now;
    lastIMUTime  = now;
}

// ════════════════════════════════════════════════════════════
//  LOOP
// ════════════════════════════════════════════════════════════
void loop()
{
    unsigned long now = millis();

    // 1. Read new commands from Serial
    readCmdVel();

    // 2. Command timeout → safe stop
    if (now - lastCmdTime > CMD_TIMEOUT_MS) {
        linear_x  = 0.0f;
        angular_z = 0.0f;
    }

    // 3. Compute target RPM (differential drive + heading hold)
    calculateTargetRPM();

    // 4. RPM feedback + PID at fixed rate
    if (updateRPM()) {
        pidLeft.Compute();
        pidRight.Compute();
        // FIX 5: Anti-windup — reset PWM output when inside deadband
        // Prevents integrator from winding up while motor is braked
        if (fabsf(targetRPM_L) < RPM_DEADBAND) { pwm_L = 0.0; }
        if (fabsf(targetRPM_R) < RPM_DEADBAND) { pwm_R = 0.0; }

        motorControl(pwm_L, pwm_R);
    }

    // 5. Publish odometry
    if (now - lastOdomTime >= ODOM_PERIOD_MS) {
        sendOdom();
        lastOdomTime = now;
    }

    // 6. Publish IMU
    if (now - lastIMUTime >= IMU_PERIOD_MS) {
        updateIMU();
        sendIMU();
        lastIMUTime = now;
    }
}

// ════════════════════════════════════════════════════════════
//  SERIAL INPUT — "linear_x,angular_z\n"
// ════════════════════════════════════════════════════════════
void readCmdVel()
{
    static String buf = "";
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            int comma = buf.indexOf(',');
            if (comma > 0) {
                float lx = buf.substring(0, comma).toFloat();
                float az = buf.substring(comma + 1).toFloat();
                if (!isnan(lx) && !isnan(az) &&
                    fabsf(lx) <= 2.0f && fabsf(az) <= 5.0f)
                {
                    linear_x  = lx;
                    angular_z = az;
                    lastCmdTime = millis();
                }
            }
            buf = "";
        } else {
            if (buf.length() < 32) buf += c;
        }
    }
}

// ════════════════════════════════════════════════════════════
//  IMU
// ════════════════════════════════════════════════════════════
void updateIMU()
{
    imu::Vector<3> euler =
        bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    currentYaw = euler.x();   // degrees, 0-360 from BNO055
}

float wrapAngle(float angle)
{
    while (angle >  180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

// ════════════════════════════════════════════════════════════
//  TARGET RPM CALCULATION (differential drive + heading hold)
// ════════════════════════════════════════════════════════════
void calculateTargetRPM()
{
    static unsigned long lastRampTime = 0;
    unsigned long now = millis();
    if (lastRampTime == 0) lastRampTime = now;
    float dt = constrain((now - lastRampTime) / 1000.0f, 0.0f, 0.1f);
    lastRampTime = now;

    smooth_linear_x = rampValue(
        smooth_linear_x,
        linear_x,
        LINEAR_ACCEL_LIMIT * dt);

    smooth_angular_z = rampValue(
        smooth_angular_z,
        angular_z,
        ANGULAR_ACCEL_LIMIT * dt);

    // ── Differential drive kinematics ──
    float motor_angular_z = smooth_angular_z * ANGULAR_CMD_SIGN;
    float v_left  = smooth_linear_x - (motor_angular_z * WHEEL_BASE * 0.5f);
    float v_right = smooth_linear_x + (motor_angular_z * WHEEL_BASE * 0.5f);

    // ── Heading-hold (only when going straight, no angular cmd) ──
    static unsigned long stableStart     = 0;
    static bool          headingLocked   = false;
    static float         lastHeadingError = 0.0f;

    if (fabsf(smooth_angular_z) < 0.08f && fabsf(smooth_linear_x) > 0.05f)
    {
        if (stableStart == 0) stableStart = millis();

        if (!headingLocked && (millis() - stableStart > HEADING_LOCK_DELAY_MS)) {
            targetYaw         = currentYaw;
            headingLocked     = true;
            lastHeadingError  = 0.0f;
            DBG("HeadingLocked");
        }

        if (headingLocked) {
            float headingError = wrapAngle(targetYaw - currentYaw);

            if (fabsf(headingError) < 0.8f) headingError = 0.0f;

            float dError = headingError - lastHeadingError;
            lastHeadingError = headingError;

            float correction = headingError * HEADING_KP
                             - dError       * HEADING_KD;

            correction = constrain(
                correction,
                -HEADING_MAX_CORRECTION,
                HEADING_MAX_CORRECTION);

            v_left  += correction;
            v_right -= correction;

            // Prevent direction flip during straight driving
            if (smooth_linear_x > 0.0f) {
                v_left  = max(v_left,  0.0f);
                v_right = max(v_right, 0.0f);
            } else {
                v_left  = min(v_left,  0.0f);
                v_right = min(v_right, 0.0f);
            }
        }
    }
    else
    {
        headingLocked    = false;
        stableStart      = 0;
        lastHeadingError = 0.0f;
        targetYaw        = currentYaw;
    }

    // ── Convert m/s → RPM ──
    targetRPM_L = (v_left  / (2.0f * PI * WHEEL_RADIUS)) * 60.0f;
    targetRPM_R = (v_right / (2.0f * PI * WHEEL_RADIUS)) * 60.0f;
}

float rampValue(float current, float target, float maxStep)
{
    float error = target - current;
    if (fabsf(error) <= maxStep) return target;
    return current + ((error > 0.0f) ? maxStep : -maxStep);
}

// ════════════════════════════════════════════════════════════
//  RPM MEASUREMENT (called at RPM_PERIOD_MS intervals)
// ════════════════════════════════════════════════════════════
bool updateRPM()
{
    unsigned long now = millis();
    if (now - prevRPMTime < RPM_PERIOD_MS) return false;

    float dt = (now - prevRPMTime) / 1000.0f;
    prevRPMTime = now;

    long rawCountL = encoderLeft.getCount();
    long rawCountR = encoderRight.getCount();
    long countL = SWAP_ENCODER_SIDES ? rawCountR : rawCountL;
    long countR = SWAP_ENCODER_SIDES ? rawCountL : rawCountR;

    long deltaL = countL - prevCountL;
    long deltaR = countR - prevCountR;

    prevCountL = countL;
    prevCountR = countR;

    // Match sendOdom(): encoder counts are negative when the robot moves forward,
    // so RPM feedback must be negated or the PID will keep adding power.
    currentRPM_L = -((float)deltaL / ENCODER_PPR) * (60.0f / dt);
    currentRPM_R = -((float)deltaR / ENCODER_PPR) * (60.0f / dt);

    DBG(String("RPM L:") + currentRPM_L + " R:" + currentRPM_R);

    return true;
}

// ════════════════════════════════════════════════════════════
//  MOTOR CONTROL
// ════════════════════════════════════════════════════════════
void motorControl(double leftPWM, double rightPWM)
{
    leftPWM  = constrain(leftPWM,  -MAX_PWM, MAX_PWM);
    rightPWM = constrain(rightPWM, -MAX_PWM, MAX_PWM);

    bool leftActive  = fabsf(targetRPM_L) >= RPM_DEADBAND;
    bool rightActive = fabsf(targetRPM_R) >= RPM_DEADBAND;

    int pwmValL = leftActive
        ? commandPWM(leftPWM, targetRPM_L, currentRPM_L, LEFT_PWM_TRIM)
        : 0;
    int pwmValR = rightActive
        ? commandPWM(rightPWM, targetRPM_R, currentRPM_R, RIGHT_PWM_TRIM)
        : 0;

    if (leftActive && rightActive) {
        balanceStraightPWM(pwmValL, pwmValR);
    }

    // ── Left motor ──
    if (!leftActive) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        ledcWrite(PWM_CH_L, 0);
    } else {
        if (targetRPM_L >= 0) {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
        } else {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
        }
        ledcWrite(PWM_CH_L, pwmValL);
    }

    // ── Right motor (physically reversed) ──
    if (!rightActive) {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        ledcWrite(PWM_CH_R, 0);
    } else {
        if (targetRPM_R >= 0) {
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
        } else {
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
        }
        ledcWrite(PWM_CH_R, pwmValR);
    }
}

void balanceStraightPWM(int &leftPWM, int &rightPWM)
{
    if (fabsf(smooth_angular_z) >= 0.04f ||
        fabsf(smooth_linear_x)  <= 0.04f ||
        fabs(targetRPM_R - targetRPM_L) >= 2.0)
    {
        return;
    }

    double speedL = fabs(currentRPM_L);
    double speedR = fabs(currentRPM_R);
    double speedError = speedR - speedL;
    double syncCorrection = constrain(
        speedError * SYNC_K,
        -MAX_SYNC_PWM,
        MAX_SYNC_PWM);

    leftPWM  = constrain(leftPWM  + (int)syncCorrection, 0, MAX_PWM);
    rightPWM = constrain(rightPWM - (int)syncCorrection, 0, MAX_PWM);
}

int commandPWM(double pidPWM, double targetRPM, double currentRPM, float trim)
{
    bool turnBoost = fabsf(smooth_angular_z) > 0.08f;
    int minPWM = turnBoost ? TURN_MIN_PWM : MIN_MOTOR_PWM;
    int startBoostPWM = turnBoost ? TURN_START_BOOST_PWM : START_BOOST_PWM;
    float pwmPerRPM = turnBoost ? TURN_PWM_PER_RPM : PWM_PER_RPM;

    int feedForwardPWM = minPWM + (int)(fabs(targetRPM) * pwmPerRPM);
    if (fabs(targetRPM) > RPM_DEADBAND && fabs(currentRPM) < 1.5) {
        feedForwardPWM = max(feedForwardPWM, startBoostPWM);
    }
    feedForwardPWM = (int)(feedForwardPWM * trim);
    double correctionPWM = (targetRPM >= 0) ? pidPWM : -pidPWM;
    int pwmVal = feedForwardPWM + (int)correctionPWM;
    return constrain(pwmVal, minPWM, MAX_PWM);
}

void stopMotors()
{
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    ledcWrite(PWM_CH_L, 0);
    ledcWrite(PWM_CH_R, 0);

    // Reset PID integrators
    pidLeft.SetMode(MANUAL);
    pidRight.SetMode(MANUAL);
    pwm_L = 0; pwm_R = 0;
    pidLeft.SetMode(AUTOMATIC);
    pidRight.SetMode(AUTOMATIC);
}

// ════════════════════════════════════════════════════════════
//  SERIAL OUTPUT — tagged so ROS node can parse reliably
// ════════════════════════════════════════════════════════════
void sendOdom()
{
    long rawCountL = encoderLeft.getCount();
    long rawCountR = encoderRight.getCount();
    Serial.print("RAW,");
    Serial.print(rawCountL);
    Serial.print(",");
    Serial.println(rawCountR);
    long countL = SWAP_ENCODER_SIDES ? rawCountR : rawCountL;
    long countR = SWAP_ENCODER_SIDES ? rawCountL : rawCountR;

    // FIX 3: Negate encoderRight to match physical forward = positive convention
    Serial.print("ODOM,");
    Serial.print(-countL);
    Serial.print(",");
    Serial.print(-countR);
    Serial.print(",");
    Serial.print(currentRPM_L, 3);
    Serial.print(",");
    Serial.println(currentRPM_R, 3);
    Serial.print("RPM,");
    Serial.print(currentRPM_L);
    Serial.print(",");
    Serial.println(currentRPM_R);
}

void sendIMU()
{
    Serial.print("IMU,");
    Serial.println(currentYaw, 3);
}
