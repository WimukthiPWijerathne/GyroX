#include <Wire.h>
#include <MPU6050_light.h>

// Motor Pin Definitions
const int AIN1 = 25;
const int AIN2 = 33;
const int PWMA = 32;
const int BIN1 = 27;
const int BIN2 = 14;
const int PWMB = 13;
const int STBY = 26;

const int PWM_FREQ = 1000;
const int PWM_RESOLUTION = 8;
const int PWM_CHANNEL_A = 0;
const int PWM_CHANNEL_B = 1;
const int PWM_MAX = 255;
const int MIN_PWM_THRESHOLD = 40; // Smooth starts
const float LOOP_INTERVAL_MS = 5.0; // 200Hz control loop
const float DEAD_BAND = 0.3; // Reduces jitter
const float MAX_ANGLE = 15.0; // Maximum angle to prevent falling damage

// PID Constants
const float KP = 12.0; // Stable for large angles
const float KI = 0.5;  // Increased for steady-state error correction
const float KD = 1.8;  // Increased for better damping
const float NON_LINEAR_GAIN = 1.2; // Non-linear control
const float ANGLE_THRESHOLD = 5.0; // Apply non-linear control earlier
const float MAX_INTEGRAL = 15.0; // Tighter integral limit
const float INTEGRAL_ERROR_THRESHOLD = 2.0; // Integral active for small errors
const float TARGET_ANGLE = 0.0;
const float MAX_PWM_NORMAL = 100.0; // Normal PWM limit
const float MAX_PWM_BOOST = 200.0; // Slightly increased for recovery

// Filtered Angle
const float ALPHA = 0.98; // Smoother low-pass filter

MPU6050 mpu(Wire);

// PID Variables
float previousError = 0.0;
float integralSum = 0.0;
float filteredAngle = 0.0;
int prevPwm = 0; // For PWM smoothing

void driveMotor(float pidValue) {
  // Check if angle exceeds safe limit
  if (abs(filteredAngle) > MAX_ANGLE) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    ledcWrite(PWM_CHANNEL_A, 0);
    ledcWrite(PWM_CHANNEL_B, 0);
    return;
  }

  // Apply deadband to reduce jitter
  if (abs(pidValue) < DEAD_BAND) {
    pidValue = 0;
  }

  // Dynamic PWM limit based on angle error
  float maxPWM = (abs(filteredAngle - TARGET_ANGLE) > ANGLE_THRESHOLD) ? MAX_PWM_BOOST : MAX_PWM_NORMAL;
  float limitedValue = constrain(pidValue, -maxPWM, maxPWM);
  int pwm = (int)abs(limitedValue);

  // Apply minimum PWM threshold for motor activation
  if (pwm > 0 && pwm < MIN_PWM_THRESHOLD) {
    pwm = MIN_PWM_THRESHOLD;
  }

  // Smooth PWM transitions using exponential smoothing
  pwm = (0.9 * prevPwm + 0.1 * pwm); // 90% previous, 10% new
  prevPwm = pwm;

  // Motor direction control
  if (limitedValue > 0) { // Forward
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else if (limitedValue < 0) { // Backward
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else { // Stop
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    pwm = 0;
  }

  ledcWrite(PWM_CHANNEL_A, pwm);
  ledcWrite(PWM_CHANNEL_B, pwm);
}

float computePID(float currentAngle, float dt) {
  float error = TARGET_ANGLE - currentAngle;

  // Non-linear proportional term for large angles
  float pTerm = KP * error;
  if (abs(error) > ANGLE_THRESHOLD) {
    pTerm *= (1.0 + NON_LINEAR_GAIN * (abs(error) - ANGLE_THRESHOLD) / ANGLE_THRESHOLD);
  }

  // Conditional integral for small errors to prevent windup
  if (abs(error) < INTEGRAL_ERROR_THRESHOLD) {
    integralSum = constrain(integralSum + error * dt, -MAX_INTEGRAL, MAX_INTEGRAL);
  }
  float iTerm = KI * integralSum;

  // Derivative term
  float derivative = (error - previousError) / dt;
  previousError = error;

  // Compute PID output
  float pidOutput = pTerm + iTerm + KD * derivative;
  return pidOutput;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(4, 15); // SDA, SCL

  // Motor setup
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMA, PWM_CHANNEL_A);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWMB, PWM_CHANNEL_B);

  // MPU6050 setup
  if (mpu.begin() != 0) {
    Serial.println(F("MPU6050 connection failed. Check wiring or address."));
    while (1);
  }

  Serial.println(F("Calibrating... Keep MPU still"));
  delay(2000); // Increased calibration time
  mpu.calcOffsets();
  Serial.println(F("Calibration complete!"));

  mpu.update();
  filteredAngle = mpu.getAngleX();
}

void loop() {
  static unsigned long lastControlTime = 0;
  static unsigned long lastDebugTime = 0;
  unsigned long now = millis();

  // Control loop at 200Hz
  if (now - lastControlTime >= LOOP_INTERVAL_MS) {
    float dt = (now - lastControlTime) / 1000.0;
    lastControlTime = now;

    mpu.update();
    float rawAngle = mpu.getAngleX();
    filteredAngle = ALPHA * filteredAngle + (1 - ALPHA) * rawAngle;

    float pidOutput = computePID(filteredAngle, dt);
    driveMotor(pidOutput);
  }

  // Debug output every 200ms
  if (now - lastDebugTime >= 200) {
    float error = TARGET_ANGLE - filteredAngle;
    float pTerm = KP * error;
    if (abs(error) > ANGLE_THRESHOLD) {
      pTerm *= (1.0 + NON_LINEAR_GAIN * (abs(error) - ANGLE_THRESHOLD) / ANGLE_THRESHOLD);
    }
    float iTerm = KI * integralSum;
    float derivative = (error - previousError) / (LOOP_INTERVAL_MS / 1000.0);
    float dTerm = KD * derivative;
    Serial.printf("Angle: %.2f | Error: %.2f | P: %.2f | I: %.2f | D: %.2f | PID: %.2f | dt: %.4f\n",
                  filteredAngle, error, pTerm, iTerm, dTerm, pTerm + iTerm + dTerm, LOOP_INTERVAL_MS / 1000.0);
    lastDebugTime = now;
  }
}