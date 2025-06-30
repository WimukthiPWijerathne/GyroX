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
const int MIN_PWM_THRESHOLD = 55;
const float LOOP_INTERVAL_MS = 5.0; // 200Hz control loop
const float DEAD_BAND = 0.1; // Deadband to prevent motor jitter

// PID Constants
const float KP = 18.0;
const float KI = 0.6;
const float KD = 1.2;
const float NON_LINEAR_GAIN = 2.0; // Gain for large angles
const float ANGLE_THRESHOLD = 10.0; // Angle threshold for non-linear control
const float MAX_INTEGRAL = 50.0; // Limit for integral windup
const float TARGET_ANGLE = 0.0;
const float MAX_PWM_NORMAL = 110.0;
const float MAX_PWM_BOOST = 180.0; // Higher PWM for large angles

// Filtered Angle
const float ALPHA = 0.95; // Smoother low-pass filter

MPU6050 mpu(Wire);

// PID Variables
float previousError = 0.0;
float integralSum = 0.0;
float filteredAngle = 0.0;

void driveMotor(float pidValue) {
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

  // Integral with anti-windup
  integralSum = constrain(integralSum + error * dt, -MAX_INTEGRAL, MAX_INTEGRAL);
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
  delay(1000);
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
    Serial.printf("Angle: %.2f | Error: %.2f | PID: %.2f\n", filteredAngle, TARGET_ANGLE - filteredAngle, computePID(filteredAngle, LOOP_INTERVAL_MS / 1000.0));
    lastDebugTime = now;
  }
}