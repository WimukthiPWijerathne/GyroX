#include <Wire.h>
#include <MPU6050_light.h>
#include <WiFi.h>
#include <WebServer.h>

// WiFi Credentials
const char* ssid = "randilsk";
const char* password = "lkci2721";
WebServer server(80);

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
const float LOOP_INTERVAL_MS = 5; // 200Hz control loop
const float DEAD_BAND = 0.1; // Deadband to prevent motor jitter

// PID Constants
float KP = 6;
float KI = 0.72;
float KD = 0.95;
const float NON_LINEAR_GAIN = 5.9;
const float ANGLE_THRESHOLD = 3.8;
const float MAX_INTEGRAL = 50.0;
float TARGET_ANGLE = 0.019;
const float MAX_PWM_NORMAL = 115.0;
const float MAX_PWM_BOOST = 235.0;

// Movement Angles
const float FORWARD_ANGLE_OFFSET = -2.0;
const float BACKWARD_ANGLE_OFFSET = 2.0;

// Filtered Angle
const float ALPHA = 0.65; // Angle low-pass filter
const float DERIVATIVE_ALPHA = 0.8; // Derivative low-pass filter

// Disturbance Observer Parameters
const float DISTURBANCE_GAIN = 0.5;
const float DISTURBANCE_FILTER = 0.9;
float estimatedDisturbance = 0.0;

// Feedforward Compensation
const float FEEDFORWARD_GAIN = 0.1;
const float VELOCITY_THRESHOLD = 0.5;

// Adaptive PID Tuning
const float KP_MAX = 25.0;
const float KP_MIN = 10.0;
const float ERROR_THRESHOLD_HIGH = 5.0;
const float ERROR_THRESHOLD_LOW = 1.0;

MPU6050 mpu(Wire);

// PID Variables
float previousError = 0.0;
float integralSum = 0.0;
float filteredAngle = 0.0;
float filteredDerivative = 0.0;
float previousAngle = 0.0;

// WiFi Command State
String currentCommand = "";
bool isBalancing = false;

void handleCommand() {
  String command;
  if (server.hasArg("command")) {
    command = server.arg("command");
  } else if (server.hasArg("plain")) {
    command = server.arg("plain");
  } else {
    server.send(400, "text/plain", "Body not found");
    return;
  }
  command.trim();
  Serial.println("Command received: " + command);

  if (command == "forward" || command == "backward" || command == "stop") {
    currentCommand = command;
    isBalancing = true;
    server.send(200, "text/plain", "Command accepted");
  } else if (command == "left") {
    currentCommand = command;
    isBalancing = false;
    integralSum = 0.0;
    server.send(200, "text/plain", "Command accepted");
  } else {
    server.send(200, "text/plain", "Command ignored");
  }
}

void driveMotor(float pidValue) {
  if (abs(pidValue) < DEAD_BAND) {
    pidValue = 0;
  }

  float maxPWM = (abs(filteredAngle - TARGET_ANGLE) > ANGLE_THRESHOLD) ? MAX_PWM_BOOST : MAX_PWM_NORMAL;
  float limitedValue = constrain(pidValue, -maxPWM, maxPWM);
  int pwm = (int)abs(limitedValue);

  if (pwm > 0 && pwm < MIN_PWM_THRESHOLD) {
    pwm = MIN_PWM_THRESHOLD;
  }

  if (limitedValue > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else if (limitedValue < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
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

  // Adaptive PID gains
  if (abs(error) > ERROR_THRESHOLD_HIGH) {
    KP = KP_MAX;
  } else if (abs(error) < ERROR_THRESHOLD_LOW) {
    KP = KP_MIN;
  } else {
    KP = KP_MIN + (KP_MAX - KP_MIN) * (abs(error) - ERROR_THRESHOLD_LOW) / 
         (ERROR_THRESHOLD_HIGH - ERROR_THRESHOLD_LOW);
  }

  // Non-linear proportional term
  float pTerm = KP * error;
  if (abs(error) > ANGLE_THRESHOLD) {
    pTerm *= (1.0 + NON_LINEAR_GAIN * (abs(error) - ANGLE_THRESHOLD) / ANGLE_THRESHOLD);
  }

  // Integral with anti-windup
  if (abs(error) < ANGLE_THRESHOLD) {
    integralSum = constrain(integralSum + error * dt, -MAX_INTEGRAL, MAX_INTEGRAL);
  }
  float iTerm = KI * integralSum;

  // Filtered derivative term
  float derivative = (error - previousError) / dt;
  filteredDerivative = DERIVATIVE_ALPHA * filteredDerivative + (1 - DERIVATIVE_ALPHA) * derivative;
  previousError = error;

  // Feedforward term based on velocity
  float velocity = (currentAngle - previousAngle) / dt;
  float feedforward = FEEDFORWARD_GAIN * velocity;
  if (abs(velocity) < VELOCITY_THRESHOLD) {
    feedforward = 0;
  }

  // Compute base PID output
  float pidOutput = pTerm + iTerm + KD * filteredDerivative + feedforward;

  // Disturbance observer (uses previous pidOutput)
  static float previousPidOutput = 0.0;
  estimatedDisturbance = DISTURBANCE_FILTER * estimatedDisturbance + 
                        (1 - DISTURBANCE_FILTER) * (pTerm - previousPidOutput + velocity);
  previousPidOutput = pidOutput;

  // Final PID output with disturbance rejection
  pidOutput -= DISTURBANCE_GAIN * estimatedDisturbance;
  return pidOutput;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(4, 15);

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

  byte status = mpu.begin();
  if (status != 0) {
    Serial.println(F("MPU6050 connection failed. Check wiring or address."));
    while (1);
  }

  Serial.println(F("Calibrating... Keep MPU still"));
  delay(1000);
  mpu.calcOffsets();
  Serial.println(F("Calibration complete!"));

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.println(WiFi.localIP());

  server.on("/command", HTTP_POST, handleCommand);
  server.on("/ping", HTTP_GET, []() {
    server.send(200, "text/plain", "pong");
  });

  server.begin();
  Serial.println("HTTP server started");

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  ledcWrite(PWM_CHANNEL_A, 0);
  ledcWrite(PWM_CHANNEL_B, 0);
}

void loop() {
  static unsigned long lastControlTime = 0;
  static unsigned long lastDebugTime = 0;
  unsigned long now = millis();

  server.handleClient();

  if (isBalancing) {
    if (currentCommand == "forward") {
      TARGET_ANGLE = 0.019 + FORWARD_ANGLE_OFFSET;
    } else if (currentCommand == "backward") {
      TARGET_ANGLE = 0.019 + BACKWARD_ANGLE_OFFSET;
    } else {
      TARGET_ANGLE = 0.019;
    }

    if (now - lastControlTime >= LOOP_INTERVAL_MS) {
      float dt = (now - lastControlTime) / 1000.0;
      lastControlTime = now;

      mpu.update();
      float rawAngle = mpu.getAngleX();
      filteredAngle = ALPHA * filteredAngle + (1 - ALPHA) * rawAngle;

      float pidOutput = computePID(filteredAngle, dt);
      driveMotor(pidOutput);

      previousAngle = filteredAngle;
    }

    if (now - lastDebugTime >= 200) {
      Serial.printf("Angle: %.2f | Error: %.2f | PID: %.2f | KP: %.2f | Disturbance: %.2f | Command: %s\n", 
                    filteredAngle, TARGET_ANGLE - filteredAngle, 
                    computePID(filteredAngle, LOOP_INTERVAL_MS / 1000.0), 
                    KP, estimatedDisturbance, currentCommand.c_str());
      lastDebugTime = now;
    }
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    ledcWrite(PWM_CHANNEL_A, 0);
    ledcWrite(PWM_CHANNEL_B, 0);
    integralSum = 0.0;
    estimatedDisturbance = 0.0;
  }
}