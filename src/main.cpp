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
const int MIN_PWM_THRESHOLD = 50;
const float LOOP_INTERVAL_MS = 0.001; // 200Hz control loop
const float DEAD_BAND = 0.15; // Deadband to prevent motor jitter

// PID Constants
const float KP = 1.42;
const float KI = 0.51;
const float KD = 0.9;
const float NON_LINEAR_GAIN = 0.5; // Gain for large angles
const float ANGLE_THRESHOLD = 1.09; // Angle threshold for non-linear control
const float MAX_INTEGRAL = 30.0; // Limit for integral windup
float TARGET_ANGLE = 0.19; // Default target angle for balance
const float MAX_PWM_NORMAL = 55.0;
const float MAX_PWM_BOOST = 90.0; // Higher PWM for large angles

// Movement Angles
const float FORWARD_ANGLE_OFFSET = -2.0; // Tilt forward to move forward
const float BACKWARD_ANGLE_OFFSET = 2.0; // Tilt backward to move backward

// Filtered Angle
const float ALPHA = 0.895; // Smoother low-pass filter

MPU6050 mpu(Wire);

// PID Variables
float previousError = 0.0;
float integralSum = 0.0;
float filteredAngle = 0.0;

// WiFi Command State
String currentCommand = "stop";

void handleCommand() {
  String command;
  // Accept both form data and raw body for maximum compatibility
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
    server.send(200, "text/plain", "Command accepted");
  } else {
    server.send(200, "text/plain", "Command ignored");
  }
}


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
  pidOutput = constrain(pidOutput, -150, 150);
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

  // WiFi setup
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

  mpu.update();
  filteredAngle = mpu.getAngleX();
}

void loop() {
  static unsigned long lastControlTime = 0;
  static unsigned long lastDebugTime = 0;
  unsigned long now = millis();

  // Handle WiFi clients
  server.handleClient();

float angleStep = 0.02;  // Smooth angle increment per loop

if (currentCommand == "forward") {
    if (TARGET_ANGLE > 0.19 + FORWARD_ANGLE_OFFSET) {
        TARGET_ANGLE -= angleStep;
    }
} else if (currentCommand == "backward") {
    if (TARGET_ANGLE < 0.19 + BACKWARD_ANGLE_OFFSET) {
        TARGET_ANGLE += angleStep;
    }
} else {
    if (TARGET_ANGLE > 0.19) {
        TARGET_ANGLE -= angleStep;
    } else if (TARGET_ANGLE < 0.19) {
        TARGET_ANGLE += angleStep;
    }
}

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
    Serial.printf("Angle: %.2f | Error: %.2f | PID: %.2f | Command: %s\n", 
                  filteredAngle, TARGET_ANGLE - filteredAngle, 
                  computePID(filteredAngle, LOOP_INTERVAL_MS / 1000.0), 
                  currentCommand.c_str());
    lastDebugTime = now;
  }
}