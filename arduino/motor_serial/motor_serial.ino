#include <EEPROM.h>
#include <Wire.h>

const int LEFT_PWM_PIN = 5;
const int LEFT_DIR_PIN = 4;
const int RIGHT_PWM_PIN = 6;
const int RIGHT_DIR_PIN = 7;

const int STEER_PWM_PIN = 9;
const int STEER_DIR_PIN = 8;

const bool LEFT_INVERT = false;
const bool RIGHT_INVERT = true;
const bool STEER_INVERT = true;

const byte AS5600_ADDRESS = 0x36;
const byte AS5600_STATUS_REGISTER = 0x0B;
const byte AS5600_RAW_ANGLE_REGISTER = 0x0C;

const unsigned long COMMAND_TIMEOUT_MS = 300;
const unsigned long ENCODER_REPORT_INTERVAL_MS = 100;
const unsigned long STEERING_LIMIT_CHECK_INTERVAL_MS = 10;
const unsigned long CENTER_CONTROL_INTERVAL_MS = 15;
const unsigned long CENTER_TIMEOUT_MS = 4000;
const unsigned long CENTER_SETTLE_MS = 250;
const unsigned long CENTER_STALL_TIMEOUT_MS = 700;
const int DRIVE_PWM_DEADZONE = 18;
const int STEER_PWM_DEADZONE = 35;
const int STEER_PWM_LIMIT = 255;
const int CENTER_TOLERANCE_RAW = 8;
const int CENTER_MIN_PWM = 170;
const int CENTER_MAX_PWM = 220;
const int CENTER_FULL_SPEED_ERROR_RAW = 180;
const int CENTER_MOTION_THRESHOLD_RAW = 3;
const uint16_t DEFAULT_STEER_LEFT_REFERENCE_RAW = 3980;
const uint16_t DEFAULT_STEER_RIGHT_REFERENCE_RAW = 3503;
const uint16_t DEFAULT_STEER_LIMIT_ALLOWANCE_RAW = 50;
const uint16_t DEFAULT_STEERING_ZERO_RAW = 3700;

const uint16_t CALIBRATION_MAGIC = 0xA562;
const uint16_t CALIBRATION_CHECK_MASK = 0x5A5A;

struct SteeringCalibration {
  uint16_t magic;
  uint16_t rightReferenceRaw;
  uint16_t centerRaw;
  uint16_t leftReferenceRaw;
  uint16_t allowanceRaw;
  uint16_t checksum;
};

String inputLine;
unsigned long lastDriveCommandMs = 0;
unsigned long lastSteerCommandMs = 0;
unsigned long lastEncoderReportMs = 0;
unsigned long lastSteeringLimitCheckMs = 0;
unsigned long lastCenterControlMs = 0;
unsigned long centeringStartedMs = 0;
unsigned long centerWithinToleranceMs = 0;
unsigned long centerLastMotionMs = 0;
bool driveActive = false;
bool steerActive = false;
bool centeringActive = false;
int currentSteerPwm = 0;
uint16_t lastSteeringLimitRaw = 0;
uint16_t centerLastMotionRaw = 0;
bool steeringZeroSet = true;
bool emergencyStopLatched = false;
uint16_t steeringZeroRaw = DEFAULT_STEERING_ZERO_RAW;
uint16_t steeringRightReferenceRaw = DEFAULT_STEER_RIGHT_REFERENCE_RAW;
uint16_t steeringLeftReferenceRaw = DEFAULT_STEER_LEFT_REFERENCE_RAW;
uint16_t steeringLimitAllowanceRaw = DEFAULT_STEER_LIMIT_ALLOWANCE_RAW;

void setup() {
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(STEER_PWM_PIN, OUTPUT);
  pinMode(STEER_DIR_PIN, OUTPUT);

  stopAll();
  Wire.begin();
  Serial.begin(115200);
  loadSteeringCalibration();
}

void loop() {
  readSerialCommands();

  const unsigned long now = millis();
  if (emergencyStopLatched) {
    stopAll();
  }
  if (driveActive && now - lastDriveCommandMs > COMMAND_TIMEOUT_MS) {
    stopDrive();
  }
  if (steerActive && now - lastSteerCommandMs > COMMAND_TIMEOUT_MS) {
    stopSteering();
  }
  if (steerActive && now - lastSteeringLimitCheckMs >= STEERING_LIMIT_CHECK_INTERVAL_MS) {
    lastSteeringLimitCheckMs = now;
    enforceSteeringLimit();
  }
  if (centeringActive && now - lastCenterControlMs >= CENTER_CONTROL_INTERVAL_MS) {
    lastCenterControlMs = now;
    updateCentering(now);
  }
  if (now - lastEncoderReportMs >= ENCODER_REPORT_INTERVAL_MS) {
    lastEncoderReportMs = now;
    reportEncoder();
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (inputLine.length() > 0) {
        handleCommand(inputLine);
        inputLine = "";
      }
    } else if (inputLine.length() < 48) {
      inputLine += ch;
    }
  }
}

void handleCommand(String command) {
  command.trim();

  if (command == "STOP") {
    stopAll();
    Serial.println("OK STOP");
    return;
  }

  if (command == "ESTOP") {
    emergencyStopLatched = true;
    stopAll();
    Serial.println("OK ESTOP LATCHED");
    return;
  }

  if (command == "RESET_ESTOP") {
    emergencyStopLatched = false;
    Serial.println("OK ESTOP RESET");
    return;
  }

  if (
    emergencyStopLatched
    && (command.startsWith("DRIVE ") || command.startsWith("STEER ") || command == "CENTER")
  ) {
    stopAll();
    Serial.println("ERR ESTOP LATCHED");
    return;
  }

  if (command.startsWith("DRIVE ")) {
    int pwm = constrain(command.substring(6).toInt(), -255, 255);
    if (abs(pwm) < DRIVE_PWM_DEADZONE) {
      pwm = 0;
    }
    setDrive(pwm);
    lastDriveCommandMs = millis();
    driveActive = pwm != 0;
    Serial.print("OK DRIVE ");
    Serial.println(pwm);
    return;
  }

  if (command.startsWith("STEER ")) {
    int pwm = constrain(command.substring(6).toInt(), -STEER_PWM_LIMIT, STEER_PWM_LIMIT);
    if (abs(pwm) < STEER_PWM_DEADZONE) {
      pwm = 0;
    }
    if (!setSteering(pwm)) {
      Serial.print("LIMIT ");
      Serial.print(pwm < 0 ? "LEFT " : "RIGHT ");
      Serial.println(lastSteeringLimitRaw);
      return;
    }
    lastSteerCommandMs = millis();
    steerActive = pwm != 0;
    Serial.print("OK STEER ");
    Serial.println(pwm);
    return;
  }

  if (command == "CENTER") {
    beginCentering();
    return;
  }

  if (command.startsWith("CONFIG ")) {
    int rightRaw = 0;
    int centerRaw = 0;
    int leftRaw = 0;
    int allowanceRaw = 0;
    if (sscanf(
          command.c_str(),
          "CONFIG %d %d %d %d",
          &rightRaw,
          &centerRaw,
          &leftRaw,
          &allowanceRaw
        ) != 4 || !steeringCalibrationValid(rightRaw, centerRaw, leftRaw, allowanceRaw)) {
      Serial.println("ERR CONFIG");
      return;
    }
    stopSteering();
    steeringRightReferenceRaw = rightRaw;
    steeringZeroRaw = centerRaw;
    steeringLeftReferenceRaw = leftRaw;
    steeringLimitAllowanceRaw = allowanceRaw;
    steeringZeroSet = true;
    saveSteeringCalibration();
    Serial.print("CONFIG ");
    Serial.print(steeringRightReferenceRaw);
    Serial.print(' ');
    Serial.print(steeringZeroRaw);
    Serial.print(' ');
    Serial.print(steeringLeftReferenceRaw);
    Serial.print(' ');
    Serial.println(steeringLimitAllowanceRaw);
    reportEncoder();
    return;
  }

  if (command == "ZERO") {
    stopSteering();
    uint16_t rawAngle = 0;
    if (!readRawAngle(rawAngle)) {
      Serial.println("ERR AS5600");
      return;
    }
    if (!steeringCalibrationValid(
          steeringRightReferenceRaw,
          rawAngle,
          steeringLeftReferenceRaw,
          steeringLimitAllowanceRaw
        )) {
      Serial.println("ERR ZERO CALIBRATION");
      return;
    }
    steeringZeroRaw = rawAngle;
    steeringZeroSet = true;
    saveSteeringCalibration();
    Serial.print("ZERO ");
    Serial.println(steeringZeroRaw);
    reportEncoder();
    return;
  }

  if (command == "STATUS") {
    reportEncoder();
    return;
  }

  stopAll();
  Serial.println("ERR UNKNOWN");
}

void setDrive(int pwm) {
  setMotor(LEFT_PWM_PIN, LEFT_DIR_PIN, pwm, LEFT_INVERT);
  setMotor(RIGHT_PWM_PIN, RIGHT_DIR_PIN, pwm, RIGHT_INVERT);
}

bool setSteering(int pwm) {
  centeringActive = false;
  centerWithinToleranceMs = 0;
  return applySteeringOutput(pwm);
}

bool applySteeringOutput(int pwm) {
  uint16_t rawAngle = 0;
  if (pwm != 0 && readRawAngle(rawAngle) && !steeringMoveAllowed(pwm, rawAngle)) {
    lastSteeringLimitRaw = rawAngle;
    stopSteering();
    return false;
  }
  currentSteerPwm = pwm;
  setMotor(STEER_PWM_PIN, STEER_DIR_PIN, pwm, STEER_INVERT);
  return true;
}

void setMotor(int pwmPin, int dirPin, int pwm, bool invert) {
  bool forward = pwm >= 0;
  if (invert) {
    forward = !forward;
  }
  digitalWrite(dirPin, forward ? HIGH : LOW);
  analogWrite(pwmPin, abs(pwm));
}

void stopDrive() {
  driveActive = false;
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
  digitalWrite(LEFT_DIR_PIN, LOW);
  digitalWrite(RIGHT_DIR_PIN, LOW);
}

void stopSteering() {
  centeringActive = false;
  centerWithinToleranceMs = 0;
  steerActive = false;
  stopSteeringOutput();
}

void stopSteeringOutput() {
  currentSteerPwm = 0;
  analogWrite(STEER_PWM_PIN, 0);
  digitalWrite(STEER_DIR_PIN, LOW);
}

void stopAll() {
  stopDrive();
  stopSteering();
}

bool readRegister(byte registerAddress, byte &value) {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(registerAddress);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(AS5600_ADDRESS, (byte)1) != 1) {
    return false;
  }
  value = Wire.read();
  return true;
}

bool readRawAngle(uint16_t &rawAngle) {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(AS5600_RAW_ANGLE_REGISTER);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(AS5600_ADDRESS, (byte)2) != 2) {
    return false;
  }
  rawAngle = ((uint16_t)(Wire.read() & 0x0F) << 8) | Wire.read();
  return true;
}

int signedRawDelta(uint16_t fromRaw, uint16_t toRaw) {
  int difference = (int)toRaw - (int)fromRaw;
  if (difference > 2047) {
    difference -= 4096;
  } else if (difference < -2048) {
    difference += 4096;
  }
  return difference;
}

uint16_t wrapRaw(int rawAngle) {
  while (rawAngle < 0) {
    rawAngle += 4096;
  }
  while (rawAngle >= 4096) {
    rawAngle -= 4096;
  }
  return (uint16_t)rawAngle;
}

int rawDirection(uint16_t referenceRaw, uint16_t centerRaw) {
  return signedRawDelta(centerRaw, referenceRaw) >= 0 ? 1 : -1;
}

int steeringLeftRawDirection() {
  return rawDirection(steeringLeftReferenceRaw, steeringZeroRaw);
}

int steeringRightRawDirection() {
  return rawDirection(steeringRightReferenceRaw, steeringZeroRaw);
}

uint16_t steeringLeftRawLimit() {
  return wrapRaw(
    (int)steeringLeftReferenceRaw
    + steeringLeftRawDirection() * (int)steeringLimitAllowanceRaw
  );
}

uint16_t steeringRightRawLimit() {
  return wrapRaw(
    (int)steeringRightReferenceRaw
    + steeringRightRawDirection() * (int)steeringLimitAllowanceRaw
  );
}

bool steeringCalibrationValid(int rightRaw, int centerRaw, int leftRaw, int allowanceRaw) {
  if (
    rightRaw < 0 || rightRaw >= 4096
    || centerRaw < 0 || centerRaw >= 4096
    || leftRaw < 0 || leftRaw >= 4096
    || allowanceRaw < 0 || allowanceRaw > 300
  ) {
    return false;
  }

  const int rightDelta = signedRawDelta(centerRaw, rightRaw);
  const int leftDelta = signedRawDelta(centerRaw, leftRaw);
  return rightDelta != 0
    && leftDelta != 0
    && (long)rightDelta * (long)leftDelta < 0
    && abs(rightDelta) + allowanceRaw < 2048
    && abs(leftDelta) + allowanceRaw < 2048;
}

bool steeringSideLimitReached(
  uint16_t rawAngle,
  uint16_t referenceRaw,
  int direction
) {
  const int currentDelta = signedRawDelta(steeringZeroRaw, rawAngle);
  const int referenceDelta = signedRawDelta(steeringZeroRaw, referenceRaw);
  const int limitMagnitude = abs(referenceDelta) + steeringLimitAllowanceRaw;
  return currentDelta * direction >= limitMagnitude;
}

float relativeAngleDegrees(uint16_t rawAngle) {
  const int physicalLeftDelta = signedRawDelta(steeringZeroRaw, rawAngle)
    * steeringLeftRawDirection();
  return physicalLeftDelta * (360.0 / 4096.0);
}

const char *steeringLimitName(uint16_t rawAngle) {
  if (steeringSideLimitReached(rawAngle, steeringLeftReferenceRaw, steeringLeftRawDirection())) {
    return "LEFT";
  }
  if (steeringSideLimitReached(rawAngle, steeringRightReferenceRaw, steeringRightRawDirection())) {
    return "RIGHT";
  }
  return "NONE";
}

bool steeringMoveAllowed(int pwm, uint16_t rawAngle) {
  if (
    pwm < 0
    && steeringSideLimitReached(rawAngle, steeringLeftReferenceRaw, steeringLeftRawDirection())
  ) {
    return false;
  }
  if (
    pwm > 0
    && steeringSideLimitReached(rawAngle, steeringRightReferenceRaw, steeringRightRawDirection())
  ) {
    return false;
  }
  return true;
}

void beginCentering() {
  uint16_t rawAngle = 0;
  if (!steeringZeroSet || !readRawAngle(rawAngle)) {
    stopSteering();
    Serial.println("ERR CENTER AS5600");
    return;
  }
  steerActive = false;
  stopSteeringOutput();
  centeringActive = true;
  centeringStartedMs = millis();
  centerWithinToleranceMs = 0;
  centerLastMotionMs = centeringStartedMs;
  centerLastMotionRaw = rawAngle;
  lastCenterControlMs = 0;
  Serial.print("OK CENTER ");
  Serial.println(steeringZeroRaw);
}

void updateCentering(unsigned long now) {
  if (now - centeringStartedMs > CENTER_TIMEOUT_MS) {
    stopSteering();
    Serial.println("CENTER TIMEOUT");
    return;
  }

  uint16_t rawAngle = 0;
  if (!readRawAngle(rawAngle)) {
    stopSteering();
    Serial.println("ERR CENTER AS5600");
    return;
  }

  const int currentDelta = signedRawDelta(steeringZeroRaw, rawAngle);
  const int absoluteError = abs(currentDelta);
  if (absoluteError <= CENTER_TOLERANCE_RAW) {
    stopSteeringOutput();
    if (centerWithinToleranceMs == 0) {
      centerWithinToleranceMs = now;
    } else if (now - centerWithinToleranceMs >= CENTER_SETTLE_MS) {
      centeringActive = false;
      Serial.print("CENTERED ");
      Serial.println(rawAngle);
    }
    return;
  }

  const int motionDelta = signedRawDelta(centerLastMotionRaw, rawAngle);
  if (abs(motionDelta) >= CENTER_MOTION_THRESHOLD_RAW) {
    centerLastMotionRaw = rawAngle;
    centerLastMotionMs = now;
  } else if (now - centerLastMotionMs > CENTER_STALL_TIMEOUT_MS) {
    stopSteering();
    Serial.println("CENTER STALL");
    return;
  }

  centerWithinToleranceMs = 0;
  const int scaledError = min(absoluteError, CENTER_FULL_SPEED_ERROR_RAW);
  const int pwmMagnitude = CENTER_MIN_PWM
    + scaledError * (CENTER_MAX_PWM - CENTER_MIN_PWM) / CENTER_FULL_SPEED_ERROR_RAW;
  const int pwm = currentDelta * steeringLeftRawDirection() > 0
    ? pwmMagnitude
    : -pwmMagnitude;
  if (!applySteeringOutput(pwm)) {
    const char *limitName = pwm < 0 ? "LEFT" : "RIGHT";
    stopSteering();
    Serial.print("LIMIT ");
    Serial.print(limitName);
    Serial.print(' ');
    Serial.println(rawAngle);
  }
}

void enforceSteeringLimit() {
  uint16_t rawAngle = 0;
  if (!readRawAngle(rawAngle) || steeringMoveAllowed(currentSteerPwm, rawAngle)) {
    return;
  }
  const int blockedPwm = currentSteerPwm;
  lastSteeringLimitRaw = rawAngle;
  stopSteering();
  Serial.print("LIMIT ");
  Serial.print(blockedPwm < 0 ? "LEFT " : "RIGHT ");
  Serial.println(rawAngle);
}

void reportEncoder() {
  uint16_t rawAngle = 0;
  byte statusValue = 0;
  if (!readRawAngle(rawAngle) || !readRegister(AS5600_STATUS_REGISTER, statusValue)) {
    Serial.println("ENC ERR");
    return;
  }

  Serial.print("ENC ");
  Serial.print(rawAngle);
  Serial.print(' ');
  if (steeringZeroSet) {
    Serial.print(relativeAngleDegrees(rawAngle), 2);
    Serial.print(' ');
    Serial.print(steeringZeroRaw);
  } else {
    Serial.print("NA -1");
  }
  Serial.print(' ');
  Serial.print(statusValue);
  Serial.print(' ');
  Serial.print(steeringLimitName(rawAngle));
  Serial.print(' ');
  Serial.print(centeringActive ? "CENTERING" : "IDLE");
  Serial.print(' ');
  Serial.print(steeringRightReferenceRaw);
  Serial.print(' ');
  Serial.print(steeringLeftReferenceRaw);
  Serial.print(' ');
  Serial.print(steeringLimitAllowanceRaw);
  Serial.print(' ');
  Serial.print(steeringRightRawLimit());
  Serial.print(' ');
  Serial.print(steeringLeftRawLimit());
  Serial.print(' ');
  Serial.println(emergencyStopLatched ? "ESTOP_LATCHED" : "ESTOP_OK");
}

uint16_t steeringCalibrationChecksum(const SteeringCalibration &calibration) {
  return calibration.magic
    ^ calibration.rightReferenceRaw
    ^ calibration.centerRaw
    ^ calibration.leftReferenceRaw
    ^ calibration.allowanceRaw
    ^ CALIBRATION_CHECK_MASK;
}

void loadSteeringCalibration() {
  SteeringCalibration calibration;
  EEPROM.get(0, calibration);
  if (
    calibration.magic == CALIBRATION_MAGIC
    && calibration.checksum == steeringCalibrationChecksum(calibration)
    && steeringCalibrationValid(
      calibration.rightReferenceRaw,
      calibration.centerRaw,
      calibration.leftReferenceRaw,
      calibration.allowanceRaw
    )
  ) {
    steeringRightReferenceRaw = calibration.rightReferenceRaw;
    steeringZeroRaw = calibration.centerRaw;
    steeringLeftReferenceRaw = calibration.leftReferenceRaw;
    steeringLimitAllowanceRaw = calibration.allowanceRaw;
  }
  steeringZeroSet = true;
}

void saveSteeringCalibration() {
  SteeringCalibration calibration;
  calibration.magic = CALIBRATION_MAGIC;
  calibration.rightReferenceRaw = steeringRightReferenceRaw;
  calibration.centerRaw = steeringZeroRaw;
  calibration.leftReferenceRaw = steeringLeftReferenceRaw;
  calibration.allowanceRaw = steeringLimitAllowanceRaw;
  calibration.checksum = steeringCalibrationChecksum(calibration);
  EEPROM.put(0, calibration);
}
