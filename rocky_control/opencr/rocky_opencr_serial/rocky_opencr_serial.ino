#include <Dynamixel2Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// OpenCR serial definitions.
#define DXL_SERIAL Serial3
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 84;

// DYNAMIXEL IDs.
const uint8_t DXL_ID_RL = 1;
const uint8_t DXL_ID_RR = 2;
const uint8_t DXL_ID_FL = 3;
const uint8_t DXL_ID_FR = 4;
const float DXL_PROTOCOL_VERSION = 2.0f;

// Conversion and limits.
const float RAD_PER_SEC_TO_RPM = 60.0f / (2.0f * M_PI);
const float MAX_VX = 0.30f;   // m/s
const float MAX_VY = 0.30f;   // m/s
const float MAX_WZ = 1.50f;   // rad/s

// Timeouts and telemetry.
const unsigned long COMMAND_TIMEOUT_MS = 250;
const unsigned long TELEMETRY_PERIOD_MS = 200;

// Robot geometry (meters).
const float HALF_WHEELBASE_D = 0.0537f;
const float HALF_TRACK_L = 0.0375f;
const float WHEEL_RADIUS_A = 0.0297f;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

char rxBuffer[80];
uint8_t rxIndex = 0;

float cmdVx = 0.0f;
float cmdVy = 0.0f;
float cmdWz = 0.0f;

unsigned long lastCommandMs = 0;
unsigned long lastTelemetryMs = 0;
bool robotStopped = true;

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

void stopRobot(bool force = false) {
  if (robotStopped && !force) {
    return;
  }

  dxl.setGoalVelocity(DXL_ID_FL, 0, UNIT_RPM);
  dxl.setGoalVelocity(DXL_ID_FR, 0, UNIT_RPM);
  dxl.setGoalVelocity(DXL_ID_RL, 0, UNIT_RPM);
  dxl.setGoalVelocity(DXL_ID_RR, 0, UNIT_RPM);
  robotStopped = true;
}

void inverseKinematics(float forwBackVel, float leftRightVel, float rotVel) {
  // Desired robot velocities in body frame.
  float u = forwBackVel;
  float v = leftRightVel;
  float r = rotVel;

  // Wheel angular velocities (rad/s).
  float w1 = (1.0f / WHEEL_RADIUS_A) * (u + v + (-HALF_WHEELBASE_D + HALF_TRACK_L) * r);  // front left
  float w2 = (1.0f / WHEEL_RADIUS_A) * (u - v + (-HALF_WHEELBASE_D + HALF_TRACK_L) * r);  // front right
  float w3 = (1.0f / WHEEL_RADIUS_A) * (u + v + ( HALF_WHEELBASE_D - HALF_TRACK_L) * r);  // rear left
  float w4 = (1.0f / WHEEL_RADIUS_A) * (u - v + ( HALF_WHEELBASE_D - HALF_TRACK_L) * r);  // rear right

  // Convert rad/s -> RPM.
  float frontLeftRpm  = w1 * RAD_PER_SEC_TO_RPM;
  float frontRightRpm = w2 * RAD_PER_SEC_TO_RPM;
  float rearLeftRpm   = w3 * RAD_PER_SEC_TO_RPM;
  float rearRightRpm  = w4 * RAD_PER_SEC_TO_RPM;

  dxl.setGoalVelocity(DXL_ID_FL, frontLeftRpm, UNIT_RPM);
  dxl.setGoalVelocity(DXL_ID_FR, frontRightRpm, UNIT_RPM);
  dxl.setGoalVelocity(DXL_ID_RL, -rearLeftRpm, UNIT_RPM);
  dxl.setGoalVelocity(DXL_ID_RR, -rearRightRpm, UNIT_RPM);
  robotStopped = false;
}

void applyCommand(float vx, float vy, float wz) {
  cmdVx = clampFloat(vx, -MAX_VX, MAX_VX);
  cmdVy = clampFloat(vy, -MAX_VY, MAX_VY);
  cmdWz = clampFloat(wz, -MAX_WZ, MAX_WZ);

  inverseKinematics(cmdVx, cmdVy, cmdWz);
  lastCommandMs = millis();
}

void processLine(const char* line) {
  float vx, vy, wz;

  if (sscanf(line, "CMD,%f,%f,%f", &vx, &vy, &wz) == 3) {
    applyCommand(vx, vy, wz);
    return;
  }

  if (strcmp(line, "STOP") == 0) {
    cmdVx = 0.0f;
    cmdVy = 0.0f;
    cmdWz = 0.0f;
    stopRobot(true);
    lastCommandMs = millis();
  }
}

void readSerialCommands() {
  while (DEBUG_SERIAL.available() > 0) {
    char c = (char)DEBUG_SERIAL.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      rxBuffer[rxIndex] = '\0';
      if (rxIndex > 0) {
        processLine(rxBuffer);
      }
      rxIndex = 0;
      continue;
    }

    if (rxIndex < sizeof(rxBuffer) - 1) {
      rxBuffer[rxIndex++] = c;
    } else {
      // Drop oversized line and start fresh.
      rxIndex = 0;
    }
  }
}

void setup() {
  DEBUG_SERIAL.begin(1000000);

  // Allow time for USB CDC attach but do not block forever.
  unsigned long waitStart = millis();
  while (!DEBUG_SERIAL && (millis() - waitStart < 2000)) {
  }

  DEBUG_SERIAL.println("OpenCR serial controller starting...");

  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  DEBUG_SERIAL.print("Ping FR: ");
  DEBUG_SERIAL.println(dxl.ping(DXL_ID_FR));
  DEBUG_SERIAL.print("Ping FL: ");
  DEBUG_SERIAL.println(dxl.ping(DXL_ID_FL));
  DEBUG_SERIAL.print("Ping RR: ");
  DEBUG_SERIAL.println(dxl.ping(DXL_ID_RR));
  DEBUG_SERIAL.print("Ping RL: ");
  DEBUG_SERIAL.println(dxl.ping(DXL_ID_RL));

  uint8_t ids[] = {DXL_ID_FR, DXL_ID_FL, DXL_ID_RR, DXL_ID_RL};
  for (int i = 0; i < 4; i++) {
    dxl.torqueOff(ids[i]);
    dxl.setOperatingMode(ids[i], OP_VELOCITY);
    dxl.torqueOn(ids[i]);
  }

  stopRobot(true);
  lastCommandMs = millis();
  lastTelemetryMs = millis();

  DEBUG_SERIAL.println("Ready. Send lines as: CMD,vx,vy,wz");
}

void loop() {
  readSerialCommands();

  unsigned long now = millis();
  if (now - lastCommandMs > COMMAND_TIMEOUT_MS) {
    stopRobot();
  }

  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    float fl = dxl.getPresentVelocity(DXL_ID_FL, UNIT_RPM);
    float fr = dxl.getPresentVelocity(DXL_ID_FR, UNIT_RPM);
    float rl = dxl.getPresentVelocity(DXL_ID_RL, UNIT_RPM);
    float rr = dxl.getPresentVelocity(DXL_ID_RR, UNIT_RPM);

    DEBUG_SERIAL.print("CMD vx:");
    DEBUG_SERIAL.print(cmdVx, 3);
    DEBUG_SERIAL.print(" vy:");
    DEBUG_SERIAL.print(cmdVy, 3);
    DEBUG_SERIAL.print(" wz:");
    DEBUG_SERIAL.print(cmdWz, 3);
    DEBUG_SERIAL.print(" | RPM FL:");
    DEBUG_SERIAL.print(fl, 1);
    DEBUG_SERIAL.print(" FR:");
    DEBUG_SERIAL.print(fr, 1);
    DEBUG_SERIAL.print(" RL:");
    DEBUG_SERIAL.print(rl, 1);
    DEBUG_SERIAL.print(" RR:");
    DEBUG_SERIAL.println(rr, 1);

    lastTelemetryMs = now;
  }
}
