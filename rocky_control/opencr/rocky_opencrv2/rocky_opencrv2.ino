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

// Timeouts and telemetry.
const unsigned long COMMAND_TIMEOUT_MS = 250;
const unsigned long TELEMETRY_PERIOD_MS = 200;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

// Binary protocol constants
#define CMD_SET_VELOCITY 0x01
#define CMD_STOP 0x02
#define BINARY_CMD_SIZE 17  // 1 byte header + 4 floats (4 bytes each)

// Dynamixel SyncWrite parameters
// Address 104 = Goal Velocity, Length = 4 bytes (int32_t)
// Format: [ID, vel_byte0, vel_byte1, vel_byte2, vel_byte3] repeated for each motor

uint8_t rxBuffer[BINARY_CMD_SIZE];
uint8_t rxIndex = 0;

unsigned long lastCommandMs = 0;
unsigned long lastTelemetryMs = 0;
bool robotStopped = true;

void stopRobot(bool force = false) {
  if (robotStopped && !force) {
    return;
  }

  // Use SyncWrite to stop all motors at once
  uint8_t sync_data[20];
  
  // All velocities = 0
  for (int i = 0; i < 20; i++) {
    if (i % 5 == 0) {
      // ID byte for each motor
      if (i == 0) sync_data[i] = DXL_ID_FL;
      else if (i == 5) sync_data[i] = DXL_ID_FR;
      else if (i == 10) sync_data[i] = DXL_ID_RL;
      else if (i == 15) sync_data[i] = DXL_ID_RR;
    } else {
      // Velocity bytes all 0
      sync_data[i] = 0x00;
    }
  }

  dxl.syncWrite(104, 4, sync_data, 20);
  robotStopped = true;
}

void setWheelVelocities(float frontLeftRpm, float frontRightRpm, float rearLeftRpm, float rearRightRpm) {
  // Prepare velocity values with sign correction for rear motors
  int32_t fl_vel = (int32_t)frontLeftRpm;
  int32_t fr_vel = (int32_t)frontRightRpm;
  int32_t rl_vel = (int32_t)(-rearLeftRpm);      // Negate rear left
  int32_t rr_vel = (int32_t)(-rearRightRpm);     // Negate rear right

  // Use SyncWrite to set Goal Velocity (address 104) for all motors at once
  // SyncWrite format: [ID, Data(LSB), Data, Data, Data(MSB)] x N motors
  uint8_t sync_data[20] = {0};  // 5 bytes per motor (1 ID + 4 data) x 4 motors
  
  // Motor 1: FL (ID=3)
  sync_data[0] = DXL_ID_FL;
  sync_data[1] = (fl_vel >> 0) & 0xFF;
  sync_data[2] = (fl_vel >> 8) & 0xFF;
  sync_data[3] = (fl_vel >> 16) & 0xFF;
  sync_data[4] = (fl_vel >> 24) & 0xFF;
  
  // Motor 2: FR (ID=4)
  sync_data[5] = DXL_ID_FR;
  sync_data[6] = (fr_vel >> 0) & 0xFF;
  sync_data[7] = (fr_vel >> 8) & 0xFF;
  sync_data[8] = (fr_vel >> 16) & 0xFF;
  sync_data[9] = (fr_vel >> 24) & 0xFF;
  
  // Motor 3: RL (ID=1)
  sync_data[10] = DXL_ID_RL;
  sync_data[11] = (rl_vel >> 0) & 0xFF;
  sync_data[12] = (rl_vel >> 8) & 0xFF;
  sync_data[13] = (rl_vel >> 16) & 0xFF;
  sync_data[14] = (rl_vel >> 24) & 0xFF;
  
  // Motor 4: RR (ID=2)
  sync_data[15] = DXL_ID_RR;
  sync_data[16] = (rr_vel >> 0) & 0xFF;
  sync_data[17] = (rr_vel >> 8) & 0xFF;
  sync_data[18] = (rr_vel >> 16) & 0xFF;
  sync_data[19] = (rr_vel >> 24) & 0xFF;

  // SyncWrite: address 104 (Goal Velocity), length 4 bytes, 4 motors
  dxl.syncWrite(104, 4, sync_data, 20);
  
  robotStopped = false;
}

void processCommand(uint8_t* buffer) {
  uint8_t cmd_type = buffer[0];

  if (cmd_type == CMD_SET_VELOCITY) {
    // Parse 4 floats from bytes 1-16
    float fl_rpm, fr_rpm, rl_rpm, rr_rpm;
    memcpy(&fl_rpm, &buffer[1], 4);
    memcpy(&fr_rpm, &buffer[5], 4);
    memcpy(&rl_rpm, &buffer[9], 4);
    memcpy(&rr_rpm, &buffer[13], 4);

    setWheelVelocities(fl_rpm, fr_rpm, rl_rpm, rr_rpm);
    lastCommandMs = millis();
  } 
  else if (cmd_type == CMD_STOP) {
    stopRobot(true);
    lastCommandMs = millis();
  }
}

void readSerialCommands() {
  while (DEBUG_SERIAL.available() > 0) {
    uint8_t byte = (uint8_t)DEBUG_SERIAL.read();
    rxBuffer[rxIndex++] = byte;

    // Check if we have a complete command
    if (rxIndex >= BINARY_CMD_SIZE) {
      processCommand(rxBuffer);
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

  DEBUG_SERIAL.println("Ready. Binary protocol: [CMD_type][FL_rpm_f][FR_rpm_f][RL_rpm_f][RR_rpm_f]");
  DEBUG_SERIAL.println("0x01 = SET_VELOCITY, 0x02 = STOP");
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

    DEBUG_SERIAL.print("RPM FL:");
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
