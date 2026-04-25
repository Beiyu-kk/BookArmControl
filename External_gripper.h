#pragma once

#include <SCServo.h>

SCSCL externalGripperBus;

bool ExternalGripper_takeBus(const char* owner) {
#if EXT_GRIPPER_USE_ARM_BUS
  return SharedBus_take(owner);
#else
  return true;
#endif
}

void ExternalGripper_releaseBus(uint32_t quietMs) {
#if EXT_GRIPPER_USE_ARM_BUS
  SharedBus_release(quietMs);
#else
  (void)quietMs;
#endif
}

void ExternalGripper_pauseArmConstant(uint32_t pauseMs = SHARED_BUS_GRIPPER_CONSTANT_PAUSE_MS) {
#if EXT_GRIPPER_USE_ARM_BUS
  SharedBus_pauseConstantMotion(pauseMs);
#else
  (void)pauseMs;
#endif
}

struct ExternalGripperFeedback {
  bool status;
  int pos;
  int speed;
  int load;
  float voltage;
  float current;
  float temper;
  byte mode;
};

ExternalGripperFeedback externalGripperFeedback = {false, 0, 0, 0, 0, 0, 0, 0};
bool externalGripperOnline = false;
int externalGripperRequestedTorque = EXT_GRIPPER_DEFAULT_TORQUE;
bool externalGripperTorqueWarned = false;

double externalGripperDegToRad(double inputAng) {
  return (inputAng / 180.0) * M_PI;
}

double externalGripperPosByRad(double radInput) {
  return round((radInput / (2 * M_PI)) * ARM_SERVO_POS_RANGE);
}

s16 externalGripperAngleToPos(double inputAng) {
  double radInput = externalGripperDegToRad(inputAng);
  s16 computePos = externalGripperPosByRad(radInput) + 1024;
  return constrain(computePos, 2047, 3150);
}

void ExternalGripper_servoInit() {
#if EXT_GRIPPER_USE_ARM_BUS
  externalGripperBus.pSerial = &Serial1;
  if (InfoPrint == 1) {
    Serial.println("External Gripper-B uses RoArm bus servo UART.");
  }
#else
  Serial2.begin(1000000, SERIAL_8N1, EXT_GRIPPER_RXD, EXT_GRIPPER_TXD);
  externalGripperBus.pSerial = &Serial2;
  delay(20);
  if (InfoPrint == 1) {
    Serial.println("External Gripper-B init UART2TTL...");
  }
#endif
}

bool ExternalGripper_getFeedback(bool quiet = false) {
  if (!ExternalGripper_takeBus("gripper feedback")) {
    externalGripperFeedback.status = false;
    externalGripperOnline = false;
    return false;
  }

  bool feedbackOk = externalGripperBus.FeedBack(EXT_GRIPPER_SERVO_ID) != -1;
  if (feedbackOk) {
    externalGripperFeedback.status = true;
    externalGripperFeedback.pos = externalGripperBus.ReadPos(EXT_GRIPPER_SERVO_ID);
    externalGripperFeedback.speed = externalGripperBus.ReadSpeed(EXT_GRIPPER_SERVO_ID);
    externalGripperFeedback.load = externalGripperBus.ReadLoad(EXT_GRIPPER_SERVO_ID);
    externalGripperFeedback.voltage = externalGripperBus.ReadVoltage(EXT_GRIPPER_SERVO_ID);
    externalGripperFeedback.current = externalGripperBus.ReadCurrent(EXT_GRIPPER_SERVO_ID);
    externalGripperFeedback.temper = externalGripperBus.ReadTemper(EXT_GRIPPER_SERVO_ID);
    externalGripperFeedback.mode = externalGripperBus.ReadMode(EXT_GRIPPER_SERVO_ID);
    externalGripperOnline = true;
    ExternalGripper_releaseBus(SHARED_BUS_READ_QUIET_MS);
    return true;
  }

  ExternalGripper_releaseBus(SHARED_BUS_READ_QUIET_MS);
  externalGripperFeedback.status = false;
  externalGripperOnline = false;
  if (!quiet && InfoPrint == 1) {
    Serial.print("External Gripper-B servo ID:");
    Serial.print(EXT_GRIPPER_SERVO_ID);
    Serial.println(" status: failed.");
  }
  return false;
}

void ExternalGripper_initCheck() {
  if (ExternalGripper_getFeedback(true)) {
    if (InfoPrint == 1) {
      Serial.println("External Gripper-B status checked.");
    }
  } else if (InfoPrint == 1) {
    Serial.println("External Gripper-B is offline, arm features remain available.");
  }
}

void ExternalGripper_torqueCtrl(u8 enableCMD) {
  if (!ExternalGripper_takeBus("gripper torque")) {
    return;
  }
  externalGripperBus.EnableTorque(EXT_GRIPPER_SERVO_ID, enableCMD);
  ExternalGripper_pauseArmConstant();
  ExternalGripper_releaseBus(SHARED_BUS_GRIPPER_CMD_QUIET_MS);
}

void ExternalGripper_setMiddle(byte servoID) {
  if (!ExternalGripper_takeBus("gripper set middle")) {
    return;
  }
  externalGripperBus.CalibrationOfs(servoID);
  ExternalGripper_pauseArmConstant();
  ExternalGripper_releaseBus(SHARED_BUS_GRIPPER_CMD_QUIET_MS);
}

void ExternalGripper_setEleMode(byte servoID, s16 torque) {
  if (!ExternalGripper_takeBus("gripper ele mode")) {
    return;
  }
  externalGripperBus.PWMMode(servoID);
  torque = constrain(torque, -200, 200);
  externalGripperBus.WritePWM(servoID, torque);
  ExternalGripper_pauseArmConstant();
  ExternalGripper_releaseBus(SHARED_BUS_GRIPPER_CMD_QUIET_MS);
}

void ExternalGripper_torqueLimit(int inputTorque) {
  externalGripperRequestedTorque = constrain(inputTorque, ST_TORQUE_MIN, ST_TORQUE_MAX);
  if (!externalGripperTorqueWarned && InfoPrint == 1) {
    Serial.println("Note: public SCServo library does not expose CF35 torque-limit registers; torque values are accepted but not applied.");
    externalGripperTorqueWarned = true;
  }
}

void ExternalGripper_dynamicAdaptation(byte inputMode, int inputTorque) {
  if (inputMode == 0) {
    ExternalGripper_torqueLimit(ST_TORQUE_MAX);
  } else if (inputMode == 1) {
    ExternalGripper_torqueLimit(inputTorque);
  }
}

void ExternalGripper_moveToAngle(double inputAng, u16 speedInput, u8 accInput, u16 torqueInput) {
  externalGripperRequestedTorque = constrain(torqueInput, ST_TORQUE_MIN, ST_TORQUE_MAX);
  if (!ExternalGripper_takeBus("gripper angle")) {
    return;
  }
  externalGripperBus.WritePosEx(
    EXT_GRIPPER_SERVO_ID,
    externalGripperAngleToPos(inputAng),
    speedInput,
    accInput
  );
  ExternalGripper_pauseArmConstant();
  ExternalGripper_releaseBus(SHARED_BUS_GRIPPER_CMD_QUIET_MS);
}

void ExternalGripper_open(u16 speedInput, u8 accInput, u16 torqueInput) {
  externalGripperRequestedTorque = constrain(torqueInput, ST_TORQUE_MIN, ST_TORQUE_MAX);
  if (!ExternalGripper_takeBus("gripper open")) {
    return;
  }
  externalGripperBus.WritePosEx(
    EXT_GRIPPER_SERVO_ID,
    2047,
    speedInput,
    accInput
  );
  ExternalGripper_pauseArmConstant();
  ExternalGripper_releaseBus(SHARED_BUS_GRIPPER_CMD_QUIET_MS);
}

void ExternalGripper_close(u16 speedInput, u8 accInput, u16 torqueInput) {
  externalGripperRequestedTorque = constrain(torqueInput, ST_TORQUE_MIN, ST_TORQUE_MAX);
  if (!ExternalGripper_takeBus("gripper close")) {
    return;
  }
  externalGripperBus.WritePosEx(
    EXT_GRIPPER_SERVO_ID,
    3150,
    speedInput,
    accInput
  );
  ExternalGripper_pauseArmConstant();
  ExternalGripper_releaseBus(SHARED_BUS_GRIPPER_CMD_QUIET_MS);
}

void Fusion_jointsAngleGripperCtrl(
  double inputBase,
  double inputShoulder,
  double inputElbow,
  double inputHand,
  double inputGripper,
  u16 inputSpd,
  u8 inputAcc,
  u16 gripperSpd,
  u8 gripperAcc,
  u16 gripperTorque
) {
  ExternalGripper_pauseArmConstant(SHARED_BUS_FUSION_CMD_QUIET_MS);
  RoArmM2_allJointsAngleCtrl(inputBase, inputShoulder, inputElbow, inputHand, inputSpd, inputAcc);
  delay(5);
  ExternalGripper_moveToAngle(inputGripper, gripperSpd, gripperAcc, gripperTorque);
  ExternalGripper_pauseArmConstant(SHARED_BUS_FUSION_CMD_QUIET_MS);
  RoArmM2_holdTorqueLock(true);
#if EXT_GRIPPER_USE_ARM_BUS
  unsigned long quietUntil = millis() + SHARED_BUS_FUSION_CMD_QUIET_MS;
  if ((long)(quietUntil - sharedBusQuietUntil) > 0) {
    sharedBusQuietUntil = quietUntil;
  }
#endif
}

void ExternalGripper_infoFeedback() {
  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = 1331;
  jsonInfoHttp["target"] = "gripper";
  jsonInfoHttp["status"] = externalGripperFeedback.status;
  jsonInfoHttp["pos"] = externalGripperFeedback.pos;
  jsonInfoHttp["speed"] = externalGripperFeedback.speed;
  jsonInfoHttp["load"] = externalGripperFeedback.load;
  jsonInfoHttp["voltage"] = externalGripperFeedback.voltage;
  jsonInfoHttp["current"] = externalGripperFeedback.current;
  jsonInfoHttp["temper"] = externalGripperFeedback.temper;
  jsonInfoHttp["mode"] = externalGripperFeedback.mode;
  jsonInfoHttp["reqTorque"] = externalGripperRequestedTorque;

  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);
}
