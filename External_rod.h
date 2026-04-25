#pragma once

#include <SCServo.h>

SMS_STS externalRodBus;

bool ExternalRod_takeBus(const char* owner) {
#if EXT_ROD_USE_ARM_BUS
  return SharedBus_take(owner);
#else
  return true;
#endif
}

void ExternalRod_releaseBus(uint32_t quietMs) {
#if EXT_ROD_USE_ARM_BUS
  SharedBus_release(quietMs);
#else
  (void)quietMs;
#endif
}

void ExternalRod_pauseArmConstant(uint32_t pauseMs = SHARED_BUS_ROD_CONSTANT_PAUSE_MS) {
#if EXT_ROD_USE_ARM_BUS
  SharedBus_pauseConstantMotion(pauseMs);
#else
  (void)pauseMs;
#endif
}

struct ExternalRodFeedback {
  bool status;
  int pos;
  int speed;
  int load;
  float voltage;
  float current;
  float temper;
  byte mode;
};

ExternalRodFeedback externalRodFeedback = {false, 0, 0, 0, 0, 0, 0, 0};
bool externalRodOnline = false;
int externalRodRequestedTorque = EXT_ROD_DEFAULT_TORQUE;
bool externalRodTorqueWarned = false;

s16 externalRodAngleToPos(double inputAng) {
  double limitedAng = constrain(inputAng, EXT_ROD_MIN_ANGLE_DEG, EXT_ROD_MAX_ANGLE_DEG);
  double pos = (limitedAng / 360.0) * ARM_SERVO_POS_RANGE;
  return constrain((s16)round(pos), 0, ARM_SERVO_POS_RANGE - 1);
}

void ExternalRod_servoInit() {
#if EXT_ROD_USE_ARM_BUS
  externalRodBus.pSerial = &Serial1;
  if (InfoPrint == 1) {
    Serial.println("External rod ST3215 uses RoArm bus servo UART.");
  }
#else
  Serial2.begin(1000000, SERIAL_8N1, EXT_ROD_RXD, EXT_ROD_TXD);
  externalRodBus.pSerial = &Serial2;
  delay(20);
  if (InfoPrint == 1) {
    Serial.println("External rod ST3215 init UART2TTL...");
  }
#endif
}

bool ExternalRod_getFeedback(bool quiet = false) {
  if (!ExternalRod_takeBus("rod feedback")) {
    externalRodFeedback.status = false;
    externalRodOnline = false;
    return false;
  }

  bool feedbackOk = externalRodBus.FeedBack(EXT_ROD_SERVO_ID) != -1;
  if (feedbackOk) {
    externalRodFeedback.status = true;
    externalRodFeedback.pos = externalRodBus.ReadPos(-1);
    externalRodFeedback.speed = externalRodBus.ReadSpeed(-1);
    externalRodFeedback.load = externalRodBus.ReadLoad(-1);
    externalRodFeedback.voltage = externalRodBus.ReadVoltage(-1);
    externalRodFeedback.current = externalRodBus.ReadCurrent(-1);
    externalRodFeedback.temper = externalRodBus.ReadTemper(-1);
    externalRodFeedback.mode = externalRodBus.ReadMode(EXT_ROD_SERVO_ID);
    externalRodOnline = true;
    ExternalRod_releaseBus(SHARED_BUS_READ_QUIET_MS);
    return true;
  }

  ExternalRod_releaseBus(SHARED_BUS_READ_QUIET_MS);
  externalRodFeedback.status = false;
  externalRodOnline = false;
  if (!quiet && InfoPrint == 1) {
    Serial.print("External rod servo ID:");
    Serial.print(EXT_ROD_SERVO_ID);
    Serial.println(" status: failed.");
  }
  return false;
}

void ExternalRod_initCheck() {
  if (ExternalRod_getFeedback(true)) {
    if (InfoPrint == 1) {
      Serial.println("External rod ST3215 status checked.");
    }
  } else if (InfoPrint == 1) {
    Serial.println("External rod ST3215 is offline, arm and gripper features remain available.");
  }
}

void ExternalRod_torqueCtrl(u8 enableCMD) {
  if (!ExternalRod_takeBus("rod torque")) {
    return;
  }
  externalRodBus.EnableTorque(EXT_ROD_SERVO_ID, enableCMD);
  ExternalRod_pauseArmConstant();
  ExternalRod_releaseBus(SHARED_BUS_ROD_CMD_QUIET_MS);
}

void ExternalRod_setMiddle(byte servoID) {
  if (!ExternalRod_takeBus("rod set middle")) {
    return;
  }
  externalRodBus.CalibrationOfs(servoID);
  ExternalRod_pauseArmConstant();
  ExternalRod_releaseBus(SHARED_BUS_ROD_CMD_QUIET_MS);
}

void ExternalRod_setServoMode(byte servoID = EXT_ROD_SERVO_ID) {
  if (!ExternalRod_takeBus("rod servo mode")) {
    return;
  }
  externalRodBus.unLockEprom(servoID);
  externalRodBus.writeByte(servoID, SMS_STS_MODE, 0);
  externalRodBus.LockEprom(servoID);
  ExternalRod_pauseArmConstant();
  ExternalRod_releaseBus(SHARED_BUS_ROD_CMD_QUIET_MS);
}

void ExternalRod_setMotorMode(byte servoID = EXT_ROD_SERVO_ID) {
  if (!ExternalRod_takeBus("rod motor mode")) {
    return;
  }
  externalRodBus.unLockEprom(servoID);
  externalRodBus.WheelMode(servoID);
  externalRodBus.LockEprom(servoID);
  ExternalRod_pauseArmConstant();
  ExternalRod_releaseBus(SHARED_BUS_ROD_CMD_QUIET_MS);
}

void ExternalRod_motorSpeed(s16 speedInput, u8 accInput) {
  if (!ExternalRod_takeBus("rod motor speed")) {
    return;
  }
  externalRodBus.WriteSpe(EXT_ROD_SERVO_ID, constrain(speedInput, -EXT_ROD_MAX_MOTOR_SPEED, EXT_ROD_MAX_MOTOR_SPEED), accInput);
  ExternalRod_pauseArmConstant();
  ExternalRod_releaseBus(SHARED_BUS_ROD_CMD_QUIET_MS);
}

void ExternalRod_torqueLimit(int inputTorque) {
  externalRodRequestedTorque = constrain(inputTorque, ST_TORQUE_MIN, ST_TORQUE_MAX);
  if (!externalRodTorqueWarned && InfoPrint == 1) {
    Serial.println("Note: public SCServo library accepts rod torque requests, but ST3215 torque-limit behavior depends on the servo firmware/register support.");
    externalRodTorqueWarned = true;
  }
}

void ExternalRod_dynamicAdaptation(byte inputMode, int inputTorque) {
  if (inputMode == 0) {
    ExternalRod_torqueLimit(ST_TORQUE_MAX);
  } else if (inputMode == 1) {
    ExternalRod_torqueLimit(inputTorque);
  }
}

void ExternalRod_moveToAngle(double inputAng, u16 speedInput, u8 accInput, u16 torqueInput) {
  externalRodRequestedTorque = constrain(torqueInput, ST_TORQUE_MIN, ST_TORQUE_MAX);
  if (!ExternalRod_takeBus("rod angle")) {
    return;
  }
  externalRodBus.WritePosEx(
    EXT_ROD_SERVO_ID,
    externalRodAngleToPos(inputAng),
    speedInput,
    accInput
  );
  ExternalRod_pauseArmConstant();
  ExternalRod_releaseBus(SHARED_BUS_ROD_CMD_QUIET_MS);
}

void ExternalRod_retract(u16 speedInput, u8 accInput, u16 torqueInput) {
  ExternalRod_moveToAngle(EXT_ROD_RETRACT_ANGLE_DEG, speedInput, accInput, torqueInput);
}

void ExternalRod_extend(u16 speedInput, u8 accInput, u16 torqueInput) {
  ExternalRod_moveToAngle(EXT_ROD_EXTEND_ANGLE_DEG, speedInput, accInput, torqueInput);
}

void Fusion_jointsGripperRodAngleCtrl(
  double inputBase,
  double inputShoulder,
  double inputElbow,
  double inputHand,
  double inputGripper,
  double inputRod,
  u16 inputSpd,
  u8 inputAcc,
  u16 gripperSpd,
  u8 gripperAcc,
  u16 gripperTorque,
  u16 rodSpd,
  u8 rodAcc,
  u16 rodTorque
) {
  ExternalRod_pauseArmConstant(SHARED_BUS_FUSION_CMD_QUIET_MS);
  RoArmM2_allJointsAngleCtrl(inputBase, inputShoulder, inputElbow, inputHand, inputSpd, inputAcc);
  delay(5);
  ExternalGripper_moveToAngle(inputGripper, gripperSpd, gripperAcc, gripperTorque);
  delay(5);
  ExternalRod_moveToAngle(inputRod, rodSpd, rodAcc, rodTorque);
  ExternalRod_pauseArmConstant(SHARED_BUS_FUSION_CMD_QUIET_MS);
  RoArmM2_holdTorqueLock(true);
#if EXT_ROD_USE_ARM_BUS
  unsigned long quietUntil = millis() + SHARED_BUS_FUSION_CMD_QUIET_MS;
  if ((long)(quietUntil - sharedBusQuietUntil) > 0) {
    sharedBusQuietUntil = quietUntil;
  }
#endif
}

void ExternalRod_infoFeedback() {
  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = 1531;
  jsonInfoHttp["target"] = "rod";
  jsonInfoHttp["status"] = externalRodFeedback.status;
  jsonInfoHttp["pos"] = externalRodFeedback.pos;
  jsonInfoHttp["speed"] = externalRodFeedback.speed;
  jsonInfoHttp["load"] = externalRodFeedback.load;
  jsonInfoHttp["voltage"] = externalRodFeedback.voltage;
  jsonInfoHttp["current"] = externalRodFeedback.current;
  jsonInfoHttp["temper"] = externalRodFeedback.temper;
  jsonInfoHttp["mode"] = externalRodFeedback.mode;
  jsonInfoHttp["reqTorque"] = externalRodRequestedTorque;

  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);
}
