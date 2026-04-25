#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifndef SHARED_BUS_LOCK_TIMEOUT_MS
#define SHARED_BUS_LOCK_TIMEOUT_MS 250
#endif

#ifndef SHARED_BUS_READ_QUIET_MS
#define SHARED_BUS_READ_QUIET_MS 8
#endif

#ifndef SHARED_BUS_ARM_WRITE_QUIET_MS
#define SHARED_BUS_ARM_WRITE_QUIET_MS 40
#endif

#ifndef SHARED_BUS_GRIPPER_CONSTANT_PAUSE_MS
#define SHARED_BUS_GRIPPER_CONSTANT_PAUSE_MS 350
#endif

SemaphoreHandle_t sharedBusMutex = nullptr;
const char* sharedBusOwner = "idle";
unsigned long sharedBusConstantPausedUntil = 0;

void SharedBus_init() {
  if (sharedBusMutex == nullptr) {
    sharedBusMutex = xSemaphoreCreateRecursiveMutex();
  }
}

bool SharedBus_take(const char* owner, uint32_t timeoutMs = SHARED_BUS_LOCK_TIMEOUT_MS) {
  SharedBus_init();
  if (sharedBusMutex == nullptr) {
    if (InfoPrint == 1) {
      Serial.println("Shared servo bus mutex init failed.");
    }
    return false;
  }

  if (xSemaphoreTakeRecursive(sharedBusMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE) {
    sharedBusOwner = owner;
    return true;
  }

  if (InfoPrint == 1) {
    Serial.print("Shared servo bus busy, owner: ");
    Serial.println(sharedBusOwner);
  }
  return false;
}

void SharedBus_release(uint32_t quietMs = 0) {
  if (quietMs > 0) {
    unsigned long quietUntil = millis() + quietMs;
    if ((long)(quietUntil - sharedBusQuietUntil) > 0) {
      sharedBusQuietUntil = quietUntil;
    }
  }
  sharedBusOwner = "idle";
  if (sharedBusMutex != nullptr) {
    xSemaphoreGiveRecursive(sharedBusMutex);
  }
}

void SharedBus_pauseConstantMotion(uint32_t pauseMs) {
  unsigned long pauseUntil = millis() + pauseMs;
  if ((long)(pauseUntil - sharedBusConstantPausedUntil) > 0) {
    sharedBusConstantPausedUntil = pauseUntil;
  }
}

bool SharedBus_canRunConstantMotion() {
  return (long)(millis() - sharedBusConstantPausedUntil) >= 0;
}
