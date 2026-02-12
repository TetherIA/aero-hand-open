#include "Homing.h"
#include "HandConfig.h"

// ----------------------- Immutable baselines -----------------------
// Always define both symbols so linker can resolve them
const ServoData sd_base_left[7] = {
  {3186,2048,1},{2048,865,-1},{0,2980,1},{4095,817,-1},{4095,817,-1},{4095,817,-1},{4095,817,-1},
};
const ServoData sd_base_right[7] = {
  {910,2048,-1},{2048,3231,1},{4095,1115,-1},{0,3278,1},{0,3278,1},{0,3278,1},{0,3278,1},
};

// ---- Servo Presence Check and Current Limit Parameters ------------
static const uint8_t NUM_SERVOS = sizeof(SERVO_IDS) / sizeof(SERVO_IDS[0]);
static const uint16_t SERVO_CURRENT_LIMIT_MAX = 1023;
static const uint8_t REG_CURRENT_LIMIT = 28;

// ----------------------- Utilities -----------------------
void resetSdToBaseline() {
#if defined(RIGHT_HAND)
  const ServoData* src = sd_base_right;
#elif defined(LEFT_HAND)
  const ServoData* src = sd_base_left;
#else
  #warning "No hand macro defined; defaulting to RIGHT_HAND baseline"
  const ServoData* src = sd_base_right;
#endif
  for (int i = 0; i < 7; ++i) sd[i] = src[i];
}

// ----------------------- Busy flag & homing core -----------------------
static volatile bool s_busy = false;
bool HOMING_isBusy() { return s_busy; }

// ----------------------- Buffer Position Degrees -----------------------
static const float BUF_DEG = 10.0f;
static const uint16_t BUF_CNT =(uint16_t) ((BUF_DEG/360.0f) *4095.0f);

// -------Servo Presence + Current Limit Check (Post Homing) -------
static void enforceServoCurrentLimitPostHoming() {
  for (uint8_t i = 0; i < NUM_SERVOS; ++i) {
    const uint8_t id = SERVO_IDS[i];
    (void)hlscl.Ping(id);
    if (hlscl.getLastError()) {
      continue;
    }
    int cur = hlscl.readWord(id, REG_CURRENT_LIMIT);
    if (hlscl.getLastError() || cur < 0) {
      continue;
    }
    if ((uint16_t)cur != SERVO_CURRENT_LIMIT_MAX) {
      (void)hlscl.unLockEprom(id);
      (void)hlscl.writeWord(id, REG_CURRENT_LIMIT, SERVO_CURRENT_LIMIT_MAX);
      (void)hlscl.LockEprom(id);
    }
  }
}
static void set_servo_limits(uint8_t servoID, uint16_t minLim, uint16_t maxLim) {
  // clamp to 12-bit range just in case
  if (minLim > 4095) minLim = 4095;
  if (maxLim > 4095) maxLim = 4095;

  // EPROM write requires unlock (this disables torque)
  hlscl.unLockEprom(servoID);
  hlscl.writeWord(servoID, HLSCL_MIN_ANGLE_LIMIT_L, minLim); // addr 9-10
  hlscl.writeWord(servoID, HLSCL_MAX_ANGLE_LIMIT_L, maxLim); // addr 11-12
  hlscl.LockEprom(servoID);

  // restore torque so motion commands work
  hlscl.EnableTorque(servoID, 1);
}

static void zero_with_current(uint8_t index, int direction, int current_limit) {
  uint8_t servoID =SERVO_IDS[index];
  int current = 0;
  int position = 0;
  // During homing, open the travel window fully so nothing gets clamped.
  set_servo_limits(servoID, 0, 0);
  hlscl.ServoMode(servoID);
  hlscl.FeedBack(servoID);
  uint32_t t0 = millis();
  while (abs(current) < current_limit) {
    hlscl.WritePosEx(servoID, 50000 * direction, 2400, 0, current_limit);
    current  = hlscl.ReadCurrent(servoID);
    position = hlscl.ReadPos(servoID);
    if (millis() - t0 > 10000) break; 
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  // Primary calibration at contact
  hlscl.WritePosEx(servoID, position, 2400, 0, 1000);
  delay(30);
  hlscl.CalibrationOfs(servoID);
  delay(30);
  position = hlscl.ReadPos(servoID);

  if (servoID == 0) {
    // Thumb abduction: hold grasp posture for a moment
    hlscl.WritePosEx(servoID, sd[index].grasp_count, 2400, 0, 1000);
    delay(250);
  } else if (servoID == 1) {
    // Thumb flexion: go to extend
    hlscl.WritePosEx(servoID, sd[index].extend_count, 2400, 0, 1000);
    delay(250);
  } else if (servoID == 2) {
    // Thumb tendon: nudge and recalibrate, then extend
    hlscl.WritePosEx(servoID, position + (direction * 2048), 2400, 0, 1000);
    delay(250);
    hlscl.CalibrationOfs(servoID);
    delay(30);
    hlscl.WritePosEx(servoID, sd[index].extend_count - (direction * 625), 2400, 0, 1000);
    delay(30);
  } else {
    // No -Operation 
    //Wrong Servo IDs
  }
}

// Home the four finger servos (logical channels 3..6) together.
static void zero_fingers_parallel_with_current(uint8_t firstIdx, uint8_t count, int current_limit) {
  if (count == 0) return;
  if (firstIdx > 6) return;
  if (firstIdx + count > 7) count = 7 - firstIdx;
  const uint32_t TIMEOUT_MS = 10000;
  // Use static storage to avoid per-call stack usage for all 7 servos.
  static bool done[7];
  static int  contactPos[7];
  static int  cur[7];
  static int  pos[7];
  // Reset arrays to per-call defaults (was previously done by initializers).
  for (uint8_t i = 0; i < 7; ++i) {
    done[i] = false;
    contactPos[i] = 0;
    cur[i] = 0;
    pos[i] = 0;
  }
  for (uint8_t i = 0; i < count; ++i) {
    uint8_t idx = firstIdx + i;
    uint8_t servoID = SERVO_IDS[idx];
    set_servo_limits(servoID, 0, 0);
    hlscl.ServoMode(servoID);
    hlscl.FeedBack(servoID);
  }
  uint32_t t0 = millis();
  while (true) {
    bool allDone = true;
    for (uint8_t i = 0; i < count; ++i) {
      uint8_t idx = firstIdx + i;
      if (!done[idx]) { allDone = false; break; }
    }
    if (allDone) break;
    if (millis() - t0 > TIMEOUT_MS) break;
    for (uint8_t i = 0; i < count; ++i) {
      uint8_t idx = firstIdx + i;
      if (done[idx]) continue;
      uint8_t servoID = SERVO_IDS[idx];
      int dir = sd[idx].servo_direction;
      hlscl.WritePosEx(servoID, 50000 * dir, 2400, 0, current_limit);
    }
    for (uint8_t i = 0; i < count; ++i) {
      uint8_t idx = firstIdx + i;
      if (done[idx]) continue;
      uint8_t servoID = SERVO_IDS[idx];
      cur[idx] = hlscl.ReadCurrent(servoID);
      pos[idx] = hlscl.ReadPos(servoID);
      if (abs(cur[idx]) >= current_limit) {
        done[idx] = true;
        contactPos[idx] = pos[idx];
        hlscl.WritePosEx(servoID, contactPos[idx], 60, 50, 1000);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  for (uint8_t i = 0; i < count; ++i) {
    uint8_t idx = firstIdx + i;
    uint8_t servoID = SERVO_IDS[idx];
    int dir = sd[idx].servo_direction;
    // If we timed out before reaching threshold, fall back to last read pos.
    int p = done[idx] ? contactPos[idx] : pos[idx];
    // Primary calibration at contact
    hlscl.WritePosEx(servoID, p, 2400, 0, 1000);
    delay(30);
    hlscl.CalibrationOfs(servoID);
    delay(30);
    p = hlscl.ReadPos(servoID);
    hlscl.WritePosEx(servoID, p + (dir * 2048), 2400, 0, 1000);
    delay(300);
    hlscl.CalibrationOfs(servoID);
    delay(30);
    hlscl.WritePosEx(servoID, sd[idx].extend_count, 2400, 0, 1000);
  }
}

void zero_all_motors() {
  resetSdToBaseline();
  if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);
  zero_with_current(0,  sd[0].servo_direction, 950);   // Thumb Abduction
  zero_with_current(1,  sd[1].servo_direction, 950);   // Thumb Flex
  zero_with_current(2,  sd[2].servo_direction, 950);   // Thumb Tendon
  zero_fingers_parallel_with_current(3, 4, 950);       // Fingers together
  // Post-homing settling moves
  hlscl.WritePosEx(SERVO_IDS[0], sd[0].extend_count, 2400, 0, 1023);   // Thumb Abduction to extend
  hlscl.WritePosEx(SERVO_IDS[1], sd[1].extend_count, 2400, 0, 1023);   // Thumb Flexion to extend
  hlscl.WritePosEx(SERVO_IDS[2], sd[2].extend_count, 2400, 0, 1023);   // Thumb Tendon to extend
  hlscl.WritePosEx(SERVO_IDS[3], sd[3].extend_count, 2400, 0, 1023);   // Index to extend
  hlscl.WritePosEx(SERVO_IDS[4], sd[4].extend_count, 2400, 0, 1023);   // Middle to extend
  hlscl.WritePosEx(SERVO_IDS[5], sd[5].extend_count, 2400, 0, 1023);   // Ring to extend
  hlscl.WritePosEx(SERVO_IDS[6], sd[6].extend_count, 2400, 0, 1023);   // Pinky to extend
  
  if (gBusMux) xSemaphoreGive(gBusMux);
}

void HOMING_start() {
  s_busy = true;
  zero_all_motors();
  enforceServoCurrentLimitPostHoming();
  s_busy = false;
}