// TetherIA - Open Source Hand
// Aero Hand Firmware Source Code
#include <Arduino.h>
#include <Wire.h>
#include <HLSCL.h>
#include <Preferences.h>
#include "HandConfig.h"
#include "Homing.h"
#include "esp_system.h"

HLSCL hlscl;
Preferences prefs;

// ---- UART pins to the servo bus (ESP32-S3 XIAO: RX=2, TX=3) ----
#define SERIAL2_TX_PIN 3
#define SERIAL2_RX_PIN 2

// ---- Servo IDs (declare at top) ----
const uint8_t SERVO_IDS[7] = { 0, 1, 2, 3, 4, 5, 6 };

ServoData sd[7];

// ---- Constants for Control Code byte ----
static const uint8_t HOMING    = 0x01;
static const uint8_t SET_ID    = 0x02;
static const uint8_t TRIM      = 0x03;
static const uint8_t CTRL_POS  = 0x11;
static const uint8_t CTRL_TOR  = 0x12;
static const uint8_t GET_POS   = 0x22;
static const uint8_t GET_VEL   = 0x23;
static const uint8_t GET_CURR  = 0x24;
static const uint8_t GET_TEMP  = 0x25;
static const uint8_t SET_SPE   = 0x31;
static const uint8_t SET_TOR   = 0x32;

// ---- Defaults for SyncWritePosEx ----
static uint16_t g_speed[7]  = {32766,32766,32766,32766,32766,32766,32766};
static uint8_t  g_accel[7]  = {0,0,0,0,0,0,0};     // 0..255
static uint16_t g_torque[7] = {700,700,700,700,700,700,700}; // 0....1000
static const uint16_t HOLD_MAG = 5;       // the minimal torque actually commanded to the motors during the torque mode. Any torque below that will not be used.

// Last commanded torque per servo (signed)
static int16_t g_lastTorqueCmd[7] = {0};

// ---- Thermal torque limiting (GLOBAL PARAMETERS) ----
static uint8_t  TEMP_CUTOFF_C    = 70;    // °C cutoff
static uint16_t HOT_TORQUE_LIMIT = 500;   // clamp torque when motor exceeds TEMP_CUTOFF_C 

// ----- Registers / constants (Mapped as per Feetech Servo HLS3606M) -----
#define REG_ID                 0x05       // ID register
#define REG_CURRENT_LIMIT      28         // decimal address (word)
#define BROADCAST_ID           0xFE
#define SCAN_MIN               0
#define SCAN_MAX               253
#define REG_BLOCK_LEN          15
#define REG_BLOCK_START        56

// ----- Structure for the Metrics of Servo -------
struct ServoMetrics {
  uint16_t pos[7];
  uint16_t vel[7];
  uint16_t cur[7];
  uint16_t tmp[7];
};
static ServoMetrics gMetrics;

// -------- Global Control Mode State  ---------
enum ControlMode{
  MODE_POS=0,
  MODE_TORQUE=2
};
static ControlMode g_currentMode = MODE_POS;

// ----- Semaphores for Metrics and Bus for acquiring lock and release it -----
static SemaphoreHandle_t gMetricsMux;
SemaphoreHandle_t gBusMux = nullptr;

// ----- Homing module API Calls (provided below as .h/.cpp) --------
bool HOMING_isBusy();
void HOMING_start();

// ---- Helper function to read latest temp from servo
static inline uint8_t getTempC(uint8_t ch) {
  uint8_t t = 0;
  if (gMetricsMux) xSemaphoreTake(gMetricsMux, portMAX_DELAY);
  t = (uint8_t)gMetrics.tmp[ch];
  if (gMetricsMux) xSemaphoreGive(gMetricsMux);
  return t;
}

static inline bool isHot(uint8_t ch) {
  return getTempC(ch) >= TEMP_CUTOFF_C;
}

static inline uint16_t u16_min(uint16_t a, uint16_t b) { return (a < b) ? a : b; }

// ---- Set-ID helpers for setting ID ---
extern void runReIdScanAndSet(uint8_t Id, uint16_t currentLimit);
static volatile int g_lastFoundId; 

// ----- Helper Functions for Set-ID Mode -----
static bool scanRequireSingleServo(uint8_t* outId, uint8_t requestedNewId) {
  uint8_t first = 0xFF;
  int count = 0;
  if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);
  for (int id = SCAN_MIN; id <= SCAN_MAX; ++id) {
    if (id == BROADCAST_ID) continue;        
    (void)hlscl.Ping((uint8_t)id);
    if (!hlscl.getLastError()) {
      if (count == 0) first = (uint8_t)id;
      ++count;
      if (count > 1) break;                    
    }
  }
  if (gBusMux) xSemaphoreGive(gBusMux);
  if (count == 1) {
    if (outId) *outId = first;
    return true;
  }
  if (count == 0) {
    uint8_t ack6[6] = { 0xFF, 0x00, requestedNewId, 0x00, 0x00, 0x00 };
    sendAckFrame(SET_ID, ack6, sizeof(ack6));
    return false;
  }
  uint8_t ack14[14] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  sendAckFrame(SET_ID, ack14, sizeof(ack14));
  return false;
}
static void sendSetIdAck(uint8_t oldId, uint8_t newId, uint16_t curLimitWord) {
  uint16_t vals[7] = {0};
  vals[0] = oldId;
  vals[1] = newId;
  vals[2] = curLimitWord;
  uint8_t out[2 + 7*2];
  out[0] = SET_ID;  // 0x03
  out[1] = 0x00;    // filler
  for (int i = 0; i < 7; ++i) {
    out[2 + 2*i + 0] = (uint8_t)(vals[i] & 0xFF);
    out[2 + 2*i + 1] = (uint8_t)((vals[i] >> 8) & 0xFF);
  }
  Serial.write(out, sizeof(out));
}
// ---- Helper function to save and loadManual extends from NVS ------
static void loadManualExtendsFromNVS() {
  prefs.begin("hand", true);  // read-only
  for (uint8_t i = 0; i < 7; ++i) {
    // Store extends per logical channel index, not servo bus ID.
    String key = "ext" + String(i);
    int v = prefs.getInt(key.c_str(), -1);
    if (v >= 0 && v <= 4095) {
      sd[i].extend_count = v;
    }
  }
}

static void saveExtendsToNVS() {
  prefs.begin("hand", false);  // RW
  for (uint8_t i = 0; i < 7; ++i) {
    // Store extends per logical channel index, not servo bus ID.
    String kext = "ext" + String(i);
    prefs.putInt(kext.c_str(), (int)sd[i].extend_count);
  }
  prefs.end();
}

// -------- Soft-Limit Safety for Servo motors
static void checkAndEnforceSoftLimits()
{
  static bool torqueLimited[7] = {false};
  static uint32_t lastCheckMs = 0;
  uint32_t now = millis();
  if (now - lastCheckMs < 20) return;
  lastCheckMs = now;

  if (g_currentMode != MODE_TORQUE) {
    // Only apply soft limits during torque control.
    for (int i = 0; i < 7; ++i) torqueLimited[i] = false;
    return;
  }

  if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);
  for (uint8_t i = 0; i < 7; ++i) {
    uint8_t id = SERVO_IDS[i];
    int pos = hlscl.ReadPos(id);
    if (pos < 0) pos += 32768;
    uint16_t raw = pos % 4096;
    uint16_t ext = sd[i].extend_count;
    uint16_t gra = sd[i].grasp_count;
    uint16_t rawMin = min(ext, gra);
    uint16_t rawMax = max(ext, gra);
    bool inRange = (raw >= rawMin && raw <= rawMax);
    if (!inRange && !torqueLimited[i]) {
      int16_t limitedTorque = (g_lastTorqueCmd[i] >= 0) ? 200 : -200;
      hlscl.WriteEle(id, limitedTorque);
      torqueLimited[i] = true;
    } else if (inRange && torqueLimited[i]) {
      hlscl.WriteEle(id, g_lastTorqueCmd[i]);
      torqueLimited[i] = false;
    }
  }
  if (gBusMux) xSemaphoreGive(gBusMux);
}


// ---- Helper function  for raw to U16 and U16 to raw ----
static inline uint16_t mapRawToU16(uint8_t ch, uint16_t raw) {
  int32_t ext  = sd[ch].extend_count;
  int32_t gra  = sd[ch].grasp_count;
  int32_t span = gra - ext;
  if (span == 0) return 0;  // avoid divide-by-zero
  int32_t val = ((int32_t)(raw - ext) * 65535L) / span;
  //Clamp
  if (val < 0) val = 0;
  if (val > 65535) val = 65535;
  return (uint16_t)val;
}
static inline uint16_t mapU16ToRaw(uint8_t ch, uint16_t u16) {
  int32_t ext = sd[ch].extend_count;
  int32_t gra = sd[ch].grasp_count;
  int32_t raw32;
  if (ext == 0 && gra == 0) {
    raw32 = ((uint64_t)u16 * 4095u) / 65535u;
  } else {
    raw32 = ext + ((int64_t)u16 * (gra - ext)) / 65535LL;
  }
  // clamp
  if (raw32 < 0)    raw32 = 0;
  if (raw32 > 4095) raw32 = 4095;
  return (uint16_t)raw32;
}

// ---- Helper Functions for u16, Decode to sign and copy values in u16 format----
static inline uint16_t leu_u16(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline int16_t decode_signmag15(uint8_t lo, uint8_t hi) {
  uint16_t mag = ((uint16_t)(hi & 0x7F) << 8) | lo;  // 15-bit magnitude
  return (hi & 0x80) ? -(int16_t)mag : (int16_t)mag;
}
static inline void copy7_u16(uint16_t dst[7], const uint16_t src[7]) {
  for (int i = 0; i < 7; ++i) dst[i] = src[i];
}

// ----- Helper function to send u16 frame for POS,SPD,VEL,CURR and sendACK Packet ----
static inline void sendU16Frame(uint8_t header, const uint16_t data[7]) {
  uint8_t out[2 + 7*2];
  out[0] = header;
  out[1] = 0x00; // filler
  for (int i = 0; i < 7; ++i) {
    out[2 + 2*i + 0] = (uint8_t)(data[i] & 0xFF);
    out[2 + 2*i + 1] = (uint8_t)((data[i] >> 8) & 0xFF);
  }
  Serial.write(out, sizeof(out)); 
}
static inline void sendAckFrame(uint8_t header, const uint8_t* payload, size_t n) {
  uint8_t out[16];
  out[0] = header;
  out[1] = 0x00; // filler
  memset(out + 2, 0, 14);
  if (payload && n) {
    if (n > 14) n = 14;
    memcpy(out + 2, payload, n);
  }
  Serial.write(out, sizeof(out));
}

// ----- Functions to Send POS, VEL, CURR, TEMP -----
void sendPositions() {
  uint16_t buf[7];
  xSemaphoreTake(gMetricsMux, portMAX_DELAY);
  copy7_u16(buf, gMetrics.pos);
  xSemaphoreGive(gMetricsMux);
  sendU16Frame(GET_POS, buf);
}
void sendVelocities() {
  uint16_t buf[7];
  xSemaphoreTake(gMetricsMux, portMAX_DELAY);
  copy7_u16(buf, gMetrics.vel);
  xSemaphoreGive(gMetricsMux);
  sendU16Frame(GET_VEL, buf);
}
void sendCurrents() {
  uint16_t buf[7];
  xSemaphoreTake(gMetricsMux, portMAX_DELAY);
  copy7_u16(buf, gMetrics.cur);
  xSemaphoreGive(gMetricsMux);
  sendU16Frame(GET_CURR, buf);
}
void sendTemps() {
  uint16_t buf[7];
  xSemaphoreTake(gMetricsMux, portMAX_DELAY);
  copy7_u16(buf, gMetrics.tmp);
  xSemaphoreGive(gMetricsMux);
  sendU16Frame(GET_TEMP, buf);
}

// ----- Task - Sync Read running always on Core 1 ----- 
static void TaskSyncRead_Core1(void *arg) {
  uint8_t  rx[REG_BLOCK_LEN];          // 15 bytes
  uint16_t pos[7], vel[7], cur[7], tmp[7];
  const TickType_t period = pdMS_TO_TICKS(10);   // Change Frequency of Running here, 5 -200 Hz, 10-100 Hz, 20 -50 Hz
  TickType_t nextWake = xTaskGetTickCount();
  for (;;) {
    // try-lock: if control is using the bus, skip this cycle
    if (gBusMux && xSemaphoreTake(gBusMux, 0) != pdTRUE) {
      vTaskDelayUntil(&nextWake, period);
      continue;
    }
    bool ok = true;
    // one TX for the whole group (15-byte slice)
    hlscl.syncReadPacketTx((uint8_t*)SERVO_IDS, 7, REG_BLOCK_START, REG_BLOCK_LEN);
    for (uint8_t i = 0; i < 7; ++i) {
      if (!hlscl.syncReadPacketRx(SERVO_IDS[i], rx)) { ok = false; break; }
      uint16_t raw = leu_u16(&rx[0]);
      pos[i] = mapRawToU16(i,raw);                     // Position (unsigned)
      vel[i] = decode_signmag15(rx[2], rx[3]);                  // velocity (signed)
      tmp[i] = rx[7];                                           // temperature (unsigned, 1 byte)
      cur[i] = decode_signmag15(rx[13], rx[14]);                // current (signed)
      //vTaskDelay(1);
    }
    if (gBusMux) xSemaphoreGive(gBusMux);
    if (ok) {
      xSemaphoreTake(gMetricsMux, portMAX_DELAY);
      for (int i = 0; i < 7; ++i) {
        gMetrics.pos[i] = pos[i];
        gMetrics.vel[i] = vel[i];
        gMetrics.tmp[i] = tmp[i];
        gMetrics.cur[i] = cur[i];
      }
      xSemaphoreGive(gMetricsMux);
    } 
    vTaskDelayUntil(&nextWake, period);
  }
}
//Set-ID and Trim Servo functions
static bool handleSetIdCmd(const uint8_t* payload) {
  // Parse request: two u16 words, little-endian
  uint16_t w0 = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8); // newId in low byte
  uint16_t w1 = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8); // requested current limit
  uint8_t  newId    = (uint8_t)(w0 & 0xFF);
  uint16_t reqLimit = (w1 > 1023) ? 1023 : w1;
  // Invalid newId → ACK with oldId=0xFF, newId, cur=0
  if (newId > 253 || newId ==BROADCAST_ID) {
    uint8_t ack[6] = { 0xFF, 0x00, newId, 0x00, 0x00, 0x00 };
    sendAckFrame(SET_ID, ack, sizeof(ack));
    return true;
  }
  // Find any servo present
  uint8_t oldId = 0xFF;
  if (!scanRequireSingleServo(&oldId, newId)) return true; 
  
  if (newId != oldId) {
  if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);
  (void)hlscl.Ping(newId);
  bool taken = !hlscl.getLastError();
  if (gBusMux) xSemaphoreGive(gBusMux);
  if (taken) {
    uint8_t ack14[14] = { oldId, 0x00, newId, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    sendAckFrame(SET_ID, ack14, sizeof(ack14));
    return true;
  }
  }
  if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);
  (void)hlscl.unLockEprom(oldId);
  (void)hlscl.writeWord(oldId, REG_CURRENT_LIMIT, reqLimit);
  uint8_t targetId = oldId;
  if (newId != oldId) {
    (void)hlscl.writeByte(oldId, REG_ID, newId);   // REG_ID = 0x05
    targetId = newId;
  }
  (void)hlscl.LockEprom(targetId);
  
  // Read back limit for ACK
  uint16_t curLimitRead = 0;
  int rd = hlscl.readWord(targetId, REG_CURRENT_LIMIT);
  if (rd >= 0) curLimitRead = (uint16_t)rd;
  if (gBusMux) xSemaphoreGive(gBusMux);
  // Build 6-byte payload: oldId(LE16), newId(LE16), curLimit(LE16) and send
  uint8_t ack[6];
  ack[0] = oldId;                      // oldId lo
  ack[1] = 0x00;                       // oldId hi
  ack[2] = targetId;                   // newId lo
  ack[3] = 0x00;                       // newId hi
  ack[4] = (uint8_t)(curLimitRead & 0xFF);
  ack[5] = (uint8_t)((curLimitRead >> 8) & 0xFF);
  sendAckFrame(SET_ID, ack, sizeof(ack));
  return true;
}
static bool handleTrimCmd(const uint8_t* payload) {
  // Parse little-endian fields
  uint16_t rawCh  = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
  uint16_t rawDeg = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
  int      ch      = (int)rawCh;         // 0..6
  int16_t  degrees = (int16_t)rawDeg;    // signed degrees
  // Validate channel (0..6)
  if (ch < 0 || ch >= 7) {
    // Optionally send a NACK or silent-accept to preserve framing
    return true;
  }
  // Degrees -> counts (≈11.375 counts/deg), clamp to 0..4095
  int delta_counts = (int)((float)degrees * 11.375f);
  int new_ext = (int)sd[ch].extend_count + delta_counts;
  if (new_ext < 0)    new_ext = 0;
  if (new_ext > 4095) new_ext = 4095;
  sd[ch].extend_count = (uint16_t)new_ext;
  // Persist to NVS
  prefs.begin("hand", false);
  prefs.putInt(String("ext" + String(ch)).c_str(), sd[ch].extend_count);
  prefs.end();
  // ACK payload: ch (u16, LE), extend_count (u16, LE)
  uint8_t ack[4];
  ack[0] = (uint8_t)(ch & 0xFF);
  ack[1] = (uint8_t)((ch >> 8) & 0xFF);
  ack[2] = (uint8_t)(sd[ch].extend_count & 0xFF);
  ack[3] = (uint8_t)((sd[ch].extend_count >> 8) & 0xFF);
  sendAckFrame(TRIM, ack, sizeof(ack));   // 16 bytes on the wire
  return true;
}
static bool handleSetSpeedCmd(const uint8_t* payload)
{
  uint16_t rawId = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
  uint16_t rawSpd = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
  if (rawId >= 7) {
    //invalid index of servo actuator - So give error code (0–6 valid)
    uint8_t ack[4] = {0xFF, 0xFF, 0x00, 0x00};
    sendAckFrame(SET_SPE, ack, sizeof(ack));
    return true;
  }
  if (rawSpd > 32766) rawSpd = 32766;  // clamp
  g_speed[rawId] = rawSpd;
  // ACK back: servo id and speed
  uint8_t ack[4];
  ack[0] = (uint8_t)(rawId & 0xFF);
  ack[1] = (uint8_t)((rawId >> 8) & 0xFF);
  ack[2] = (uint8_t)(rawSpd & 0xFF);
  ack[3] = (uint8_t)((rawSpd >> 8) & 0xFF);
  sendAckFrame(SET_SPE, ack, sizeof(ack));
  return true;
}
static bool handleSetTorCmd(const uint8_t* payload)
{
  uint16_t rawId = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
  uint16_t rawTor = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
  // Validate ID range (0..6)
  if (rawId >= 7) {
    uint8_t ack[4] = {0xFF, 0xFF, 0x00, 0x00};  // invalid ID ack
    sendAckFrame(SET_TOR, ack, sizeof(ack));
    return true;
  }
  if (rawTor > 1023) rawTor = 1023;
  // Update the per-servo torque value
  g_torque[rawId] = rawTor;
  uint8_t ack[4];
  ack[0] = (uint8_t)(rawId & 0xFF);
  ack[1] = (uint8_t)((rawId >> 8) & 0xFF);
  ack[2] = (uint8_t)(rawTor & 0xFF);
  ack[3] = (uint8_t)((rawTor >> 8) & 0xFF);
  sendAckFrame(SET_TOR, ack, sizeof(ack));
  return true;
}

// ----- Returns true if a valid 16-byte frame was consumed and handled -----
static bool handleHostFrame(uint8_t op) {
  // Wait until full frame is buffered: filler + 14 payload
  while (Serial.available() < 15) { /* wait */ }
  uint8_t buf[15];
  for (int i = 0; i < 15; ++i) {
    int ch = Serial.read(); if (ch < 0) return false;
    buf[i] = (uint8_t)ch;
  }
  const uint8_t* payload = &buf[1]; // buf[0] = filler 0x00 (ignored)

  switch (op) {
    case CTRL_POS: {
      int16_t pos[7];
      for (int i = 0; i < 7; ++i) {
        uint16_t u16 = (uint16_t)payload[2*i] | ((uint16_t)payload[2*i+1] << 8);
        uint8_t  ch  = i;
        pos[i] = mapU16ToRaw(ch, u16);
      }

      // Torque Servo limit - If motor crosses the TEMP_CUTOFF_C, then keep HOT_TORQUE_LIMIT as the torque
      uint16_t torque_eff[7];
      for (int i = 0; i < 7; ++i) {
        uint16_t base = g_torque[i]; // 0..1023
        if (isHot((uint8_t)i)) {
          base = u16_min(base, HOT_TORQUE_LIMIT);
        }
        torque_eff[i] = base;
      }

      if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);
      if (g_currentMode != MODE_POS) {
        for (int i = 0; i < 7; ++i) {
          uint8_t id = SERVO_IDS[i];
          hlscl.ServoMode(id);
        }
        g_currentMode = MODE_POS;
      }
      hlscl.SyncWritePosEx((uint8_t*)SERVO_IDS, 7, pos, g_speed, g_accel, torque_eff);
      if (gBusMux) xSemaphoreGive(gBusMux);
      return true;
    }

    case CTRL_TOR: 
    {   
      int16_t torque_cmd[7]; 
      for (int i = 0; i < 7; ++i) 
      {
        uint16_t mag = (uint16_t)payload[2*i] | ((uint16_t)payload[2*i + 1] << 8); 
        if (mag > 1000) mag = 1000; 
        if (mag < HOLD_MAG) mag = HOLD_MAG;
        if (isHot((uint8_t)i)) {
          mag = u16_min(mag, HOT_TORQUE_LIMIT);
        }
        int grasp_sign = (sd[i].extend_count > sd[i].grasp_count) ? +1 : -1;
        torque_cmd[i] = (int16_t)(grasp_sign * (int)mag);
      } 
      for (int i = 0; i < 7; ++i) {
        g_lastTorqueCmd[i] = torque_cmd[i];
      }

      if (gBusMux) xSemaphoreTake(gBusMux, portMAX_DELAY);

      if (g_currentMode != MODE_TORQUE) 
      { 
        for (int i = 0; i < 7; ++i) 
        { 
          uint8_t id = SERVO_IDS[i]; 
          hlscl.EleMode(id); 
        } 
        g_currentMode = MODE_TORQUE;
      } 
      for (int i = 0; i < 7; ++i) 
      { 
        uint8_t id = SERVO_IDS[i]; 
        hlscl.WriteEle(id, torque_cmd[i]); 
      } 
      if (gBusMux) xSemaphoreGive(gBusMux); 
      return true; 
    }

    case SET_TOR: {
      return handleSetTorCmd(payload);
    }

    case SET_SPE: {
      return handleSetSpeedCmd(payload);
    }

    case HOMING: {
      HOMING_start();   // blocks
      saveExtendsToNVS();
      sendAckFrame(HOMING, nullptr, 0);
      return true;
    }

    case SET_ID: {
      return handleSetIdCmd(payload);
    }

    case TRIM: {
      return handleTrimCmd(payload);
    }

    case GET_POS: {
      sendPositions();
      return true;
    }

    case GET_VEL: {
      sendVelocities();
      return true;
    }

    case GET_TEMP: {
      sendTemps();
      return true;
    }

    case GET_CURR: {
      sendCurrents();
      return true;
    }

    default:
      //Unknown Control Code — consume frame to preserve alignment
      return true;
  }
}

void setup() {
  // USB debug
  Serial.begin(921600);
  delay(100);

  // Servo bus UART @ 1 Mbps
  Serial2.begin(1000000, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN);
  hlscl.pSerial = &Serial2;

  resetSdToBaseline();
  prefs.begin("hand", false);
  loadManualExtendsFromNVS();
  #if defined(LEFT_HAND)
    Serial.println("[BOOT] Hand Type: LEFT_HAND");
  #elif defined(RIGHT_HAND)
    Serial.println("[BOOT] Hand Type: RIGHT_HAND");
  #else
    Serial.println("[BOOT] Hand Type: UNKNOWN");
  #endif

  esp_reset_reason_t reason = esp_reset_reason();
  bool do_homing = false;
  if (reason == ESP_RST_POWERON) {
    do_homing = true;
  }
  else if (reason == ESP_RST_BROWNOUT) {
    do_homing = true;   // Power-dipped but not fully to 0V, optional but recommended for safety
  }
  if (do_homing) {
    Serial.println("[BOOT] Power-on detected → homing");
    HOMING_start();
    saveExtendsToNVS();
  } else {
    Serial.println("[BOOT] Non-power reset → skipping homing");
  }
  // ---- Presence Check on Every Boot -----
  Serial.println("\n[Init] Pinging servos...");
  for (uint8_t i = 0; i < 7; ++i) {
    uint8_t id = SERVO_IDS[i];
    int resp = hlscl.Ping(id);
    if (!hlscl.getLastError()) {
      Serial.print("  ID "); Serial.print(id); Serial.println(": OK");
    } else {
      Serial.print("  ID "); Serial.print(id); Serial.println(": NO REPLY");
    }
  }
  //Syncreadbegin to Start the syncread
  hlscl.syncReadBegin(sizeof(SERVO_IDS), REG_BLOCK_LEN, /*rx_fix*/ 8);

  for (int i = 0; i < 7; ++i){
  Serial.printf("Servo %d dir=%d\n", i, sd[i].servo_direction);}
  //Initialisation of Mutex and Task serial pinned to Core 1
  gBusMux =xSemaphoreCreateMutex();
  gMetricsMux = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(TaskSyncRead_Core1, "SyncRead", 4096, NULL, 1, NULL, 1); // run on Core1
}

void loop() {
  static uint32_t last_cmd_ms = 0; 

  // Gate all host input during homing
  if (HOMING_isBusy()) {
    while (Serial.available()) { Serial.read(); }
    vTaskDelay(pdMS_TO_TICKS(5));
    return;
  }

  // ------ Soft Limit Check -------
  checkAndEnforceSoftLimits();

  // Process exactly one complete 16-byte frame when available
  if (Serial.available() >= 16) {
    int op = Serial.read();
    if (op >= 0) {
      if (handleHostFrame((uint8_t)op)) {
        last_cmd_ms = millis();
        return;
      }
    }
  }
}