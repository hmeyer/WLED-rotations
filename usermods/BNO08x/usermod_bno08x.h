// force the compiler to show a warning to confirm that this file is included
#warning **** Included USERMOD_BNO08x ****

#pragma once

#include "wled.h"
#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#define _USE_MATH_DEFINES
#include <cmath>


struct Quaternion {
  float r = 0;
  float i = 0;
  float j = 0;
  float k = 0;
  bool non_zero() {
    return r != 0 || i != 0 || j != 0 || k != 0;
  }
  void fromRVWAcc(sh2_RotationVectorWAcc_t &rv) {
    r = rv.real;
    i = rv.i;
    j = rv.j;
    k = rv.k;
  }
  void conjugate() {
    i = -i;
    j = -j;
    k = -k;
  }
  Quaternion& operator*=(const Quaternion& rhs) {
    float nr = r * rhs.r - i * rhs.i - j * rhs.j - k * rhs.k;
    float ni = r * rhs.i + i * rhs.r + j * rhs.k - k * rhs.j;
    float nj = r * rhs.j - i * rhs.k + j * rhs.r + k * rhs.i;
    k = r * rhs.k + i * rhs.j - j * rhs.i + k * rhs.r;
    j = nj; i = ni; r = nr;
    return *this; 
  }
};

struct Euler {
  float roll;
  float pitch;
  float yaw;
  void fromQuaterion(const Quaternion &q) {
    float sqr = sq(q.r);
    float sqi = sq(q.i);
    float sqj = sq(q.j);
    float sqk = sq(q.k);

    roll = atan2(2.0 * (q.j * q.k + q.i * q.r), (-sqi - sqj + sqk + sqr));
    pitch = asin(-2.0 * (q.i * q.k - q.j * q.r) / (sqi + sqj + sqk + sqr));
    yaw = atan2(2.0 * (q.i * q.j + q.k * q.r), (sqi - sqj - sqk + sqr));
  }
};

inline uint16_t rad2uint16(float rad) {
  const float res_max = 65536; 
  float res = rad * res_max / (2 * M_PI);
  return std::fmod(res, res_max);
}

inline float rad2norm(float rad) {
  float res = rad / (2 * M_PI);
  res = std::fmod(res, 1.0);
  if (res < 0.0) {
    res += 1.0;
  }
  return res;
}

class UsermodBNO08x : public Usermod
{
private:
  
  // set the default pins based on the architecture, these get overridden by Usermod menu settings
  #ifdef ARDUINO_ARCH_ESP32 // ESP32 boards
    const int8_t DEFAULT_SCL = 22;
    const int8_t DEFAULT_SDA = 21;
  #else // ESP8266 boards
    const int8_t DEFAULT_SCL = 5;
    const int8_t DEFAULT_SDA = 4;
    //uint8_t RST_PIN = 16; // Uncoment for Heltec WiFi-Kit-8
  #endif
  int8_t ioPin[2] = {DEFAULT_SCL, DEFAULT_SDA};        // I2C pins: SCL, SDA...defaults to Arch hardware pins but overridden at setup()
  bool relative_to_initial = false;  // Rotations are relative to initial orientation.
  bool initDone = false;


  Adafruit_BNO08x bno08x{};
  sh2_SensorValue_t sensorValue;
  Quaternion initial_conjugate_q;
  Quaternion current_q;
  Euler euler;

  void setReports(void) {
    DEBUG_PRINTLN(F("Setting desired reports."));
    if (! bno08x.enableReport(SH2_ARVR_STABILIZED_RV)) {
      DEBUG_PRINTLN(F("Could not enable ARVR stabilized rotation vector."));
    }    
  }  


public:
  void setup()
  {
    bool HW_Pins_Used = (ioPin[0]==DEFAULT_SCL && ioPin[1]==DEFAULT_SDA); // note whether architecture-based hardware SCL/SDA pins used
    PinOwner po = PinOwner::UM_BNO08x; // defaults to being pinowner for SCL/SDA pins
    PinManagerPinType pins[2] = { { ioPin[0], true }, { ioPin[1], true } };  // allocate pins
    if (HW_Pins_Used) po = PinOwner::HW_I2C; // allow multiple allocations of HW I2C bus pins
    if (!pinManager.allocateMultiplePins(pins, 2, po)) { return; }
    

    Wire.begin(ioPin[1], ioPin[0]);

    if (!bno08x.begin_I2C())
    {
      DEBUG_PRINTLN(F("Could not find BNO08x sensor!"));
    }
    else
    {
      DEBUG_PRINTLN(F("Found BNO08x sensor! Success."));
      for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
        DEBUG_PRINT(F("Part "));
        DEBUG_PRINT(bno08x.prodIds.entry[n].swPartNumber);
        DEBUG_PRINT(F(": Version: "));
        DEBUG_PRINT(bno08x.prodIds.entry[n].swVersionMajor);
        DEBUG_PRINT(F("."));
        DEBUG_PRINT(bno08x.prodIds.entry[n].swVersionMinor);
        DEBUG_PRINT(F("."));
        DEBUG_PRINT(bno08x.prodIds.entry[n].swVersionPatch);
        DEBUG_PRINT(F(" Build "));
        DEBUG_PRINTLN(bno08x.prodIds.entry[n].swBuildNumber);
      }
    }
    setReports();
    initDone=true;
  }

  void loop()
  {
    if (bno08x.wasReset()) {
      DEBUG_PRINT(F("sensor was reset "));
      setReports();
    }
    if (!bno08x.getSensorEvent(&sensorValue)) {
      return;
    }
  switch (sensorValue.sensorId) {
    case SH2_ARVR_STABILIZED_RV:
      current_q.fromRVWAcc(sensorValue.un.arvrStabilizedRV);
      if (relative_to_initial) {
        if (!initial_conjugate_q.non_zero()) {
          initial_conjugate_q.fromRVWAcc(sensorValue.un.arvrStabilizedRV);
          initial_conjugate_q.conjugate();
        }
        current_q *= initial_conjugate_q;
      }
      euler.fromQuaterion(current_q);
      // DEBUG_PRINT(F("ARVR Stabilized Rotation Vector - yaw: "));
      // DEBUG_PRINT(euler.yaw);
      // DEBUG_PRINT(F(" pitch: "));
      // DEBUG_PRINT(euler.pitch);
      // DEBUG_PRINT(F(" roll: "));
      // DEBUG_PRINT(euler.roll);
      // DEBUG_PRINT(F(" accuracy: "));
      // DEBUG_PRINTLN(sensorValue.un.arvrStabilizedRV.accuracy);
      break;
  }
  }

  //   /*
  //    * API calls te enable data exchange between WLED modules
  //    */
  uint16_t getRoll16() {
    return rad2uint16(euler.roll);
  }
  uint16_t getPitch16() {
    return rad2uint16(euler.pitch);
  }
  uint16_t getYaw16() {
    return rad2uint16(euler.yaw);
  }
  /// These functions return values normalized within [0;1).
  float getRollNorm() {
    return rad2norm(euler.roll);
  }
  float getPitchNorm() {
    return rad2norm(euler.pitch);
  }
  float getYawNorm() {
    return rad2norm(euler.yaw);
  }

    

  // Save Usermod Config Settings
  void addToConfig(JsonObject& root)
  {
    JsonObject top = root.createNestedObject(F("BNO08x"));
    JsonArray io_pin = top.createNestedArray(F("pin"));
    for (byte i=0; i<2; i++) io_pin.add(ioPin[i]);
    top[F("help4Pins")] = F("SCL,SDA"); // help for Settings page
    DEBUG_PRINTLN(F("BNO08x config saved."));
  }

  // Read Usermod Config Settings
  bool readFromConfig(JsonObject& root)
  {
    // default settings values could be set here (or below using the 3-argument getJsonValue()) instead of in the class definition or constructor
    // setting them inside readFromConfig() is slightly more robust, handling the rare but plausible use case of single value being missing after boot (e.g. if the cfg.json was manually edited and a value was removed)


    int8_t newPin[2]; for (byte i=0; i<2; i++) newPin[i] = ioPin[i]; // prepare to note changed pins

    JsonObject top = root[F("BNO08x")];
    if (top.isNull()) {
      DEBUG_PRINT(F("BNO08x"));
      DEBUG_PRINTLN(F(": No config found. (Using defaults.)"));
      return false;
    }
    bool configComplete = !top.isNull();

    // A 3-argument getJsonValue() assigns the 3rd argument as a default value if the Json value is missing
    for (byte i=0; i<2; i++) configComplete &= getJsonValue(top[F("pin")][i], newPin[i], ioPin[i]);

    DEBUG_PRINT(FPSTR(F("BNO08x")));
    if (!initDone) {
      // first run: reading from cfg.json
      for (byte i=0; i<2; i++) ioPin[i] = newPin[i];
      DEBUG_PRINTLN(F(" config loaded."));
    } else {
      DEBUG_PRINTLN(F(" config (re)loaded."));
      // changing parameters from settings page
      bool pinsChanged = false;
      for (byte i=0; i<2; i++) if (ioPin[i] != newPin[i]) { pinsChanged = true; break; } // check if any pins changed
      if (pinsChanged) { //if pins changed, deallocate old pins and allocate new ones
        PinOwner po = PinOwner::UM_BNO08x;
        if (ioPin[0]==DEFAULT_SCL && ioPin[1]==DEFAULT_SDA) po = PinOwner::HW_I2C;  // allow multiple allocations of HW I2C bus pins
        pinManager.deallocateMultiplePins((const uint8_t *)ioPin, 2, po);  // deallocate pins
        for (byte i=0; i<2; i++) ioPin[i] = newPin[i];
        setup();
      }
      // use "return !top["newestParameter"].isNull();" when updating Usermod with new features
      return !top[F("pin")].isNull();
    }

    return configComplete;
  }

  uint16_t getId() {
    return USERMOD_ID_BNO08x;
  }
};


