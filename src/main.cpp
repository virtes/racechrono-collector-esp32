#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Preferences.h>
#include <Wire.h>

namespace {

constexpr char kDeviceName[] = "RaceChrono Brake BLE";
constexpr char kRaceChronoServiceUuid[] = "00001ff8-0000-1000-8000-00805f9b34fb";
constexpr char kCanMainCharacteristicUuid[] = "00000001-0000-1000-8000-00805f9b34fb";
constexpr char kCanFilterCharacteristicUuid[] = "00000002-0000-1000-8000-00805f9b34fb";

constexpr uint32_t kBrakePressurePid = 0x00000101;
constexpr uint32_t kThrottlePositionPid = 0x00000102;
constexpr uint32_t kBatteryVoltagePid = 0x00000103;
constexpr uint16_t kDefaultNotifyIntervalMs = 20;
constexpr uint16_t kBleTxLogIntervalMs = 5000;
constexpr uint16_t kMaxBrakePressureBar = 250;
constexpr uint16_t kStatusLogIntervalMs = 10000;
constexpr uint8_t kLedPin = 5;
constexpr uint8_t kLedPwmChannel = 0;
constexpr uint16_t kLedPwmFrequencyHz = 1000;
constexpr uint8_t kLedPwmResolutionBits = 8;
constexpr uint16_t kLedPwmMaxDuty = (1U << kLedPwmResolutionBits) - 1;
constexpr uint16_t kLedThrottleMaxDuty = 180;
constexpr float kLedThrottleGamma = 1.7F;
constexpr float kLedBrakePressureThresholdBar = 15.0F;
constexpr float kLedBrakePressureFullBar = 100.0F;
constexpr uint16_t kLedBrakeMinDuty = 8;
constexpr uint16_t kLedBrakeMaxDuty = kLedThrottleMaxDuty;
constexpr uint16_t kLedFastBlinkIntervalMs = 100;
constexpr uint16_t kLedLongBlinkOnMs = 700;
constexpr uint16_t kLedLongBlinkOffMs = 350;
constexpr uint16_t kLedConnectedBlinkOnMs = 100;
constexpr uint16_t kLedConnectedBlinkIntervalMs = 5000;
constexpr uint16_t kLedDisconnectedBlinkOnMs = 700;
constexpr uint16_t kLedDisconnectedBlinkIntervalMs = 2000;
constexpr float kLedThrottleThresholdPercent = 1.0F;
constexpr uint8_t kAdcSdaPin = 19;
constexpr uint8_t kAdcSclPin = 22;
constexpr uint32_t kAdcI2cClockHz = 100000;
constexpr uint16_t kAdcReadIntervalMs = 5000;
constexpr uint16_t kAdcMissingLogIntervalMs = 5000;
constexpr float kAds1115VoltsPerBit = 6.144F / 32768.0F;
constexpr uint8_t kThrottleAdcChannel = 0;
constexpr uint8_t kBrakePressureAdcChannel = 1;
constexpr uint8_t kBatteryAdcChannel = 2;
constexpr uint16_t kThrottleAdcReadIntervalMs = 20;
constexpr uint16_t kBrakePressureAdcReadIntervalMs = 20;
constexpr uint16_t kBatteryAdcReadIntervalMs = 1000;
constexpr uint16_t kThrottleZeroCalibrationMs = 1000;
constexpr uint16_t kThrottleOpenCalibrationPauseMs = 2000;
constexpr uint16_t kThrottleOpenCalibrationMs = 2000;
constexpr uint16_t kThrottleCalibrationSampleIntervalMs = 20;
constexpr float kDefaultThrottleZeroVolts = 0.95F;
constexpr float kDefaultThrottleFullVolts = 2.4F;
constexpr float kMaxThrottleZeroCalibrationVolts = 1.2F;
constexpr float kMinThrottleFullCalibrationVolts = 1.5F;
constexpr float kMaxThrottleCalibrationDeltaVolts = 0.1F;
constexpr float kThrottleAdcFilterAlpha = 0.3F;
constexpr float kThrottlePercentDeadband = 0.2F;
constexpr float kBrakePressureZeroVolts = 0.5F;
constexpr float kBrakePressureFullVolts = 4.5F;
constexpr float kBatteryDividerMultiplier = 2.013564F;
constexpr char kThrottleCalibrationPrefsNamespace[] = "throttle";
constexpr char kThrottleZeroPrefsKey[] = "zero_v";
constexpr char kThrottleFullPrefsKey[] = "full_v";

BLEServer *bleServer = nullptr;
BLECharacteristic *canMainCharacteristic = nullptr;
Preferences throttleCalibrationPrefs;

bool bleClientConnected = false;
bool bleWasConnected = false;
bool allowAllPids = true;
bool brakePressurePidAllowed = true;
bool throttlePositionPidAllowed = true;
bool batteryVoltagePidAllowed = true;
uint16_t notifyIntervalMs = kDefaultNotifyIntervalMs;
uint32_t lastCanNotifyMs = 0;
uint32_t lastBleTxLogMs = 0;
uint32_t lastStatusLogMs = 0;
uint32_t lastLedToggleMs = 0;
uint32_t lastLedIdleBlinkMs = 0;
uint32_t lastAdcReadMs = 0;
uint32_t lastAdcMissingLogMs = 0;
uint32_t lastThrottleAdcReadMs = 0;
uint32_t lastBrakePressureAdcReadMs = 0;
uint32_t lastBatteryAdcReadMs = 0;
uint8_t adcI2cAddress = 0;
int16_t throttleAdcRaw = 0;
float throttleAdcVolts = 0.0F;
float throttleFilteredVolts = 0.0F;
float throttlePercent = 0.0F;
float throttleZeroVolts = kDefaultThrottleZeroVolts;
float throttleFullVolts = kDefaultThrottleFullVolts;
bool throttleAdcValid = false;
bool throttleFilterInitialized = false;
bool throttleCalibrationPrefsReady = false;
int16_t brakePressureAdcRaw = 0;
float brakePressureAdcVolts = 0.0F;
float brakePressureBar = 0.0F;
bool brakePressureAdcValid = false;
int16_t batteryAdcRaw = 0;
float batteryAdcVolts = 0.0F;
float batteryVolts = 0.0F;
bool batteryAdcValid = false;
bool ledOn = false;
bool ledIdleBlinkOn = false;
bool ledIdleConnected = false;
bool ledSensorActive = false;

uint16_t readUint16Be(const uint8_t *data) {
  return (static_cast<uint16_t>(data[0]) << 8) | data[1];
}

uint32_t readUint32Be(const uint8_t *data) {
  return (static_cast<uint32_t>(data[0]) << 24) |
         (static_cast<uint32_t>(data[1]) << 16) |
         (static_cast<uint32_t>(data[2]) << 8) |
         data[3];
}

void writeUint16Be(uint8_t *data, uint16_t value) {
  data[0] = static_cast<uint8_t>(value >> 8);
  data[1] = static_cast<uint8_t>(value & 0xFF);
}

void writeUint32Le(uint8_t *data, uint32_t value) {
  data[0] = static_cast<uint8_t>(value & 0xFF);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  data[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  data[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
}

bool isI2cDevicePresent(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

bool writeAds1115Register(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(adcI2cAddress);
  Wire.write(reg);
  Wire.write(static_cast<uint8_t>(value >> 8));
  Wire.write(static_cast<uint8_t>(value & 0xFF));
  return Wire.endTransmission() == 0;
}

bool readAds1115Register(uint8_t reg, int16_t &value) {
  Wire.beginTransmission(adcI2cAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(static_cast<int>(adcI2cAddress), 2) != 2) {
    return false;
  }

  const uint16_t raw = (static_cast<uint16_t>(Wire.read()) << 8) | Wire.read();
  value = static_cast<int16_t>(raw);
  return true;
}

bool readAds1115SingleEnded(uint8_t channel, int16_t &rawValue, float &volts) {
  if (channel > 3 || adcI2cAddress == 0) {
    return false;
  }

  constexpr uint8_t kConversionRegister = 0x00;
  constexpr uint8_t kConfigRegister = 0x01;
  const uint16_t mux = static_cast<uint16_t>(0x04 + channel) << 12;
  const uint16_t config =
      0x8000 | mux | 0x0000 | 0x0100 | 0x00E0 | 0x0003;

  if (!writeAds1115Register(kConfigRegister, config)) {
    return false;
  }

  delay(2);
  if (!readAds1115Register(kConversionRegister, rawValue)) {
    return false;
  }

  volts = static_cast<float>(rawValue) * kAds1115VoltsPerBit;
  return true;
}

bool isThrottleZeroCalibrationValid(float volts) {
  return volts >= 0.0F && volts < kMaxThrottleZeroCalibrationVolts;
}

bool isThrottleFullCalibrationValid(float volts) {
  return volts > kMinThrottleFullCalibrationVolts;
}

bool isThrottleCalibrationStable(float minVolts, float maxVolts) {
  return maxVolts - minVolts <= kMaxThrottleCalibrationDeltaVolts;
}

void loadThrottleCalibration() {
  throttleCalibrationPrefsReady =
      throttleCalibrationPrefs.begin(kThrottleCalibrationPrefsNamespace, false);

  if (!throttleCalibrationPrefsReady) {
    Serial.printf(
        "Throttle calibration NVS unavailable, using defaults zero=%.3fV full=%.3fV\n",
        throttleZeroVolts,
        throttleFullVolts);
    return;
  }

  const float storedZeroVolts =
      throttleCalibrationPrefs.getFloat(kThrottleZeroPrefsKey,
                                        kDefaultThrottleZeroVolts);
  const float storedFullVolts =
      throttleCalibrationPrefs.getFloat(kThrottleFullPrefsKey,
                                        kDefaultThrottleFullVolts);

  throttleZeroVolts = isThrottleZeroCalibrationValid(storedZeroVolts)
                          ? storedZeroVolts
                          : kDefaultThrottleZeroVolts;
  throttleFullVolts = isThrottleFullCalibrationValid(storedFullVolts)
                          ? storedFullVolts
                          : kDefaultThrottleFullVolts;

  Serial.printf("Throttle calibration loaded: zero=%.3fV full=%.3fV\n",
                throttleZeroVolts,
                throttleFullVolts);
}

void saveThrottleCalibrationValue(const char *key, float volts) {
  if (!throttleCalibrationPrefsReady) {
    Serial.printf("Throttle calibration NVS unavailable, not saving %.3fV\n",
                  volts);
    return;
  }

  throttleCalibrationPrefs.putFloat(key, volts);
}

float adcVoltsToThrottlePercent(float volts) {
  const float throttleSpanVolts = throttleFullVolts - throttleZeroVolts;
  if (throttleSpanVolts <= 0.01F) {
    return volts > throttleZeroVolts ? 100.0F : 0.0F;
  }

  return constrain((volts - throttleZeroVolts) * 100.0F /
                       throttleSpanVolts,
                   0.0F,
                   100.0F);
}

float adcVoltsToBrakePressureBar(float volts) {
  constexpr float kBrakePressureSpanVolts =
      kBrakePressureFullVolts - kBrakePressureZeroVolts;

  return constrain((volts - kBrakePressureZeroVolts) *
                       static_cast<float>(kMaxBrakePressureBar) /
                       kBrakePressureSpanVolts,
                   0.0F,
                   static_cast<float>(kMaxBrakePressureBar));
}

float adcVoltsToBatteryVolts(float volts) {
  return max(0.0F, volts * kBatteryDividerMultiplier);
}

void invalidateThrottleAdc() {
  throttleAdcValid = false;
  throttleFilterInitialized = false;
  throttlePercent = 0.0F;
}

void invalidateBrakePressureAdc() {
  brakePressureAdcValid = false;
  brakePressureAdcRaw = 0;
  brakePressureAdcVolts = 0.0F;
  brakePressureBar = 0.0F;
}

void invalidateBatteryAdc() {
  batteryAdcValid = false;
  batteryAdcRaw = 0;
  batteryAdcVolts = 0.0F;
  batteryVolts = 0.0F;
}

void invalidateAdcMeasurements() {
  invalidateThrottleAdc();
  invalidateBrakePressureAdc();
  invalidateBatteryAdc();
}

float updateThrottleFilteredVolts(float volts, bool resetFilter) {
  if (resetFilter || !throttleFilterInitialized) {
    throttleFilteredVolts = volts;
    throttleFilterInitialized = true;
    return throttleFilteredVolts;
  }

  throttleFilteredVolts +=
      kThrottleAdcFilterAlpha * (volts - throttleFilteredVolts);
  return throttleFilteredVolts;
}

void updateThrottleAdcState(int16_t rawValue, float volts, bool resetFilter) {
  throttleAdcRaw = rawValue;
  throttleAdcVolts = volts;

  const float filteredVolts = updateThrottleFilteredVolts(volts, resetFilter);
  const float nextThrottlePercent = adcVoltsToThrottlePercent(filteredVolts);
  if (resetFilter || !throttleAdcValid ||
      fabsf(nextThrottlePercent - throttlePercent) >= kThrottlePercentDeadband) {
    throttlePercent = nextThrottlePercent;
  }

  throttleAdcValid = true;
}

void updateBrakePressureAdcState(int16_t rawValue, float volts) {
  brakePressureAdcRaw = rawValue;
  brakePressureAdcVolts = volts;
  brakePressureBar = adcVoltsToBrakePressureBar(volts);
  brakePressureAdcValid = true;
}

void updateBatteryAdcState(int16_t rawValue, float volts) {
  batteryAdcRaw = rawValue;
  batteryAdcVolts = volts;
  batteryVolts = adcVoltsToBatteryVolts(volts);
  batteryAdcValid = true;
}

void updateThrottleFromAdc(uint32_t now) {
  if (adcI2cAddress == 0 ||
      now - lastThrottleAdcReadMs < kThrottleAdcReadIntervalMs) {
    return;
  }

  lastThrottleAdcReadMs = now;

  int16_t rawValue = 0;
  float volts = 0.0F;
  if (!readAds1115SingleEnded(kThrottleAdcChannel, rawValue, volts)) {
    Serial.printf("ADS1115 throttle A%u read failed at 0x%02X, will retry scan\n",
                  kThrottleAdcChannel,
                  adcI2cAddress);
    adcI2cAddress = 0;
    invalidateAdcMeasurements();
    return;
  }

  updateThrottleAdcState(rawValue, volts, false);
}

void updateBrakePressureFromAdc(uint32_t now) {
  if (adcI2cAddress == 0 ||
      now - lastBrakePressureAdcReadMs < kBrakePressureAdcReadIntervalMs) {
    return;
  }

  lastBrakePressureAdcReadMs = now;

  int16_t rawValue = 0;
  float volts = 0.0F;
  if (!readAds1115SingleEnded(kBrakePressureAdcChannel, rawValue, volts)) {
    Serial.printf("ADS1115 brake pressure A%u read failed at 0x%02X, will retry scan\n",
                  kBrakePressureAdcChannel,
                  adcI2cAddress);
    adcI2cAddress = 0;
    invalidateAdcMeasurements();
    return;
  }

  updateBrakePressureAdcState(rawValue, volts);
}

void updateBatteryFromAdc(uint32_t now) {
  if (adcI2cAddress == 0 ||
      now - lastBatteryAdcReadMs < kBatteryAdcReadIntervalMs) {
    return;
  }

  lastBatteryAdcReadMs = now;

  int16_t rawValue = 0;
  float volts = 0.0F;
  if (!readAds1115SingleEnded(kBatteryAdcChannel, rawValue, volts)) {
    Serial.printf("ADS1115 battery A%u read failed at 0x%02X, will retry scan\n",
                  kBatteryAdcChannel,
                  adcI2cAddress);
    adcI2cAddress = 0;
    invalidateAdcMeasurements();
    return;
  }

  updateBatteryAdcState(rawValue, volts);
}

void setLedDuty(uint16_t duty) {
  duty = min<uint16_t>(duty, kLedPwmMaxDuty);
  ledOn = duty > 0;
  ledcWrite(kLedPwmChannel, duty);
}

void setLed(bool on) {
  setLedDuty(on ? kLedPwmMaxDuty : 0);
}

uint16_t throttlePercentToLedDuty(float percent) {
  percent = constrain(percent, 0.0F, 100.0F);
  const float normalized = percent / 100.0F;
  return static_cast<uint16_t>(
      lroundf(powf(normalized, kLedThrottleGamma) *
              static_cast<float>(kLedThrottleMaxDuty)));
}

uint16_t brakePressureBarToLedDuty(float pressureBar) {
  if (pressureBar <= kLedBrakePressureThresholdBar) {
    return 0;
  }

  pressureBar = constrain(pressureBar,
                          kLedBrakePressureThresholdBar,
                          kLedBrakePressureFullBar);
  const float normalized =
      (pressureBar - kLedBrakePressureThresholdBar) /
      (kLedBrakePressureFullBar - kLedBrakePressureThresholdBar);
  return static_cast<uint16_t>(
      lroundf(static_cast<float>(kLedBrakeMinDuty) +
              normalized * static_cast<float>(kLedBrakeMaxDuty -
                                              kLedBrakeMinDuty)));
}

void startLedFastBlink(uint32_t now) {
  lastLedToggleMs = now;
  setLed(true);
}

void updateLedFastBlink(uint32_t now) {
  if (now - lastLedToggleMs < kLedFastBlinkIntervalMs) {
    return;
  }

  lastLedToggleMs = now;
  setLed(!ledOn);
}

void showLongLedBlinks(uint8_t count) {
  for (uint8_t blink = 0; blink < count; ++blink) {
    setLed(true);
    delay(kLedLongBlinkOnMs);
    setLed(false);

    if (blink + 1 < count) {
      delay(kLedLongBlinkOffMs);
    }
  }
}

void startLedIdleBlink(uint32_t now) {
  ledIdleBlinkOn = false;
  ledIdleConnected = bleClientConnected;
  lastLedIdleBlinkMs = now;
  setLed(false);
}

void updateLedIdleBlink(uint32_t now) {
  uint16_t sensorLedDuty = 0;

  if (throttleAdcValid && throttlePercent > kLedThrottleThresholdPercent) {
    sensorLedDuty = max<uint16_t>(sensorLedDuty,
                                  throttlePercentToLedDuty(throttlePercent));
  }

  if (brakePressureAdcValid &&
      brakePressureBar > kLedBrakePressureThresholdBar) {
    sensorLedDuty = max<uint16_t>(sensorLedDuty,
                                  brakePressureBarToLedDuty(brakePressureBar));
  }

  if (sensorLedDuty > 0) {
    ledSensorActive = true;
    ledIdleBlinkOn = false;
    setLedDuty(sensorLedDuty);
    return;
  }

  if (ledSensorActive) {
    ledSensorActive = false;
    ledIdleBlinkOn = false;
    lastLedIdleBlinkMs = now;
    setLed(false);
    return;
  }

  const bool connected = bleClientConnected;
  const uint16_t blinkOnMs =
      connected ? kLedConnectedBlinkOnMs : kLedDisconnectedBlinkOnMs;
  const uint16_t blinkIntervalMs =
      connected ? kLedConnectedBlinkIntervalMs : kLedDisconnectedBlinkIntervalMs;

  if (ledIdleConnected != connected) {
    ledIdleConnected = connected;
    ledIdleBlinkOn = false;
    lastLedIdleBlinkMs = now;
    setLed(false);
    return;
  }

  if (ledIdleBlinkOn) {
    if (now - lastLedIdleBlinkMs >= blinkOnMs) {
      ledIdleBlinkOn = false;
      setLed(false);
    }
    return;
  }

  if (now - lastLedIdleBlinkMs >= blinkIntervalMs) {
    lastLedIdleBlinkMs = now;
    ledIdleBlinkOn = true;
    setLed(true);
  }
}

bool readThrottleCalibrationAverage(uint16_t durationMs,
                                    const char *label,
                                    float &averageVolts,
                                    float &minVolts,
                                    float &maxVolts,
                                    int16_t &averageRaw,
                                    uint32_t &sampleCount) {
  if (adcI2cAddress == 0) {
    Serial.printf(
        "Throttle %s calibration skipped: ADS1115 not found, using zero=%.3fV full=%.3fV\n",
        label,
        throttleZeroVolts,
        throttleFullVolts);
    return false;
  }

  Serial.printf("Calibrating throttle %s from ADS1115 A%u for %u ms...\n",
                label,
                kThrottleAdcChannel,
                durationMs);

  const uint32_t startMs = millis();
  uint32_t lastSampleMs = 0;
  sampleCount = 0;
  float voltsSum = 0.0F;
  minVolts = 0.0F;
  maxVolts = 0.0F;
  int32_t rawSum = 0;
  bool readFailed = false;

  while (millis() - startMs < durationMs) {
    const uint32_t now = millis();
    updateLedFastBlink(now);

    if (sampleCount > 0 &&
        now - lastSampleMs < kThrottleCalibrationSampleIntervalMs) {
      delay(1);
      continue;
    }

    int16_t rawValue = 0;
    float volts = 0.0F;
    if (readAds1115SingleEnded(kThrottleAdcChannel, rawValue, volts)) {
      lastSampleMs = now;
      if (sampleCount == 0) {
        minVolts = volts;
        maxVolts = volts;
      } else {
        minVolts = min(minVolts, volts);
        maxVolts = max(maxVolts, volts);
      }
      ++sampleCount;
      rawSum += rawValue;
      voltsSum += volts;
    } else {
      Serial.printf("Throttle %s calibration read failed at 0x%02X\n",
                    label,
                    adcI2cAddress);
      readFailed = true;
      break;
    }
  }

  if (readFailed || sampleCount == 0) {
    Serial.printf(
        "Throttle %s calibration failed, using zero=%.3fV full=%.3fV\n",
        label,
        throttleZeroVolts,
        throttleFullVolts);
    return false;
  }

  averageVolts = voltsSum / static_cast<float>(sampleCount);
  averageRaw = static_cast<int16_t>(lroundf(static_cast<float>(rawSum) /
                                            static_cast<float>(sampleCount)));
  return true;
}

void updateThrottleCalibrationAdcState(float volts, int16_t raw) {
  updateThrottleAdcState(raw, volts, true);
  lastThrottleAdcReadMs = millis();
}

bool calibrateThrottleZero() {
  float averageVolts = 0.0F;
  float minVolts = 0.0F;
  float maxVolts = 0.0F;
  int16_t averageRaw = 0;
  uint32_t sampleCount = 0;

  if (!readThrottleCalibrationAverage(kThrottleZeroCalibrationMs,
                                      "zero",
                                      averageVolts,
                                      minVolts,
                                      maxVolts,
                                      averageRaw,
                                      sampleCount)) {
    throttleAdcValid = false;
    return false;
  }

  updateThrottleCalibrationAdcState(averageVolts, averageRaw);

  if (!isThrottleCalibrationStable(minVolts, maxVolts)) {
    Serial.printf(
        "Throttle zero calibration rejected: avg=%.3fV min=%.3fV max=%.3fV delta=%.3fV samples=%lu > %.3fV, using saved %.3fV\n",
        averageVolts,
        minVolts,
        maxVolts,
        maxVolts - minVolts,
        static_cast<unsigned long>(sampleCount),
        kMaxThrottleCalibrationDeltaVolts,
        throttleZeroVolts);
    return false;
  }

  if (!isThrottleZeroCalibrationValid(averageVolts)) {
    Serial.printf(
        "Throttle zero calibration rejected: %.3fV raw=%d samples=%lu >= %.3fV, using saved %.3fV\n",
        averageVolts,
        averageRaw,
        static_cast<unsigned long>(sampleCount),
        kMaxThrottleZeroCalibrationVolts,
        throttleZeroVolts);
    return false;
  }

  throttleZeroVolts = averageVolts;
  throttlePercent = 0.0F;
  saveThrottleCalibrationValue(kThrottleZeroPrefsKey, throttleZeroVolts);

  Serial.printf("Throttle zero calibrated: %.3fV raw=%d samples=%lu, full=%.3fV\n",
                throttleZeroVolts,
                throttleAdcRaw,
                static_cast<unsigned long>(sampleCount),
                throttleFullVolts);
  return true;
}

bool calibrateThrottleFull() {
  float averageVolts = 0.0F;
  float minVolts = 0.0F;
  float maxVolts = 0.0F;
  int16_t averageRaw = 0;
  uint32_t sampleCount = 0;

  if (!readThrottleCalibrationAverage(kThrottleOpenCalibrationMs,
                                      "full",
                                      averageVolts,
                                      minVolts,
                                      maxVolts,
                                      averageRaw,
                                      sampleCount)) {
    throttleAdcValid = false;
    return false;
  }

  updateThrottleCalibrationAdcState(averageVolts, averageRaw);

  if (!isThrottleCalibrationStable(minVolts, maxVolts)) {
    Serial.printf(
        "Throttle full calibration rejected: avg=%.3fV min=%.3fV max=%.3fV delta=%.3fV samples=%lu > %.3fV, using saved %.3fV\n",
        averageVolts,
        minVolts,
        maxVolts,
        maxVolts - minVolts,
        static_cast<unsigned long>(sampleCount),
        kMaxThrottleCalibrationDeltaVolts,
        throttleFullVolts);
    return false;
  }

  if (!isThrottleFullCalibrationValid(averageVolts)) {
    Serial.printf(
        "Throttle full calibration rejected: %.3fV raw=%d samples=%lu <= %.3fV, using saved %.3fV\n",
        averageVolts,
        averageRaw,
        static_cast<unsigned long>(sampleCount),
        kMinThrottleFullCalibrationVolts,
        throttleFullVolts);
    return false;
  }

  throttleFullVolts = averageVolts;
  throttlePercent = adcVoltsToThrottlePercent(averageVolts);
  saveThrottleCalibrationValue(kThrottleFullPrefsKey, throttleFullVolts);

  Serial.printf("Throttle full calibrated: %.3fV raw=%d samples=%lu, zero=%.3fV\n",
                throttleFullVolts,
                throttleAdcRaw,
                static_cast<unsigned long>(sampleCount),
                throttleZeroVolts);
  return true;
}

void calibrateThrottle() {
  startLedFastBlink(millis());
  const bool zeroCalibrated = calibrateThrottleZero();

  Serial.printf("Throttle calibration pause for %u ms\n",
                kThrottleOpenCalibrationPauseMs);
  setLed(true);
  delay(kThrottleOpenCalibrationPauseMs);

  startLedFastBlink(millis());
  const bool fullCalibrated = calibrateThrottleFull();

  setLed(false);
  delay(kLedLongBlinkOffMs);
  if (zeroCalibrated && fullCalibrated) {
    showLongLedBlinks(1);
  } else if (!zeroCalibrated) {
    showLongLedBlinks(2);
  } else {
    showLongLedBlinks(3);
  }

  startLedIdleBlink(millis());
}

uint8_t findAds1115Address() {
  for (uint8_t address = 0x48; address <= 0x4B; ++address) {
    if (isI2cDevicePresent(address)) {
      return address;
    }
  }

  return 0;
}

void startAdc() {
  Wire.begin(kAdcSdaPin, kAdcSclPin);
  Wire.setClock(kAdcI2cClockHz);

  adcI2cAddress = findAds1115Address();
  if (adcI2cAddress == 0) {
    Serial.printf("ADS1115 ADC not found on I2C SDA=%u SCL=%u\n", kAdcSdaPin, kAdcSclPin);
    return;
  }

  Serial.printf(
      "ADS1115 ADC found at 0x%02X on I2C SDA=%u SCL=%u\n",
      adcI2cAddress,
      kAdcSdaPin,
      kAdcSclPin);
}

void printAdcReadings(uint32_t now) {
  if (adcI2cAddress == 0) {
    if (now - lastAdcMissingLogMs >= kAdcMissingLogIntervalMs) {
      lastAdcMissingLogMs = now;
      adcI2cAddress = findAds1115Address();
      if (adcI2cAddress == 0) {
        Serial.printf("ADS1115 ADC still not found on I2C SDA=%u SCL=%u\n",
                      kAdcSdaPin,
                      kAdcSclPin);
      } else {
        Serial.printf("ADS1115 ADC found at 0x%02X\n", adcI2cAddress);
      }
    }

    return;
  }

  if (now - lastAdcReadMs < kAdcReadIntervalMs) {
    return;
  }

  lastAdcReadMs = now;

  int16_t rawValues[4] = {};
  float volts[4] = {};
  for (uint8_t channel = 0; channel < 4; ++channel) {
    if (!readAds1115SingleEnded(channel, rawValues[channel], volts[channel])) {
      Serial.printf("ADS1115 ADC read failed at 0x%02X, will retry scan\n", adcI2cAddress);
      adcI2cAddress = 0;
      invalidateAdcMeasurements();
      return;
    }
  }

  updateThrottleAdcState(rawValues[kThrottleAdcChannel],
                         volts[kThrottleAdcChannel],
                         false);
  updateBrakePressureAdcState(rawValues[kBrakePressureAdcChannel],
                              volts[kBrakePressureAdcChannel]);
  updateBatteryAdcState(rawValues[kBatteryAdcChannel],
                        volts[kBatteryAdcChannel]);

  Serial.printf(
      "ADC ADS1115 A0=%d %.3fV filtered=%.3fV throttle=%.1f%%, A1=%d %.3fV brake=%.1f bar, A2=%d %.3fV battery=%.3fV, A3=%d %.3fV\n",
      rawValues[0],
      volts[0],
      throttleFilteredVolts,
      throttlePercent,
      rawValues[1],
      volts[1],
      brakePressureBar,
      rawValues[2],
      volts[2],
      batteryVolts,
      rawValues[3],
      volts[3]);
}

uint16_t pressureBarToCentibar(float pressureBar) {
  pressureBar = constrain(pressureBar, 0.0F, static_cast<float>(kMaxBrakePressureBar));
  return static_cast<uint16_t>(lroundf(pressureBar * 100.0F));
}

uint16_t voltsToMillivolts(float volts) {
  volts = constrain(volts, 0.0F, 65.535F);
  return static_cast<uint16_t>(lroundf(volts * 1000.0F));
}

bool shouldSendPid(uint32_t pid) {
  return allowAllPids ||
         (pid == kBrakePressurePid && brakePressurePidAllowed) ||
         (pid == kThrottlePositionPid && throttlePositionPidAllowed) ||
         (pid == kBatteryVoltagePid && batteryVoltagePidAllowed);
}

bool publishCanValue(uint32_t pid, uint16_t rawValue) {
  if (!bleClientConnected || canMainCharacteristic == nullptr ||
      !shouldSendPid(pid)) {
    return false;
  }

  uint8_t packet[8] = {};
  writeUint32Le(packet, pid);
  writeUint16Be(packet + 4, rawValue);

  canMainCharacteristic->setValue(packet, sizeof(packet));
  canMainCharacteristic->notify();
  return true;
}

void publishTelemetry(float pressureBar,
                      float throttlePercent,
                      float batteryVolts) {
  const uint16_t pressureCentibar = pressureBarToCentibar(pressureBar);
  const uint16_t throttleCentipercent =
      static_cast<uint16_t>(lroundf(constrain(throttlePercent, 0.0F, 100.0F) * 100.0F));
  const uint16_t batteryMillivolts = voltsToMillivolts(batteryVolts);

  const bool brakeSent = publishCanValue(kBrakePressurePid, pressureCentibar);
  const bool throttleSent = publishCanValue(kThrottlePositionPid, throttleCentipercent);
  const bool batterySent = publishCanValue(kBatteryVoltagePid, batteryMillivolts);

  const uint32_t now = millis();
  if (now - lastBleTxLogMs >= kBleTxLogIntervalMs) {
    lastBleTxLogMs = now;
    Serial.printf(
        "BLE TX brake=%s PID=0x%08lX pressure=%.2f bar (%u), throttle=%s PID=0x%08lX throttle=%.2f %% (%u), battery=%s PID=0x%08lX voltage=%.3f V (%u mV)\n",
        brakeSent ? "sent" : "filtered",
        static_cast<unsigned long>(kBrakePressurePid),
        pressureBar,
        pressureCentibar,
        throttleSent ? "sent" : "filtered",
        static_cast<unsigned long>(kThrottlePositionPid),
        throttlePercent,
        throttleCentipercent,
        batterySent ? "sent" : "filtered",
        static_cast<unsigned long>(kBatteryVoltagePid),
        batteryVolts,
        batteryMillivolts);
  }
}

class RaceChronoServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    bleClientConnected = true;
  }

  void onDisconnect(BLEServer *) override {
    bleClientConnected = false;
  }
};

class RaceChronoCanFilterCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    const std::string value = characteristic->getValue();
    if (value.empty()) {
      return;
    }

    const auto *data = reinterpret_cast<const uint8_t *>(value.data());
    const size_t length = value.length();
    const uint8_t commandId = data[0];

    if (commandId == 0) {
      allowAllPids = false;
      brakePressurePidAllowed = false;
      throttlePositionPidAllowed = false;
      batteryVoltagePidAllowed = false;
      Serial.println("RaceChrono filter: deny all PIDs");
      return;
    }

    if (commandId == 1 && length >= 3) {
      allowAllPids = true;
      brakePressurePidAllowed = true;
      throttlePositionPidAllowed = true;
      batteryVoltagePidAllowed = true;
      notifyIntervalMs = max<uint16_t>(1, readUint16Be(data + 1));
      Serial.printf("RaceChrono filter: allow all PIDs, interval=%u ms\n", notifyIntervalMs);
      return;
    }

    if (commandId == 2 && length >= 7) {
      const uint16_t interval = max<uint16_t>(1, readUint16Be(data + 1));
      const uint32_t pid = readUint32Be(data + 3);
      const bool knownPid = pid == kBrakePressurePid ||
                            pid == kThrottlePositionPid ||
                            pid == kBatteryVoltagePid;
      if (pid == kBrakePressurePid) {
        brakePressurePidAllowed = true;
      } else if (pid == kThrottlePositionPid) {
        throttlePositionPidAllowed = true;
      } else if (pid == kBatteryVoltagePid) {
        batteryVoltagePidAllowed = true;
      }
      if (knownPid) {
        notifyIntervalMs = interval;
      }
      Serial.printf(
          "RaceChrono filter: allow PID=0x%08lX, interval=%u ms%s\n",
          static_cast<unsigned long>(pid),
          interval,
          knownPid ? "" : " (ignored)");
    }
  }
};

void startRaceChronoBle() {
  BLEDevice::init(kDeviceName);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new RaceChronoServerCallbacks());

  BLEService *raceChronoService = bleServer->createService(kRaceChronoServiceUuid);

  canMainCharacteristic = raceChronoService->createCharacteristic(
      kCanMainCharacteristicUuid,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  canMainCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *canFilterCharacteristic = raceChronoService->createCharacteristic(
      kCanFilterCharacteristicUuid,
      BLECharacteristic::PROPERTY_WRITE);
  canFilterCharacteristic->setCallbacks(new RaceChronoCanFilterCallbacks());

  uint8_t initialPacket[8] = {};
  writeUint32Le(initialPacket, kBrakePressurePid);
  writeUint16Be(initialPacket + 4, pressureBarToCentibar(brakePressureBar));
  canMainCharacteristic->setValue(initialPacket, sizeof(initialPacket));

  raceChronoService->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(kRaceChronoServiceUuid);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}

}  // namespace

void setup() {
  Serial.begin(115200);
  ledcSetup(kLedPwmChannel, kLedPwmFrequencyHz, kLedPwmResolutionBits);
  ledcAttachPin(kLedPin, kLedPwmChannel);
  setLed(false);

  Serial.println();
  Serial.println("racechrono-collector-esp32 started");
  Serial.printf("RaceChrono BLE CAN PID 0x%08lX: brake pressure, centibar\n",
                static_cast<unsigned long>(kBrakePressurePid));
  Serial.printf("RaceChrono BLE CAN PID 0x%08lX: throttle position, centipercent\n",
                static_cast<unsigned long>(kThrottlePositionPid));
  Serial.printf("RaceChrono BLE CAN PID 0x%08lX: battery voltage, millivolts\n",
                static_cast<unsigned long>(kBatteryVoltagePid));

  loadThrottleCalibration();
  startAdc();
  calibrateThrottle();
  startRaceChronoBle();
  Serial.printf("BLE advertising as \"%s\"\n", kDeviceName);
}

void loop() {
  const uint32_t now = millis();

  updateThrottleFromAdc(now);
  updateBrakePressureFromAdc(now);
  updateBatteryFromAdc(now);
  updateLedIdleBlink(now);

  if (bleClientConnected && now - lastCanNotifyMs >= notifyIntervalMs) {
    lastCanNotifyMs = now;
    publishTelemetry(brakePressureBar, throttlePercent, batteryVolts);
  }

  printAdcReadings(now);

  if (!bleClientConnected && bleWasConnected) {
    delay(500);
    bleServer->startAdvertising();
    Serial.println("BLE client disconnected, advertising restarted");
    bleWasConnected = false;
  }

  if (bleClientConnected && !bleWasConnected) {
    Serial.println("BLE client connected");
    bleWasConnected = true;
  }

  if (now - lastStatusLogMs >= kStatusLogIntervalMs) {
    lastStatusLogMs = now;

    Serial.printf(
        "uptime=%lu ms, free_heap=%u bytes, cpu=%u MHz, ble=%s, brake=%.1f bar (A1=%s %.3fV raw=%d), throttle=%.1f %% (A0=%s %.3fV raw=%d), battery=%.3f V (A2=%s %.3fV raw=%d)\n",
        static_cast<unsigned long>(now),
        ESP.getFreeHeap(),
        ESP.getCpuFreqMHz(),
        bleClientConnected ? "connected" : "advertising",
        brakePressureBar,
        brakePressureAdcValid ? "ok" : "missing",
        brakePressureAdcVolts,
        brakePressureAdcRaw,
        throttlePercent,
        throttleAdcValid ? "ok" : "missing",
        throttleAdcVolts,
        throttleAdcRaw,
        batteryVolts,
        batteryAdcValid ? "ok" : "missing",
        batteryAdcVolts,
        batteryAdcRaw);
  }
}
