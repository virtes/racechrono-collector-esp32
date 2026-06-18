#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>
#include <Wire.h>

namespace {

constexpr char kDeviceName[] = "RaceExporter";
constexpr char kRaceChronoServiceUuid[] = "00001ff8-0000-1000-8000-00805f9b34fb";
constexpr char kCanMainCharacteristicUuid[] = "00000001-0000-1000-8000-00805f9b34fb";
constexpr char kCanFilterCharacteristicUuid[] = "00000002-0000-1000-8000-00805f9b34fb";
constexpr char kGpsMainCharacteristicUuid[] = "00000003-0000-1000-8000-00805f9b34fb";
constexpr char kGpsTimeCharacteristicUuid[] = "00000004-0000-1000-8000-00805f9b34fb";

constexpr uint32_t kBrakePressurePid = 0x00000101;
constexpr uint32_t kThrottlePositionPid = 0x00000102;
constexpr uint32_t kBatteryVoltagePid = 0x00000103;
constexpr uint32_t kAfrPid = 0x00000105;
constexpr uint16_t kDefaultNotifyIntervalMs = 20;
constexpr uint16_t kMinNotifyIntervalMs = 1;
constexpr uint16_t kBleReconnectQuietMs = 750;
constexpr uint16_t kBleTxLogIntervalMs = 30000;
constexpr uint16_t kMaxBrakePressureBar = 250;
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
constexpr uint8_t kAdcSdaPin = 22;
constexpr uint8_t kAdcSclPin = 19;
constexpr uint32_t kAdcI2cClockHz = 400000;
constexpr int8_t kDisplayResetPin = -1;
constexpr uint8_t kDisplayWidth = 128;
constexpr uint8_t kDisplayHeight = 64;
constexpr uint8_t kDisplayPrimaryI2cAddress = 0x3C;
constexpr uint8_t kDisplaySecondaryI2cAddress = 0x3D;
constexpr uint16_t kDisplayUpdateIntervalMs = 500;
constexpr uint16_t kAdcReadIntervalMs = 5000;
constexpr uint16_t kAdcMissingLogIntervalMs = 5000;
constexpr float kAds1115VoltsPerBit = 6.144F / 32768.0F;
constexpr uint8_t kThrottleAdcChannel = 0;
constexpr uint8_t kBrakePressureAdcChannel = 1;
constexpr uint8_t kAfrAdcChannel = 2;
constexpr uint16_t kThrottleAdcReadIntervalMs = 20;
constexpr uint16_t kBrakePressureAdcReadIntervalMs = 20;
constexpr uint16_t kAfrAdcReadIntervalMs = 20;
constexpr uint8_t kBatteryAdcPin = 35;
constexpr uint8_t kBatteryAdcResolutionBits = 12;
constexpr uint16_t kBatteryAdcReadIntervalMs = 1000;
constexpr uint8_t kBatteryAdcSampleCount = 8;
constexpr uint8_t kGpsRxPin = 16;
constexpr uint8_t kGpsTxPin = 17;
constexpr uint32_t kGpsBaudRates[] = {115200, 9600, 38400, 57600, 4800};
constexpr uint8_t kGpsBaudRateCount =
    sizeof(kGpsBaudRates) / sizeof(kGpsBaudRates[0]);
constexpr uint32_t kGpsTargetBaudRate = 115200;
constexpr uint32_t kGpsInitialBaudRate = kGpsTargetBaudRate;
constexpr uint16_t kGpsFreshAgeMs = 2500;
constexpr uint16_t kGpsDebugLogIntervalMs = 30000;
constexpr uint16_t kGpsBaudProbeIntervalMs = 6000;
constexpr uint8_t kGpsDebugRawSampleSize = 96;
constexpr uint8_t kGpsDebugHexSampleByteCount = 32;
constexpr uint16_t kGpsSignalConfigPollTimeoutMs = 700;
constexpr uint16_t kThrottleZeroCalibrationMs = 1000;
constexpr uint16_t kThrottleOpenCalibrationPauseMs = 2000;
constexpr uint16_t kThrottleOpenCalibrationMs = 2000;
constexpr uint16_t kThrottleCalibrationSampleIntervalMs = 20;
constexpr float kDefaultThrottleZeroVolts = 0.95F;
constexpr float kDefaultThrottleFullVolts = 2.4F;
constexpr float kMaxThrottleZeroCalibrationVolts = 1.3F;
constexpr float kMinThrottleFullCalibrationVolts = 1.5F;
constexpr float kMaxThrottleCalibrationDeltaVolts = 0.1F;
constexpr float kThrottleAdcFilterAlpha = 0.3F;
constexpr float kThrottlePercentDeadband = 0.2F;
constexpr float kBrakePressureSupplyVolts = 5.10F;
constexpr float kBrakePressureZeroRatio = 0.12F;
constexpr float kBrakePressureFullRatio = 0.87F;
constexpr float kBrakePressureZeroVolts =
    kBrakePressureSupplyVolts * kBrakePressureZeroRatio;
constexpr float kBrakePressureFullVolts =
    kBrakePressureSupplyVolts * kBrakePressureFullRatio;
constexpr float kBrakePressureHoldDeadbandBar = 0.25F;
constexpr float kBatteryDividerMultiplier = 1.951091F;
constexpr float kBatteryAdcFilterAlpha = 0.2F;
constexpr float kAfrDividerMultiplier = 2.0F;
constexpr float kAfrCalibrationLowVolts = 2.47F;
constexpr float kAfrCalibrationLow = 15.3F;
constexpr float kAfrCalibrationHighVolts = 4.764F;
constexpr float kAfrCalibrationHigh = 20.0F;
constexpr float kAfrCalibrationAfrPerVolt =
    (kAfrCalibrationHigh - kAfrCalibrationLow) /
    (kAfrCalibrationHighVolts - kAfrCalibrationLowVolts);
constexpr float kAfrAtZeroVolts =
    kAfrCalibrationLow -
    kAfrCalibrationLowVolts * kAfrCalibrationAfrPerVolt;
constexpr char kThrottleCalibrationPrefsNamespace[] = "throttle";
constexpr char kThrottleZeroPrefsKey[] = "zero_v";
constexpr char kThrottleFullPrefsKey[] = "full_v";
constexpr char kGpsConfigPrefsNamespace[] = "gps";
constexpr char kGpsGlonassProfilePrefsKey[] = "glo_prof";

constexpr uint8_t kGpsUbxClassCfg = 0x06;
constexpr uint8_t kGpsUbxClassAck = 0x05;
constexpr uint8_t kGpsUbxIdAckNak = 0x00;
constexpr uint8_t kGpsUbxIdAckAck = 0x01;
constexpr uint8_t kGpsUbxIdCfgPrt = 0x00;
constexpr uint8_t kGpsUbxIdValset = 0x8A;
constexpr uint8_t kGpsUbxIdValget = 0x8B;
constexpr uint8_t kGpsUbxValsetLayerRam = 0x01;
constexpr uint8_t kGpsUbxValsetLayerBatteryBackedRam = 0x02;
constexpr uint8_t kGpsUbxValsetLayerFlash = 0x04;
constexpr uint8_t kGpsUbxValsetLayerPersistent =
    kGpsUbxValsetLayerRam |
    kGpsUbxValsetLayerBatteryBackedRam |
    kGpsUbxValsetLayerFlash;
constexpr uint8_t kGpsUbxValgetLayerRam = 0x00;
constexpr uint16_t kGpsUbxProtocolUbx = 0x0001;
constexpr uint16_t kGpsUbxProtocolNmea = 0x0002;
constexpr uint32_t kGpsSignalGpsEnaKey = 0x1031001F;
constexpr uint32_t kGpsSignalGpsL1CaEnaKey = 0x10310001;
constexpr uint32_t kGpsSignalSbasEnaKey = 0x10310020;
constexpr uint32_t kGpsSignalSbasL1CaEnaKey = 0x10310005;
constexpr uint32_t kGpsSignalGalEnaKey = 0x10310021;
constexpr uint32_t kGpsSignalGalE1EnaKey = 0x10310007;
constexpr uint32_t kGpsSignalBdsEnaKey = 0x10310022;
constexpr uint32_t kGpsSignalBdsB1EnaKey = 0x1031000D;
constexpr uint32_t kGpsSignalQzssEnaKey = 0x10310024;
constexpr uint32_t kGpsSignalQzssL1CaEnaKey = 0x10310012;
constexpr uint32_t kGpsSignalGloEnaKey = 0x10310025;
constexpr uint32_t kGpsSignalGloL1EnaKey = 0x10310018;

BLEServer *bleServer = nullptr;
BLECharacteristic *canMainCharacteristic = nullptr;
BLECharacteristic *gpsMainCharacteristic = nullptr;
BLECharacteristic *gpsTimeCharacteristic = nullptr;
BLE2902 *canMainNotifyDescriptor = nullptr;
BLE2902 *gpsMainNotifyDescriptor = nullptr;
BLE2902 *gpsTimeNotifyDescriptor = nullptr;
Preferences throttleCalibrationPrefs;
Preferences gpsConfigPrefs;
Adafruit_SSD1306 display(kDisplayWidth,
                         kDisplayHeight,
                         &Wire,
                         kDisplayResetPin);
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
TinyGPSCustom gpsGgaFixQuality(gps, "GPGGA", 6);

bool bleClientConnected = false;
bool bleWasConnected = false;
bool allowAllPids = true;
bool brakePressurePidAllowed = true;
bool throttlePositionPidAllowed = true;
bool batteryVoltagePidAllowed = true;
bool afrPidAllowed = true;
uint16_t notifyIntervalMs = kDefaultNotifyIntervalMs;
uint32_t bleConnectedAtMs = 0;
uint32_t lastCanNotifyMs = 0;
uint32_t lastBleTxLogMs = 0;
uint32_t lastLedToggleMs = 0;
uint32_t lastLedIdleBlinkMs = 0;
uint32_t lastAdcReadMs = 0;
uint32_t lastAdcMissingLogMs = 0;
uint32_t lastDisplayUpdateMs = 0;
uint32_t lastThrottleAdcReadMs = 0;
uint32_t lastBrakePressureAdcReadMs = 0;
uint32_t lastBatteryAdcReadMs = 0;
uint32_t lastAfrAdcReadMs = 0;
uint32_t lastGpsDebugLogMs = 0;
uint32_t lastGpsDebugCharsProcessed = 0;
uint32_t lastGpsDebugPassedChecksum = 0;
uint32_t lastGpsDebugFailedChecksum = 0;
uint32_t lastGpsDebugSentencesWithFix = 0;
uint32_t lastGpsBaudProbeMs = 0;
uint32_t lastGpsBaudProbePassedChecksum = 0;
uint32_t gpsCurrentBaudRate = kGpsInitialBaudRate;
uint16_t gpsDebugDollarCount = 0;
uint16_t gpsDebugStarCount = 0;
uint16_t gpsDebugLineFeedCount = 0;
uint16_t gpsDebugUbxHeaderCount = 0;
uint8_t gpsCurrentBaudRateIndex = 0;
uint8_t gpsDebugPreviousByte = 0;
uint8_t gpsDebugRawSampleLength = 0;
uint8_t gpsDebugHexSampleLength = 0;
char gpsDebugRawSample[kGpsDebugRawSampleSize + 1] = {};
char gpsDebugHexSample[kGpsDebugHexSampleByteCount * 3 + 1] = {};
uint32_t gpsLastDateHourValue = 0;
uint8_t gpsSyncBits = 0;
uint8_t adcI2cAddress = 0;
uint8_t displayI2cAddress = 0;
int16_t ads1115AdcRaw[4] = {};
float ads1115AdcVolts[4] = {};
bool ads1115AdcValid[4] = {};
int16_t throttleAdcRaw = 0;
float throttleAdcVolts = 0.0F;
float throttleFilteredVolts = 0.0F;
float throttlePercent = 0.0F;
float throttleZeroVolts = kDefaultThrottleZeroVolts;
float throttleFullVolts = kDefaultThrottleFullVolts;
bool throttleAdcValid = false;
bool throttleFilterInitialized = false;
bool throttleCalibrationPrefsReady = false;
bool gpsConfigPrefsReady = false;
bool gpsGlonassProfilePersisted = false;
int16_t brakePressureAdcRaw = 0;
float brakePressureAdcVolts = 0.0F;
float brakePressureBar = 0.0F;
bool brakePressureAdcValid = false;
bool brakePressureFilterInitialized = false;
int16_t afrAdcRaw = 0;
float afrAdcVolts = 0.0F;
float afrSignalVolts = 0.0F;
float afr = kAfrAtZeroVolts;
bool afrAdcValid = false;
int16_t batteryAdcRaw = 0;
float batteryAdcVolts = 0.0F;
float batteryMeasuredVolts = 0.0F;
float batteryVolts = 0.0F;
bool batteryAdcValid = false;
bool batteryFilterInitialized = false;
bool ledOn = false;
bool ledIdleBlinkOn = false;
bool ledIdleConnected = false;
bool ledSensorActive = false;
bool displayReady = false;
bool gpsMainDirty = true;
bool gpsTimeDirty = true;
bool gpsDateHourInitialized = false;
bool gpsLocationWasFresh = false;
bool gpsAltitudeWasFresh = false;
bool gpsSpeedWasFresh = false;
bool gpsCourseWasFresh = false;
bool gpsTimeWasFresh = false;
bool gpsDateWasFresh = false;

uint16_t readUint16Be(const uint8_t *data) {
  return (static_cast<uint16_t>(data[0]) << 8) | data[1];
}

uint32_t readUint32Be(const uint8_t *data) {
  return (static_cast<uint32_t>(data[0]) << 24) |
         (static_cast<uint32_t>(data[1]) << 16) |
         (static_cast<uint32_t>(data[2]) << 8) |
         data[3];
}

uint32_t readUint32Le(const uint8_t *data) {
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

void writeUint16Be(uint8_t *data, uint16_t value) {
  data[0] = static_cast<uint8_t>(value >> 8);
  data[1] = static_cast<uint8_t>(value & 0xFF);
}

void writeUint24Be(uint8_t *data, uint32_t value) {
  data[0] = static_cast<uint8_t>((value >> 16) & 0xFF);
  data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  data[2] = static_cast<uint8_t>(value & 0xFF);
}

void writeUint32Be(uint8_t *data, uint32_t value) {
  data[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
  data[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
  data[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
  data[3] = static_cast<uint8_t>(value & 0xFF);
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

void updateAds1115ChannelState(uint8_t channel, int16_t rawValue, float volts) {
  if (channel > 3) {
    return;
  }

  ads1115AdcRaw[channel] = rawValue;
  ads1115AdcVolts[channel] = volts;
  ads1115AdcValid[channel] = true;
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
  updateAds1115ChannelState(channel, rawValue, volts);
  return true;
}

bool readBatteryAdc(int16_t &rawValue, float &volts) {
  uint32_t rawSum = 0;
  uint32_t millivoltsSum = 0;

  (void)analogRead(kBatteryAdcPin);
  delayMicroseconds(50);

  for (uint8_t i = 0; i < kBatteryAdcSampleCount; ++i) {
    rawSum += analogRead(kBatteryAdcPin);
    millivoltsSum += analogReadMilliVolts(kBatteryAdcPin);
    delayMicroseconds(50);
  }

  rawValue = static_cast<int16_t>(
      lroundf(static_cast<float>(rawSum) /
              static_cast<float>(kBatteryAdcSampleCount)));
  volts = static_cast<float>(millivoltsSum) /
          static_cast<float>(kBatteryAdcSampleCount) / 1000.0F;
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

void loadGpsConfigState() {
  gpsConfigPrefsReady = gpsConfigPrefs.begin(kGpsConfigPrefsNamespace, false);
  if (!gpsConfigPrefsReady) {
    Serial.println("GNSS config NVS unavailable");
    return;
  }

  gpsGlonassProfilePersisted =
      gpsConfigPrefs.getBool(kGpsGlonassProfilePrefsKey, false);
  if (gpsGlonassProfilePersisted) {
    Serial.println("GNSS config NVS: GLONASS profile was previously ACKed as persistent");
    Serial.println("GNSS profile expected: GPS=on, SBAS=on, Galileo=on, BeiDou=off, QZSS=on, GLONASS=on");
  }
}

void markGpsGlonassProfilePersisted() {
  gpsGlonassProfilePersisted = true;
  if (!gpsConfigPrefsReady) {
    Serial.println("GNSS config NVS unavailable, persistent ACK flag not saved");
    return;
  }

  gpsConfigPrefs.putBool(kGpsGlonassProfilePrefsKey, true);
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

float adcVoltsToAfr(float volts) {
  const float signalVolts = max(0.0F, volts * kAfrDividerMultiplier);
  const float afr =
      kAfrCalibrationLow +
      (signalVolts - kAfrCalibrationLowVolts) * kAfrCalibrationAfrPerVolt;

  return constrain(afr,
                   kAfrAtZeroVolts,
                   kAfrCalibrationHigh);
}

void invalidateThrottleAdc() {
  throttleAdcValid = false;
  throttleFilterInitialized = false;
  throttlePercent = 0.0F;
}

void invalidateBrakePressureAdc() {
  brakePressureAdcValid = false;
  brakePressureFilterInitialized = false;
  brakePressureAdcRaw = 0;
  brakePressureAdcVolts = 0.0F;
  brakePressureBar = 0.0F;
}

void invalidateAfrAdc() {
  afrAdcValid = false;
  afrAdcRaw = 0;
  afrAdcVolts = 0.0F;
  afrSignalVolts = 0.0F;
  afr = kAfrAtZeroVolts;
}

void invalidateBatteryAdc() {
  batteryAdcValid = false;
  batteryFilterInitialized = false;
  batteryAdcRaw = 0;
  batteryAdcVolts = 0.0F;
  batteryMeasuredVolts = 0.0F;
  batteryVolts = 0.0F;
}

void invalidateAds1115Measurements() {
  for (uint8_t channel = 0; channel < 4; ++channel) {
    ads1115AdcRaw[channel] = 0;
    ads1115AdcVolts[channel] = 0.0F;
    ads1115AdcValid[channel] = false;
  }

  invalidateThrottleAdc();
  invalidateBrakePressureAdc();
  invalidateAfrAdc();
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

float updateBatteryFilteredVolts(float volts) {
  if (!batteryFilterInitialized) {
    batteryVolts = volts;
    batteryFilterInitialized = true;
    return batteryVolts;
  }

  batteryVolts += kBatteryAdcFilterAlpha * (volts - batteryVolts);
  return batteryVolts;
}

float updateBrakePressureFilteredBar(float pressureBar) {
  if (!brakePressureFilterInitialized) {
    brakePressureBar = pressureBar;
    brakePressureFilterInitialized = true;
    return brakePressureBar;
  }

  if (fabsf(pressureBar - brakePressureBar) >= kBrakePressureHoldDeadbandBar) {
    brakePressureBar = pressureBar;
  }

  return brakePressureBar;
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
  updateBrakePressureFilteredBar(adcVoltsToBrakePressureBar(volts));
  brakePressureAdcValid = true;
}

void updateAfrAdcState(int16_t rawValue, float volts) {
  afrAdcRaw = rawValue;
  afrAdcVolts = volts;
  afrSignalVolts = max(0.0F, volts * kAfrDividerMultiplier);
  afr = adcVoltsToAfr(volts);
  afrAdcValid = true;
}

void updateBatteryAdcState(int16_t rawValue, float volts) {
  batteryAdcRaw = rawValue;
  batteryAdcVolts = volts;
  batteryMeasuredVolts = adcVoltsToBatteryVolts(volts);
  updateBatteryFilteredVolts(batteryMeasuredVolts);
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
    invalidateAds1115Measurements();
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
    invalidateAds1115Measurements();
    return;
  }

  updateBrakePressureAdcState(rawValue, volts);
}

void updateAfrFromAdc(uint32_t now) {
  if (adcI2cAddress == 0 || now - lastAfrAdcReadMs < kAfrAdcReadIntervalMs) {
    return;
  }

  lastAfrAdcReadMs = now;

  int16_t rawValue = 0;
  float volts = 0.0F;
  if (!readAds1115SingleEnded(kAfrAdcChannel, rawValue, volts)) {
    Serial.printf("ADS1115 AFR A%u read failed at 0x%02X, will retry scan\n",
                  kAfrAdcChannel,
                  adcI2cAddress);
    adcI2cAddress = 0;
    invalidateAds1115Measurements();
    return;
  }

  updateAfrAdcState(rawValue, volts);
}

void updateBatteryFromAdc(uint32_t now) {
  if (batteryAdcValid &&
      now - lastBatteryAdcReadMs < kBatteryAdcReadIntervalMs) {
    return;
  }

  lastBatteryAdcReadMs = now;

  int16_t rawValue = 0;
  float volts = 0.0F;
  if (!readBatteryAdc(rawValue, volts)) {
    Serial.printf("ESP32 battery ADC GPIO%u read failed\n", kBatteryAdcPin);
    invalidateBatteryAdc();
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

uint8_t findSsd1306Address() {
  if (isI2cDevicePresent(kDisplayPrimaryI2cAddress)) {
    return kDisplayPrimaryI2cAddress;
  }
  if (isI2cDevicePresent(kDisplaySecondaryI2cAddress)) {
    return kDisplaySecondaryI2cAddress;
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

void startDisplay() {
  displayI2cAddress = findSsd1306Address();
  if (displayI2cAddress == 0) {
    Serial.printf("SSD1306 display not found on I2C SDA=%u SCL=%u\n",
                  kAdcSdaPin,
                  kAdcSclPin);
    return;
  }

  if (!display.begin(SSD1306_SWITCHCAPVCC, displayI2cAddress, true, false)) {
    Serial.printf("SSD1306 display init failed at 0x%02X\n",
                  displayI2cAddress);
    displayI2cAddress = 0;
    return;
  }

  displayReady = true;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("ADS1115");
  display.display();

  Serial.printf(
      "SSD1306 display found at 0x%02X, resolution=%ux%u, showing TH/BR/AF/BAT volts\n",
      displayI2cAddress,
      kDisplayWidth,
      kDisplayHeight);
}

void startBatteryAdc() {
  pinMode(kBatteryAdcPin, INPUT);
  analogReadResolution(kBatteryAdcResolutionBits);
  analogSetPinAttenuation(kBatteryAdcPin, ADC_11db);

  Serial.printf(
      "Battery voltage ADC on ESP32 GPIO%u, resolution=%u bits, attenuation=11dB\n",
      kBatteryAdcPin,
      kBatteryAdcResolutionBits);
}

void writeGpsUbxByte(uint8_t value, uint8_t &checksumA, uint8_t &checksumB) {
  gpsSerial.write(value);
  checksumA += value;
  checksumB += checksumA;
}

void updateGpsUbxChecksum(uint8_t value, uint8_t &checksumA, uint8_t &checksumB) {
  checksumA += value;
  checksumB += checksumA;
}

void writeGpsUbxPacket(uint8_t messageClass,
                       uint8_t messageId,
                       const uint8_t *payload,
                       uint16_t payloadLength) {
  uint8_t checksumA = 0;
  uint8_t checksumB = 0;

  gpsSerial.write(0xB5);
  gpsSerial.write(0x62);
  writeGpsUbxByte(messageClass, checksumA, checksumB);
  writeGpsUbxByte(messageId, checksumA, checksumB);
  writeGpsUbxByte(static_cast<uint8_t>(payloadLength & 0xFF),
                  checksumA,
                  checksumB);
  writeGpsUbxByte(static_cast<uint8_t>(payloadLength >> 8),
                  checksumA,
                  checksumB);

  for (uint16_t i = 0; i < payloadLength; ++i) {
    writeGpsUbxByte(payload[i], checksumA, checksumB);
  }

  gpsSerial.write(checksumA);
  gpsSerial.write(checksumB);
  gpsSerial.flush();
}

enum GpsUbxPollResult {
  kGpsUbxPollSuccess,
  kGpsUbxPollTimeout,
  kGpsUbxPollNak,
  kGpsUbxPollTooLong
};

struct GpsUbxPollStats {
  uint32_t bytesRead;
  uint16_t ubxHeaders;
  uint16_t validFrames;
  uint16_t unexpectedFrames;
  uint8_t lastClass;
  uint8_t lastId;
};

struct GpsSignalConfigItem {
  const char *constellationName;
  const char *signalName;
  uint32_t constellationKey;
  uint32_t signalKey;
  bool constellationKnown;
  bool constellationEnabled;
  bool signalKnown;
  bool signalEnabled;
};

GpsUbxPollResult readGpsUbxPacket(uint8_t expectedClass,
                                  uint8_t expectedId,
                                  uint8_t *payload,
                                  uint16_t payloadCapacity,
                                  uint16_t &payloadLength,
                                  uint16_t timeoutMs,
                                  GpsUbxPollStats *stats = nullptr) {
  uint8_t state = 0;
  uint8_t messageClass = 0;
  uint8_t messageId = 0;
  uint8_t checksumA = 0;
  uint8_t checksumB = 0;
  uint8_t expectedChecksumA = 0;
  uint8_t expectedChecksumB = 0;
  uint16_t messageLength = 0;
  uint16_t payloadIndex = 0;
  bool tooLong = false;
  const uint32_t startedMs = millis();

  while (millis() - startedMs < timeoutMs) {
    while (gpsSerial.available() > 0) {
      const uint8_t value = static_cast<uint8_t>(gpsSerial.read());
      if (stats != nullptr) {
        ++stats->bytesRead;
      }
      gps.encode(static_cast<char>(value));

      switch (state) {
        case 0:
          state = (value == 0xB5) ? 1 : 0;
          break;
        case 1:
          if (value == 0x62) {
            if (stats != nullptr) {
              ++stats->ubxHeaders;
            }
            state = 2;
          } else {
            state = (value == 0xB5) ? 1 : 0;
          }
          break;
        case 2:
          messageClass = value;
          checksumA = 0;
          checksumB = 0;
          updateGpsUbxChecksum(value, checksumA, checksumB);
          state = 3;
          break;
        case 3:
          messageId = value;
          updateGpsUbxChecksum(value, checksumA, checksumB);
          state = 4;
          break;
        case 4:
          messageLength = value;
          updateGpsUbxChecksum(value, checksumA, checksumB);
          state = 5;
          break;
        case 5:
          messageLength |= static_cast<uint16_t>(value) << 8;
          updateGpsUbxChecksum(value, checksumA, checksumB);
          payloadIndex = 0;
          tooLong = messageLength > payloadCapacity;
          state = (messageLength == 0) ? 7 : 6;
          break;
        case 6:
          updateGpsUbxChecksum(value, checksumA, checksumB);
          if (!tooLong) {
            payload[payloadIndex] = value;
          }
          ++payloadIndex;
          if (payloadIndex >= messageLength) {
            state = 7;
          }
          break;
        case 7:
          expectedChecksumA = value;
          state = 8;
          break;
        case 8:
          expectedChecksumB = value;
          if (expectedChecksumA == checksumA && expectedChecksumB == checksumB) {
            if (stats != nullptr) {
              ++stats->validFrames;
              stats->lastClass = messageClass;
              stats->lastId = messageId;
            }
            if (messageClass == kGpsUbxClassAck &&
                messageId == kGpsUbxIdAckNak &&
                !tooLong &&
                messageLength >= 2 &&
                payload[0] == expectedClass &&
                payload[1] == expectedId) {
              return kGpsUbxPollNak;
            }

            if (messageClass == expectedClass && messageId == expectedId) {
              if (tooLong) {
                return kGpsUbxPollTooLong;
              }

              payloadLength = messageLength;
              return kGpsUbxPollSuccess;
            }

            if (stats != nullptr) {
              ++stats->unexpectedFrames;
            }
          }
          state = 0;
          break;
      }
    }

    delay(1);
  }

  return kGpsUbxPollTimeout;
}

void writeGpsValgetKey(uint8_t *payload, uint16_t &offset, uint32_t key) {
  writeUint32Le(payload + offset, key);
  offset += 4;
}

void sendGpsSignalConfigValget() {
  uint8_t payload[4 + 12 * 4] = {};
  uint16_t offset = 0;
  payload[offset++] = 0x00;  // version
  payload[offset++] = kGpsUbxValgetLayerRam;
  payload[offset++] = 0x00;  // position, little-endian
  payload[offset++] = 0x00;

  writeGpsValgetKey(payload, offset, kGpsSignalGpsEnaKey);
  writeGpsValgetKey(payload, offset, kGpsSignalGpsL1CaEnaKey);
  writeGpsValgetKey(payload, offset, kGpsSignalSbasEnaKey);
  writeGpsValgetKey(payload, offset, kGpsSignalSbasL1CaEnaKey);
  writeGpsValgetKey(payload, offset, kGpsSignalGalEnaKey);
  writeGpsValgetKey(payload, offset, kGpsSignalGalE1EnaKey);
  writeGpsValgetKey(payload, offset, kGpsSignalBdsEnaKey);
  writeGpsValgetKey(payload, offset, kGpsSignalBdsB1EnaKey);
  writeGpsValgetKey(payload, offset, kGpsSignalQzssEnaKey);
  writeGpsValgetKey(payload, offset, kGpsSignalQzssL1CaEnaKey);
  writeGpsValgetKey(payload, offset, kGpsSignalGloEnaKey);
  writeGpsValgetKey(payload, offset, kGpsSignalGloL1EnaKey);

  writeGpsUbxPacket(kGpsUbxClassCfg, kGpsUbxIdValget, payload, offset);
}

void sendGpsSignalConfigEnableGlonass(uint8_t layerMask) {
  uint8_t payload[4 + 4 * 5] = {};
  uint16_t offset = 0;
  payload[offset++] = 0x00;  // version
  payload[offset++] = layerMask;
  payload[offset++] = 0x00;  // transaction
  payload[offset++] = 0x00;  // reserved

  writeGpsValgetKey(payload, offset, kGpsSignalBdsEnaKey);
  payload[offset++] = 0x00;
  writeGpsValgetKey(payload, offset, kGpsSignalBdsB1EnaKey);
  payload[offset++] = 0x00;
  writeGpsValgetKey(payload, offset, kGpsSignalGloEnaKey);
  payload[offset++] = 0x01;
  writeGpsValgetKey(payload, offset, kGpsSignalGloL1EnaKey);
  payload[offset++] = 0x01;

  writeGpsUbxPacket(kGpsUbxClassCfg, kGpsUbxIdValset, payload, offset);
}

void printGpsUbxPollStats(const GpsUbxPollStats &stats) {
  Serial.printf("bytes=%lu, ubx_headers=%u, valid_ubx=%u, unexpected_ubx=%u, last=0x%02X/0x%02X",
                static_cast<unsigned long>(stats.bytesRead),
                stats.ubxHeaders,
                stats.validFrames,
                stats.unexpectedFrames,
                stats.lastClass,
                stats.lastId);
}

bool waitGpsUbxAck(uint8_t targetClass,
                   uint8_t targetId,
                   uint16_t timeoutMs,
                   GpsUbxPollStats &stats) {
  uint8_t payload[8] = {};
  uint16_t payloadLength = 0;
  const GpsUbxPollResult result =
      readGpsUbxPacket(kGpsUbxClassAck,
                       kGpsUbxIdAckAck,
                       payload,
                       sizeof(payload),
                       payloadLength,
                       timeoutMs,
                       &stats);

  return result == kGpsUbxPollSuccess &&
         payloadLength >= 2 &&
         payload[0] == targetClass &&
         payload[1] == targetId;
}

const char *gpsSignalConfigText(bool known, bool enabled) {
  if (!known) {
    return "?";
  }

  return enabled ? "on" : "off";
}

void updateGpsSignalConfigItem(GpsSignalConfigItem &item,
                               uint32_t key,
                               bool enabled) {
  if (key == item.constellationKey) {
    item.constellationKnown = true;
    item.constellationEnabled = enabled;
  } else if (key == item.signalKey) {
    item.signalKnown = true;
    item.signalEnabled = enabled;
  }
}

bool readGpsSignalConfigItems(GpsSignalConfigItem *items,
                              uint8_t itemCount,
                              const char *context) {
  uint8_t responsePayload[96] = {};
  uint16_t responsePayloadLength = 0;
  GpsUbxPollStats stats = {};

  sendGpsSignalConfigValget();
  const GpsUbxPollResult result =
      readGpsUbxPacket(kGpsUbxClassCfg,
                       kGpsUbxIdValget,
                       responsePayload,
                       sizeof(responsePayload),
                       responsePayloadLength,
                       kGpsSignalConfigPollTimeoutMs,
                       &stats);

  if (result == kGpsUbxPollNak) {
    Serial.printf("GNSS signal config (CFG-SIGNAL RAM): unavailable (%s: receiver rejected UBX-CFG-VALGET; ",
                  context);
    printGpsUbxPollStats(stats);
    Serial.println(")");
    return false;
  }
  if (result == kGpsUbxPollTooLong) {
    Serial.printf("GNSS signal config (CFG-SIGNAL RAM): unavailable (%s: UBX-CFG-VALGET response too large; ",
                  context);
    printGpsUbxPollStats(stats);
    Serial.println(")");
    return false;
  }
  if (result != kGpsUbxPollSuccess || responsePayloadLength < 4) {
    Serial.printf("GNSS signal config (CFG-SIGNAL RAM): unavailable (%s: no UBX-CFG-VALGET response; ",
                  context);
    printGpsUbxPollStats(stats);
    Serial.println(")");
    return false;
  }

  uint16_t offset = 4;
  while (offset + 5 <= responsePayloadLength) {
    const uint32_t key = readUint32Le(responsePayload + offset);
    const bool enabled = responsePayload[offset + 4] != 0;
    offset += 5;

    for (uint8_t i = 0; i < itemCount; ++i) {
      updateGpsSignalConfigItem(items[i], key, enabled);
    }
  }

  return true;
}

void printGpsSignalConfigItems(const GpsSignalConfigItem *items,
                               uint8_t itemCount) {
  Serial.print("GNSS signal config (CFG-SIGNAL RAM): ");
  for (uint8_t i = 0; i < itemCount; ++i) {
    const GpsSignalConfigItem &item = items[i];
    Serial.printf("%s=%s(%s=%s)",
                  item.constellationName,
                  gpsSignalConfigText(item.constellationKnown,
                                      item.constellationEnabled),
                  item.signalName,
                  gpsSignalConfigText(item.signalKnown, item.signalEnabled));
    Serial.print((i + 1 < itemCount) ? ", " : "\n");
  }
}

void resetGpsSignalConfigItems(GpsSignalConfigItem *items) {
  items[0] = {"GPS", "L1CA", kGpsSignalGpsEnaKey, kGpsSignalGpsL1CaEnaKey};
  items[1] = {"SBAS", "L1CA", kGpsSignalSbasEnaKey, kGpsSignalSbasL1CaEnaKey};
  items[2] = {"Galileo", "E1", kGpsSignalGalEnaKey, kGpsSignalGalE1EnaKey};
  items[3] = {"BeiDou", "B1", kGpsSignalBdsEnaKey, kGpsSignalBdsB1EnaKey};
  items[4] = {"QZSS", "L1CA", kGpsSignalQzssEnaKey, kGpsSignalQzssL1CaEnaKey};
  items[5] = {"GLONASS", "L1", kGpsSignalGloEnaKey, kGpsSignalGloL1EnaKey};
}

bool isGpsSignalItemEnabled(const GpsSignalConfigItem &item) {
  return item.constellationKnown &&
         item.constellationEnabled &&
         item.signalKnown &&
         item.signalEnabled;
}

bool enableGpsGlonassWithLayers(uint8_t layerMask,
                                const char *layerName,
                                const char *reason) {
  GpsUbxPollStats stats = {};
  sendGpsSignalConfigEnableGlonass(layerMask);
  const bool acknowledged =
      waitGpsUbxAck(kGpsUbxClassCfg, kGpsUbxIdValset, 700, stats);
  Serial.printf("GNSS signal config: GLONASS profile ack=%s, layers=%s, BeiDou=off, GLONASS=on (%s; ",
                acknowledged ? "yes" : "no",
                layerName,
                reason);
  printGpsUbxPollStats(stats);
  Serial.println(")");
  return acknowledged;
}

bool enableGpsGlonassBestEffort(const char *reason) {
  if (enableGpsGlonassWithLayers(kGpsUbxValsetLayerPersistent,
                                 "RAM+BBR+Flash",
                                 reason)) {
    markGpsGlonassProfilePersisted();
    return true;
  }

  if (enableGpsGlonassWithLayers(kGpsUbxValsetLayerRam |
                                     kGpsUbxValsetLayerBatteryBackedRam,
                                 "RAM+BBR",
                                 "Flash rejected, fallback")) {
    return true;
  }

  return enableGpsGlonassWithLayers(kGpsUbxValsetLayerRam,
                                    "RAM",
                                    "persistent layers rejected, runtime fallback");
}

void logAndEnableGpsGlonassIfNeeded() {
  constexpr uint8_t kItemCount = 6;
  constexpr uint8_t kGlonassItemIndex = 5;
  GpsSignalConfigItem items[kItemCount] = {};
  resetGpsSignalConfigItems(items);

  if (!readGpsSignalConfigItems(items, kItemCount, "before GLONASS enable")) {
    if (gpsGlonassProfilePersisted) {
      Serial.println("GNSS signal config: status unavailable, persistent GLONASS profile already ACKed; skipping GPS flash write");
      Serial.println("GNSS profile expected: GPS=on, SBAS=on, Galileo=on, BeiDou=off, QZSS=on, GLONASS=on");
      return;
    }

    enableGpsGlonassBestEffort("status unavailable, forced by user request");
    return;
  }

  printGpsSignalConfigItems(items, kItemCount);
  const GpsSignalConfigItem &glonassItem = items[kGlonassItemIndex];
  if (isGpsSignalItemEnabled(glonassItem)) {
    Serial.println("GNSS signal config: GLONASS already enabled");
    return;
  }

  if (!glonassItem.constellationKnown || !glonassItem.signalKnown) {
    Serial.println("GNSS signal config: GLONASS status unknown, not changing receiver config");
    return;
  }

  const bool acknowledged = enableGpsGlonassBestEffort("GLONASS was disabled");
  if (!acknowledged) {
    return;
  }

  delay(50);
  resetGpsSignalConfigItems(items);
  if (readGpsSignalConfigItems(items, kItemCount, "after GLONASS enable")) {
    printGpsSignalConfigItems(items, kItemCount);
  }
}

void sendGpsUbxSetUartNmea(bool enableUbxOutput = false) {
  const uint16_t outputProtocols =
      enableUbxOutput ? (kGpsUbxProtocolUbx | kGpsUbxProtocolNmea)
                      : kGpsUbxProtocolNmea;
  uint8_t payload[] = {
      0x01, 0x00,              // UART1
      0x00, 0x00,              // txReady disabled
      0xD0, 0x08, 0x00, 0x00,  // 8N1
      0x00, 0x00, 0x00, 0x00,  // baud, filled below
      0x03, 0x00,              // input: UBX + NMEA
      0x00, 0x00,              // output, filled below
      0x00, 0x00,              // flags
      0x00, 0x00               // reserved
  };
  writeUint32Le(payload + 8, kGpsTargetBaudRate);
  payload[14] = static_cast<uint8_t>(outputProtocols & 0xFF);
  payload[15] = static_cast<uint8_t>(outputProtocols >> 8);

  writeGpsUbxPacket(0x06, 0x00, payload, sizeof(payload));
}

void beginGpsSerial(uint32_t baudRate) {
  gpsCurrentBaudRate = baudRate;
  gpsSerial.begin(gpsCurrentBaudRate, SERIAL_8N1, kGpsRxPin, kGpsTxPin);
}

void configureGpsAtCurrentBaud(const char *reason, bool logCommand = true) {
  sendGpsUbxSetUartNmea();
  if (logCommand) {
    Serial.printf("GPS config: sent UBX-CFG-PRT at %lu baud, target NMEA %lu baud, output=NMEA (%s)\n",
                  static_cast<unsigned long>(gpsCurrentBaudRate),
                  static_cast<unsigned long>(kGpsTargetBaudRate),
                  reason);
  }

  if (gpsCurrentBaudRate != kGpsTargetBaudRate) {
    delay(50);
    beginGpsSerial(kGpsTargetBaudRate);
  }
}

void configureGpsAtCommonBaudRates(const char *reason) {
  for (uint8_t i = 0; i < kGpsBaudRateCount; ++i) {
    beginGpsSerial(kGpsBaudRates[i]);
    delay(50);
    sendGpsUbxSetUartNmea();
  }

  beginGpsSerial(kGpsTargetBaudRate);
  Serial.printf("GPS config: sent UBX-CFG-PRT at %u common baud rates, target NMEA %lu baud, output=NMEA (%s)\n",
                kGpsBaudRateCount,
                static_cast<unsigned long>(kGpsTargetBaudRate),
                reason);
}

void enableGpsUbxOutputForDiagnostics() {
  sendGpsUbxSetUartNmea(true);
  Serial.printf("GPS config: sent UBX-CFG-PRT at %lu baud, target NMEA %lu baud, output=UBX+NMEA (diagnostics)\n",
                static_cast<unsigned long>(gpsCurrentBaudRate),
                static_cast<unsigned long>(kGpsTargetBaudRate));
  delay(50);
}

void restoreGpsNmeaOnlyOutput() {
  sendGpsUbxSetUartNmea();
  Serial.printf("GPS config: sent UBX-CFG-PRT at %lu baud, target NMEA %lu baud, output=NMEA (diagnostics done)\n",
                static_cast<unsigned long>(gpsCurrentBaudRate),
                static_cast<unsigned long>(kGpsTargetBaudRate));
}

void startGps() {
  gpsCurrentBaudRateIndex = 0;
  beginGpsSerial(kGpsTargetBaudRate);
  const uint32_t now = millis();
  lastGpsDebugLogMs = now;
  lastGpsBaudProbeMs = now;

  Serial.printf("GPS UART2 started: GPS TX -> ESP32 GPIO%u RX2, ESP32 GPIO%u TX2 -> GPS RX, baud=%lu\n",
                kGpsRxPin,
                kGpsTxPin,
                static_cast<unsigned long>(gpsCurrentBaudRate));
  configureGpsAtCommonBaudRates("startup");
  enableGpsUbxOutputForDiagnostics();
  logAndEnableGpsGlonassIfNeeded();
  restoreGpsNmeaOnlyOutput();
}

void formatDisplayAdsVoltageLine(char *line,
                                 size_t lineSize,
                                 const char *label,
                                 uint8_t channel) {
  if (channel <= 3 && ads1115AdcValid[channel]) {
    snprintf(line, lineSize, "%s %.3fV", label, ads1115AdcVolts[channel]);
    return;
  }

  snprintf(line, lineSize, "%s --.--V", label);
}

void formatDisplayBatteryLine(char *line, size_t lineSize) {
  if (batteryAdcValid) {
    snprintf(line, lineSize, "BAT %.2fV", batteryVolts);
    return;
  }

  snprintf(line, lineSize, "BAT --.--V");
}

void updateDisplay(uint32_t now) {
  if (!displayReady || now - lastDisplayUpdateMs < kDisplayUpdateIntervalMs) {
    return;
  }

  lastDisplayUpdateMs = now;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);

  char line[16] = {};
  formatDisplayAdsVoltageLine(line, sizeof(line), "TH", kThrottleAdcChannel);
  display.setCursor(0, 0);
  display.print(line);

  formatDisplayAdsVoltageLine(line,
                              sizeof(line),
                              "BR",
                              kBrakePressureAdcChannel);
  display.setCursor(0, 16);
  display.print(line);

  formatDisplayAdsVoltageLine(line, sizeof(line), "AF", kAfrAdcChannel);
  display.setCursor(0, 32);
  display.print(line);

  formatDisplayBatteryLine(line, sizeof(line));
  display.setCursor(0, 48);
  display.print(line);

  display.display();
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
      invalidateAds1115Measurements();
      return;
    }
  }

  updateThrottleAdcState(rawValues[kThrottleAdcChannel],
                         volts[kThrottleAdcChannel],
                         false);
  updateBrakePressureAdcState(rawValues[kBrakePressureAdcChannel],
                              volts[kBrakePressureAdcChannel]);
  updateAfrAdcState(rawValues[kAfrAdcChannel], volts[kAfrAdcChannel]);

  Serial.printf(
      "ADC throttle A0=%.3fV -> %.1f%%, brake A1=%.3fV -> %.1f bar, AFR A2=%.3fV -> %.2f, battery GPIO%u raw=%d adc=%.3fV measured=%.3fV filtered=%.3fV\n",
      volts[kThrottleAdcChannel],
      throttlePercent,
      volts[kBrakePressureAdcChannel],
      brakePressureBar,
      volts[kAfrAdcChannel],
      afr,
      kBatteryAdcPin,
      batteryAdcRaw,
      batteryAdcVolts,
      batteryMeasuredVolts,
      batteryVolts);
}

uint16_t pressureBarToCentibar(float pressureBar) {
  pressureBar = constrain(pressureBar, 0.0F, static_cast<float>(kMaxBrakePressureBar));
  return static_cast<uint16_t>(lroundf(pressureBar * 100.0F));
}

uint16_t voltsToMillivolts(float volts) {
  volts = constrain(volts, 0.0F, 65.535F);
  return static_cast<uint16_t>(lroundf(volts * 1000.0F));
}

uint16_t afrToCentiAfr(float afr) {
  afr = constrain(afr, kAfrAtZeroVolts, kAfrCalibrationHigh);
  return static_cast<uint16_t>(lroundf(afr * 100.0F));
}

bool isGpsLocationFresh() {
  return gps.location.isValid() && gps.location.age() <= kGpsFreshAgeMs;
}

bool isGpsAltitudeFresh() {
  return gps.altitude.isValid() && gps.altitude.age() <= kGpsFreshAgeMs;
}

bool isGpsSpeedFresh() {
  return gps.speed.isValid() && gps.speed.age() <= kGpsFreshAgeMs;
}

bool isGpsCourseFresh() {
  return gps.course.isValid() && gps.course.age() <= kGpsFreshAgeMs;
}

bool isGpsTimeFresh() {
  return gps.time.isValid() && gps.time.age() <= kGpsFreshAgeMs;
}

bool isGpsDateFresh() {
  return gps.date.isValid() && gps.date.age() <= kGpsFreshAgeMs;
}

void formatGpsUnsigned(char *buffer,
                       size_t bufferSize,
                       bool valid,
                       uint32_t value) {
  if (!valid) {
    snprintf(buffer, bufferSize, "--");
    return;
  }

  snprintf(buffer, bufferSize, "%lu", static_cast<unsigned long>(value));
}

void formatGpsFloat(char *buffer,
                    size_t bufferSize,
                    bool valid,
                    double value,
                    uint8_t decimals) {
  if (!valid) {
    snprintf(buffer, bufferSize, "--");
    return;
  }

  char format[8] = {};
  snprintf(format, sizeof(format), "%%.%uf", decimals);
  snprintf(buffer, bufferSize, format, value);
}

const char *gpsDebugState(uint32_t charsDelta,
                          uint32_t passedDelta,
                          uint32_t failedDelta,
                          uint16_t ubxHeaderCount) {
  if (charsDelta == 0) {
    return "no_uart";
  }
  if (ubxHeaderCount > 0 && passedDelta == 0) {
    return "ubx_binary";
  }
  if (passedDelta == 0 && failedDelta > 0) {
    return "bad_nmea";
  }
  if (!isGpsLocationFresh()) {
    return "no_fix";
  }

  return "fix";
}

void appendGpsDebugHexByte(uint8_t value) {
  if (gpsDebugHexSampleLength >= kGpsDebugHexSampleByteCount) {
    return;
  }

  const size_t offset = static_cast<size_t>(gpsDebugHexSampleLength) * 3;
  snprintf(gpsDebugHexSample + offset,
           sizeof(gpsDebugHexSample) - offset,
           "%02X ",
           value);
  ++gpsDebugHexSampleLength;
}

void captureGpsDebugByte(char value) {
  const uint8_t rawValue = static_cast<uint8_t>(value);
  if (gpsDebugPreviousByte == 0xB5 && rawValue == 0x62) {
    ++gpsDebugUbxHeaderCount;
  }
  gpsDebugPreviousByte = rawValue;
  appendGpsDebugHexByte(rawValue);

  if (value == '$') {
    ++gpsDebugDollarCount;
  } else if (value == '*') {
    ++gpsDebugStarCount;
  } else if (value == '\n') {
    ++gpsDebugLineFeedCount;
  }

  if (gpsDebugRawSampleLength >= kGpsDebugRawSampleSize) {
    return;
  }

  if (value == '\r') {
    return;
  }

  if (value == '\n') {
    gpsDebugRawSample[gpsDebugRawSampleLength++] = '|';
  } else if (value >= 32 && value <= 126) {
    gpsDebugRawSample[gpsDebugRawSampleLength++] = value;
  } else {
    gpsDebugRawSample[gpsDebugRawSampleLength++] = '.';
  }
  gpsDebugRawSample[gpsDebugRawSampleLength] = '\0';
}

void updateGpsBaudProbe(uint32_t now) {
  const uint32_t passedChecksum = gps.passedChecksum();
  if (passedChecksum > 0) {
    if (passedChecksum != lastGpsBaudProbePassedChecksum) {
      lastGpsBaudProbePassedChecksum = passedChecksum;
      lastGpsBaudProbeMs = now;
    }
    return;
  }

  if (now - lastGpsBaudProbeMs < kGpsBaudProbeIntervalMs) {
    return;
  }

  lastGpsBaudProbeMs = now;
  const uint32_t probeBaudRate = kGpsBaudRates[gpsCurrentBaudRateIndex];
  beginGpsSerial(probeBaudRate);
  delay(50);
  configureGpsAtCurrentBaud("baud probe", false);
  gpsCurrentBaudRateIndex =
      (gpsCurrentBaudRateIndex + 1) % kGpsBaudRateCount;
}

uint32_t gpsTimeFromHourStart() {
  if (!isGpsTimeFresh()) {
    return 0;
  }

  return static_cast<uint32_t>(gps.time.minute()) * 30000UL +
         static_cast<uint32_t>(gps.time.second()) * 500UL +
         static_cast<uint32_t>(gps.time.centisecond()) * 5UL;
}

uint32_t gpsDateHourValue() {
  if (!isGpsDateFresh() || !isGpsTimeFresh()) {
    return 0;
  }

  const uint16_t year =
      static_cast<uint16_t>(constrain(gps.date.year(), 2000, 2099));
  const uint8_t month =
      static_cast<uint8_t>(constrain(gps.date.month(), 1, 12));
  const uint8_t day =
      static_cast<uint8_t>(constrain(gps.date.day(), 1, 31));
  const uint8_t hour = min<uint8_t>(gps.time.hour(), 23);

  return static_cast<uint32_t>(year - 2000) * 8928UL +
         static_cast<uint32_t>(month - 1) * 744UL +
         static_cast<uint32_t>(day - 1) * 24UL +
         hour;
}

void syncGpsDateHourValue() {
  const uint32_t nextDateHourValue = gpsDateHourValue();
  if (!gpsDateHourInitialized) {
    gpsDateHourInitialized = true;
    gpsLastDateHourValue = nextDateHourValue;
    gpsTimeDirty = true;
    return;
  }

  if (nextDateHourValue != gpsLastDateHourValue) {
    gpsLastDateHourValue = nextDateHourValue;
    gpsSyncBits = (gpsSyncBits + 1) & 0x07;
    gpsTimeDirty = true;
  }
}

void updateGpsFreshnessState() {
  const bool locationFresh = isGpsLocationFresh();
  const bool altitudeFresh = isGpsAltitudeFresh();
  const bool speedFresh = isGpsSpeedFresh();
  const bool courseFresh = isGpsCourseFresh();
  const bool timeFresh = isGpsTimeFresh();
  const bool dateFresh = isGpsDateFresh();

  if (locationFresh != gpsLocationWasFresh ||
      altitudeFresh != gpsAltitudeWasFresh ||
      speedFresh != gpsSpeedWasFresh ||
      courseFresh != gpsCourseWasFresh ||
      timeFresh != gpsTimeWasFresh) {
    gpsMainDirty = true;
  }

  if (timeFresh != gpsTimeWasFresh || dateFresh != gpsDateWasFresh) {
    gpsTimeDirty = true;
  }

  gpsLocationWasFresh = locationFresh;
  gpsAltitudeWasFresh = altitudeFresh;
  gpsSpeedWasFresh = speedFresh;
  gpsCourseWasFresh = courseFresh;
  gpsTimeWasFresh = timeFresh;
  gpsDateWasFresh = dateFresh;
}

uint8_t gpsFixQuality() {
  if (gpsGgaFixQuality.isValid() && gpsGgaFixQuality.age() <= kGpsFreshAgeMs) {
    return min<uint8_t>(static_cast<uint8_t>(atoi(gpsGgaFixQuality.value())), 3);
  }

  return isGpsLocationFresh() ? 1 : 0;
}

uint8_t gpsLockedSatellites() {
  if (!gps.satellites.isValid() || gps.satellites.age() > kGpsFreshAgeMs) {
    return 0x3F;
  }

  return min<uint8_t>(static_cast<uint8_t>(gps.satellites.value()), 0x3E);
}

uint8_t gpsHdopRaw() {
  if (!gps.hdop.isValid() || gps.hdop.age() > kGpsFreshAgeMs) {
    return 0xFF;
  }

  const long hdopTenths = lround(gps.hdop.hdop() * 10.0);
  return static_cast<uint8_t>(constrain(hdopTenths, 0L, 0xFEL));
}

uint32_t gpsLatLonRaw(double degrees) {
  return static_cast<uint32_t>(static_cast<int32_t>(
      llround(degrees * 10000000.0)));
}

uint16_t gpsAltitudeRaw() {
  if (!isGpsAltitudeFresh()) {
    return 0xFFFF;
  }

  const double meters = gps.altitude.meters();
  const double shiftedMeters = meters + 500.0;
  if (shiftedMeters >= 0.0 && shiftedMeters <= 6053.5) {
    return static_cast<uint16_t>(llround(shiftedMeters * 10.0)) & 0x7FFF;
  }

  return (static_cast<uint16_t>(llround(shiftedMeters)) & 0x7FFF) | 0x8000;
}

uint16_t gpsSpeedRaw() {
  if (!isGpsSpeedFresh()) {
    return 0xFFFF;
  }

  const double speedKmph = max(0.0, gps.speed.kmph());
  if (speedKmph <= 655.35) {
    return static_cast<uint16_t>(llround(speedKmph * 100.0)) & 0x7FFF;
  }

  return (static_cast<uint16_t>(llround(speedKmph * 10.0)) & 0x7FFF) | 0x8000;
}

uint16_t gpsBearingRaw() {
  if (!isGpsCourseFresh()) {
    return 0xFFFF;
  }

  double bearingDegrees = fmod(gps.course.deg(), 360.0);
  if (bearingDegrees < 0.0) {
    bearingDegrees += 360.0;
  }

  return static_cast<uint16_t>(llround(bearingDegrees * 100.0));
}

void buildGpsMainPacket(uint8_t *packet) {
  memset(packet, 0, 20);
  const uint32_t syncTime =
      (static_cast<uint32_t>(gpsSyncBits) << 21) |
      (gpsTimeFromHourStart() & 0x1FFFFF);
  writeUint24Be(packet, syncTime);

  packet[3] = static_cast<uint8_t>((gpsFixQuality() << 6) |
                                   (gpsLockedSatellites() & 0x3F));

  if (isGpsLocationFresh()) {
    writeUint32Be(packet + 4, gpsLatLonRaw(gps.location.lat()));
    writeUint32Be(packet + 8, gpsLatLonRaw(gps.location.lng()));
  } else {
    writeUint32Be(packet + 4, 0x7FFFFFFFUL);
    writeUint32Be(packet + 8, 0x7FFFFFFFUL);
  }

  writeUint16Be(packet + 12, gpsAltitudeRaw());
  writeUint16Be(packet + 14, gpsSpeedRaw());
  writeUint16Be(packet + 16, gpsBearingRaw());
  packet[18] = gpsHdopRaw();
  packet[19] = 0xFF;
}

void buildGpsTimePacket(uint8_t *packet) {
  memset(packet, 0, 3);
  const uint32_t syncDateHour =
      (static_cast<uint32_t>(gpsSyncBits) << 21) |
      (gpsLastDateHourValue & 0x1FFFFF);
  writeUint24Be(packet, syncDateHour);
}

bool canSendBleNotification(BLE2902 *descriptor) {
  return bleClientConnected &&
         descriptor != nullptr &&
         descriptor->getNotifications() &&
         millis() - bleConnectedAtMs >= kBleReconnectQuietMs;
}

void updateGpsCharacteristicValues(bool notify) {
  syncGpsDateHourValue();

  if (gpsTimeCharacteristic != nullptr && gpsTimeDirty) {
    uint8_t packet[3] = {};
    buildGpsTimePacket(packet);
    gpsTimeCharacteristic->setValue(packet, sizeof(packet));
    if (notify && canSendBleNotification(gpsTimeNotifyDescriptor)) {
      gpsTimeCharacteristic->notify();
    }
    gpsTimeDirty = false;
  }

  if (gpsMainCharacteristic != nullptr && gpsMainDirty) {
    uint8_t packet[20] = {};
    buildGpsMainPacket(packet);
    gpsMainCharacteristic->setValue(packet, sizeof(packet));
    if (notify && canSendBleNotification(gpsMainNotifyDescriptor)) {
      gpsMainCharacteristic->notify();
    }
    gpsMainDirty = false;
  }
}

void printGpsDebugLog(uint32_t now) {
  if (now - lastGpsDebugLogMs < kGpsDebugLogIntervalMs) {
    return;
  }

  lastGpsDebugLogMs = now;

  const uint32_t charsProcessed = gps.charsProcessed();
  const uint32_t passedChecksum = gps.passedChecksum();
  const uint32_t failedChecksum = gps.failedChecksum();
  const uint32_t sentencesWithFix = gps.sentencesWithFix();
  const uint32_t charsDelta = charsProcessed - lastGpsDebugCharsProcessed;
  const uint32_t passedDelta = passedChecksum - lastGpsDebugPassedChecksum;
  const uint32_t failedDelta = failedChecksum - lastGpsDebugFailedChecksum;
  const uint32_t fixDelta = sentencesWithFix - lastGpsDebugSentencesWithFix;
  const uint16_t dollarCount = gpsDebugDollarCount;
  const uint16_t starCount = gpsDebugStarCount;
  const uint16_t lineFeedCount = gpsDebugLineFeedCount;
  const uint16_t ubxHeaderCount = gpsDebugUbxHeaderCount;

  lastGpsDebugCharsProcessed = charsProcessed;
  lastGpsDebugPassedChecksum = passedChecksum;
  lastGpsDebugFailedChecksum = failedChecksum;
  lastGpsDebugSentencesWithFix = sentencesWithFix;

  char satellites[8] = {};
  char hdop[8] = {};
  char latitude[16] = {};
  char longitude[16] = {};
  char speed[12] = {};
  char course[12] = {};
  formatGpsUnsigned(satellites,
                    sizeof(satellites),
                    gps.satellites.isValid(),
                    gps.satellites.value());
  formatGpsFloat(hdop,
                 sizeof(hdop),
                 gps.hdop.isValid(),
                 gps.hdop.hdop(),
                 1);
  formatGpsFloat(latitude,
                 sizeof(latitude),
                 isGpsLocationFresh(),
                 gps.location.lat(),
                 7);
  formatGpsFloat(longitude,
                 sizeof(longitude),
                 isGpsLocationFresh(),
                 gps.location.lng(),
                 7);
  formatGpsFloat(speed,
                 sizeof(speed),
                 isGpsSpeedFresh(),
                 gps.speed.kmph(),
                 2);
  formatGpsFloat(course,
                 sizeof(course),
                 isGpsCourseFresh(),
                 gps.course.deg(),
                 2);

  const char *state =
      gpsDebugState(charsDelta, passedDelta, failedDelta, ubxHeaderCount);
  if (passedChecksum > 0) {
    Serial.printf(
        "GPS diag state=%s baud=%lu ok=%lu (+%lu), fix_sent=%lu (+%lu), used=%s, hdop=%s, lat=%s, lon=%s, speed=%s km/h, course=%s deg\n",
        state,
        static_cast<unsigned long>(gpsCurrentBaudRate),
        static_cast<unsigned long>(passedChecksum),
        static_cast<unsigned long>(passedDelta),
        static_cast<unsigned long>(sentencesWithFix),
        static_cast<unsigned long>(fixDelta),
        satellites,
        hdop,
        latitude,
        longitude,
        speed,
        course);
  } else {
    Serial.printf(
        "GPS diag state=%s RX=GPIO%u baud=%lu chars=%lu (+%lu), ok=%lu (+%lu), checksum_fail=%lu (+%lu), $=%u, *=%u, lf=%u, ubx=%u, raw=\"%s\", hex=\"%s\"\n",
        state,
        kGpsRxPin,
        static_cast<unsigned long>(gpsCurrentBaudRate),
        static_cast<unsigned long>(charsProcessed),
        static_cast<unsigned long>(charsDelta),
        static_cast<unsigned long>(passedChecksum),
        static_cast<unsigned long>(passedDelta),
        static_cast<unsigned long>(failedChecksum),
        static_cast<unsigned long>(failedDelta),
        dollarCount,
        starCount,
        lineFeedCount,
        ubxHeaderCount,
        gpsDebugRawSample,
        gpsDebugHexSample);
  }

  gpsDebugDollarCount = 0;
  gpsDebugStarCount = 0;
  gpsDebugLineFeedCount = 0;
  gpsDebugUbxHeaderCount = 0;
  gpsDebugRawSampleLength = 0;
  gpsDebugHexSampleLength = 0;
  gpsDebugRawSample[0] = '\0';
  gpsDebugHexSample[0] = '\0';
}

void updateGpsFromSerial(uint32_t now) {
  while (gpsSerial.available() > 0) {
    const char value = static_cast<char>(gpsSerial.read());
    captureGpsDebugByte(value);
    if (gps.encode(value)) {
      gpsMainDirty = true;
    }
  }

  updateGpsFreshnessState();

  if (gpsMainDirty || gpsTimeDirty) {
    updateGpsCharacteristicValues(true);
  }

  printGpsDebugLog(now);
  updateGpsBaudProbe(now);
}

bool shouldSendPid(uint32_t pid) {
  return allowAllPids ||
         (pid == kBrakePressurePid && brakePressurePidAllowed) ||
         (pid == kThrottlePositionPid && throttlePositionPidAllowed) ||
         (pid == kBatteryVoltagePid && batteryVoltagePidAllowed) ||
         (pid == kAfrPid && afrPidAllowed);
}

bool publishCanValue(uint32_t pid, uint16_t rawValue) {
  if (!canSendBleNotification(canMainNotifyDescriptor) ||
      canMainCharacteristic == nullptr ||
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

uint16_t normalizeNotifyInterval(uint16_t requestedIntervalMs) {
  return max<uint16_t>(kMinNotifyIntervalMs, requestedIntervalMs);
}

void setBleNotificationDescriptors(bool enabled) {
  if (canMainNotifyDescriptor != nullptr) {
    canMainNotifyDescriptor->setNotifications(enabled);
  }
  if (gpsMainNotifyDescriptor != nullptr) {
    gpsMainNotifyDescriptor->setNotifications(enabled);
  }
  if (gpsTimeNotifyDescriptor != nullptr) {
    gpsTimeNotifyDescriptor->setNotifications(enabled);
  }
}

void publishTelemetry(float pressureBar,
                      float throttlePercent,
                      float batteryVolts,
                      float afr) {
  const uint16_t pressureCentibar = pressureBarToCentibar(pressureBar);
  const uint16_t throttleCentipercent =
      static_cast<uint16_t>(lroundf(constrain(throttlePercent, 0.0F, 100.0F) * 100.0F));
  const uint16_t batteryMillivolts = voltsToMillivolts(batteryVolts);
  const uint16_t afrCentivalue = afrToCentiAfr(afr);

  publishCanValue(kBrakePressurePid, pressureCentibar);
  publishCanValue(kThrottlePositionPid, throttleCentipercent);
  publishCanValue(kBatteryVoltagePid, batteryMillivolts);
  publishCanValue(kAfrPid, afrCentivalue);

  const uint32_t now = millis();

  if (now - lastBleTxLogMs >= kBleTxLogIntervalMs) {
    lastBleTxLogMs = now;
    Serial.printf(
        "BLE TX brake=%5.1f bar, throttle=%5.1f%%, battery=%6.3fV, AFR=%5.2f\n",
        pressureBar,
        throttlePercent,
        batteryVolts,
        afr);
  }
}

class RaceChronoServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    bleClientConnected = true;
    bleConnectedAtMs = millis();
    lastCanNotifyMs = bleConnectedAtMs;
  }

  void onDisconnect(BLEServer *) override {
    bleClientConnected = false;
    setBleNotificationDescriptors(false);
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
      afrPidAllowed = false;
      Serial.println("RaceChrono filter: deny all PIDs");
      return;
    }

    if (commandId == 1 && length >= 3) {
      allowAllPids = true;
      brakePressurePidAllowed = true;
      throttlePositionPidAllowed = true;
      batteryVoltagePidAllowed = true;
      afrPidAllowed = true;
      const uint16_t requestedIntervalMs = readUint16Be(data + 1);
      notifyIntervalMs = normalizeNotifyInterval(requestedIntervalMs);
      Serial.printf(
          "RaceChrono filter: allow all PIDs, interval=%u ms%s\n",
          notifyIntervalMs,
          requestedIntervalMs == notifyIntervalMs ? "" : " (clamped)");
      return;
    }

    if (commandId == 2 && length >= 7) {
      const uint16_t requestedIntervalMs = readUint16Be(data + 1);
      const uint16_t interval = normalizeNotifyInterval(requestedIntervalMs);
      const uint32_t pid = readUint32Be(data + 3);
      const bool knownPid = pid == kBrakePressurePid ||
                            pid == kThrottlePositionPid ||
                            pid == kBatteryVoltagePid ||
                            pid == kAfrPid;
      if (pid == kBrakePressurePid) {
        brakePressurePidAllowed = true;
      } else if (pid == kThrottlePositionPid) {
        throttlePositionPidAllowed = true;
      } else if (pid == kBatteryVoltagePid) {
        batteryVoltagePidAllowed = true;
      } else if (pid == kAfrPid) {
        afrPidAllowed = true;
      }
      if (knownPid) {
        notifyIntervalMs = interval;
      }
      Serial.printf(
          "RaceChrono filter: allow PID=0x%08lX, interval=%u ms%s\n",
          static_cast<unsigned long>(pid),
          interval,
          knownPid ? (requestedIntervalMs == interval ? "" : " (clamped)") :
                     " (ignored)");
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
  canMainNotifyDescriptor = new BLE2902();
  canMainCharacteristic->addDescriptor(canMainNotifyDescriptor);

  BLECharacteristic *canFilterCharacteristic = raceChronoService->createCharacteristic(
      kCanFilterCharacteristicUuid,
      BLECharacteristic::PROPERTY_WRITE);
  canFilterCharacteristic->setCallbacks(new RaceChronoCanFilterCallbacks());

  gpsMainCharacteristic = raceChronoService->createCharacteristic(
      kGpsMainCharacteristicUuid,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  gpsMainNotifyDescriptor = new BLE2902();
  gpsMainCharacteristic->addDescriptor(gpsMainNotifyDescriptor);

  gpsTimeCharacteristic = raceChronoService->createCharacteristic(
      kGpsTimeCharacteristicUuid,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  gpsTimeNotifyDescriptor = new BLE2902();
  gpsTimeCharacteristic->addDescriptor(gpsTimeNotifyDescriptor);

  uint8_t initialPacket[8] = {};
  writeUint32Le(initialPacket, kBrakePressurePid);
  writeUint16Be(initialPacket + 4, pressureBarToCentibar(brakePressureBar));
  canMainCharacteristic->setValue(initialPacket, sizeof(initialPacket));
  updateGpsCharacteristicValues(false);

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
  Serial.printf("RaceChrono BLE CAN PID 0x%08lX: AFR, centi-AFR\n",
                static_cast<unsigned long>(kAfrPid));
  Serial.printf("RaceChrono BLE GPS: native GPS feature on characteristics 0x0003/0x0004\n");

  loadThrottleCalibration();
  loadGpsConfigState();
  startGps();
  startBatteryAdc();
  startAdc();
  startDisplay();
  calibrateThrottle();
  startRaceChronoBle();
  Serial.printf("BLE advertising as \"%s\"\n", kDeviceName);
}

void loop() {
  const uint32_t now = millis();

  updateThrottleFromAdc(now);
  updateBrakePressureFromAdc(now);
  updateAfrFromAdc(now);
  updateBatteryFromAdc(now);
  updateGpsFromSerial(now);
  updateLedIdleBlink(now);
  updateDisplay(now);

  if (bleClientConnected && now - lastCanNotifyMs >= notifyIntervalMs) {
    lastCanNotifyMs = now;
    publishTelemetry(brakePressureBar, throttlePercent, batteryVolts, afr);
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
}
