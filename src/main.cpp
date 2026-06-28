#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <Wire.h>

namespace {

constexpr char kDeviceName[] = "RaceExporter";
constexpr char kRaceChronoServiceUuid[] = "00001ff8-0000-1000-8000-00805f9b34fb";
constexpr char kCanMainCharacteristicUuid[] = "00000001-0000-1000-8000-00805f9b34fb";
constexpr char kCanFilterCharacteristicUuid[] = "00000002-0000-1000-8000-00805f9b34fb";
constexpr char kGpsMainCharacteristicUuid[] = "00000003-0000-1000-8000-00805f9b34fb";
constexpr char kGpsTimeCharacteristicUuid[] = "00000004-0000-1000-8000-00805f9b34fb";

constexpr uint32_t kTelemetryPid = 0x00000101;
constexpr uint8_t kCanPacketPidLength = 4;
constexpr uint8_t kCanMaxPayloadLength = 16;
constexpr uint8_t kTelemetryPayloadLength = 8;
constexpr uint8_t kTelemetryBrakePressureOffset = 0;
constexpr uint8_t kTelemetryThrottlePositionOffset = 2;
constexpr uint8_t kTelemetryAfrOffset = 4;
constexpr uint8_t kTelemetryBatteryVoltageOffset = 6;
constexpr uint16_t kDefaultNotifyIntervalMs = 50;
constexpr uint16_t kMinNotifyIntervalMs = 50;
constexpr uint16_t kBleReconnectQuietMs = 750;
constexpr uint16_t kBleConnMinIntervalUnits = 0x0C;   // 15 ms
constexpr uint16_t kBleConnMaxIntervalUnits = 0x18;   // 30 ms
constexpr uint16_t kBleConnLatency = 0;
constexpr uint16_t kBleConnTimeoutUnits = 400;        // 4 s supervision timeout
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
constexpr uint32_t kAdcI2cClockHz = 50000;
constexpr uint16_t kAdcI2cTimeoutMs = 25;
constexpr uint16_t kAdcLogIntervalMs = 5000;
constexpr uint16_t kAdcMissingLogIntervalMs = 5000;
constexpr float kAds1115VoltsPerBit = 6.144F / 32768.0F;
constexpr uint8_t kThrottleAdcChannel = 0;
constexpr uint8_t kBrakePressureAdcChannel = 1;
constexpr uint8_t kAfrAdcChannel = 2;
constexpr uint16_t kThrottleAdcReadIntervalMs = 50;
constexpr uint16_t kBrakePressureAdcReadIntervalMs = 50;
constexpr uint16_t kAfrAdcReadIntervalMs = 100;
constexpr uint8_t kBatteryAdcPin = 35;
constexpr uint8_t kBatteryAdcResolutionBits = 12;
constexpr uint16_t kBatteryAdcReadIntervalMs = 1000;
constexpr uint8_t kBatteryAdcSampleCount = 8;
constexpr uint8_t kGpsRxPin = 16;
constexpr uint8_t kGpsTxPin = 17;
constexpr size_t kGpsSerialRxBufferSize = 8192;
constexpr uint32_t kGpsBaudRates[] = {115200, 9600, 38400, 57600, 4800};
constexpr uint8_t kGpsBaudRateCount =
    sizeof(kGpsBaudRates) / sizeof(kGpsBaudRates[0]);
constexpr uint32_t kGpsTargetBaudRate = 115200;
constexpr uint32_t kGpsInitialBaudRate = kGpsTargetBaudRate;
constexpr uint8_t kGpsNavigationRateHz = 10;
constexpr uint16_t kGpsMeasurementRateMs = 1000 / kGpsNavigationRateHz;
constexpr uint16_t kGpsFreshAgeMs = 2500;
constexpr uint16_t kGpsDebugLogIntervalMs = 30000;
constexpr uint16_t kGpsBaudProbeIntervalMs = 6000;
constexpr uint8_t kGpsDebugRawSampleSize = 96;
constexpr uint8_t kGpsDebugHexSampleByteCount = 32;
constexpr uint16_t kGpsUbxMaxPayloadLength = 768;
constexpr uint8_t kGpsUbxNavSatHeaderLength = 8;
constexpr uint8_t kGpsUbxNavSatSvLength = 12;
constexpr uint8_t kGpsUbxNavSatUartRate = 10;
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
constexpr float kDefaultBrakePressureZeroVolts =
    kBrakePressureSupplyVolts * kBrakePressureZeroRatio;
constexpr float kBrakePressureFullVolts =
    kBrakePressureSupplyVolts * kBrakePressureFullRatio;
constexpr uint16_t kBrakeZeroCalibrationMs = 1000;
constexpr float kMinBrakeZeroCalibrationVolts = 0.5F;
constexpr float kMaxBrakeZeroCalibrationVolts = 0.7F;
constexpr float kBrakePressureHoldDeadbandBar = 0.25F;
constexpr float kBatteryDividerMultiplier = 1.951091F;
constexpr float kBatteryAdcFilterAlpha = 0.2F;
constexpr float kAfrAdcFilterAlpha = 0.2F;
constexpr char kThrottleCalibrationPrefsNamespace[] = "throttle";
constexpr char kThrottleZeroPrefsKey[] = "zero_v";
constexpr char kThrottleFullPrefsKey[] = "full_v";
constexpr char kBrakeCalibrationPrefsNamespace[] = "brake";
constexpr char kBrakeZeroPrefsKey[] = "zero_v";
constexpr char kGpsConfigPrefsNamespace[] = "gps";
constexpr char kGpsGlonassProfilePrefsKey[] = "glo_prof";

constexpr uint8_t kGpsUbxClassNav = 0x01;
constexpr uint8_t kGpsUbxClassCfg = 0x06;
constexpr uint8_t kGpsUbxClassAck = 0x05;
constexpr uint8_t kGpsUbxIdAckNak = 0x00;
constexpr uint8_t kGpsUbxIdAckAck = 0x01;
constexpr uint8_t kGpsUbxIdCfgPrt = 0x00;
constexpr uint8_t kGpsUbxIdCfgMsg = 0x01;
constexpr uint8_t kGpsUbxIdCfgRate = 0x08;
constexpr uint8_t kGpsUbxIdNavDop = 0x04;
constexpr uint8_t kGpsUbxIdNavPvt = 0x07;
constexpr uint8_t kGpsUbxIdNavSat = 0x35;
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

struct AfrScalePoint {
  float afr;
  float volts;
};

// Keep points sorted by voltage; values between points are interpolated.
constexpr AfrScalePoint kAfrScalePoints[] = {
    {10.0F, -0.501F},
    {11.1F, -0.174F},
    {11.2F, -0.138F},
    {14.1F, 0.564F},
    {14.3F, 0.606F},
    {14.4F, 0.873F},
    {14.6F, 0.903F},
    {15.0F, 0.999F},
    {15.3F, 1.239F},
    {15.8F, 1.383F},
    {18.4F, 2.001F},
    {19.4F, 2.253F},
    {20.0F, 2.388F},
};
constexpr size_t kAfrScalePointCount =
    sizeof(kAfrScalePoints) / sizeof(kAfrScalePoints[0]);
static_assert(kAfrScalePointCount >= 2,
              "AFR scale needs at least two calibration points");

struct GpsNavSatState {
  bool valid;
  uint32_t updatedMs;
  uint8_t numSvs;
  uint8_t visibleCount;
  uint8_t usedCount;
  uint8_t bestCno;
};

struct GpsNavState {
  bool locationValid;
  bool altitudeValid;
  bool speedValid;
  bool courseValid;
  bool timeValid;
  bool dateValid;
  bool dopValid;
  uint32_t pvtUpdatedMs;
  uint32_t dopUpdatedMs;
  uint32_t iTowMs;
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t fixType;
  uint8_t flags;
  uint8_t numSv;
  int32_t lonDeg1e7;
  int32_t latDeg1e7;
  int32_t heightMslMm;
  int32_t groundSpeedMmps;
  int32_t headingMotionDeg1e5;
  uint16_t hDopCentis;
};

struct GpsUbxStreamParser {
  uint8_t state;
  uint8_t messageClass;
  uint8_t messageId;
  uint8_t checksumA;
  uint8_t checksumB;
  uint8_t expectedChecksumA;
  uint16_t payloadLength;
  uint16_t payloadIndex;
  bool tooLong;
  uint8_t payload[kGpsUbxMaxPayloadLength];
};

NimBLECharacteristic *canMainCharacteristic = nullptr;
NimBLECharacteristic *gpsMainCharacteristic = nullptr;
NimBLECharacteristic *gpsTimeCharacteristic = nullptr;
Preferences throttleCalibrationPrefs;
Preferences brakeCalibrationPrefs;
Preferences gpsConfigPrefs;
HardwareSerial gpsSerial(2);
GpsNavState gpsNav = {};
GpsNavSatState gpsNavSat = {};
GpsUbxStreamParser gpsUbxStreamParser = {};

bool bleClientConnected = false;
bool canMainSubscribed = false;
bool gpsMainSubscribed = false;
bool gpsTimeSubscribed = false;
uint32_t bleNotifyOkCount = 0;
uint32_t bleNotifyFailCount = 0;
bool allowAllPids = true;
bool telemetryPidAllowed = true;
uint16_t notifyIntervalMs = kDefaultNotifyIntervalMs;
uint32_t bleConnectedAtMs = 0;
uint32_t lastCanNotifyMs = 0;
uint32_t lastBleTxLogMs = 0;
uint32_t lastLedToggleMs = 0;
uint32_t lastLedIdleBlinkMs = 0;
uint32_t lastAdcLogMs = 0;
uint32_t lastAdcMissingLogMs = 0;
uint32_t lastThrottleAdcReadMs = 0;
uint32_t lastBrakePressureAdcReadMs = 0;
uint32_t lastBatteryAdcReadMs = 0;
uint32_t lastAfrAdcReadMs = 0;
uint32_t lastGpsDebugLogMs = 0;
uint32_t lastGpsDebugBytesProcessed = 0;
uint32_t lastGpsDebugValidUbxPackets = 0;
uint32_t lastGpsDebugChecksumFailures = 0;
uint32_t lastGpsDebugPvtPackets = 0;
uint32_t lastGpsDebugDopPackets = 0;
uint32_t lastGpsDebugSatPackets = 0;
uint32_t lastGpsBaudProbeMs = 0;
uint32_t lastGpsBaudProbePvtPackets = 0;
uint32_t gpsCurrentBaudRate = kGpsInitialBaudRate;
uint32_t gpsUbxBytesProcessed = 0;
uint32_t gpsUbxValidPacketCount = 0;
uint32_t gpsUbxChecksumFailureCount = 0;
uint32_t gpsUbxUnexpectedPacketCount = 0;
uint32_t gpsUbxTooLongPacketCount = 0;
uint32_t gpsUbxPvtPacketCount = 0;
uint32_t gpsUbxDopPacketCount = 0;
uint32_t gpsUbxSatPacketCount = 0;
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
uint32_t adcReadFailureCount = 0;
uint32_t adcBusRecoveryCount = 0;
uint32_t adcNotFoundCount = 0;
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
bool brakeCalibrationPrefsReady = false;
bool gpsConfigPrefsReady = false;
bool gpsGlonassProfilePersisted = false;
float brakePressureZeroVolts = kDefaultBrakePressureZeroVolts;
int16_t brakePressureAdcRaw = 0;
float brakePressureAdcVolts = 0.0F;
float brakePressureBar = 0.0F;
bool brakePressureAdcValid = false;
bool brakePressureFilterInitialized = false;
int16_t afrAdcRaw = 0;
float afrAdcVolts = 0.0F;
float afrFilteredVolts = 0.0F;
float afr = kAfrScalePoints[0].afr;
bool afrAdcValid = false;
bool afrFilterInitialized = false;
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
bool telemetryDirty = false;
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

uint16_t readUint16Le(const uint8_t *data) {
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
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

int32_t readInt32Le(const uint8_t *data) {
  return static_cast<int32_t>(readUint32Le(data));
}

void writeUint16Le(uint8_t *data, uint16_t value) {
  data[0] = static_cast<uint8_t>(value & 0xFF);
  data[1] = static_cast<uint8_t>(value >> 8);
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

constexpr uint8_t kAdcConversionRegister = 0x00;
constexpr uint8_t kAdcConfigRegister = 0x01;
// 860 SPS -> конверсия ~1.16 мс; 2 мс с запасом.
constexpr uint16_t kAdcConversionWaitMs = 2;

uint16_t makeAds1115Config(uint8_t channel) {
  const uint16_t mux = static_cast<uint16_t>(0x04 + channel) << 12;
  return 0x8000 | mux | 0x0000 | 0x0100 | 0x00E0 | 0x0003;
}

// Блокирующее чтение: используется только в калибровке на старте, где
// неблокирующий секвенсор еще не работает.
bool readAds1115SingleEnded(uint8_t channel, int16_t &rawValue, float &volts) {
  if (channel > 3 || adcI2cAddress == 0) {
    return false;
  }

  if (!writeAds1115Register(kAdcConfigRegister, makeAds1115Config(channel))) {
    return false;
  }

  delay(kAdcConversionWaitMs);
  if (!readAds1115Register(kAdcConversionRegister, rawValue)) {
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

bool isBrakeZeroCalibrationValid(float volts) {
  return volts >= kMinBrakeZeroCalibrationVolts &&
         volts <= kMaxBrakeZeroCalibrationVolts;
}

void loadBrakeCalibration() {
  brakeCalibrationPrefsReady =
      brakeCalibrationPrefs.begin(kBrakeCalibrationPrefsNamespace, false);

  if (!brakeCalibrationPrefsReady) {
    Serial.printf(
        "Brake zero calibration NVS unavailable, using default zero=%.3fV full=%.3fV\n",
        brakePressureZeroVolts,
        kBrakePressureFullVolts);
    return;
  }

  const float storedZeroVolts =
      brakeCalibrationPrefs.getFloat(kBrakeZeroPrefsKey,
                                     kDefaultBrakePressureZeroVolts);

  brakePressureZeroVolts = isBrakeZeroCalibrationValid(storedZeroVolts)
                               ? storedZeroVolts
                               : kDefaultBrakePressureZeroVolts;

  Serial.printf("Brake zero calibration loaded: zero=%.3fV full=%.3fV\n",
                brakePressureZeroVolts,
                kBrakePressureFullVolts);
}

void saveBrakeCalibrationValue(const char *key, float volts) {
  if (!brakeCalibrationPrefsReady) {
    Serial.printf("Brake zero calibration NVS unavailable, not saving %.3fV\n",
                  volts);
    return;
  }

  brakeCalibrationPrefs.putFloat(key, volts);
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
  const float brakePressureSpanVolts =
      kBrakePressureFullVolts - brakePressureZeroVolts;
  if (brakePressureSpanVolts <= 0.01F) {
    return volts > brakePressureZeroVolts
               ? static_cast<float>(kMaxBrakePressureBar)
               : 0.0F;
  }

  return constrain((volts - brakePressureZeroVolts) *
                       static_cast<float>(kMaxBrakePressureBar) /
                       brakePressureSpanVolts,
                   0.0F,
                   static_cast<float>(kMaxBrakePressureBar));
}

float adcVoltsToBatteryVolts(float volts) {
  return max(0.0F, volts * kBatteryDividerMultiplier);
}

float adcVoltsToAfr(float volts) {
  if (volts <= kAfrScalePoints[0].volts) {
    return kAfrScalePoints[0].afr;
  }

  for (size_t index = 1; index < kAfrScalePointCount; ++index) {
    const AfrScalePoint &lower = kAfrScalePoints[index - 1];
    const AfrScalePoint &upper = kAfrScalePoints[index];
    if (volts <= upper.volts) {
      const float spanVolts = upper.volts - lower.volts;
      if (spanVolts <= 0.000001F) {
        return upper.afr;
      }

      const float ratio = (volts - lower.volts) / spanVolts;
      return lower.afr + ratio * (upper.afr - lower.afr);
    }
  }

  return kAfrScalePoints[kAfrScalePointCount - 1].afr;
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
  afrFilterInitialized = false;
  afrAdcRaw = 0;
  afrAdcVolts = 0.0F;
  afrFilteredVolts = 0.0F;
  afr = kAfrScalePoints[0].afr;
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

void recoverI2cBus();
uint8_t findAds1115Address();

void handleAdcReadFailure(const char *label, uint8_t channel) {
  ++adcReadFailureCount;
  Serial.printf(
      "ADS1115 %s A%u read failed at 0x%02X, recovering I2C bus (fails=%lu)\n",
      label, channel, adcI2cAddress,
      static_cast<unsigned long>(adcReadFailureCount));
  invalidateAds1115Measurements();
  recoverI2cBus();
  // Сразу пробуем переподключиться, чтобы не ждать следующего цикла ре-скана.
  adcI2cAddress = findAds1115Address();
  if (adcI2cAddress == 0) {
    ++adcNotFoundCount;
  }
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

float updateAfrFilteredVolts(float volts) {
  if (!afrFilterInitialized) {
    afrFilteredVolts = volts;
    afrFilterInitialized = true;
    return afrFilteredVolts;
  }

  afrFilteredVolts += kAfrAdcFilterAlpha * (volts - afrFilteredVolts);
  return afrFilteredVolts;
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
  telemetryDirty = true;
}

void updateBrakePressureAdcState(int16_t rawValue, float volts) {
  brakePressureAdcRaw = rawValue;
  brakePressureAdcVolts = volts;
  updateBrakePressureFilteredBar(adcVoltsToBrakePressureBar(volts));
  brakePressureAdcValid = true;
  telemetryDirty = true;
}

void updateAfrAdcState(int16_t rawValue, float volts) {
  afrAdcRaw = rawValue;
  afrAdcVolts = volts;
  afr = adcVoltsToAfr(updateAfrFilteredVolts(volts));
  afrAdcValid = true;
  telemetryDirty = true;
}

void updateBatteryAdcState(int16_t rawValue, float volts) {
  batteryAdcRaw = rawValue;
  batteryAdcVolts = volts;
  batteryMeasuredVolts = adcVoltsToBatteryVolts(volts);
  updateBatteryFilteredVolts(batteryMeasuredVolts);
  batteryAdcValid = true;
  telemetryDirty = true;
}

// Неблокирующий секвенсор ADS1115: в полете всегда одна конверсия (mux общий),
// каналы обходятся по кругу. Вместо delay(2) loop выходит и забирает результат
// на следующей итерации, не блокируя нотификации/GPS/LED.
enum class AdcSeqPhase : uint8_t { Idle, Converting };

struct AdcRuntimeChannel {
  uint8_t channel;
  uint16_t intervalMs;
  uint32_t *lastStartMs;
  const char *label;
};

AdcRuntimeChannel adcRuntimeChannels[] = {
    {kThrottleAdcChannel, kThrottleAdcReadIntervalMs, &lastThrottleAdcReadMs,
     "throttle"},
    {kBrakePressureAdcChannel, kBrakePressureAdcReadIntervalMs,
     &lastBrakePressureAdcReadMs, "brake pressure"},
    {kAfrAdcChannel, kAfrAdcReadIntervalMs, &lastAfrAdcReadMs, "AFR"},
};
constexpr uint8_t kAdcRuntimeChannelCount =
    sizeof(adcRuntimeChannels) / sizeof(adcRuntimeChannels[0]);

AdcSeqPhase adcSeqPhase = AdcSeqPhase::Idle;
uint8_t adcSeqChannel = 0;
uint32_t adcSeqConvStartedMs = 0;
uint8_t adcSeqRotation = 0;

void dispatchAdcResult(uint8_t channel, int16_t raw, float volts) {
  switch (channel) {
    case kThrottleAdcChannel:
      updateThrottleAdcState(raw, volts, false);
      break;
    case kBrakePressureAdcChannel:
      updateBrakePressureAdcState(raw, volts);
      break;
    case kAfrAdcChannel:
      updateAfrAdcState(raw, volts);
      break;
    default:
      break;
  }
  updateAds1115ChannelState(channel, raw, volts);
}

void serviceAds(uint32_t now) {
  if (adcI2cAddress == 0) {
    return;
  }

  if (adcSeqPhase == AdcSeqPhase::Converting) {
    if (now - adcSeqConvStartedMs < kAdcConversionWaitMs) {
      return;
    }

    int16_t raw = 0;
    if (!readAds1115Register(kAdcConversionRegister, raw)) {
      adcSeqPhase = AdcSeqPhase::Idle;
      const AdcRuntimeChannel &c = adcRuntimeChannels[adcSeqRotation];
      handleAdcReadFailure(c.label, adcSeqChannel);
      return;
    }

    const float volts = static_cast<float>(raw) * kAds1115VoltsPerBit;
    dispatchAdcResult(adcSeqChannel, raw, volts);
    adcSeqPhase = AdcSeqPhase::Idle;
    return;
  }

  for (uint8_t i = 0; i < kAdcRuntimeChannelCount; ++i) {
    const uint8_t idx = (adcSeqRotation + i) % kAdcRuntimeChannelCount;
    AdcRuntimeChannel &c = adcRuntimeChannels[idx];
    if (now - *c.lastStartMs < c.intervalMs) {
      continue;
    }

    if (!writeAds1115Register(kAdcConfigRegister, makeAds1115Config(c.channel))) {
      adcSeqRotation = idx;
      handleAdcReadFailure(c.label, c.channel);
      return;
    }

    *c.lastStartMs = now;
    adcSeqChannel = c.channel;
    adcSeqRotation = idx;
    adcSeqConvStartedMs = now;
    adcSeqPhase = AdcSeqPhase::Converting;
    return;
  }
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

bool readBrakeCalibrationAverage(uint16_t durationMs,
                                 float &averageVolts,
                                 float &minVolts,
                                 float &maxVolts,
                                 int16_t &averageRaw,
                                 uint32_t &sampleCount) {
  if (adcI2cAddress == 0) {
    Serial.printf(
        "Brake zero calibration skipped: ADS1115 not found, using zero=%.3fV full=%.3fV\n",
        brakePressureZeroVolts,
        kBrakePressureFullVolts);
    return false;
  }

  Serial.printf("Calibrating brake zero from ADS1115 A%u for %u ms...\n",
                kBrakePressureAdcChannel,
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

    if (sampleCount > 0 &&
        now - lastSampleMs < kThrottleCalibrationSampleIntervalMs) {
      delay(1);
      continue;
    }

    int16_t rawValue = 0;
    float volts = 0.0F;
    if (readAds1115SingleEnded(kBrakePressureAdcChannel, rawValue, volts)) {
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
      Serial.printf("Brake zero calibration read failed at 0x%02X\n",
                    adcI2cAddress);
      readFailed = true;
      break;
    }
  }

  if (readFailed || sampleCount == 0) {
    Serial.printf(
        "Brake zero calibration failed, using zero=%.3fV full=%.3fV\n",
        brakePressureZeroVolts,
        kBrakePressureFullVolts);
    return false;
  }

  averageVolts = voltsSum / static_cast<float>(sampleCount);
  averageRaw = static_cast<int16_t>(lroundf(static_cast<float>(rawSum) /
                                            static_cast<float>(sampleCount)));
  return true;
}

bool calibrateBrakeZero() {
  float averageVolts = 0.0F;
  float minVolts = 0.0F;
  float maxVolts = 0.0F;
  int16_t averageRaw = 0;
  uint32_t sampleCount = 0;

  if (!readBrakeCalibrationAverage(kBrakeZeroCalibrationMs,
                                 averageVolts,
                                 minVolts,
                                 maxVolts,
                                 averageRaw,
                                 sampleCount)) {
    invalidateBrakePressureAdc();
    return false;
  }

  updateBrakePressureAdcState(averageRaw, averageVolts);

  if (!isBrakeZeroCalibrationValid(averageVolts)) {
    Serial.printf(
        "Brake zero calibration rejected: %.3fV raw=%d samples=%lu outside %.3f..%.3fV, using saved %.3fV\n",
        averageVolts,
        averageRaw,
        static_cast<unsigned long>(sampleCount),
        kMinBrakeZeroCalibrationVolts,
        kMaxBrakeZeroCalibrationVolts,
        brakePressureZeroVolts);
    return false;
  }

  brakePressureZeroVolts = averageVolts;
  brakePressureBar = 0.0F;
  saveBrakeCalibrationValue(kBrakeZeroPrefsKey, brakePressureZeroVolts);

  Serial.printf("Brake zero calibrated: %.3fV raw=%d samples=%lu, full=%.3fV\n",
                brakePressureZeroVolts,
                brakePressureAdcRaw,
                static_cast<unsigned long>(sampleCount),
                kBrakePressureFullVolts);
  return true;
}

void calibrateBrakeZeroAtStartup() {
  Serial.println("Brake zero calibration: release brake pedal");
  calibrateBrakeZero();
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

void recoverI2cBus() {
  ++adcBusRecoveryCount;
  Wire.end();
  pinMode(kAdcSclPin, OUTPUT_OPEN_DRAIN);
  pinMode(kAdcSdaPin, INPUT_PULLUP);

  // Прокачиваем до 9 клоков, чтобы slave отпустил залипший SDA после
  // оборванной транзакции (характерно для Error 263 -> -1).
  // Половина периода SCL для kAdcI2cClockHz: при 50 кГц это 10 мкс,
  // чтобы slave успевал отпустить залипший SDA на сниженной частоте.
  const uint32_t halfPeriodUs = 500000UL / kAdcI2cClockHz;
  for (int i = 0; i < 9; ++i) {
    digitalWrite(kAdcSclPin, LOW);
    delayMicroseconds(halfPeriodUs);
    digitalWrite(kAdcSclPin, HIGH);
    delayMicroseconds(halfPeriodUs);
    if (digitalRead(kAdcSdaPin)) {
      break;
    }
  }

  Wire.begin(kAdcSdaPin, kAdcSclPin);
  Wire.setClock(kAdcI2cClockHz);
  Wire.setTimeOut(kAdcI2cTimeoutMs);
}

void startAdc() {
  Wire.begin(kAdcSdaPin, kAdcSclPin);
  Wire.setClock(kAdcI2cClockHz);
  Wire.setTimeOut(kAdcI2cTimeoutMs);

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

void sendGpsUbxSetUartProtocols(uint16_t outputProtocols) {
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

  writeGpsUbxPacket(kGpsUbxClassCfg, kGpsUbxIdCfgPrt, payload, sizeof(payload));
}

void sendGpsUbxSetNavigationRate10Hz() {
  uint8_t payload[6] = {};
  writeUint16Le(payload, kGpsMeasurementRateMs);
  writeUint16Le(payload + 2, 1);     // navigation solution every measurement
  writeUint16Le(payload + 4, 0);     // UTC time reference

  writeGpsUbxPacket(kGpsUbxClassCfg, kGpsUbxIdCfgRate, payload, sizeof(payload));
}

void sendGpsUbxSetMessageRate(uint8_t messageClass,
                              uint8_t messageId,
                              uint8_t uartRate) {
  uint8_t payload[] = {
      messageClass,
      messageId,
      0x00,      // I2C/DDC
      uartRate,  // UART1
      0x00,      // UART2
      0x00,      // USB
      0x00,      // SPI
      0x00       // reserved
  };

  writeGpsUbxPacket(kGpsUbxClassCfg, kGpsUbxIdCfgMsg, payload, sizeof(payload));
}

void beginGpsSerial(uint32_t baudRate) {
  gpsCurrentBaudRate = baudRate;
  gpsSerial.begin(gpsCurrentBaudRate, SERIAL_8N1, kGpsRxPin, kGpsTxPin);
}

void configureGpsAtCurrentBaud(const char *reason, bool logCommand = true) {
  sendGpsUbxSetUartProtocols(kGpsUbxProtocolUbx);
  if (logCommand) {
    Serial.printf("GPS config: sent UBX-CFG-PRT at %lu baud, target UBX %lu baud, output=UBX (%s)\n",
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
    sendGpsUbxSetUartProtocols(kGpsUbxProtocolUbx);
  }

  beginGpsSerial(kGpsTargetBaudRate);
  Serial.printf("GPS config: sent UBX-CFG-PRT at %u common baud rates, target UBX %lu baud, output=UBX (%s)\n",
                kGpsBaudRateCount,
                static_cast<unsigned long>(kGpsTargetBaudRate),
                reason);
}

void logGpsConfigAck(const char *commandName,
                     uint8_t targetClass,
                     uint8_t targetId,
                     const char *reason) {
  GpsUbxPollStats stats = {};
  const bool acknowledged = waitGpsUbxAck(targetClass, targetId, 700, stats);
  Serial.printf("GPS config: %s ack=%s (%s; ",
                commandName,
                acknowledged ? "yes" : "no",
                reason);
  printGpsUbxPollStats(stats);
  Serial.println(")");
}

void configureGpsRuntimeUbxMessages(const char *reason, bool waitForAck = true) {
  sendGpsUbxSetNavigationRate10Hz();
  if (waitForAck) {
    logGpsConfigAck("UBX-CFG-RATE 10Hz",
                    kGpsUbxClassCfg,
                    kGpsUbxIdCfgRate,
                    reason);
  }

  sendGpsUbxSetMessageRate(kGpsUbxClassNav, kGpsUbxIdNavPvt, 1);
  if (waitForAck) {
    logGpsConfigAck("UBX-CFG-MSG NAV-PVT UART1",
                    kGpsUbxClassCfg,
                    kGpsUbxIdCfgMsg,
                    reason);
  }

  sendGpsUbxSetMessageRate(kGpsUbxClassNav, kGpsUbxIdNavDop, 1);
  if (waitForAck) {
    logGpsConfigAck("UBX-CFG-MSG NAV-DOP UART1",
                    kGpsUbxClassCfg,
                    kGpsUbxIdCfgMsg,
                    reason);
  }

  sendGpsUbxSetMessageRate(kGpsUbxClassNav,
                           kGpsUbxIdNavSat,
                           kGpsUbxNavSatUartRate);
  if (waitForAck) {
    logGpsConfigAck("UBX-CFG-MSG NAV-SAT UART1",
                    kGpsUbxClassCfg,
                    kGpsUbxIdCfgMsg,
                    reason);
  }

  Serial.printf("GPS config: native UBX NAV-PVT/NAV-DOP at %uHz, NAV-SAT at %uHz, baud=%lu (%s)\n",
                kGpsNavigationRateHz,
                kGpsNavigationRateHz / kGpsUbxNavSatUartRate,
                static_cast<unsigned long>(gpsCurrentBaudRate),
                reason);
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
  logAndEnableGpsGlonassIfNeeded();
  configureGpsRuntimeUbxMessages("startup");
}

void printAdcReadings(uint32_t now) {
  if (adcI2cAddress == 0) {
    if (now - lastAdcMissingLogMs >= kAdcMissingLogIntervalMs) {
      lastAdcMissingLogMs = now;
      recoverI2cBus();
      adcI2cAddress = findAds1115Address();
      if (adcI2cAddress == 0) {
        ++adcNotFoundCount;
        Serial.printf("ADS1115 ADC still not found on I2C SDA=%u SCL=%u\n",
                      kAdcSdaPin,
                      kAdcSclPin);
      } else {
        Serial.printf("ADS1115 ADC found at 0x%02X\n", adcI2cAddress);
      }
    }

    return;
  }

  if (now - lastAdcLogMs < kAdcLogIntervalMs) {
    return;
  }

  lastAdcLogMs = now;

  Serial.printf(
      "ADC throttle A0=%.3fV -> %.1f%%, brake A1=%.3fV -> %.1f bar, AFR A2=%.3fV -> %.2f, battery GPIO%u raw=%d adc=%.3fV measured=%.3fV filtered=%.3fV, errors fails=%lu recoveries=%lu notfound=%lu\n",
      throttleAdcVolts,
      throttlePercent,
      brakePressureAdcVolts,
      brakePressureBar,
      afrAdcVolts,
      afr,
      kBatteryAdcPin,
      batteryAdcRaw,
      batteryAdcVolts,
      batteryMeasuredVolts,
      batteryVolts,
      static_cast<unsigned long>(adcReadFailureCount),
      static_cast<unsigned long>(adcBusRecoveryCount),
      static_cast<unsigned long>(adcNotFoundCount));
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
  afr = constrain(afr,
                  kAfrScalePoints[0].afr,
                  kAfrScalePoints[kAfrScalePointCount - 1].afr);
  return static_cast<uint16_t>(lroundf(afr * 100.0F));
}

bool isGpsLocationFresh() {
  return gpsNav.locationValid &&
         millis() - gpsNav.pvtUpdatedMs <= kGpsFreshAgeMs;
}

bool isGpsPvtFresh() {
  return gpsUbxPvtPacketCount > 0 &&
         millis() - gpsNav.pvtUpdatedMs <= kGpsFreshAgeMs;
}

bool isGpsAltitudeFresh() {
  return gpsNav.altitudeValid &&
         millis() - gpsNav.pvtUpdatedMs <= kGpsFreshAgeMs;
}

bool isGpsSpeedFresh() {
  return gpsNav.speedValid &&
         millis() - gpsNav.pvtUpdatedMs <= kGpsFreshAgeMs;
}

bool isGpsCourseFresh() {
  return gpsNav.courseValid &&
         millis() - gpsNav.pvtUpdatedMs <= kGpsFreshAgeMs;
}

bool isGpsTimeFresh() {
  return gpsNav.timeValid &&
         millis() - gpsNav.pvtUpdatedMs <= kGpsFreshAgeMs;
}

bool isGpsDateFresh() {
  return gpsNav.dateValid &&
         millis() - gpsNav.pvtUpdatedMs <= kGpsFreshAgeMs;
}

bool isGpsDopFresh() {
  return gpsNav.dopValid &&
         millis() - gpsNav.dopUpdatedMs <= kGpsFreshAgeMs;
}

bool isGpsSatFresh() {
  return gpsNavSat.valid &&
         millis() - gpsNavSat.updatedMs <= kGpsFreshAgeMs;
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

const char *gpsDebugState(uint32_t bytesDelta,
                          uint32_t validUbxDelta,
                          uint32_t checksumFailureDelta,
                          uint32_t pvtDelta,
                          uint16_t dollarCount) {
  if (bytesDelta == 0) {
    return "no_uart";
  }
  if (validUbxDelta == 0 && dollarCount > 0) {
    return "nmea_output";
  }
  if (validUbxDelta == 0 && checksumFailureDelta > 0) {
    return "bad_ubx";
  }
  if (pvtDelta == 0) {
    return "no_nav_pvt";
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

bool isGpsPvtLocationValid(uint8_t fixType, uint8_t flags) {
  const bool gnssFixOk = (flags & 0x01) != 0;
  return gnssFixOk && (fixType == 2 || fixType == 3 || fixType == 4);
}

void updateGpsFromNavPvt(const uint8_t *payload, uint16_t payloadLength) {
  if (payloadLength < 92) {
    return;
  }

  const uint32_t now = millis();
  const uint8_t valid = payload[11];
  const uint8_t fixType = payload[20];
  const uint8_t flags = payload[21];
  const bool locationValid = isGpsPvtLocationValid(fixType, flags);

  gpsNav.pvtUpdatedMs = now;
  gpsNav.iTowMs = readUint32Le(payload);
  gpsNav.year = readUint16Le(payload + 4);
  gpsNav.month = payload[6];
  gpsNav.day = payload[7];
  gpsNav.hour = payload[8];
  gpsNav.minute = payload[9];
  gpsNav.second = payload[10];
  gpsNav.fixType = fixType;
  gpsNav.flags = flags;
  gpsNav.numSv = payload[23];
  gpsNav.lonDeg1e7 = readInt32Le(payload + 24);
  gpsNav.latDeg1e7 = readInt32Le(payload + 28);
  gpsNav.heightMslMm = readInt32Le(payload + 36);
  gpsNav.groundSpeedMmps = readInt32Le(payload + 60);
  gpsNav.headingMotionDeg1e5 = readInt32Le(payload + 64);
  gpsNav.locationValid = locationValid;
  gpsNav.altitudeValid = locationValid && (fixType == 3 || fixType == 4);
  gpsNav.speedValid = locationValid;
  gpsNav.courseValid = locationValid;
  gpsNav.dateValid = (valid & 0x01) != 0;
  gpsNav.timeValid = (valid & 0x02) != 0;

  ++gpsUbxPvtPacketCount;
  gpsMainDirty = true;
}

void updateGpsFromNavDop(const uint8_t *payload, uint16_t payloadLength) {
  if (payloadLength < 18) {
    return;
  }

  gpsNav.hDopCentis = readUint16Le(payload + 12);
  gpsNav.dopUpdatedMs = millis();
  gpsNav.dopValid = true;
  ++gpsUbxDopPacketCount;
}

void updateGpsFromNavSat(const uint8_t *payload, uint16_t payloadLength) {
  if (payloadLength < kGpsUbxNavSatHeaderLength) {
    return;
  }

  const uint8_t numSvs = payload[5];
  const uint16_t expectedLength =
      static_cast<uint16_t>(kGpsUbxNavSatHeaderLength) +
      static_cast<uint16_t>(numSvs) * kGpsUbxNavSatSvLength;
  if (payloadLength < expectedLength) {
    return;
  }

  uint8_t visibleCount = 0;
  uint8_t usedCount = 0;
  uint8_t bestCno = 0;
  for (uint8_t i = 0; i < numSvs; ++i) {
    const uint16_t offset =
        static_cast<uint16_t>(kGpsUbxNavSatHeaderLength) +
        static_cast<uint16_t>(i) * kGpsUbxNavSatSvLength;
    const uint8_t cno = payload[offset + 2];
    const uint32_t flags = readUint32Le(payload + offset + 8);

    if (cno > 0) {
      ++visibleCount;
      if (cno > bestCno) {
        bestCno = cno;
      }
    }
    if ((flags & 0x01) != 0) {
      ++usedCount;
    }
  }

  gpsNavSat.numSvs = numSvs;
  gpsNavSat.visibleCount = visibleCount;
  gpsNavSat.usedCount = usedCount;
  gpsNavSat.bestCno = bestCno;
  gpsNavSat.updatedMs = millis();
  gpsNavSat.valid = true;
  ++gpsUbxSatPacketCount;
}

void handleGpsUbxFrame(uint8_t messageClass,
                       uint8_t messageId,
                       const uint8_t *payload,
                       uint16_t payloadLength,
                       bool tooLong) {
  ++gpsUbxValidPacketCount;

  if (tooLong) {
    ++gpsUbxTooLongPacketCount;
    return;
  }

  if (messageClass == kGpsUbxClassNav && messageId == kGpsUbxIdNavPvt) {
    updateGpsFromNavPvt(payload, payloadLength);
    return;
  }

  if (messageClass == kGpsUbxClassNav && messageId == kGpsUbxIdNavDop) {
    updateGpsFromNavDop(payload, payloadLength);
    return;
  }

  if (messageClass == kGpsUbxClassNav && messageId == kGpsUbxIdNavSat) {
    updateGpsFromNavSat(payload, payloadLength);
    return;
  }

  ++gpsUbxUnexpectedPacketCount;
}

void processGpsUbxByte(uint8_t value) {
  ++gpsUbxBytesProcessed;

  switch (gpsUbxStreamParser.state) {
    case 0:
      gpsUbxStreamParser.state = (value == 0xB5) ? 1 : 0;
      break;
    case 1:
      if (value == 0x62) {
        gpsUbxStreamParser.state = 2;
      } else {
        gpsUbxStreamParser.state = (value == 0xB5) ? 1 : 0;
      }
      break;
    case 2:
      gpsUbxStreamParser.messageClass = value;
      gpsUbxStreamParser.checksumA = 0;
      gpsUbxStreamParser.checksumB = 0;
      updateGpsUbxChecksum(value,
                           gpsUbxStreamParser.checksumA,
                           gpsUbxStreamParser.checksumB);
      gpsUbxStreamParser.state = 3;
      break;
    case 3:
      gpsUbxStreamParser.messageId = value;
      updateGpsUbxChecksum(value,
                           gpsUbxStreamParser.checksumA,
                           gpsUbxStreamParser.checksumB);
      gpsUbxStreamParser.state = 4;
      break;
    case 4:
      gpsUbxStreamParser.payloadLength = value;
      updateGpsUbxChecksum(value,
                           gpsUbxStreamParser.checksumA,
                           gpsUbxStreamParser.checksumB);
      gpsUbxStreamParser.state = 5;
      break;
    case 5:
      gpsUbxStreamParser.payloadLength |= static_cast<uint16_t>(value) << 8;
      gpsUbxStreamParser.payloadIndex = 0;
      gpsUbxStreamParser.tooLong =
          gpsUbxStreamParser.payloadLength > kGpsUbxMaxPayloadLength;
      updateGpsUbxChecksum(value,
                           gpsUbxStreamParser.checksumA,
                           gpsUbxStreamParser.checksumB);
      gpsUbxStreamParser.state =
          (gpsUbxStreamParser.payloadLength == 0) ? 7 : 6;
      break;
    case 6:
      updateGpsUbxChecksum(value,
                           gpsUbxStreamParser.checksumA,
                           gpsUbxStreamParser.checksumB);
      if (!gpsUbxStreamParser.tooLong) {
        gpsUbxStreamParser.payload[gpsUbxStreamParser.payloadIndex] = value;
      }
      ++gpsUbxStreamParser.payloadIndex;
      if (gpsUbxStreamParser.payloadIndex >= gpsUbxStreamParser.payloadLength) {
        gpsUbxStreamParser.state = 7;
      }
      break;
    case 7:
      gpsUbxStreamParser.expectedChecksumA = value;
      gpsUbxStreamParser.state = 8;
      break;
    case 8:
      if (gpsUbxStreamParser.expectedChecksumA == gpsUbxStreamParser.checksumA &&
          value == gpsUbxStreamParser.checksumB) {
        handleGpsUbxFrame(gpsUbxStreamParser.messageClass,
                          gpsUbxStreamParser.messageId,
                          gpsUbxStreamParser.payload,
                          gpsUbxStreamParser.payloadLength,
                          gpsUbxStreamParser.tooLong);
      } else {
        ++gpsUbxChecksumFailureCount;
      }
      gpsUbxStreamParser.state = 0;
      break;
    default:
      gpsUbxStreamParser.state = 0;
      break;
  }
}

void updateGpsBaudProbe(uint32_t now) {
  const uint32_t pvtPackets = gpsUbxPvtPacketCount;
  if (pvtPackets != lastGpsBaudProbePvtPackets) {
    lastGpsBaudProbePvtPackets = pvtPackets;
    lastGpsBaudProbeMs = now;
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
  configureGpsRuntimeUbxMessages("baud probe", false);
  gpsCurrentBaudRateIndex =
      (gpsCurrentBaudRateIndex + 1) % kGpsBaudRateCount;
}

uint32_t gpsTimeFromHourStart() {
  if (!isGpsTimeFresh()) {
    return 0;
  }

  return static_cast<uint32_t>(gpsNav.minute) * 30000UL +
         static_cast<uint32_t>(gpsNav.second) * 500UL +
         (gpsNav.iTowMs % 1000UL) / 2UL;
}

uint32_t gpsDateHourValue() {
  if (!isGpsDateFresh() || !isGpsTimeFresh()) {
    return 0;
  }

  const uint16_t year =
      static_cast<uint16_t>(constrain(gpsNav.year, 2000, 2099));
  const uint8_t month =
      static_cast<uint8_t>(constrain(gpsNav.month, 1, 12));
  const uint8_t day =
      static_cast<uint8_t>(constrain(gpsNav.day, 1, 31));
  const uint8_t hour = min<uint8_t>(gpsNav.hour, 23);

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
  if (!isGpsLocationFresh()) {
    return 0;
  }

  const uint8_t carrierSolution = (gpsNav.flags >> 6) & 0x03;
  if (carrierSolution > 0) {
    return 3;
  }

  if ((gpsNav.flags & 0x02) != 0) {
    return 2;
  }

  return 1;
}

uint8_t gpsLockedSatellites() {
  if (!isGpsPvtFresh()) {
    return 0x3F;
  }

  return min<uint8_t>(gpsNav.numSv, 0x3E);
}

uint8_t gpsHdopRaw() {
  if (!isGpsDopFresh()) {
    return 0xFF;
  }

  const long hdopTenths = (static_cast<long>(gpsNav.hDopCentis) + 5L) / 10L;
  return static_cast<uint8_t>(constrain(hdopTenths, 0L, 0xFEL));
}

uint16_t gpsAltitudeRaw() {
  if (!isGpsAltitudeFresh()) {
    return 0xFFFF;
  }

  const double meters = static_cast<double>(gpsNav.heightMslMm) / 1000.0;
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

  const double speedKmph =
      max(0.0, static_cast<double>(gpsNav.groundSpeedMmps) * 0.0036);
  if (speedKmph <= 655.35) {
    return static_cast<uint16_t>(llround(speedKmph * 100.0)) & 0x7FFF;
  }

  return (static_cast<uint16_t>(llround(speedKmph * 10.0)) & 0x7FFF) | 0x8000;
}

uint16_t gpsBearingRaw() {
  if (!isGpsCourseFresh()) {
    return 0xFFFF;
  }

  double bearingDegrees =
      fmod(static_cast<double>(gpsNav.headingMotionDeg1e5) / 100000.0, 360.0);
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
    writeUint32Be(packet + 4, static_cast<uint32_t>(gpsNav.latDeg1e7));
    writeUint32Be(packet + 8, static_cast<uint32_t>(gpsNav.lonDeg1e7));
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

bool canSendBleNotification(NimBLECharacteristic *characteristic) {
  if (!bleClientConnected ||
      characteristic == nullptr ||
      millis() - bleConnectedAtMs < kBleReconnectQuietMs) {
    return false;
  }

  if (characteristic == canMainCharacteristic) {
    return canMainSubscribed;
  }
  if (characteristic == gpsMainCharacteristic) {
    return gpsMainSubscribed;
  }
  if (characteristic == gpsTimeCharacteristic) {
    return gpsTimeSubscribed;
  }
  return false;
}

void notifyBleCharacteristic(NimBLECharacteristic *characteristic) {
  if (characteristic->notify()) {
    ++bleNotifyOkCount;
  } else {
    ++bleNotifyFailCount;
  }
}

void updateGpsCharacteristicValues(bool notify) {
  syncGpsDateHourValue();

  if (gpsTimeCharacteristic != nullptr && gpsTimeDirty) {
    uint8_t packet[3] = {};
    buildGpsTimePacket(packet);
    gpsTimeCharacteristic->setValue(packet, sizeof(packet));
    if (notify && canSendBleNotification(gpsTimeCharacteristic)) {
      notifyBleCharacteristic(gpsTimeCharacteristic);
    }
    gpsTimeDirty = false;
  }

  if (gpsMainCharacteristic != nullptr && gpsMainDirty) {
    uint8_t packet[20] = {};
    buildGpsMainPacket(packet);
    gpsMainCharacteristic->setValue(packet, sizeof(packet));
    if (notify && canSendBleNotification(gpsMainCharacteristic)) {
      notifyBleCharacteristic(gpsMainCharacteristic);
    }
    gpsMainDirty = false;
  }
}

void printGpsDebugLog(uint32_t now) {
  if (now - lastGpsDebugLogMs < kGpsDebugLogIntervalMs) {
    return;
  }

  lastGpsDebugLogMs = now;

  const uint32_t bytesProcessed = gpsUbxBytesProcessed;
  const uint32_t validUbxPackets = gpsUbxValidPacketCount;
  const uint32_t checksumFailures = gpsUbxChecksumFailureCount;
  const uint32_t pvtPackets = gpsUbxPvtPacketCount;
  const uint32_t dopPackets = gpsUbxDopPacketCount;
  const uint32_t satPackets = gpsUbxSatPacketCount;
  const uint32_t bytesDelta = bytesProcessed - lastGpsDebugBytesProcessed;
  const uint32_t validDelta = validUbxPackets - lastGpsDebugValidUbxPackets;
  const uint32_t checksumFailureDelta =
      checksumFailures - lastGpsDebugChecksumFailures;
  const uint32_t pvtDelta = pvtPackets - lastGpsDebugPvtPackets;
  const uint32_t dopDelta = dopPackets - lastGpsDebugDopPackets;
  const uint32_t satDelta = satPackets - lastGpsDebugSatPackets;
  const uint16_t dollarCount = gpsDebugDollarCount;
  const uint16_t starCount = gpsDebugStarCount;
  const uint16_t lineFeedCount = gpsDebugLineFeedCount;
  const uint16_t ubxHeaderCount = gpsDebugUbxHeaderCount;

  lastGpsDebugBytesProcessed = bytesProcessed;
  lastGpsDebugValidUbxPackets = validUbxPackets;
  lastGpsDebugChecksumFailures = checksumFailures;
  lastGpsDebugPvtPackets = pvtPackets;
  lastGpsDebugDopPackets = dopPackets;
  lastGpsDebugSatPackets = satPackets;

  char satellites[8] = {};
  char visibleSats[8] = {};
  char listedSats[8] = {};
  char usedNavSats[8] = {};
  char bestCno[8] = {};
  char hdop[8] = {};
  char latitude[16] = {};
  char longitude[16] = {};
  char speed[12] = {};
  char course[12] = {};
  formatGpsUnsigned(satellites,
                    sizeof(satellites),
                    isGpsPvtFresh(),
                    gpsNav.numSv);
  formatGpsUnsigned(visibleSats,
                    sizeof(visibleSats),
                    isGpsSatFresh(),
                    gpsNavSat.visibleCount);
  formatGpsUnsigned(listedSats,
                    sizeof(listedSats),
                    isGpsSatFresh(),
                    gpsNavSat.numSvs);
  formatGpsUnsigned(usedNavSats,
                    sizeof(usedNavSats),
                    isGpsSatFresh(),
                    gpsNavSat.usedCount);
  formatGpsUnsigned(bestCno,
                    sizeof(bestCno),
                    isGpsSatFresh() && gpsNavSat.visibleCount > 0,
                    gpsNavSat.bestCno);
  formatGpsFloat(hdop,
                 sizeof(hdop),
                 isGpsDopFresh(),
                 static_cast<double>(gpsNav.hDopCentis) / 100.0,
                 1);
  formatGpsFloat(latitude,
                 sizeof(latitude),
                 isGpsLocationFresh(),
                 static_cast<double>(gpsNav.latDeg1e7) / 10000000.0,
                 7);
  formatGpsFloat(longitude,
                 sizeof(longitude),
                 isGpsLocationFresh(),
                 static_cast<double>(gpsNav.lonDeg1e7) / 10000000.0,
                 7);
  formatGpsFloat(speed,
                 sizeof(speed),
                 isGpsSpeedFresh(),
                 static_cast<double>(gpsNav.groundSpeedMmps) * 0.0036,
                 2);
  formatGpsFloat(course,
                 sizeof(course),
                 isGpsCourseFresh(),
                 static_cast<double>(gpsNav.headingMotionDeg1e5) / 100000.0,
                 2);

  const char *state =
      gpsDebugState(bytesDelta,
                    validDelta,
                    checksumFailureDelta,
                    pvtDelta,
                    dollarCount);
  Serial.printf(
      "GPS diag state=%s RX=GPIO%u baud=%lu bytes=%lu (+%lu), ubx=%lu (+%lu), pvt=%lu (+%lu), dop=%lu (+%lu), sat=%lu (+%lu), checksum_fail=%lu (+%lu), too_long=%lu, other=%lu, fix_type=%u, used=%s, visible=%s, listed=%s, used_nav=%s, best_cno=%s, hdop=%s, lat=%s, lon=%s, speed=%s km/h, course=%s deg, $=%u, *=%u, lf=%u, headers=%u, raw=\"%s\", hex=\"%s\"\n",
      state,
      kGpsRxPin,
      static_cast<unsigned long>(gpsCurrentBaudRate),
      static_cast<unsigned long>(bytesProcessed),
      static_cast<unsigned long>(bytesDelta),
      static_cast<unsigned long>(validUbxPackets),
      static_cast<unsigned long>(validDelta),
      static_cast<unsigned long>(pvtPackets),
      static_cast<unsigned long>(pvtDelta),
      static_cast<unsigned long>(dopPackets),
      static_cast<unsigned long>(dopDelta),
      static_cast<unsigned long>(satPackets),
      static_cast<unsigned long>(satDelta),
      static_cast<unsigned long>(checksumFailures),
      static_cast<unsigned long>(checksumFailureDelta),
      static_cast<unsigned long>(gpsUbxTooLongPacketCount),
      static_cast<unsigned long>(gpsUbxUnexpectedPacketCount),
      gpsNav.fixType,
      satellites,
      visibleSats,
      listedSats,
      usedNavSats,
      bestCno,
      hdop,
      latitude,
      longitude,
      speed,
      course,
      dollarCount,
      starCount,
      lineFeedCount,
      ubxHeaderCount,
      gpsDebugRawSample,
      gpsDebugHexSample);

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
    const uint8_t value = static_cast<uint8_t>(gpsSerial.read());
    captureGpsDebugByte(static_cast<char>(value));
    processGpsUbxByte(value);
  }

  updateGpsFreshnessState();

  if (gpsMainDirty || gpsTimeDirty) {
    updateGpsCharacteristicValues(true);
  }

  printGpsDebugLog(now);
  updateGpsBaudProbe(now);
}

bool shouldSendPid(uint32_t pid) {
  return allowAllPids || (pid == kTelemetryPid && telemetryPidAllowed);
}

bool publishCanPayload(uint32_t pid,
                       const uint8_t *payload,
                       size_t payloadLength) {
  if (!canSendBleNotification(canMainCharacteristic) ||
      canMainCharacteristic == nullptr ||
      payloadLength == 0 ||
      payloadLength > kCanMaxPayloadLength ||
      !shouldSendPid(pid)) {
    return false;
  }

  uint8_t packet[kCanPacketPidLength + kCanMaxPayloadLength] = {};
  writeUint32Le(packet, pid);
  memcpy(packet + kCanPacketPidLength, payload, payloadLength);

  canMainCharacteristic->setValue(packet, kCanPacketPidLength + payloadLength);
  notifyBleCharacteristic(canMainCharacteristic);
  return true;
}

uint16_t normalizeNotifyInterval(uint16_t requestedIntervalMs) {
  return max<uint16_t>(kMinNotifyIntervalMs, requestedIntervalMs);
}

void buildTelemetryPayload(uint8_t *payload,
                           float pressureBar,
                           float throttlePercent,
                           float afr,
                           float batteryVolts) {
  const uint16_t pressureCentibar = pressureBarToCentibar(pressureBar);
  const uint16_t throttleCentipercent = static_cast<uint16_t>(
      lroundf(constrain(throttlePercent, 0.0F, 100.0F) * 100.0F));
  const uint16_t afrCentivalue = afrToCentiAfr(afr);
  const uint16_t batteryMillivolts = voltsToMillivolts(batteryVolts);

  memset(payload, 0, kTelemetryPayloadLength);
  writeUint16Be(payload + kTelemetryBrakePressureOffset, pressureCentibar);
  writeUint16Be(payload + kTelemetryThrottlePositionOffset,
                throttleCentipercent);
  writeUint16Be(payload + kTelemetryAfrOffset, afrCentivalue);
  writeUint16Be(payload + kTelemetryBatteryVoltageOffset,
                batteryMillivolts);
}

void publishTelemetry(float pressureBar,
                      float throttlePercent,
                      float afr,
                      float batteryVolts) {
  uint8_t payload[kTelemetryPayloadLength] = {};
  buildTelemetryPayload(payload, pressureBar, throttlePercent, afr,
                        batteryVolts);

  publishCanPayload(kTelemetryPid, payload, sizeof(payload));
}

void printBleTxLog(uint32_t now,
                   float pressureBar,
                   float throttlePercent,
                   float batteryVolts,
                   float afr) {
  if (now - lastBleTxLogMs >= kBleTxLogIntervalMs) {
    lastBleTxLogMs = now;
    Serial.printf(
        "BLE TX brake=%5.1f bar, throttle=%5.1f%%, battery=%6.3fV, AFR=%5.2f, notify_ok=%lu fail=%lu\n",
        pressureBar,
        throttlePercent,
        batteryVolts,
        afr,
        static_cast<unsigned long>(bleNotifyOkCount),
        static_cast<unsigned long>(bleNotifyFailCount));
  }
}

const char *describeBleDisconnectReason(int reason) {
  switch (reason) {
    case 0x005:
      return "authentication failure (stale phone bond?)";
    case 0x00F:
      return "insufficient encryption";
    case 0x013:
      return "remote user terminated";
    case 0x016:
      return "local host terminated";
    case 0x018:
      return "pairing not supported";
    case 0x022:
      return "link layer response timeout";
    case 0x03D:
      return "MIC failure (bond key mismatch?)";
    case 0x03E:
      return "connection failed to establish";
    default:
      return nullptr;
  }
}

bool isLikelyStaleBondDisconnect(int reason) {
  return reason == 0x005 || reason == 0x00F || reason == 0x03D;
}

void logBleStaleBondHint() {
  Serial.println(
      "BLE hint: forget \"RaceExporter\" in phone Bluetooth settings, then "
      "reconnect in RaceChrono");
}

void configureRaceChronoBleSecurity() {
  NimBLEDevice::setSecurityAuth(false, false, false);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEDevice::setSecurityInitKey(0);
  NimBLEDevice::setSecurityRespKey(0);

  const int bondCount = NimBLEDevice::getNumBonds();
  if (bondCount > 0) {
    Serial.printf("BLE clearing %d stale bond(s) from NVS\n", bondCount);
    NimBLEDevice::deleteAllBonds();
  }

  Serial.println("BLE security: open connection, bonding disabled");
}

class RaceChronoDeviceCallbacks : public NimBLEDeviceCallbacks {
  int onStoreStatus(struct ble_store_status_event *event, void *) override {
    if (event == nullptr) {
      return 0;
    }

    if (event->event_code == BLE_STORE_EVENT_FULL ||
        event->event_code == BLE_STORE_EVENT_OVERFLOW) {
      Serial.println("BLE bond store full; clearing bonds");
      NimBLEDevice::deleteAllBonds();
    }
    return 0;
  }
};

class NotifySubscriptionCallbacks : public NimBLECharacteristicCallbacks {
 public:
  explicit NotifySubscriptionCallbacks(bool *subscribedFlag)
      : subscribedFlag_(subscribedFlag) {}

  void onSubscribe(NimBLECharacteristic *characteristic, NimBLEConnInfo &,
                   uint16_t subValue) override {
    *subscribedFlag_ = (subValue & 0x0001) != 0;
    Serial.printf("BLE subscribe: char=%s value=%u\n",
                  characteristic->getUUID().toString().c_str(),
                  subValue);
  }

 private:
  bool *subscribedFlag_;
};

class RaceChronoServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *, NimBLEConnInfo &connInfo) override {
    bleClientConnected = true;
    bleConnectedAtMs = millis();
    lastCanNotifyMs = bleConnectedAtMs;
    telemetryDirty = true;
    Serial.printf(
        "BLE connected: addr=%s, interval=%u units, latency=%u, timeout=%u units, "
        "mtu=%u, bonded=%s, encrypted=%s\n",
        connInfo.getAddress().toString().c_str(),
        connInfo.getConnInterval(),
        connInfo.getConnLatency(),
        connInfo.getConnTimeout(),
        connInfo.getMTU(),
        connInfo.isBonded() ? "yes" : "no",
        connInfo.isEncrypted() ? "yes" : "no");
  }

  void onMTUChange(uint16_t mtu, NimBLEConnInfo &connInfo) override {
    Serial.printf("BLE MTU updated: addr=%s mtu=%u\n",
                  connInfo.getAddress().toString().c_str(),
                  mtu);
  }

  void onConnParamsUpdate(NimBLEConnInfo &connInfo) override {
    Serial.printf(
        "BLE params updated: addr=%s interval=%u units, latency=%u, timeout=%u units\n",
        connInfo.getAddress().toString().c_str(),
        connInfo.getConnInterval(),
        connInfo.getConnLatency(),
        connInfo.getConnTimeout());
  }

  void onAuthenticationComplete(NimBLEConnInfo &connInfo) override {
    Serial.printf(
        "BLE auth complete: bonded=%s encrypted=%s authenticated=%s addr=%s\n",
        connInfo.isBonded() ? "yes" : "no",
        connInfo.isEncrypted() ? "yes" : "no",
        connInfo.isAuthenticated() ? "yes" : "no",
        connInfo.getAddress().toString().c_str());
    if (!connInfo.isEncrypted()) {
      Serial.println(
          "BLE auth failed: phone may have a stale bond for RaceExporter");
      logBleStaleBondHint();
    }
  }

  void onDisconnect(NimBLEServer *server, NimBLEConnInfo &, int reason) override {
    bleClientConnected = false;
    canMainSubscribed = false;
    gpsMainSubscribed = false;
    gpsTimeSubscribed = false;
    server->startAdvertising();
    const char *reasonText = describeBleDisconnectReason(reason);
    if (reasonText != nullptr) {
      Serial.printf(
          "BLE disconnected: reason=0x%03X (%s), advertising restarted\n",
          reason,
          reasonText);
    } else {
      Serial.printf("BLE disconnected: reason=0x%03X, advertising restarted\n",
                    reason);
    }
    if (isLikelyStaleBondDisconnect(reason)) {
      logBleStaleBondHint();
    }
  }
};

class RaceChronoCanFilterCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &) override {
    const NimBLEAttValue value = characteristic->getValue();
    if (value.length() == 0) {
      return;
    }

    const uint8_t *data = value.data();
    const size_t length = value.length();
    const uint8_t commandId = data[0];

    if (commandId == 0) {
      allowAllPids = false;
      telemetryPidAllowed = false;
      Serial.println("RaceChrono filter: deny all PIDs");
      return;
    }

    if (commandId == 1 && length >= 3) {
      allowAllPids = true;
      telemetryPidAllowed = true;
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
      const bool knownPid = pid == kTelemetryPid;
      if (knownPid) {
        telemetryPidAllowed = true;
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
  static RaceChronoDeviceCallbacks deviceCallbacks;
  NimBLEDevice::setDeviceCallbacks(&deviceCallbacks);
  NimBLEDevice::init(kDeviceName);
  configureRaceChronoBleSecurity();

  NimBLEServer *bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new RaceChronoServerCallbacks());

  NimBLEService *raceChronoService =
      bleServer->createService(kRaceChronoServiceUuid);

  canMainCharacteristic = raceChronoService->createCharacteristic(
      kCanMainCharacteristicUuid,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  canMainCharacteristic->setCallbacks(
      new NotifySubscriptionCallbacks(&canMainSubscribed));

  NimBLECharacteristic *canFilterCharacteristic =
      raceChronoService->createCharacteristic(kCanFilterCharacteristicUuid,
                                              NIMBLE_PROPERTY::WRITE);
  canFilterCharacteristic->setCallbacks(new RaceChronoCanFilterCallbacks());

  gpsMainCharacteristic = raceChronoService->createCharacteristic(
      kGpsMainCharacteristicUuid,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  gpsMainCharacteristic->setCallbacks(
      new NotifySubscriptionCallbacks(&gpsMainSubscribed));

  gpsTimeCharacteristic = raceChronoService->createCharacteristic(
      kGpsTimeCharacteristicUuid,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  gpsTimeCharacteristic->setCallbacks(
      new NotifySubscriptionCallbacks(&gpsTimeSubscribed));

  uint8_t initialPacket[kCanPacketPidLength + kTelemetryPayloadLength] = {};
  writeUint32Le(initialPacket, kTelemetryPid);
  buildTelemetryPayload(initialPacket + kCanPacketPidLength,
                        brakePressureBar,
                        throttlePercent,
                        afr,
                        batteryVolts);
  canMainCharacteristic->setValue(initialPacket, sizeof(initialPacket));
  updateGpsCharacteristicValues(false);

  NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(kRaceChronoServiceUuid);
  advertising->enableScanResponse(true);
  const bool nameSet = advertising->setName(kDeviceName);
  advertising->setPreferredParams(kBleConnMinIntervalUnits,
                                  kBleConnMaxIntervalUnits);
  const bool advertisingStarted = NimBLEDevice::startAdvertising();
  Serial.printf("BLE advertising start: started=%s active=%s name=%s\n",
                advertisingStarted ? "yes" : "no",
                advertising->isAdvertising() ? "yes" : "no",
                nameSet ? "yes" : "no");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  ledcSetup(kLedPwmChannel, kLedPwmFrequencyHz, kLedPwmResolutionBits);
  ledcAttachPin(kLedPin, kLedPwmChannel);
  setLed(false);

  Serial.println();
  Serial.println("race-exporter-esp32 started");
  Serial.printf(
      "RaceChrono BLE CAN PID 0x%08lX: telemetry, brake@0 centibar, throttle@2 centipercent, AFR@4 centi-AFR, battery@6 millivolts\n",
      static_cast<unsigned long>(kTelemetryPid));
  Serial.printf("RaceChrono BLE GPS: native GPS feature on characteristics 0x0003/0x0004\n");

  loadThrottleCalibration();
  loadBrakeCalibration();
  loadGpsConfigState();
  startGps();
  startBatteryAdc();
  startAdc();
  calibrateBrakeZeroAtStartup();
  calibrateThrottle();
  startRaceChronoBle();
  Serial.printf("BLE advertising as \"%s\"\n", kDeviceName);
}

void loop() {
  const uint32_t now = millis();

  serviceAds(now);
  updateBatteryFromAdc(now);
  updateGpsFromSerial(now);
  updateLedIdleBlink(now);

  if (canSendBleNotification(canMainCharacteristic) && telemetryDirty &&
      now - lastCanNotifyMs >= notifyIntervalMs) {
    // Фазовый сдвиг базы на интервал, а не на now, чтобы джиттер цикла не
    // накапливался и средняя частота держалась около 1000/notifyIntervalMs.
    lastCanNotifyMs += notifyIntervalMs;
    // Если отстали больше чем на интервал (длинный столл), ресинхронизируемся
    // к now, чтобы не выдать пачку догоняющих нотификаций.
    if (now - lastCanNotifyMs >= notifyIntervalMs) {
      lastCanNotifyMs = now;
    }
    publishTelemetry(brakePressureBar, throttlePercent, afr, batteryVolts);
    telemetryDirty = false;
    printBleTxLog(now, brakePressureBar, throttlePercent, batteryVolts, afr);
  }

  printAdcReadings(now);
}
