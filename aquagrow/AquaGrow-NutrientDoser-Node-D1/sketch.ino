/**
 * @file sketch.ino
 * @brief AquaGrow NutrientDoser Node D1 simulation using Modest IoT Nano Framework.
 * @author Iker Gabriel Barturen Panez
 *
 * All application classes remain in this sketch, as required for the exercise.
 * Sensor events reach the Device coordinator; the coordinator sends commands
 * to the servo and LCD actuators. The framework scheduler runs in the background.
 */
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ModestIoT.h>
#include <Wire.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/** @brief Named hardware, calibration, timing, and UI constants. */
namespace AquaGrowConfig {
constexpr uint8_t kTankTriggerPin = 5;
constexpr uint8_t kTankEchoPin = 18;
constexpr uint8_t kProximityTriggerPin = 19;
constexpr uint8_t kProximityEchoPin = 23;
constexpr uint8_t kLoadCellDataPin = 26;
constexpr uint8_t kLoadCellClockPin = 27;
constexpr uint8_t kServoPin = 13;
constexpr uint8_t kLcdAddress = 0x27;
constexpr uint8_t kLcdSdaPin = 21;
constexpr uint8_t kLcdSclPin = 22;
constexpr uint8_t kLcdColumns = 16;
constexpr uint8_t kLcdRows = 2;
constexpr uint8_t kTelemetryQueueDepth = 4;

constexpr float kTankFullDistanceCm = 6.0f;
constexpr float kTankEmptyDistanceCm = 40.0f;
constexpr float kTankCapacityMl = 5000.0f;
constexpr float kGardenerThresholdCm = 25.0f;
constexpr float kScaleMinimumGrams = 0.0f;
constexpr float kScaleMaximumGrams = 5000.0f;
constexpr long kScaleMinimumRaw = 0;
constexpr long kScaleMaximumRaw = 2100;  // Wokwi 5 kg HX711 raw range.
constexpr uint8_t kHx711BitCount = 24;
constexpr int32_t kHx711SignBit = 0x800000L;
constexpr int32_t kHx711RawMask = 0xFFFFFFL;
constexpr int kSuppressEventId = -1;
constexpr unsigned long kHx711WaitYieldMs = 1UL;
constexpr unsigned long kSerialBaudRate = 115200UL;
constexpr uint8_t kHx711GainPulses = 1;
constexpr uint8_t kHx711ClockDelayUs = 1;
constexpr unsigned long kHx711ReadyTimeoutMs = 100UL;
constexpr uint32_t kScaleSampleMs = 500UL;
constexpr uint32_t kScaleMinimumSampleMs = 100UL;
constexpr float kWeightChangeEpsilonGrams = 0.1f;
constexpr float kStartDosingBelowGrams = 500.0f;
constexpr float kStopDosingAtGrams = 1500.0f;
constexpr unsigned long kSchedulerTickMs = 250UL;
constexpr unsigned long kTelemetryIntervalMs = 5000UL;
constexpr int kReviewEventId = 900;

constexpr char kCompanyBanner[] = "AquaGrow | NutrientDoser Node D1";
constexpr char kDeveloperBanner[] = "Developer: Iker Gabriel Barturen Panez";
constexpr char kStudentCodeBanner[] = "Student code: U202312629";
constexpr char kTankPendingLine[] = "Tank:---% Wait";
constexpr char kTrayPendingLine[] = "Tray:----g Wait";
constexpr char kTankLineFormat[] = "Tank:%3u%% %s";
constexpr char kTrayLineFormat[] = "Tray:%4ug %s";
constexpr char kEmptyLabel[] = "Empty";
constexpr char kBlankLabel[] = "";
constexpr char kWaitingLabel[] = "WAIT";
constexpr char kReadyLabel[] = "IDLE";
constexpr char kDosingLabel[] = "DOSE";
constexpr char kTargetLabel[] = "DONE";
constexpr char kModeStarting[] = "STARTING";
constexpr char kModeWaiting[] = "WAITING_FOR_SENSORS";
constexpr char kModeReady[] = "READY";
constexpr char kModeDosing[] = "DOSING";
constexpr char kModeEmpty[] = "EMPTY";
constexpr char kModeTarget[] = "TARGET_REACHED";
constexpr char kTelemetryPrefix[] = "TRACKING ";
constexpr char kDeviceId[] = "AQUAGROW-D1";

constexpr char kJsonDeviceId[] = "deviceId";
constexpr char kJsonUptimeMs[] = "uptimeMs";
constexpr char kJsonTankPercent[] = "tankPercent";
constexpr char kJsonTankVolumeMl[] = "tankVolumeMl";
constexpr char kJsonTankEmpty[] = "tankEmpty";
constexpr char kJsonTrayWeightGrams[] = "trayWeightGrams";
constexpr char kJsonGardenerNear[] = "gardenerNear";
constexpr char kJsonValveOpen[] = "valveOpen";
constexpr char kJsonMode[] = "mode";
constexpr char kJsonDosingCycles[] = "dosingCycles";
constexpr char kJsonProximityEvents[] = "proximityEvents";
constexpr char kJsonModeTransitions[] = "modeTransitions";
}

/** @brief Operational states evaluated by the device coordinator. */
enum class AquaGrowMode : uint8_t {
  STARTING,
  WAITING_FOR_SENSORS,
  READY,
  DOSING,
  EMPTY,
  TARGET_REACHED
};

/**
 * @brief Immutable snapshot of device-maker tracking metrics.
 * @details Uses the ModestIoT TelemetryPackage contract. In this offline
 * prototype, serialized JSON is written to Serial; no broker is specified.
 */
class AquaGrowTrackingPackage final : public TelemetryPackage {
 public:
  AquaGrowTrackingPackage(unsigned long uptimeMs, unsigned tankPercent,
                          unsigned tankVolumeMl, bool tankEmpty,
                          unsigned trayWeightGrams, bool gardenerNear,
                          bool valveOpen, const char* mode,
                          unsigned long dosingCycles,
                          unsigned long proximityEvents,
                          unsigned long modeTransitions)
      : uptimeMs_(uptimeMs), tankPercent_(tankPercent),
        tankVolumeMl_(tankVolumeMl), tankEmpty_(tankEmpty),
        trayWeightGrams_(trayWeightGrams), gardenerNear_(gardenerNear),
        valveOpen_(valveOpen), mode_(mode), dosingCycles_(dosingCycles),
        proximityEvents_(proximityEvents), modeTransitions_(modeTransitions) {}

  /** @brief Populate an ArduinoJson document with the tracking snapshot. */
  void serialize(JsonDocument& destination) const override {
    using namespace AquaGrowConfig;
    destination[kJsonDeviceId] = kDeviceId;
    destination[kJsonUptimeMs] = uptimeMs_;
    destination[kJsonTankPercent] = tankPercent_;
    destination[kJsonTankVolumeMl] = tankVolumeMl_;
    destination[kJsonTankEmpty] = tankEmpty_;
    destination[kJsonTrayWeightGrams] = trayWeightGrams_;
    destination[kJsonGardenerNear] = gardenerNear_;
    destination[kJsonValveOpen] = valveOpen_;
    destination[kJsonMode] = mode_;
    destination[kJsonDosingCycles] = dosingCycles_;
    destination[kJsonProximityEvents] = proximityEvents_;
    destination[kJsonModeTransitions] = modeTransitions_;
  }

 private:
  const unsigned long uptimeMs_;
  const unsigned tankPercent_;
  const unsigned tankVolumeMl_;
  const bool tankEmpty_;
  const unsigned trayWeightGrams_;
  const bool gardenerNear_;
  const bool valveOpen_;
  const char* const mode_;
  const unsigned long dosingCycles_;
  const unsigned long proximityEvents_;
  const unsigned long modeTransitions_;
};

/**
 * @brief Reads a Wokwi 5 kg HX711 and emits ModestIoT measurement events.
 * @details The adapter stays in this sketch to meet the single-file rule.
 * Its raw range is calibrated to 0â€“5000 grams using Wokwi's 0â€“2100 scale.
 */
class AquaGrowLoadCellAmplifier final : public Sensor {
 public:
  AquaGrowLoadCellAmplifier(uint8_t dataPin, uint8_t clockPin,
                            EventHandler* parentHandler)
      : Sensor(dataPin, parentHandler), clockPin_(clockPin) {
    pinMode(pin, INPUT);
    pinMode(clockPin_, OUTPUT);
    digitalWrite(clockPin_, LOW);
  }

  /** @brief Return the last valid tray mass in grams. */
  float getWeightInGrams() const { return weightGrams_; }

 protected:
  /** @brief Capture one 24-bit HX711 conversion and notify the Device. */
  void processEvent(Event& event) override {
    using namespace AquaGrowConfig;
    if (event.identifier != MEASURE_DATA_REQUESTED_EVENT_IDENTIFIER) {
      event.identifier = kSuppressEventId;
      return;
    }

    const unsigned long startedAt = millis();
    while (digitalRead(pin) != LOW) {
      if (millis() - startedAt >= kHx711ReadyTimeoutMs) {
        event.identifier = kSuppressEventId;
        return;
      }
      delay(kHx711WaitYieldMs);
    }

    int32_t raw = 0;
    for (uint8_t bit = 0; bit < kHx711BitCount; ++bit) {
      digitalWrite(clockPin_, HIGH);
      delayMicroseconds(kHx711ClockDelayUs);
      raw = (raw << 1) | digitalRead(pin);
      digitalWrite(clockPin_, LOW);
      delayMicroseconds(kHx711ClockDelayUs);
    }
    for (uint8_t pulse = 0; pulse < kHx711GainPulses; ++pulse) {
      digitalWrite(clockPin_, HIGH);
      delayMicroseconds(kHx711ClockDelayUs);
      digitalWrite(clockPin_, LOW);
      delayMicroseconds(kHx711ClockDelayUs);
    }

    if (raw & kHx711SignBit) raw |= ~kHx711RawMask;
    if (raw < kScaleMinimumRaw) raw = kScaleMinimumRaw;
    if (raw > kScaleMaximumRaw) raw = kScaleMaximumRaw;
    const float measuredGrams = static_cast<float>(raw) *
                                kScaleMaximumGrams /
                                static_cast<float>(kScaleMaximumRaw);
    if (!hasReading_ ||
        fabsf(measuredGrams - weightGrams_) > kWeightChangeEpsilonGrams) {
      weightGrams_ = measuredGrams;
      hasReading_ = true;
      event = Event(DATA_READ_EVENT_IDENTIFIER, pin);
    } else {
      event.identifier = kSuppressEventId;
    }
  }

  /** @brief Default sample period for framework versions with per-slot timing. */
  uint32_t getDefaultSamplingIntervalInMilliseconds() const {
    return AquaGrowConfig::kScaleSampleMs;
  }

  /** @brief Minimum safe HX711 sample period. */
  uint32_t getMinimalSamplingIntervalInMilliseconds() const {
    return AquaGrowConfig::kScaleMinimumSampleMs;
  }

 private:
  const uint8_t clockPin_;
  float weightGrams_ = 0.0f;
  bool hasReading_ = false;
};

/**
 * @brief Coordinates two HC-SR04 sensors, a 5 kg HX711, a servo valve,
 *        a 16x2 I2C LCD, and telemetry.
 * @details The ModestIoT scheduler polls each sensor and requests a periodic
 * state review. This allows loop() to remain inhibited.
 */
class AquaGrowDevice final : public Device {
 public:
  AquaGrowDevice()
      : Device(AquaGrowConfig::kSchedulerTickMs),
        tankSensor_(AquaGrowConfig::kTankTriggerPin,
                    AquaGrowConfig::kTankEchoPin, this),
        proximitySensor_(AquaGrowConfig::kProximityTriggerPin,
                         AquaGrowConfig::kProximityEchoPin, this),
        trayScale_(AquaGrowConfig::kLoadCellDataPin,
                   AquaGrowConfig::kLoadCellClockPin, this),
        dosingValve_(AquaGrowConfig::kServoPin),
        display_(AquaGrowConfig::kLcdAddress,
                 AquaGrowConfig::kLcdColumns,
                 AquaGrowConfig::kLcdRows) {}

  /** @brief Print identity, initialize the screen, and start background polling. */
  void begin() {
    using namespace AquaGrowConfig;
    Serial.begin(kSerialBaudRate);
    Serial.println(kCompanyBanner);
    Serial.println(kDeveloperBanner);
    Serial.println(kStudentCodeBanner);

    display_.handle(CharacterLcdDisplay::TURN_BACKLIGHT_ON_COMMAND);
    display_.setLineBuffer(0, kTankPendingLine);
    display_.setLineBuffer(1, kTrayPendingLine);
    display_.handle(CharacterLcdDisplay::UPDATE_TEXT_COMMAND);

    appendSensorToScheduler(&tankSensor_,
                            Sensor::MEASURE_DATA_REQUESTED_EVENT_IDENTIFIER);
    appendSensorToScheduler(&proximitySensor_,
                            Sensor::MEASURE_DATA_REQUESTED_EVENT_IDENTIFIER);
    appendSensorToScheduler(&trayScale_,
                            Sensor::MEASURE_DATA_REQUESTED_EVENT_IDENTIFIER);
    appendSensorToScheduler(this, kReviewEventId);
    initializeAsynchronousEngine(kTelemetryQueueDepth);
  }

  /** @brief Receive sensor facts and periodic review requests. */
  void on(Event event) override {
    using namespace AquaGrowConfig;
    if (event.identifier == Sensor::DATA_READ_EVENT_IDENTIFIER) {
      if (event.sourceId == kTankTriggerPin) {
        tankDistanceCm_ = tankSensor_.getDistanceInCentimeters();
        tankSeen_ = tankDistanceCm_ >= 0.0f;
      } else if (event.sourceId == kProximityTriggerPin) {
        const float distance = proximitySensor_.getDistanceInCentimeters();
        const bool nearNow = distance >= 0.0f &&
                             distance <= kGardenerThresholdCm;
        if (nearNow && !gardenerNear_) {
          ++proximityEvents_;
        }
        gardenerNear_ = nearNow;
        proximitySeen_ = distance >= 0.0f;
      } else if (event.sourceId == kLoadCellDataPin) {
        trayWeightGrams_ = trayScale_.getWeightInGrams();
        if (trayWeightGrams_ < kScaleMinimumGrams) {
          trayWeightGrams_ = kScaleMinimumGrams;
        }
        weightSeen_ = true;
      }
    } else if (event.identifier != kReviewEventId) {
      return;
    }
    evaluateAndPublish();
  }

 private:
  /** @brief Clamp the calibrated tank depth to a percentage. */
  unsigned tankPercentage() const {
    using namespace AquaGrowConfig;
    if (!tankSeen_) return 0;
    float percent = (kTankEmptyDistanceCm - tankDistanceCm_) *
                    100.0f /
                    (kTankEmptyDistanceCm - kTankFullDistanceCm);
    if (percent < 0.0f) percent = 0.0f;
    if (percent > 100.0f) percent = 100.0f;
    return static_cast<unsigned>(lroundf(percent));
  }

  /** @brief Select the next state, respecting empty-tank and target cutoffs. */
  AquaGrowMode decideMode() const {
    using namespace AquaGrowConfig;
    if (tankSeen_ && tankDistanceCm_ >= kTankEmptyDistanceCm) {
      return AquaGrowMode::EMPTY;
    }
    if (!tankSeen_ || !proximitySeen_ || !weightSeen_) {
      return AquaGrowMode::WAITING_FOR_SENSORS;
    }
    if (trayWeightGrams_ >= kStopDosingAtGrams) {
      return AquaGrowMode::TARGET_REACHED;
    }
    if (mode_ == AquaGrowMode::DOSING) {
      return AquaGrowMode::DOSING;
    }
    if (trayWeightGrams_ < kStartDosingBelowGrams && gardenerNear_) {
      return AquaGrowMode::DOSING;
    }
    return AquaGrowMode::READY;
  }

  /** @brief Translate an operational state into a stable telemetry label. */
  static const char* modeName(AquaGrowMode mode) {
    using namespace AquaGrowConfig;
    switch (mode) {
      case AquaGrowMode::STARTING: return kModeStarting;
      case AquaGrowMode::WAITING_FOR_SENSORS: return kModeWaiting;
      case AquaGrowMode::READY: return kModeReady;
      case AquaGrowMode::DOSING: return kModeDosing;
      case AquaGrowMode::EMPTY: return kModeEmpty;
      case AquaGrowMode::TARGET_REACHED: return kModeTarget;
    }
    return kModeWaiting;
  }

  /** @brief Return a compact state label that fits on the 16x2 LCD. */
  static const char* shortModeName(AquaGrowMode mode) {
    using namespace AquaGrowConfig;
    switch (mode) {
      case AquaGrowMode::DOSING: return kDosingLabel;
      case AquaGrowMode::TARGET_REACHED: return kTargetLabel;
      case AquaGrowMode::READY: return kReadyLabel;
      default: return kWaitingLabel;
    }
  }

  /** @brief Send intent to the servo actuator only when the valve changes. */
  void applyValveCommand(bool shouldOpen) {
    if (shouldOpen == valveOpen_) return;
    if (shouldOpen) {
      dosingValve_.handle(ServoMotor::LOCK_TO_MAXIMUM_LIMIT_COMMAND);
      ++dosingCycles_;
    } else {
      dosingValve_.handle(ServoMotor::LOCK_TO_MINIMUM_LIMIT_COMMAND);
    }
    valveOpen_ = shouldOpen;
  }

  /** @brief Refresh both LCD rows with level, weight, and empty state. */
  void renderDisplay() {
    using namespace AquaGrowConfig;
    char tankLine[kLcdColumns + 1];
    char trayLine[kLcdColumns + 1];

    if (tankSeen_) {
      snprintf(tankLine, sizeof(tankLine), kTankLineFormat,
               tankPercentage(),
               tankDistanceCm_ >= kTankEmptyDistanceCm
                   ? kEmptyLabel : kBlankLabel);
    } else {
      snprintf(tankLine, sizeof(tankLine), "%s", kTankPendingLine);
    }

    if (weightSeen_) {
      const float boundedWeight =
          trayWeightGrams_ > kScaleMaximumGrams
              ? kScaleMaximumGrams : trayWeightGrams_;
      snprintf(trayLine, sizeof(trayLine), kTrayLineFormat,
               static_cast<unsigned>(lroundf(boundedWeight)),
               shortModeName(mode_));
    } else {
      snprintf(trayLine, sizeof(trayLine), "%s", kTrayPendingLine);
    }

    display_.setLineBuffer(0, tankLine);
    display_.setLineBuffer(1, trayLine);
    display_.handle(CharacterLcdDisplay::UPDATE_TEXT_COMMAND);
  }

  /** @brief Emit a JSON snapshot for the Device Maker's tracking console. */
  void publishTracking(unsigned long now) {
    using namespace AquaGrowConfig;
    const unsigned percent = tankPercentage();
    const unsigned volumeMl = static_cast<unsigned>(
        lroundf(kTankCapacityMl * percent / 100.0f));
    const unsigned weightGrams = weightSeen_
        ? static_cast<unsigned>(lroundf(trayWeightGrams_))
        : 0U;
    AquaGrowTrackingPackage snapshot(
        now, percent, volumeMl,
        tankSeen_ && tankDistanceCm_ >= kTankEmptyDistanceCm,
        weightGrams, gardenerNear_, valveOpen_, modeName(mode_),
        dosingCycles_, proximityEvents_, modeTransitions_);
    Serial.print(kTelemetryPrefix);
    JsonDocument document;
    snapshot.serialize(document);
    serializeJson(document, Serial);
    Serial.println();
  }

  /** @brief Apply state transitions, update outputs, and report tracking. */
  void evaluateAndPublish() {
    using namespace AquaGrowConfig;
    const unsigned long now = millis();
    const AquaGrowMode nextMode = decideMode();
    const bool changed = nextMode != mode_;
    if (changed) {
      mode_ = nextMode;
      ++modeTransitions_;
    }
    applyValveCommand(mode_ == AquaGrowMode::DOSING);
    renderDisplay();
    if (changed || now - lastTelemetryMs_ >= kTelemetryIntervalMs) {
      publishTracking(now);
      lastTelemetryMs_ = now;
    }
  }

  UltrasonicSensor tankSensor_;
  UltrasonicSensor proximitySensor_;
  AquaGrowLoadCellAmplifier trayScale_;
  ServoMotor dosingValve_;
  CharacterLcdDisplay display_;

  bool tankSeen_ = false;
  bool proximitySeen_ = false;
  bool weightSeen_ = false;
  bool gardenerNear_ = false;
  bool valveOpen_ = false;
  float tankDistanceCm_ = -1.0f;
  float trayWeightGrams_ = 0.0f;
  AquaGrowMode mode_ = AquaGrowMode::STARTING;
  unsigned long dosingCycles_ = 0;
  unsigned long proximityEvents_ = 0;
  unsigned long modeTransitions_ = 0;
  unsigned long lastTelemetryMs_ = 0;
};

/** @brief Construct the one device after the Arduino runtime starts. */
void setup() {
  Wire.begin(AquaGrowConfig::kLcdSdaPin, AquaGrowConfig::kLcdSclPin);
  static AquaGrowDevice device;
  device.begin();
}

/** @brief Inhibit Arduino loop; ModestIoT tasks perform all work. */
void loop() {
  vTaskDelay(portMAX_DELAY);
}

