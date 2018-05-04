#include "config.hpp"


namespace config {

namespace debug {
const uint32_t baud = 2000000;
} //namespace debug

namespace imu {
const uint8_t i2cAddress = 0x68;
namespace lowPassFilter {
// EepromSetting<int8_t> common(SettingId::imuLpf_common, 2, 50, 0, 6, 1);
// namespace angularVelocity {
// EepromSetting<bool> state(SettingId::imuLpfAv_state, false, 55, 0, 1);
// EepromSetting<float> alpha(SettingId::imuLpfAv_alpha, 1, 60, 0.025, 1.0, 0.025); //3
// float oneMinusAlpha = 1 - alpha();
// } //namespace angularVelocity
// namespace acceleration {
// EepromSetting<bool> state(SettingId::imuLpfAcc_state, false, 65, 0, 1);
// EepromSetting<float> alpha(SettingId::imuLpfAcc_alpha, 1, 70, 0.025, 1.0, 0.025); //5
// float oneMinusAlpha = 1 - alpha();
// } //namespace acceleration
} //namespace lowPassFilter
// namespace complementary {
// EepromSetting<float> alpha(SettingId::imuComplementary_alpha, 0.980, 75, 0.7, 0.998, 0.001); //6
// float oneMinusAlpha = 1 - alpha();
// } //namespace complementary
// const float epsilon = 0.001;
} //namespace imu

namespace communication {
const uint32_t commandTimeout = 350000;
const int8_t powerAmplification = 3;
const int8_t dataRate = 1;
const int8_t retryDelay = 1;
const int8_t retryCount = 1;
const int8_t crcLength = 2;
// EepromSetting<int8_t> powerAmplification(SettingId::comm_pa, 3, 80, 0, 3, 1);
// EepromSetting<int8_t> dataRate(SettingId::comm_dataRate, 1, 85, 0, 2, 1);
// EepromSetting<int8_t> retryDelay(SettingId::comm_retryDelay, 1, 90, 0, 15, 1);
// EepromSetting<int8_t> retryCount(SettingId::comm_retryCount, 1, 95, 0, 15, 1);
// EepromSetting<int8_t> crcLength(SettingId::comm_crcLength, 2, 100, 0, 2, 1);
namespace telemetry {
EepromSetting<int8_t> type(SettingId::commTelemetry_type, 1, 105, 1, 4, 1);
} //namespace telemetry
} //namespace communication

namespace regulation {
namespace inner {
EepromSetting<float> P(SettingId::regInner_p, 0.15, 110, 0.1, 0.7, 0.025);
EepromSetting<float> yawP(SettingId::regInner_yawP, 0.8, 115, 0.1, 1.0, 0.025);
const float yawI = 0.03f;
const int16_t outputLimit = 70;
// EepromSetting<int16_t> outputLimit(SettingId::regInner_outputLimit, 70, 120, 10, 127, 1);
} //namespace inner
namespace outer {
EepromSetting<float> P(SettingId::regOuter_p, 2.3, 125, 1.5, 3.3, 0.025);
EepromSetting<float> I(SettingId::regOuter_i, 0.5, 130, 0.0, 1.8, 0.025);
const uint32_t updateRate = 12406;
// EepromSetting<uint32_t> updateRate(SettingId::regOuter_updateRate, 13750, 135, 2000, 30000, 500);
const int16_t outputLimit = 250;
// EepromSetting<int16_t> outputLimit(SettingId::regOuter_outputLimit, 250, 140, 10, 250, 1);
} //namespace outer
const uint8_t minimumRegulationThrottle = 3;
const uint8_t maximumBaseThrottle = 120;
// EepromSetting<uint8_t> minimumRegulationThrottle(SettingId::reg_minRegThrottle, 2, 145, 0, 15, 1);
// EepromSetting<uint8_t> maximumBaseThrottle(SettingId::reg_maxBaseThrottle, 117, 150, 90, 127, 1);
namespace altitude {
EepromSetting<float> P(SettingId::regAlt_p, 0.5, 155, 0.0, 2, 0.1);
EepromSetting<float> D(SettingId::regAlt_d, 0.2, 160, 0.0, 2, 0.1);
} //namespace altitude
} //namespace regulation

namespace indication {
const uint32_t period = 200000;
EepromSetting<uint8_t> armsLevel(SettingId::indication_armsLevel, 12, 155, 0, 12, 1);
EepromSetting<bool> lamp(SettingId::indication_lamp, false, 160, 0, 1);
} //namespace indication

namespace battery {
const uint32_t updateRate = 500000;
const float alpha = 0.98;
const float lowVoltage = 11.1;
const float criticalVoltage = 10.0f;
} //namespace battery

void init() {
  // imu::lowPassFilter::common.init();
  // imu::lowPassFilter::angularVelocity::state.init();
  // imu::lowPassFilter::angularVelocity::alpha.init();
  // imu::lowPassFilter::angularVelocity::oneMinusAlpha = 1 - imu::lowPassFilter::angularVelocity::alpha();
  // imu::lowPassFilter::acceleration::state.init();
  // imu::lowPassFilter::acceleration::alpha.init();
  // imu::lowPassFilter::acceleration::oneMinusAlpha = 1 - imu::lowPassFilter::acceleration::alpha();
  // imu::complementary::alpha.init();
  // imu::complementary::oneMinusAlpha = 1 - imu::complementary::alpha();

  // communication::powerAmplification.init();
  // communication::dataRate.init();
  // communication::retryDelay.init();
  // communication::retryCount.init();
  // communication::crcLength.init();
  communication::telemetry::type.init();

  regulation::inner::P.init();
  regulation::inner::yawP.init();
  // regulation::inner::outputLimit.init();
  regulation::outer::P.init();
  regulation::outer::I.init();
  // regulation::outer::updateRate.init();
  // regulation::outer::outputLimit.init();
  // regulation::minimumRegulationThrottle.init();
  // regulation::maximumBaseThrottle.init();
  regulation::altitude::P.init();
  regulation::altitude::D.init();

  indication::armsLevel.init();
  indication::lamp.init();
}
} //namespace config
