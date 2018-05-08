#include <MADwa-RF24_TMRh20.hpp>

float p = 0.0f;
int32_t updateRate = 0;

mad::communication::RF24_TMRh20 com(Role::drone);

void setup() {
  Serial.begin(2000000);
  bool initSuccess = true;

  com.initialize();
  initSuccess &= com.setPALevel(config::communication::paLevel);
  initSuccess &= com.setDataRate(config::communication::dataRate);
  initSuccess &= com.setRetryDelay(config::communication::retryDelay);
  initSuccess &= com.setRetryCount(config::communication::retryCount);
  initSuccess &= com.setCrcLength(config::communication::crcLength);
  initSuccess &= com.testConnection();

  com.attachSetting(SettingId::p, &changeP);
  com.attachSetting(SettingId::updateRate, &changeUpdateRate);
}

void loop() {

}

void changeP(float newP) {
  p = newP;
}
void changeUpdateRate(float newUpdateRate) {
  updateRate = newUpdateRate;
}