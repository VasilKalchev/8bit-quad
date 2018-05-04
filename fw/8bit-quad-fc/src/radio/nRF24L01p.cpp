#include "nRF24L01p.hpp"

nRF24L01p_RF24::nRF24L01p_RF24(Role role, uint8_t ce, uint8_t csn)
  : _rf24(ce, csn), _role(role) {}

void nRF24L01p_RF24::initialize() {
  _rf24.begin();

// for (auto i = 0)
  _rf24.flush_tx();
  _rf24.read(0, 0);

  //RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
  //-18dBm, -12dBm,-6dBM, and 0dBm
  _rf24.setPALevel(RF24_PA_MAX);

  //RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
  _rf24.setDataRate(RF24_1MBPS);

  //Delay between retries (0 - 15 % 250us), (0 - 250us, 15 - 4000us)
  _rf24.setRetries(1, 1);

  //RF24_CRC_DISABLED, RF24_CRC_8 for 8-bit or RF24_CRC_16 for 16-bit
  _rf24.setCRCLength(RF24_CRC_16);

  _rf24.enableAckPayload();
  _rf24.enableDynamicPayloads();

  if (_role == Role::drone) {
    _rf24.openWritingPipe(_pipe[1]);
    _rf24.openReadingPipe(1, _pipe[0]);
  } else {
    _rf24.openWritingPipe(_pipe[0]);
    _rf24.openReadingPipe(1, _pipe[1]);
  }

  _rf24.startListening();
}

void nRF24L01p_RF24::setPALevel(uint8_t paLevel) {
  _rf24.setPALevel(paLevel);
}
void nRF24L01p_RF24::setDataRate(uint8_t dataRate) {
  _rf24.setDataRate(dataRate);
}
void nRF24L01p_RF24::setRetries(uint8_t delay, uint8_t count) {
  _rf24.setRetries(delay, count);
}
void nRF24L01p_RF24::setCRCLength(uint8_t crcLength) {
  _rf24.setCRCLength(crcLength);
}

void nRF24L01p_RF24::reset() {
  // _rf24.powerDown();
  // _rf24.powerUp();
  // if (_role == Role::drone) {
  //   _rf24.openWritingPipe(_pipe[1]);
  //   _rf24.openReadingPipe(1, _pipe[0]);
  // } else {
  //   _rf24.openWritingPipe(_pipe[0]);
  //   _rf24.openReadingPipe(1, _pipe[1]);
  // }
  // _rf24.flush_tx();
  _rf24.read(0, 32);
  // initialize();
}

bool nRF24L01p_RF24::available() {
  return _rf24.available();
}
// MessageType nRF24L01p_RF24::available() {
//   if (_rf24.available() == false) {
//     return MessageType::noMessage;
//   } else {
//     _rf24.read(&_rawMessage, 32);
//     switch ((MessageType)_rawMessage[0]) {
//     case MessageType::Command:
//       memcpy(command, _rawMessage, sizeof(command));
//       break;
//     case MessageType::Setting:
//       memcpy(command, _rawMessage, sizeof(command));
//       break;
//     case MessageType::Telemetry:
//       memcpy(command, _rawMessage, sizeof(command));
//       break;
//     default:
//       break;
//     }
//     return (MessageType)_rawMessage[0];
//   }
//   // return _rf24.available();
// }

void nRF24L01p_RF24::startListening() {
  _rf24.startListening();
}

void nRF24L01p_RF24::stopListening() {
  _rf24.stopListening();
}

bool nRF24L01p_RF24::write(const void *buffer, uint8_t length) {
  _rf24.stopListening();
  bool s = _rf24.write(buffer, length);
  _rf24.startListening();
  return s;
}

void nRF24L01p_RF24::read(void *buffer, uint8_t length) {
  _rf24.read(buffer, length);
}

void nRF24L01p_RF24::deserialize(void *dst, const void *src, uint8_t length) {
  memcpy(dst, src, length);
}

void nRF24L01p_RF24::setResponse(const void* buffer, uint8_t length) {
  uint8_t pipeNumber;
  (_role == Role::drone) ? pipeNumber = 1 : pipeNumber = 0;
  _rf24.writeAckPayload(pipeNumber, buffer, length);
}

bool nRF24L01p_RF24::isChipConnected() {
  return _rf24.isChipConnected();
}