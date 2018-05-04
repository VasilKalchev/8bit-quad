// #include "nRF24L01p.hpp"
#include <SPI.h>
#include "RF24.h"
#include "printf.h"
#include <util/delay.h>

const uint64_t _pipe[5] = { 0x3A3A3A3AD2LL, 0x3A3A3A3AC3LL, 0x3A3A3A3AB4LL, 0x3A3A3A3AA5LL, 0x3A3A3A3A96LL };

RF24 rf24(7, 8);

bool drone = false;

struct Command {
  uint8_t messageType;
  uint8_t throttle;
  uint8_t roll;
};

struct Setting {
  uint8_t messageType;
  uint8_t code;
  uint16_t int_;
};

struct Telemetry {
  uint8_t messageType;
  uint32_t pitch;
  float roll;
};

void setup() {
  // Configure Timer 2 to replace Timer 0.
  TIMSK0 &= ~(1 << TOIE0); // disable overflow interrupt for timer0
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); //outputs, fast PWM with TOP = 0xff
  TCCR2B = _BV(CS22);   // clock at F_CPU / 64
  TIMSK2 |= (1 << TOIE2); // enable overflow interrupt for timer2

  Serial.begin(9600);
  Serial.println("R2 comms test");

  rf24.begin();
  rf24.setPALevel(RF24_PA_MAX);
  rf24.setDataRate(RF24_1MBPS);
  rf24.setRetries(5, 5);
  rf24.setCRCLength(RF24_CRC_16);


  if (drone) {
    rf24.openWritingPipe(_pipe[1]);
    rf24.openReadingPipe(1, _pipe[0]);
  } else {
    rf24.openWritingPipe(_pipe[0]);
    rf24.openReadingPipe(1, _pipe[1]);
  }

  rf24.enableAckPayload();

  rf24.startListening();

  printf_begin();
  rf24.printDetails();
}

void loop() {
  static Command command;
  static Setting setting;
  static Telemetry telemetry;

  command.messageType = 123;
  setting.messageType = 51;


  static uint32_t lastMicros1 = micros();
  if (micros() > lastMicros1 + 250000) {
    lastMicros1 = micros();
    rf24.stopListening();
    command.throttle += 1;
    command.roll -= 1;
    if (rf24.write(&command, sizeof(command))) {
      Serial.println("Command: Success");
    } else {
      Serial.println("Command: Fail");
    }
    rf24.startListening();
  }

  static uint32_t lastMicros2 = micros();
  if (micros() > lastMicros2 + 500000) {
    lastMicros2 = micros();
    rf24.stopListening();
    setting.code += 5;
    setting.int_ -= 5;
    if (rf24.write(&setting, sizeof(setting))) {
      Serial.println("Setting: Success");
    } else {
      Serial.println("Setting: Fail");
    }
    rf24.startListening();
  }

        static uint32_t cntr = 0;
  if (rf24.available()) {
    while (rf24.available()) {
      uint8_t message[32];
      rf24.read(&message, 32);
      Serial.print("Message type: "); Serial.print((uint8_t)message[0]);
      switch ((uint8_t)message[0]) {
        case 51:
          memcpy(&setting, &message, sizeof(setting));
          Serial.print(" Setting code: "); Serial.println(setting.code);
        break;
        case 12:
          memcpy(&telemetry, &message, sizeof(telemetry));
          Serial.print(" Telemetry roll: "); Serial.println(telemetry.roll);
        break;
        default:
          cntr++;
          Serial.print(" UNRECOGNIZED "); Serial.println(cntr);
          // rf24.read(0, 32);
        break;
      }
    }
  }
}