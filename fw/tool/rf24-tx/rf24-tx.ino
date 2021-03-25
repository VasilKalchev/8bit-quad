#include <SPI.h>
#include "printf.h"
#include "RF24.h"

static uint64_t const pipe_sg[5] = { 0x3A3A3A3AD2LL, 0x3A3A3A3AC3LL, 0x3A3A3A3AB4LL, 0x3A3A3A3AA5LL, 0x3A3A3A3A96LL };
RF24 radio(7, 8);


void setup() {
  Serial.begin(2000000);

  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
    while (1) {} // hold in infinite loop
  }

  // print example's introductory prompt
  Serial.println(F("RF24/examples/AcknowledgementPayloads"));

  // role variable is hardcoded to RX behavior, inform the user of this
  Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));

  // RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
  // RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_MED=-6dBM, and RF24_PA_HIGH=0dBm
  // -18dBm, -12dBm, -6dBM, and 0dBm
  // 7mA     7.5mA   9mA        11.3mA @ VDD = 3V, load impedance = 15+j88ohm
  // RF24_PA_MIN = 0, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX, RF24_PA_ERROR
  radio.setPALevel(RF24_PA_HIGH);

  // RF24_250KBPS for 250kbps, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
  radio.setDataRate(RF24_1MBPS);

  // Delay between retries (0 - 15 % 250us), retries count (0 - 15)
  radio.setRetries(15, 15);

  // Frequency channel (0 - 125 % 1MHz).
  // 250kbps and 1Mbps bandwidth: 1MHz, 2Mbps bandwidth: 2MHz
  // 0..82 - Wi-Fi, Bluetooth...
  // 83..99 - illegal
  // 100 - licensed
  // 101..119 - free
  // 120..125 - used in defense
  radio.setChannel(108);

  // RF24_CRC_DISABLED, RF24_CRC_8 for 8-bit or RF24_CRC_16 for 16-bit
  radio.setCRCLength(RF24_CRC_16);

  // to use ACK payloads, we need to enable dynamic payload lengths (for all nodes)
  radio.enableDynamicPayloads();    // ACK payloads are dynamically sized

  // Acknowledgement packets have no payloads by default. We need to enable
  // this feature for all nodes (TX & RX) to use ACK payloads.
  radio.enableAckPayload();

  radio.openWritingPipe(pipe_sg[0]);
  radio.openReadingPipe(1, pipe_sg[1]);

  radio.stopListening();

  // For debugging info
  // printf_begin();             // needed only once for printing details
  // radio.printDetails();       // (smaller) function that prints raw register values
  // radio.printPrettyDetails(); // (larger) function that prints human readable data
}

void loop() {

  radio.stopListening();

  static uint32_t packet = 0;
  static uint32_t tries = 0;
  static uint32_t successes = 0;
  static uint32_t failures = 0;
  static uint32_t rec_cnt = 0;
  static uint32_t rec_empty_cnt = 0;

  uint32_t m0 = micros();
  bool wr_status = radio.write(&packet, sizeof(packet));
  uint32_t m1 = micros();


  if (wr_status == true) {

    uint8_t pipe;
    if (radio.available(&pipe)) {
      uint32_t packet_rx;
      radio.read(&packet_rx, sizeof(packet_rx));
      Serial.print(F("Reply ")); Serial.print(radio.getDynamicPayloadSize());
      Serial.print(F(" bytes on pipe "));
      Serial.print(pipe);
      Serial.print("\n");

      rec_cnt++;
    } else {
      Serial.print("Reply empty\n");
      rec_empty_cnt++;
    }

    Serial.print("y - ");
    successes++;
    packet++;

  } else {
    Serial.print("n - ");
    failures++;
  }
  Serial.print(packet); Serial.print(" / ");
  Serial.print(tries); Serial.print(", ");
  Serial.print("t: "); Serial.print(m1-m0); Serial.print(", ");
  Serial.print("s: "); Serial.print(successes); Serial.print(", ");
  Serial.print("f: "); Serial.print(failures); Serial.print(", ");
  Serial.print("\n");

  ++tries;

  delay(100);
}