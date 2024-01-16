#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

char anchor_addr[] = "84:00:5B:D5:A9:9A:E2:9C";
uint16_t Adelay = 16605;
//16550
//16620

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

const int NUMBER_OF_DISTANCES = 3;  // Set the number of distances to average

const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS = 4;

void setup() {
  Serial.begin(115200);
  delay(1000);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  DW1000.setAntennaDelay(Adelay);

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  DW1000Ranging.startAsAnchor(anchor_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
}

void loop() {
  DW1000Ranging.loop();
}

void newRange() {

  float dist = 0.0;
  for (int i = 0; i < NUMBER_OF_DISTANCES; i++) {
    dist += DW1000Ranging.getDistantDevice()->getRange();
  }
  dist = dist / NUMBER_OF_DISTANCES;

  // Tijd verkrijgen (optioneel)
  unsigned long currentTime = millis();

  // Gegevens verzenden via seriële communicatie
  Serial.print(currentTime);
  Serial.print(",");
  Serial.println(dist);
}

void newDevice(DW1000Device *device) {
  //Serial.print("Device added: ");
  //Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device) {
  //Serial.print("Delete inactive device: ");
  //Serial.println(device->getShortAddress(), HEX);
}



