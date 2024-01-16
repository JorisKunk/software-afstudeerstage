#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define OLED_RESET     4    // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

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
  Serial.println("Anchor config and start");
  Serial.print("Antenna delay");
  Serial.println(Adelay);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);  
  display.print("OLED Test");
  display.display();

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
  Serial.println(dist);


  checkDistanceTooBig(dist);
}

void newDevice(DW1000Device *device) {
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device) {
  Serial.print("Delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}

void checkDistanceTooBig(float dis){
  if(dis> 3.00){
    Serial.println("WARNING...DISTANCE BIGGER THAN 3 M");
      // Display the distance on the OLED
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("WARNING... DISTANCE TOO BIG    ");
    display.print(dis);
    display.print(" m");
    display.display();  
  }else{
    // Display the distance on the OLED
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Distance");
    display.setCursor(0, 30);
    display.print(dis);
    display.print(" m");
    display.display();
  }
}


