#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET 4
#define SCREEN_ADDRESS 0x3D

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ESP32_UWB pin definitions
 
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

char this_anchor_addr[] = "84:00:22:EA:82:60:3B:9C";
float this_anchor_target_distance = 1;

uint16_t this_anchor_Adelay = 16600;
uint16_t Adelay_delta = 100;

const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS = 4;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting Auto Calibration");
  Serial.print("Starting Adelay "); Serial.println(this_anchor_Adelay);
  Serial.print("Measured distance "); Serial.println(this_anchor_target_distance);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  Wire.begin(21, 22);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  display.clearDisplay();
  display.display();

  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ);
  DW1000.setAntennaDelay(this_anchor_Adelay);

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  DW1000Ranging.startAsAnchor(this_anchor_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
}

void loop() {
  DW1000Ranging.loop();
}

void newRange() {
  static float last_delta = 0.0;
  Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), DEC);

  float dist = 0;
  for (int i = 0; i < 100; i++) {
    // get and average 100 measurements
    dist += DW1000Ranging.getDistantDevice()->getRange();
  }
  dist /= 100.0;
  Serial.print(",");
  Serial.print(dist); 
  if (Adelay_delta < 3) {
    Serial.print(", final Adelay ");
    Serial.println(this_anchor_Adelay);
    while(1);  //done calibrating
  }
 
  float this_delta = dist - this_anchor_target_distance;  //error in measured distance
 
  if ( this_delta * last_delta < 0.0) Adelay_delta = Adelay_delta / 2; //sign changed, reduce step size
    last_delta = this_delta;
  
  if (this_delta > 0.0 ) this_anchor_Adelay += Adelay_delta; //new trial Adelay
  else this_anchor_Adelay -= Adelay_delta;
  
  Serial.print(", Adelay = ");
  Serial.println (this_anchor_Adelay);
  DW1000.setAntennaDelay(this_anchor_Adelay);

  // Display the results on the OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Adelay: ");
  display.println(this_anchor_Adelay);
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.print("Distance: ");
  display.print(dist);
  display.println(" m");
  display.display();
}

void newDevice(DW1000Device *device) {
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device) {
  Serial.print("Delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}
