#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

#include <Wire.h>
#include <Adafruit_SSD1306.h>
 
//oled variables and initalisation
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define OLED_RESET     4    // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D // 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ESP32_UWB pin definitions
 
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4
 
// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin
 
 
char this_anchor_addr[] = "84:00:22:EA:82:60:3B:9C";
float this_anchor_target_distance = 2; //measured distance to anchor in m
 
uint16_t this_anchor_Adelay = 16600; //starting value
uint16_t Adelay_delta = 100; //initial binary search step size
 
 
void setup()
{
  Serial.begin(115200);
  while (!Serial);
  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  Wire.begin(); // SDA to pin 21, SCL to pin 22 (adjust based on your wiring)
 
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);  
  display.print("dist Test");
  display.setCursor(0, 15);  

  display.display();

  Serial.print("Starting Adelay "); Serial.println(this_anchor_Adelay);
  Serial.print("Measured distance "); Serial.println(this_anchor_target_distance);
  
  DW1000.setAntennaDelay(this_anchor_Adelay);
 
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  //Enable the filter to smooth the distance
  //DW1000Ranging.useRangeFilter(true);
 
  //start the module as anchor, don't assign random short address
  DW1000Ranging.startAsAnchor(this_anchor_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
 
}
 
void loop()
{
  DW1000Ranging.loop();
}
 
void newRange()
{
  static float last_delta = 0.0;
  //Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), DEC);
 
  float dist = 0;
  for (int i = 0; i < 100; i++) {
    // get and average 100 measurements
    dist += DW1000Ranging.getDistantDevice()->getRange();
  }
  dist /= 100.0;
  //Serial.print(",");

  // Display the distance on the OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("Distance:");
  display.setCursor(64, 0);
  display.print(dist);
  display.print(" m");

  Serial.println(dist);


  //Serial.print(dist); 
  if (Adelay_delta < 3) {
    Serial.print(", final Adelay ");
    Serial.println(this_anchor_Adelay);
//    Serial.print("Check: stored Adelay = ");
//    Serial.println(DW1000.getAntennaDelay());
    while(1);  //done calibrating
  }
 
  float this_delta = dist - this_anchor_target_distance;  //error in measured distance
 
  if ( this_delta * last_delta < 0.0) Adelay_delta = Adelay_delta / 2; //sign changed, reduce step size
    last_delta = this_delta;
  
  if (this_delta > 0.0 ) this_anchor_Adelay += Adelay_delta; //new trial Adelay
  else this_anchor_Adelay -= Adelay_delta;
  
  //Serial.print(", Adelay = ");
  //Serial.println (this_anchor_Adelay);

  display.setCursor(0,15);
  display.print("Adelay");
  display.setCursor(60,15);
  display.print(this_anchor_Adelay);
  display.display();

//  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  DW1000.setAntennaDelay(this_anchor_Adelay);
}
 
void newDevice(DW1000Device *device)
{
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}
 
void inactiveDevice(DW1000Device *device)
{
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}