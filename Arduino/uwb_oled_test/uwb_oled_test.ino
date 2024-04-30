#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define OLED_RESET     4    // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D // 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS = 4;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("oled test");

  //SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  Serial.println("esp32 uwb SPI connection succes");

  Wire.begin(21, 22); // SDA to pin 21, SCL to pin 22 (adjust based on your wiring)
  Serial.println("I2C pins defined. sda to IO21 and scl to IO22 on esp32 uwb");


  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  else{Serial.println("oled allocation succes");}

  display.clearDisplay();
  Serial.println("display cleared");

  display.setCursor(15, 0);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.print("Test");
  display.display();

  Serial.println("display text");
  
}

void loop() {}

