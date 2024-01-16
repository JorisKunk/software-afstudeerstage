#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels
#define OLED_RESET     4    // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C // 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);  
  display.print("OLED Test");
  display.setCursor(0, 15);  
  display.print("OLED Test");
  display.setCursor(0, 30);  
  display.print("OLED Test");
  display.setCursor(0, 45);  
  display.print("OLED Test");
  display.setCursor(0, 60);  
  display.print("OLED Test");

  display.display();
}

void loop() {
  // Your main code here (leave loop empty for a simple test)
}
