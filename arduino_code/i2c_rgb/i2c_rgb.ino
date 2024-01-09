
#include <FastLED.h>
#include <Wire.h>

#define DATA_PIN        5
#define NUM_LEDS 142

#define R 0
#define G 1
#define B 2

int RGB_value[3] = {0, 25, 0};

CRGB leds[NUM_LEDS];

void setup() {
  Serial.begin(115200);
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed
  Wire.begin(0x42);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
}

void receiveEvent(int howMany)
{
  int i=0;
  while(Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    if (i > 2) {
      Serial.print("Received too much data, dropping: ");
      Serial.println(c);
      continue;
    }
    RGB_value[i++] = c;  
  }
  Serial.print("Received new RBG value: ");
  Serial.print(RGB_value[R]);
  Serial.print(", ");
  Serial.print(RGB_value[G]);
  Serial.print(", ");
  Serial.println(RGB_value[B]);
}

void loop() {
  int speed = 50;
  int height = 100;
  unsigned long currentMillis = millis();
  for (int i = 0; i < NUM_LEDS; i++) {
    // int position = (i * height) + (currentMillis / speed);
    // int brightness = (sin(position * 0.15) + 1) * 127;
    leds[i].r = RGB_value[R];
    leds[i].g = RGB_value[G];
    leds[i].b = RGB_value[B];
    // leds[i].subtractFromRGB(255 - brightness);
  }
  
  FastLED.show();
  
  delay(10);


}
