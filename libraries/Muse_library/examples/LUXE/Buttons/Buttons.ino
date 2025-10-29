// ESP Muse Luxe NeoPixel (c) 2024 RASPiAudio
// This code displays the French flag colors based on button presses on the Muse Luxe's programmable RGB LED

#include <Arduino.h>
#include "museWrover.h"

// Set up the NeoPixel library with number of pixels and pin configuration
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  // Initialize the NeoPixel strip object (REQUIRED for the Muse Luxe's RGB LED)
  pixels.begin();
  pixels.show();  // Initialize all to 'off'

  // Initialize button pins
  pinMode(BUTTON_PAUSE, INPUT_PULLUP);
  pinMode(BUTTON_VOL_MINUS, INPUT_PULLUP);
  pinMode(BUTTON_VOL_PLUS, INPUT_PULLUP);
}

void loop() {
  if (digitalRead(BUTTON_VOL_MINUS) == LOW) {
    // Display blue when Volume- button is pressed
    pixels.setPixelColor(0, pixels.Color(0, 0, 255));
    pixels.show();

  } else if (digitalRead(BUTTON_PAUSE) == LOW) {
    // Display white when Pause/Play button is pressed
    pixels.setPixelColor(0, pixels.Color(255, 255, 255));
    pixels.show();

  } else if (digitalRead(BUTTON_VOL_PLUS) == LOW) {
    // Display red when Volume+ button is pressed
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();
    
  } else {
    // Turn off the LED if no button is pressed
    pixels.clear();
    pixels.show();
  }
}
