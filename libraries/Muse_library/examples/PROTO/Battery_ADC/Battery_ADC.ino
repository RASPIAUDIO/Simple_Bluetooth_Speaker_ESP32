// ESP Muse Luxe NeoPixel (c) 2024 RASPiAudio
// This code monitors battery level using simple ADC for older and new version of the Luxe and changes LED color from green (full) to blinking red (empty)
// Adapted to use the MuseLuxe library for NeoPixel control

#include <Arduino.h>
#include "museWrover.h"

// Instantiate the MuseLuxe class
MuseLuxe museluxe;

// Define the battery pin if not already defined
#ifndef BATTERY_PIN
#define BATTERY_PIN 34 // Example GPIO pin for battery monitoring; adjust as needed
#endif

// Define battery thresholds if not already defined
#ifndef BATTERY_FULL
#define BATTERY_FULL 3000 // Example ADC value for full battery; adjust based on calibration
#endif

#ifndef BATTERY_LOW
#define BATTERY_LOW 1000 // Example ADC value for low battery; adjust based on calibration
#endif

void setup() {
    // Initialize the MuseLuxe library (handles NeoPixel initialization)
    museluxe.begin();

    // Initialize ADC for battery level monitoring
    analogReadResolution(12); // Set ADC resolution to 12 bits

    // Initialize Serial for debugging
    Serial.begin(115200);
    Serial.println("Battery monitoring started");
}

void loop() {
    // Read the battery level using ADC
    int batteryLevel = analogRead(BATTERY_PIN);
    Serial.print("Battery Level: ");
    Serial.println(batteryLevel);

    // Battery level indication
    if (batteryLevel >= BATTERY_FULL) {
        // Full battery - Green
        Serial.println("Battery Full - Green");
        museluxe.setPixelColor(museluxe.getColor(0, 255, 0)); // Green
        museluxe.showPixel();
    } 
    else if (batteryLevel < BATTERY_LOW) {
        // Low battery - Blinking Red
        Serial.println("Battery Low - Blinking Red");
        museluxe.setPixelColor(museluxe.getColor(255, 0, 0)); // Red
        museluxe.showPixel();
        delay(500);
        museluxe.setPixelColor(museluxe.getColor(0, 0, 0)); // Off
        museluxe.showPixel();
        delay(500);
    } 
    else {
        // Mid battery - Orange
        Serial.println("Battery Mid - Orange");
        museluxe.setPixelColor(museluxe.getColor(255, 165, 0)); // Orange
        museluxe.showPixel();
    }

    delay(1000); // Update every second
}
