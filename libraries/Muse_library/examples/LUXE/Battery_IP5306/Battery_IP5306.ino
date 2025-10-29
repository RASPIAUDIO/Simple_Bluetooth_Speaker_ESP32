// BatteryIP5306.ino - Example Sketch for Muse Luxe with IP5306

#include <Arduino.h>
#include "museWrover.h"

// Instantiate the MuseLuxe class
MuseLuxe museluxe;

// Function to blink the NeoPixel red
void blinkRed(MuseLuxe &museluxe) {
    for (int i = 0; i < 10; i++) {  // Blink 10 times
        museluxe.setPixelColor(museluxe.getColor(255, 0, 0));  // Red
        museluxe.showPixel();
        delay(500);  // Wait 0.5 second
        museluxe.setPixelColor(museluxe.getColor(0, 0, 0));  // Off
        museluxe.showPixel();
        delay(500);  // Wait 0.5 second
    }
}

// Function to pulse the NeoPixel (e.g., green)
void pulseColor(MuseLuxe &museluxe, uint8_t r, uint8_t g, uint8_t b, int duration_ms = 1000, int steps = 100) {
    for(int i = 0; i <= steps; i++) {
        float brightness = (float)i / steps;
        museluxe.setPixelColor(museluxe.getColor(r * brightness, g * brightness, b * brightness));
        museluxe.showPixel();
        delay(duration_ms / steps);
    }
    for(int i = steps; i >= 0; i--) {
        float brightness = (float)i / steps;
        museluxe.setPixelColor(museluxe.getColor(r * brightness, g * brightness, b * brightness));
        museluxe.showPixel();
        delay(duration_ms / steps);
    }
}

void setup() {
    // Initialize the MuseLuxe library
    museluxe.begin();
}

void loop() {
    // Get the battery percentage
    uint8_t batteryLevel = museluxe.getBatteryPercentage();

    Serial.print("Battery Level: ");
    Serial.print(batteryLevel);
    Serial.println("%");

    // Log IP5306 statuses
    bool chargerEnabled = museluxe.isCharging();
    uint8_t vinCurrent = museluxe.getVinCurrent();
    uint8_t voltagePressure = museluxe.getVoltagePressure();

    Serial.print("Charger Enabled: ");
    Serial.println(chargerEnabled ? "Yes" : "No");

    Serial.print("Vin Current: ");
    Serial.print(vinCurrent); // Already calculated in getVinCurrent()
    Serial.println(" mA");

    Serial.print("Voltage Pressure: ");
    Serial.print(voltagePressure); // Already calculated in getVoltagePressure()
    Serial.println(" mV");

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
        blinkRed(museluxe);
    } 
    else {
        // Mid battery - Orange
        Serial.println("Battery Mid - Orange");
        museluxe.setPixelColor(museluxe.getColor(255, 165, 0)); // Orange
        museluxe.showPixel();
    }

    // If charging, perform pulse effect
    if (chargerEnabled) {
        Serial.println("Charging - Pulsing LED");
        pulseColor(museluxe, 0, 255, 0, 1000, 100); // Pulse green over 1 second with 100 steps
    }

    delay(1000); // Update every second
}
