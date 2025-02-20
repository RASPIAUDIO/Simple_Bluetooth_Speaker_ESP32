# Introduction

This is a simple example demonstrating how to use the Muse Luxe as a Bluetooth speaker. This code is from the app included with the Muse Luxe.

This program is intended to be used with the ESP32 Muse Luxe speaker, a portable and affordable Bluetooth speaker that is fully programmable. The ESP32 Muse Luxe is a commercial product available for purchase here: [ESP32 Muse Luxe](https://raspiaudio.com/produit/esp-muse-luxe).

Voice recordings are provided courtesy of [Aurélie Loilier](http://aurelieloilier.com/) (all rights reserved). To load the voice, you need to perform an ESP32 data sketch upload and select LittleFS.

# Features

- Bluetooth and Wi-Fi capabilities via ESP32
- Line input
- SD card media reader
- RGB LEDs
- Microphone
- Low battery sensor

# Compilation Instructions

1. Select `Tool -> Board -> ESP32 Dev Module`
2. Choose the partition scheme "Huge app"
3. Use `Tool -> ESP32 Sketch Data Upload` to upload the WAV files used by the program
4. Upload your sketch
5. Ensure the following options are selected:
     - ![image](https://github.com/user-attachments/assets/4920e7a3-65a3-4098-b42b-2be1af66a60d)

# Factory Test

To initiate the factory test, hold the three top buttons while powering on the device. More information on the factory test can be found here: [Muse Luxe Factory Test](https://github.com/RASPIAUDIO/Muse-Luxe-Factory-Test)

# To Be Done

- Implement low battery warning
- Enable SD card reading
