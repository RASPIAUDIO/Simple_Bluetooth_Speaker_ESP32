# Introduction

This is a simple example demonstrating how to use the Muse Luxe as a Bluetooth speaker. This code is from the app included with the Muse Luxe.

This program is intended to be used with the ESP32 Muse Luxe speaker, a portable and affordable Bluetooth speaker that is fully programmable. The ESP32 Muse Luxe is a commercial product available for purchase here: [ESP32 Muse Luxe](https://raspiaudio.com/produit/esp-muse-luxe).

Voice recordings are provided courtesy of [Aurélie Loilier](http://aurelieloilier.com/) (all rights reserved). To load the voice, you need to perform an ESP32 data sketch upload and select LittleFS.

# Quick Start

To quickly load the precompiled binary, follow these steps:

1. Go to [app.raspiaudio.com](https://app.raspiaudio.com).
2. Select "Luxe-Bluetooth."
3. Use a Chrome browser for best compatibility.
4. Connect your device using a USB cable.

# Features
- Intro startup wav play
- Bluetooth A2DP
- Codec volume control
- Heaphone output with jack detection
- RGB LEDs
- Low battery sensor

# Compilation Instructions
compiled with arduino 3.1.0
1. Select `Tool -> Board -> ESP32 Dev Module`
2. Choose the partition scheme "Huge app"
3. Use `Tool -> ESP32 Sketch Data Upload` to upload the WAV files used by the program
4. Upload your sketch
5. Ensure the following options are selected:
![image](https://github.com/user-attachments/assets/1d91c99f-238e-474a-956e-7092ff2ced8c)


# Factory Test

To initiate the factory test, hold the three top buttons while powering on the device. More information on the factory test can be found here: [Muse Luxe Factory Test](https://github.com/RASPIAUDIO/Muse-Luxe-Factory-Test)

# To Be Done

- Implement low battery warning
- Enable SD card reading
- SD card media reader
- Microphone support for phone handfree


