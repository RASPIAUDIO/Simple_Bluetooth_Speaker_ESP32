# Muse Library
This is a library to ease devlopment with Raspiaudio's muse devices, for now it supports :
- Muse Luxe [buy here](https://raspiaudio.com/product/esp-muse-luxe)
- Muse Radio 
- Muse Proto 
- Muse Manga
  

## Overview

The **Muse Library** is an Arduino library designed for the ESP32-based Muse devices. It provides seamless control over the Muse  NeoPixel LED, interfaces with the ES8388 codec for audio, and includes support for various other peripherals. 

## Features

- **NeoPixel Control**: Easily set and display colors on the Muse Luxe's integrated NeoPixel LED.
- **Audio Playback**: Control audio playback using the ES8388 codec and integrate streaming or audio functionalities.
- **Battery monitoring**: Control audio playback using the ES8388 codec and integrate streaming or audio functionalities.

- **Audio recording**

## Table of Contents

- [Installation](#installation)
- [Dependencies](#dependencies)


## Installation

### Using Arduino Library Manager

1. Open the Arduino IDE.
2. Navigate to **Sketch** > **Include Library** > **Manage Libraries**.
3. In the Library Manager, search for `Muse_library`.
4. Click **Install** on the Muse library by `Raspiaudio`.
5. Select int Tools-Board-ESP32 Wrover


## Dependencies
- Tested on ESP32 version 3.0.7
- [Audio](https://github.com/schreibfaul1/ESP32-audioI2S) by schreibfaul1 YOU NEED TO INSTALL THIS LIBRARY MANUALLY search for "ESP32-audioI2S-master" in the library manager
- [Adafruit NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel) by Adafruit
- [ESP32Encoder](https://github.com/madhephaestus/ESP32Encoder)
- [ESP32-A2DP](https://github.com/pschatzmann/ESP32-A2DP)
- [TFT_eSPI](https://github.com/Bodmer/TFT_eSPI)
- [esp32-sh1106-oled](https://github.com/davidperrenoud/Adafruit_SH1106) **replace the User_Setup.h of this repository in the TFT-eSPI library directory to apply the screen preferences**


Ensure that the libraries are installed before using the Muse_library.



