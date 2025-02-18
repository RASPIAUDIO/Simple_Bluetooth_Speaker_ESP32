# Introduction
This program is intended to be used with the ESP32 Muse Luxe speaker, a portable and affordable bluetooth speaker that is fully programmable .
ESP32 Muse Luxe is a commercial product that can be purchase here: https://raspiaudio.com/produit/esp-muse-luxe

Full tutorial could be found here : https://forum.raspiaudio.com/t/esp-muse-luxe-bluetooth-speaker/294


Voices recorded with the courtesy of [Aurélie Loilier](http://aurelieloilier.com/) (all rights reserved)


# Features
- ESP32 offers : Bluetooth, Wifi
- line Input
- SD card media reader
- RGB leds
- Microphone
- Low battery sensor

#  To compile
- Select Tool-Board-ESP32 dev Module
- Partition "Huge app"
- Select Toll-ESP32 Sketch data upload this will upload the wav files use by the programmable
- Upload

# Factory test
- To trigger the factory test move the file testToDo in the data directory, then upload SPIFF data.


#  To be done
- Low battery warning
