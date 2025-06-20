# ESP MUSE – Bluetooth Speaker Firmware (Luxe & Proto)

This repository contains the **complete source code and binaries** that turn an ESP-MUSE Luxe or Proto board into a fully featured Bluetooth speaker.

| Feature | Status |
|---------|--------|
| A2DP sink (stereo) | ✅ |
| Codec or direct I²S (board-selectable) | ✅ |
| Head-phone jack detection (Luxe) | ✅ |
| RGB NeoPixel status LED | ✅ |
| Low-battery indicator | ✅ |
| Custom start-up jingle | ✅ |
| **Runtime configurator (Web app)** | ✅ |

---

## New! Online Configurator 🚀

You can now change the **Bluetooth name**, the **LED colour**, and upload your own **start-up jingle** (≤ 2 s WAV) **without recompiling**:

**▶︎ [Open the RaspiAudio ESP MUSE Customizator](https://raspiaudio.github.io/BT/)**

![image](https://github.com/user-attachments/assets/dfecd49f-d9f6-4836-aa32-6df99d91cdbb)


The page also includes an **ESP-Web-Tools** button that flashes the latest release `.bin` straight from your browser.

> The configurator’s source code lives in the same repo: see [`raspiaudio_customizator.html`](./raspiaudio_customizator.html).

---

## Quick Start (Flash pre-built firmware)

1. Open <https://raspiaudio.github.io/BT/> in Chrome / Edge.  
2. In step 1, click **“Flash latest firmware”** and choose your ESP32 Muse board.  
3. After flashing, hit **“Connect ESP32”** and tweak the settings in step 2.  
4. Enjoy your personalised speaker!

---

## Building from Source

1. **Arduino IDE ≥ 3.1.0**  
2. Board: **ESP32 Dev Module**  
3. Partition scheme: **Huge App**  
4. Optional WAV files → `Tools > ESP32 Sketch Data Upload` (LittleFS)  
5. `Upload` and you’re done.

> The default board can be switched by commenting / uncommenting  
> `#define BOARD_LUXE` or `#define BOARD_PROTO` in `main.ino`.

---

## Factory Test

Hold the three top buttons while power-cycling → the firmware jumps to the factory-test partition. Details in the separate test repo: <https://github.com/RASPIAUDIO/Muse-Luxe-Factory-Test>.

---

## Road-map / Ideas

* OTA update over Wi-Fi  
* SD-card media playback  
* Built-in microphone → hands-free / voice-assistant  
* Battery percentage pop-ups over AVRCP

---

## Credits

* **Hardware:** [RaspiAudio ESP-MUSE Luxe & Proto](https://raspiaudio.com)  
* **Voice prompts:** © [Aurélie Loilier](http://aurelieloilier.com/) – all rights reserved.

Happy hacking!
