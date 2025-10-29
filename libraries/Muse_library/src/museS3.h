// museluxe.h - Header file for ESP Muse Luxe NeoPixel, Battery Management System (BMS) using IP5306, and ES8388 Audio Codec
// (c) 2024 RASPiAudio

#ifndef MUSELUXE_H
#define MUSELUXE_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h> // https://github.com/adafruit/Adafruit_NeoPixel
#include <stdint.h>
#include <SD_MMC.h>
#include "Free_Fonts.h"
#include <driver/gpio.h>
#include <esp_system.h>


//SD_MMC card GPIOs
#define SD_MMC_clk	     14
#define SD_MMC_cmd           15
#define SD_MMC_d0            2



// I2S GPIOs
#define I2S_SDOUT     17
#define I2S_BCLK       5
#define I2S_LRCK      16
#define I2S_MCLK       0
#define I2S_SDIN       4

// I2C GPIOs
#define IIC_CLK       11
#define IIC_DATA      18

// Amplifier enable
#define GPIO_PA_EN    GPIO_NUM_46

// Pin Definitions
#define NEOPIXEL_PIN 22        // Pin for the RGB LED on Muse Luxe
#define NUMPIXELS 1            // Muse Luxe has 1 RGB LED

#define IR              GPIO_NUM_47

// Encoders definition
#define ENC_A2          6
#define ENC_B2          7
#define ENC_A1          3
#define ENC_B1          42

#define CLICK2      GPIO_NUM_45
#define CLICK1      GPIO_NUM_48
#define backLight   GPIO_NUM_41

// Battery Level Thresholds
#define BATTERY_FULL 75
#define BATTERY_LOW  25
#define BATTERY_PIN 13 //for legacy ADC pin (not the I2C bms mode)

#define KEYs_ADC        1

class MuseRadio{
public:
    void begin();
    int button_get_level(int);

};



class ES8388
{

    bool write_reg(uint8_t slave_add, uint8_t reg_add, uint8_t data);
    bool read_reg(uint8_t slave_add, uint8_t reg_add, uint8_t &data);
    bool identify(int sda, int scl, uint32_t frequency);

public:
    bool begin(int sda = -1, int scl = -1, uint32_t frequency = 400000U);

    enum ES8388_OUT
    {
        ES_MAIN, // this is the DAC output volume (both outputs)
        ES_OUT1, // this is the additional gain for OUT1
        ES_OUT2  // this is the additional gain for OUT2
    };

    void mute(const ES8388_OUT out, const bool muted);
    void volume(const ES8388_OUT out, const uint8_t vol);
    void microphone_volume(const uint8_t vol);   
    void ALC(const bool valid);
    void Amp_D(const bool valid);      
};
#endif // MUSELUXE_H
