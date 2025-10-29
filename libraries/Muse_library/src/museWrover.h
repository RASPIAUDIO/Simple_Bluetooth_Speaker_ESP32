// museluxe.h - Header file for ESP Muse Luxe NeoPixel, Battery Management System (BMS) using IP5306, and ES8388 Audio Codec
// (c) 2024 RASPiAudio

#ifndef MUSELUXE_H
#define MUSELUXE_H


#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <driver/gpio.h>
#include <esp32-hal-rmt.h>
#include <esp_system.h>
//SD card
#define SD_CS         13

// GPIOs for SPI
#define SPI_MOSI      15
#define SPI_MISO      2
#define SPI_SCK       14

// I2S GPIOs
#define I2S_SDOUT     26
#define I2S_BCLK       5
#define I2S_LRCK      25
#define I2S_MCLK       0
#define I2S_SDIN      35

// I2C GPIOs
#define IIC_CLK       23
#define IIC_DATA      18

// Amplifier enable
#define GPIO_PA_EN    GPIO_NUM_21

// Amplifier gain (Proto)
#define GPIO_PROTO_GAIN GPIO_NUM_23

// Jack detect
#define Jack_Detect GPIO_NUM_27
// Sd Detect
#define SD_Detect     GPIO_NUM_34      
// Pin Definitions
#define NEOPIXEL_PIN 22        // Pin for the RGB LED on Muse Luxe

// Buttons definition (Luxe)
#define BUTTON_PAUSE GPIO_NUM_12  // Pause/Play button
#define BUTTON_VOL_MINUS GPIO_NUM_32 // Volume - button
#define BUTTON_VOL_PLUS GPIO_NUM_19  // Volume + button

// Button definition (Proto)
#define BUTTON_PROTO_ZERO GPIO_NUM_0

// Button definition (Manga)
#define BUTTON_MANGA_CLICK GPIO_NUM_0
#define MANGA_ENC_A 32
#define MANGA_ENC_B 19

// Battery Level Thresholds
#define BATTERY_FULL 75
#define BATTERY_LOW  25
#define BATTERY_PIN 34 //for legacy ADC pin (not the I2C bms mode)

// IP5306 for Battery management system I2C Address and Registers
#define IP5306_I2C_ADDRESS 0x75
#define IP5306_REG_SYS_0    0x00
#define IP5306_REG_SYS_1    0x01
#define IP5306_REG_SYS_2    0x02
#define IP5306_REG_CHG_0    0x20
#define IP5306_REG_CHG_1    0x21
#define IP5306_REG_CHG_2    0x22
#define IP5306_REG_CHG_3    0x23
#define IP5306_REG_CHG_4    0x24
#define IP5306_REG_READ_0   0x70
#define IP5306_REG_READ_1   0x71
#define IP5306_REG_READ_2   0x72
#define IP5306_REG_READ_3   0x77
#define IP5306_REG_READ_4   0x78
// Macros to read specific bits from IP5306 registers
#define IP5306_GetKeyOffEnabled()               ip5306_get_bits(IP5306_REG_SYS_0, 0, 1)
#define IP5306_GetBoostOutputEnabled()          ip5306_get_bits(IP5306_REG_SYS_0, 1, 1)
#define IP5306_GetPowerOnLoadEnabled()          ip5306_get_bits(IP5306_REG_SYS_0, 2, 1)
#define IP5306_GetChargerEnabled()              ip5306_get_bits(IP5306_REG_SYS_0, 4, 1)
#define IP5306_GetBoostEnabled()                ip5306_get_bits(IP5306_REG_SYS_0, 5, 1)
#define IP5306_GetLowBatShutdownEnable()        ip5306_get_bits(IP5306_REG_SYS_1, 0, 1)
#define IP5306_GetBoostAfterVin()               ip5306_get_bits(IP5306_REG_SYS_1, 2, 1)
#define IP5306_GetShortPressBoostSwitchEnable() ip5306_get_bits(IP5306_REG_SYS_1, 5, 1)
#define IP5306_GetFlashlightClicks()            ip5306_get_bits(IP5306_REG_SYS_1, 6, 1)
#define IP5306_GetBoostOffClicks()              ip5306_get_bits(IP5306_REG_SYS_1, 7, 1)
#define IP5306_GetLightLoadShutdownTime()       ip5306_get_bits(IP5306_REG_SYS_2, 2, 2)
#define IP5306_GetLongPressTime()               ip5306_get_bits(IP5306_REG_SYS_2, 4, 1)
#define IP5306_GetChargingFullStopVoltage()     ip5306_get_bits(IP5306_REG_CHG_0, 0, 2)
#define IP5306_GetChargeUnderVoltageLoop()      ip5306_get_bits(IP5306_REG_CHG_1, 2, 3)
#define IP5306_GetEndChargeCurrentDetection()  ip5306_get_bits(IP5306_REG_CHG_1, 6, 2)
#define IP5306_GetVoltagePressure()             ip5306_get_bits(IP5306_REG_CHG_2, 0, 2)
#define IP5306_GetChargeCutoffVoltage()         ip5306_get_bits(IP5306_REG_CHG_2, 2, 2)
#define IP5306_GetChargeCCLoop()                ip5306_get_bits(IP5306_REG_CHG_3, 5, 1)
#define IP5306_GetVinCurrent()                  ip5306_get_bits(IP5306_REG_CHG_4, 0, 5)
#define IP5306_GetShortPressDetected()          ip5306_get_bits(IP5306_REG_READ_3, 0, 1)
#define IP5306_GetLongPressDetected()           ip5306_get_bits(IP5306_REG_READ_3, 1, 1)
#define IP5306_GetDoubleClickDetected()         ip5306_get_bits(IP5306_REG_READ_3, 2, 1)
#define IP5306_GetPowerSource()                 ip5306_get_bits(IP5306_REG_READ_0, 3, 1) // 0: BATTERY, 1: VIN (USB)
#define IP5306_GetBatteryFull()                 ip5306_get_bits(IP5306_REG_READ_1, 3, 1)
#define IP5306_GetOutputLoad()                  ip5306_get_bits(IP5306_REG_READ_2, 2, 1)
#define IP5306_GetLevelLeds()                   ((~ip5306_get_bits(IP5306_REG_READ_4, 4, 4)) & 0x0F)
#define IP5306_LEDS2PCT(byte)  \
  ((byte & 0x01 ? 25 : 0) + \
   (byte & 0x02 ? 25 : 0) + \
   (byte & 0x04 ? 25 : 0) + \
   (byte & 0x08 ? 25 : 0))
   
   
#define ES8388_ADDR 0x10   
   
// ES8388 register 
#define ES8388_CONTROL1 0x00
#define ES8388_CONTROL2 0x01
#define ES8388_CHIPPOWER 0x02
#define ES8388_ADCPOWER 0x03
#define ES8388_DACPOWER 0x04
#define ES8388_CHIPLOPOW1 0x05
#define ES8388_CHIPLOPOW2 0x06
#define ES8388_ANAVOLMANAG 0x07
#define ES8388_MASTERMODE 0x08

// ADC 
#define ES8388_ADCCONTROL1 0x09
#define ES8388_ADCCONTROL2 0x0a
#define ES8388_ADCCONTROL3 0x0b
#define ES8388_ADCCONTROL4 0x0c
#define ES8388_ADCCONTROL5 0x0d
#define ES8388_ADCCONTROL6 0x0e
#define ES8388_ADCCONTROL7 0x0f
#define ES8388_ADCCONTROL8 0x10
#define ES8388_ADCCONTROL9 0x11
#define ES8388_ADCCONTROL10 0x12
#define ES8388_ADCCONTROL11 0x13
#define ES8388_ADCCONTROL12 0x14
#define ES8388_ADCCONTROL13 0x15
#define ES8388_ADCCONTROL14 0x16

// DAC 
#define ES8388_DACCONTROL1 0x17
#define ES8388_DACCONTROL2 0x18
#define ES8388_DACCONTROL3 0x19
#define ES8388_DACCONTROL4 0x1a
#define ES8388_DACCONTROL5 0x1b
#define ES8388_DACCONTROL6 0x1c
#define ES8388_DACCONTROL7 0x1d
#define ES8388_DACCONTROL8 0x1e
#define ES8388_DACCONTROL9 0x1f
#define ES8388_DACCONTROL10 0x20
#define ES8388_DACCONTROL11 0x21
#define ES8388_DACCONTROL12 0x22
#define ES8388_DACCONTROL13 0x23
#define ES8388_DACCONTROL14 0x24
#define ES8388_DACCONTROL15 0x25
#define ES8388_DACCONTROL16 0x26
#define ES8388_DACCONTROL17 0x27
#define ES8388_DACCONTROL18 0x28
#define ES8388_DACCONTROL19 0x29
#define ES8388_DACCONTROL20 0x2a
#define ES8388_DACCONTROL21 0x2b
#define ES8388_DACCONTROL22 0x2c
#define ES8388_DACCONTROL23 0x2d
#define ES8388_DACCONTROL24 0x2e
#define ES8388_DACCONTROL25 0x2f
#define ES8388_DACCONTROL26 0x30
#define ES8388_DACCONTROL27 0x31
#define ES8388_DACCONTROL28 0x32
#define ES8388_DACCONTROL29 0x33
#define ES8388_DACCONTROL30 0x34
class MuseNeoPixel {
public:
    explicit MuseNeoPixel(int pin = NEOPIXEL_PIN, uint32_t freqHz = 10'000'000);
    bool begin();
    void setPixelColor(uint32_t color);
    void show();
    uint32_t color(uint8_t r, uint8_t g, uint8_t b) const;

private:
    void updateFrame();

    int pin_;
    uint32_t freqHz_;
    bool initialized_;
    uint8_t grb_[3];
    rmt_data_t frame_[24];
};

class MuseLuxe {
public:
    MuseLuxe();
    void begin();
    uint8_t getBatteryPercentage(); // Method to return battery percentage

    // Public methods to control the NeoPixel
    void setPixelColor(uint32_t color);
    void showPixel();
    uint32_t getColor(uint8_t r, uint8_t g, uint8_t b);

    // Public methods to get IP5306 statuses
    bool isCharging();             // Returns true if charging
    uint8_t getVinCurrent();       // Returns VIN current in mA
    uint8_t getVoltagePressure();  // Returns voltage pressure in mV


private:
    MuseNeoPixel neoPixel_;
    uint8_t getBatteryLevel();

    // IP5306 helper functions
    int ip5306_get_reg(uint8_t reg);
    int ip5306_set_reg(uint8_t reg, uint8_t value);
    uint8_t ip5306_get_bits(uint8_t reg, uint8_t index, uint8_t bits);
    void ip5306_set_bits(uint8_t reg, uint8_t index, uint8_t bits, uint8_t value);
};



class ES8388
{

//    bool write_reg(uint8_t slave_add, uint8_t reg_add, uint8_t data);
//    bool read_reg(uint8_t slave_add, uint8_t reg_add, uint8_t &data);
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
    bool write_reg(uint8_t slave_add, uint8_t reg_add, uint8_t data);
    bool read_reg(uint8_t slave_add, uint8_t reg_add, uint8_t &data);  
    void select_out1();
    void select_out2();  
    void select_internal_microphone();
    void select_external_microphone();

};
#endif // MUSELUXE_H
