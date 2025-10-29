// museluxe.cpp - Implementation file for ESP Muse Luxe NeoPixel, Battery Management System (BMS) using IP5306, and ES8388 Audio Codec
// (c) 2024 RASPiAudio
#ifdef ARDUINO_ESP32_DEV

#include "museWrover.h"

#define ES8388_ADDR 0x10  

namespace {
constexpr uint8_t kNeoPixelSymbolCount = 24;
constexpr uint8_t kNeoPixelBitOneHighTicks = 8;  // ~0.8µs at 10MHz
constexpr uint8_t kNeoPixelBitOneLowTicks  = 5;  // ~0.5µs at 10MHz
constexpr uint8_t kNeoPixelBitZeroHighTicks = 4; // ~0.4µs at 10MHz
constexpr uint8_t kNeoPixelBitZeroLowTicks  = 9; // ~0.9µs at 10MHz
constexpr uint32_t kNeoPixelResetDelayUs = 80;
}

MuseNeoPixel::MuseNeoPixel(int pin, uint32_t freqHz)
    : pin_(pin), freqHz_(freqHz), initialized_(false), grb_{0, 0, 0}, frame_{} {}

bool MuseNeoPixel::begin() {
    if (initialized_) {
        return true;
    }
    if (!rmtInit(pin_, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, freqHz_)) {
        return false;
    }
    if (!rmtSetEOT(pin_, LOW)) {
        return false;
    }
    initialized_ = true;
    return true;
}

void MuseNeoPixel::setPixelColor(uint32_t color) {
    grb_[0] = (color >> 16) & 0xFF;  // G
    grb_[1] = (color >> 8) & 0xFF;   // R
    grb_[2] = color & 0xFF;          // B
}

uint32_t MuseNeoPixel::color(uint8_t r, uint8_t g, uint8_t b) const {
    return (static_cast<uint32_t>(g) << 16) |
           (static_cast<uint32_t>(r) << 8) |
           static_cast<uint32_t>(b);
}

void MuseNeoPixel::updateFrame() {
    size_t symbolIndex = 0;
    for (uint8_t component = 0; component < 3; ++component) {
        uint8_t value = grb_[component];
        for (int8_t bit = 7; bit >= 0; --bit) {
            bool isOne = (value >> bit) & 0x01;
            frame_[symbolIndex].level0 = 1;
            frame_[symbolIndex].duration0 = isOne ? kNeoPixelBitOneHighTicks : kNeoPixelBitZeroHighTicks;
            frame_[symbolIndex].level1 = 0;
            frame_[symbolIndex].duration1 = isOne ? kNeoPixelBitOneLowTicks : kNeoPixelBitZeroLowTicks;
            ++symbolIndex;
        }
    }
}

void MuseNeoPixel::show() {
    if (!initialized_ && !begin()) {
        return;
    }
    updateFrame();
    if (!rmtWrite(pin_, frame_, kNeoPixelSymbolCount, RMT_WAIT_FOR_EVER)) {
        return;
    }
    delayMicroseconds(kNeoPixelResetDelayUs);
}


MuseLuxe::MuseLuxe() : neoPixel_() {}


void MuseLuxe::begin() {
    // Initialize the NeoPixel strip object (REQUIRED for the Muse Luxe's RGB LED)
    neoPixel_.begin();
    neoPixel_.setPixelColor(neoPixel_.color(0, 0, 0));
    neoPixel_.show(); // Initialize all to 'off'

    // Initialize I2C for IP5306 and ES8388 communication
    Wire.begin(IIC_DATA, IIC_CLK);

    // Initialize button pins
    pinMode(BUTTON_PAUSE, INPUT_PULLUP);
    pinMode(BUTTON_VOL_MINUS, INPUT_PULLUP);
    pinMode(BUTTON_VOL_PLUS, INPUT_PULLUP);

    // Initialize Serial for debugging
    Serial.begin(115200);
    Serial.println("Muse Luxe initialized");

}

uint8_t MuseLuxe::getBatteryPercentage() {
    return getBatteryLevel();
}

uint8_t MuseLuxe::getBatteryLevel() {
    Wire.beginTransmission(IP5306_I2C_ADDRESS);
    Wire.write(IP5306_REG_READ_4);
    // Explicitly cast address and size to uint8_t
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)IP5306_I2C_ADDRESS, (uint8_t)1)) {
        uint8_t level = Wire.read();
        return IP5306_LEDS2PCT(~level & 0x0F); // LED[0-4] State (inverted)
    }
    Serial.println("Error: Unable to read battery level from IP5306.");
    return 0;
}

// NeoPixel control methods
void MuseLuxe::setPixelColor(uint32_t color) {
    neoPixel_.setPixelColor(color);
}

void MuseLuxe::showPixel() {
    neoPixel_.show();
}

uint32_t MuseLuxe::getColor(uint8_t r, uint8_t g, uint8_t b) {
    return neoPixel_.color(r, g, b);
}

// IP5306 helper functions
int MuseLuxe::ip5306_get_reg(uint8_t reg) {
    Wire.beginTransmission(IP5306_I2C_ADDRESS);
    Wire.write(reg);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)IP5306_I2C_ADDRESS, (uint8_t)1)) {
        return Wire.read();
    }
    return -1;
}

int MuseLuxe::ip5306_set_reg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(IP5306_I2C_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    if (Wire.endTransmission(true) == 0) {
        return 0;
    }
    return -1;
}

uint8_t MuseLuxe::ip5306_get_bits(uint8_t reg, uint8_t index, uint8_t bits) {
    int value = ip5306_get_reg(reg);
    if (value < 0) {
        Serial.printf("ip5306_get_bits fail: 0x%02x\n", reg);
        return 0;
    }
    return (value >> index) & ((1 << bits) - 1);
}

void MuseLuxe::ip5306_set_bits(uint8_t reg, uint8_t index, uint8_t bits, uint8_t value) {
    uint8_t mask = (1 << bits) - 1;
    int v = ip5306_get_reg(reg);
    if (v < 0) {
        Serial.printf("ip5306_set_reg fail: 0x%02x\n", reg);
        return;
    }
    v &= ~(mask << index);
    v |= ((value & mask) << index);
    if (ip5306_set_reg(reg, v)) {
        Serial.printf("ip5306_set_bits fail: 0x%02x\n", reg);
    }
}

// Public methods to get IP5306 statuses
bool MuseLuxe::isCharging() {
    // IP5306_GetPowerSource() returns 1 if VIN (USB) is connected (charging), 0 if battery
    return IP5306_GetPowerSource();
}

uint8_t MuseLuxe::getVinCurrent() {
    return (IP5306_GetVinCurrent() * 100) + 50; // ImA = (v * 100) + 50
}

uint8_t MuseLuxe::getVoltagePressure() {
    return IP5306_GetVoltagePressure() * 14; // Voltage Pressure = bits * 14mV
} 

//////////////////////////////////////////////////////////////////////////////////////////




bool ES8388::begin(int sda, int scl, uint32_t frequency)
{
    bool res = identify(sda, scl, frequency);

    if (res == true)
    {
        // reset
        write_reg(ES8388_ADDR, 0x00, 0x80);
        write_reg(ES8388_ADDR, 0x00, 0x00);

        // mute
        write_reg(ES8388_ADDR, 25, 0x04); // Reg 25 decimal = 0x19
        write_reg(ES8388_ADDR, 1, 0x50);

        // power up
        write_reg(ES8388_ADDR, 2, 0x00);

        // slave mode
        write_reg(ES8388_ADDR, 8, 0x00);

        // DAC power-down
        write_reg(ES8388_ADDR, 4, 0xC0);

        // vmidsel/500k ADC/DAC idem
        write_reg(ES8388_ADDR, 0, 0x12);
        write_reg(ES8388_ADDR, 1, 0x00);

        // I2S 16 bits
        write_reg(ES8388_ADDR, 23, 0x18); 

        // sample freq 256
        write_reg(ES8388_ADDR, 24, 0x02); 

        // LIN2/RIN2 for mixer
        //write_reg(ES8388_ADDR, 38, 0x09); //

        // left DAC to left mixer
        write_reg(ES8388_ADDR, 39, 0x90); 

        // right DAC to right mixer
        write_reg(ES8388_ADDR, 42, 0x90); // 

        // DACLRC ADCLRC idem
        write_reg(ES8388_ADDR, 43, 0x80); // 
        write_reg(ES8388_ADDR, 45, 0x00); // 

        // DAC volume max
        write_reg(ES8388_ADDR, 27, 0x00); // 
        write_reg(ES8388_ADDR, 26, 0x00); // 

        write_reg(ES8388_ADDR, 2, 0xF0);
        write_reg(ES8388_ADDR, 2, 0x00);
        write_reg(ES8388_ADDR, 29, 0x1C); // 

        // DAC power-up LOUT1/ROUT1 enabled
        write_reg(ES8388_ADDR, 4, 0x30);

        res |=write_reg(ES8388_ADDR, 8, 0x00); // ES8388 in I2S slave mode
        res |=write_reg(ES8388_ADDR, 43, 0x80); // Set ADC and DAC to have the same LRCK
        res |=write_reg(ES8388_ADDR, 0, 0x05); // Start up reference
        res |=write_reg(ES8388_ADDR, 1, 0x40); // Start up reference
        res |=write_reg(ES8388_ADDR, 3, 0x00); // Power on ADC and LIN/RIN input
        res |=write_reg(ES8388_ADDR, 9, 0x77); // MicBoost PGA = +21dB
        res |=write_reg(ES8388_ADDR, 10, 0x00); // LIN1 and RIN1 used as single-ended input

         //write_reg(ES8388_ADDR, 0x0A, 0x50); // LIN2 and RIN2 used as single-ended input
        res |=write_reg(ES8388_ADDR, 12, 0x00); // I2S – 24bits, Ldata = LADC, Rdata = RADC
        res |=write_reg(ES8388_ADDR, 13, 0x02); // MCLK/LRCK = 256
        res |=write_reg(ES8388_ADDR, 16, 0x00); // LADC volume = 0dB
        res |=write_reg(ES8388_ADDR, 17, 0x00); // RADC volume = 0dB
        // ALC

/*
        res |=write_reg(ES8388_ADDR, 18, 0xE2); // ALC enable, PGA Max. Gain=23.5dB, Min. Gain=0dB
        res |=write_reg(ES8388_ADDR, 19, 0xA0); // ALC Target=-4.5dB, ALC Hold time=0ms
        res |=write_reg(ES8388_ADDR, 20, 0x12); // Decay time=820µs, Attack time=416µs
        res |=write_reg(ES8388_ADDR, 21, 0x06); // ALC mode
       res |=write_reg(ES8388_ADDR, 22, 0xC3); // Noise gate=-40.5dB, NGG=0x01(mute ADC)
        
*/

// 1) ALC enable, PGA max gain
res |= write_reg(ES8388_ADDR, 18, 0xE2);

// 2) ALC Target = 0 dB
res |= write_reg(ES8388_ADDR, 19, 0xE0);

// 3) Decay = 5,2 ms, Attack = 2,6 ms
res |= write_reg(ES8388_ADDR, 20, 0x1E);

// 4) ALC mode (restez en mode standard si besoin)
res |= write_reg(ES8388_ADDR, 21, 0x06);

// 5) Noise gate : seuil –60 dB, hold gain, activé
res |= write_reg(ES8388_ADDR, 22, 0x5B);



        // unmute
        write_reg(ES8388_ADDR, 25, 0x00); // 

        // amp validation
        write_reg(ES8388_ADDR, 46, 0x21); // Reg 46  value 33 
        write_reg(ES8388_ADDR, 47, 0x21); // Reg 47  value 33 


    }

    return res;
}

/**
 * @brief (un)mute one of the two outputs or main dac output of the ES8388 by switching of the output register bits. Does not really mute the selected output, causes an attenuation. 
 * hence should be used in conjunction with appropriate volume setting. Main dac output mute does mute both outputs
 * 
 * @param out
 * @param muted
 */

void ES8388::mute(const ES8388_OUT out, const bool muted)
{
    uint8_t reg_addr;
    uint8_t mask_mute;
    uint8_t mask_val;

    switch (out)
    {
    case ES_OUT1:
        reg_addr = ES8388_DACPOWER;
        mask_mute = (3 << 4);
        mask_val = muted ? 0 : mask_mute;
        break;
    case ES_OUT2:
        reg_addr = ES8388_DACPOWER;
        mask_mute = (3 << 2);
        mask_val = muted ? 0 : mask_mute;
        break;
    case ES_MAIN:
    default:
        reg_addr = ES8388_DACCONTROL3;
        mask_mute = 1 << 2;
        mask_val = muted ? mask_mute : 0;
        break;
    }

    uint8_t reg;
    if (read_reg(ES8388_ADDR, reg_addr, reg))
    {
        reg = (reg & ~mask_mute) | (mask_val & mask_mute);
        write_reg(ES8388_ADDR, reg_addr, reg);
    }
}


 /* @brief Set volume gain for the main dac, or for one of the two output channels. Final gain = main gain + out channel gain 
 * 
 * @param out which gain setting to control
 * @param vol 0-100 (100 is max)
 */
 
 
void ES8388::volume(const ES8388_OUT out, const uint8_t vol)
{
    const uint32_t max_vol = 100; // max input volume value

    const int32_t max_vol_val = out == ES8388_OUT::ES_MAIN ? 96 : 0x21; // max register value for ES8388 out volume

    uint8_t lreg = 0, rreg = 0;

    switch (out)
    {
    case ES_MAIN:
        lreg = ES8388_DACCONTROL4;
        rreg = ES8388_DACCONTROL5;
        break;
    case ES_OUT1:
        lreg = ES8388_DACCONTROL24;
        rreg = ES8388_DACCONTROL25;
        break;
    case ES_OUT2:
        lreg = ES8388_DACCONTROL26;
        rreg = ES8388_DACCONTROL27;
        break;
    }

    uint8_t vol_val = vol > max_vol ? max_vol_val : (max_vol_val * vol) / max_vol;

    // main dac volume control is reverse scale (lowest value is loudest)
    // hence we reverse the calculated value
    if (out == ES_MAIN)
    {
        vol_val = max_vol_val - vol_val;
    }

    write_reg(ES8388_ADDR, lreg, vol_val);
    write_reg(ES8388_ADDR, rreg, vol_val);
}


/**
 * @brief Set volume gain for ADC (microphone)
 * @param vol 0-96  (96 is max)
 */
 
 
void ES8388::microphone_volume(const uint8_t vol)
{
    const uint8_t max_vol = 96; // max input volume value
    uint8_t vol_val;
    
    if (vol > max_vol) vol_val = 0; else vol_val = (max_vol - vol) * 2;
    write_reg(ES8388_ADDR, ES8388_ADCCONTROL8, vol_val);
//    write_reg(ES8388_ADDR, ES8388_ADCCONTROL9, vol_val);    
}



void ES8388::Amp_D(const bool valid)
{
if(!valid)
{
  digitalWrite(GPIO_PA_EN, LOW);
  delayMicroseconds(300);
  digitalWrite(GPIO_PA_EN, HIGH);
}
else
{
  digitalWrite(GPIO_PA_EN, LOW);
  delayMicroseconds(300);
  digitalWrite(GPIO_PA_EN,HIGH);
  delayMicroseconds(5);
  digitalWrite(GPIO_PA_EN,LOW);
  delayMicroseconds(5);
  digitalWrite(GPIO_PA_EN,HIGH);
}
}
/**
 * @brief Test if device with I2C address for ES8388 is connected to the I2C bus 
 * 
 * @param sda which pin to use for I2C SDA
 * @param scl which pin to use for I2C SCL
 * @param frequency which frequency to use as I2C bus frequency
 * @return true device was found
 * @return false device was not found
 */
 
bool ES8388::identify(int sda, int scl, uint32_t frequency)
{
    Wire.begin(sda, scl, frequency);
    Wire.beginTransmission(ES8388_ADDR);
    return Wire.endTransmission() == 0;
}
bool ES8388::write_reg(uint8_t slave_add, uint8_t reg_add, uint8_t data)
{
    Wire.beginTransmission(slave_add);
    Wire.write(reg_add);
    Wire.write(data);
    return Wire.endTransmission() == 0;
}

bool ES8388::read_reg(uint8_t slave_add, uint8_t reg_add, uint8_t &data)
{
    bool retval = false;
    Wire.beginTransmission(slave_add);
    Wire.write(reg_add);
    Wire.endTransmission(false);
    Wire.requestFrom((uint16_t)slave_add, (uint8_t)1, true);
    if (Wire.available() >= 1)
    {
        data = Wire.read();
        retval = true;
    }
    return retval;
}
void ES8388::select_out1()
{
write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x30); 
}
void ES8388::select_out2()
{
write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x0c); 
}
void ES8388::select_internal_microphone()
{
// left data => left ADC left data => right ADC
  write_reg(ES8388_ADDR, 12, 0x40);   
}
void ES8388::select_external_microphone()
{
// right data => left ADC right data => right ADC
  write_reg(ES8388_ADDR, 12, 0x80);    
}

#endif
