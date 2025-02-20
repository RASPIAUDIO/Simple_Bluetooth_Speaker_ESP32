// museluxe.cpp - Implementation file for ESP Muse Luxe NeoPixel, Battery Management System (BMS) using IP5306, and ES8388 Audio Codec
// (c) 2024 RASPiAudio
#ifdef ARDUINO_ESP32_DEV

#include "museWrover.h"




MuseLuxe::MuseLuxe() : pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800) {}

void MuseLuxe::begin() {
    // Initialize the NeoPixel strip object (REQUIRED for the Muse Luxe's RGB LED)
    pixels.begin();
    pixels.show(); // Initialize all to 'off'

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
    pixels.setPixelColor(0, color);
}

void MuseLuxe::showPixel() {
    pixels.show();
}

uint32_t MuseLuxe::getColor(uint8_t r, uint8_t g, uint8_t b) {
    return pixels.Color(r, g, b);
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

        // mute DAC during setup, power up all systems, slave mode 
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x04);
        res &= write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50);
        res &= write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00);
        res &= write_reg(ES8388_ADDR, ES8388_MASTERMODE, 0x00);

        // power up DAC and enable LOUT1+2 / ROUT1+2, ADC sample rate = DAC sample rate 
        res &= write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x3e);
        res &= write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x12);

        // DAC I2S setup: 16 bit word length, I2S format; MCLK / Fs = 256
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL1, 0x18);
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL2, 0x02);

        // DAC to output route mixer configuration: ADC MIX TO OUTPUT 
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x1B);
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x90);
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x90);

        // DAC and ADC use same LRCK, enable MCLK input; output resistance setup 
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80);
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL23, 0x00);

        // DAC volume control: 0dB (maximum, unattenuated)  
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL5, 0x00);
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL4, 0x00);

        // power down ADC while configuring; volume: +9dB for both channels 
        res &= write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0xff);
        res &= write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, 0x88); // 

        // select LINPUT1 / RINPUT1 as ADC input; mono; 16 bit word length, format right-justified, MCLK / Fs = 256 
        res &= write_reg(ES8388_ADDR, ES8388_ADCCONTROL2, 0x00); // 
        res &= write_reg(ES8388_ADDR, ES8388_ADCCONTROL3, 0x08); // 00
        res &= write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, 0x0c);
        res &= write_reg(ES8388_ADDR, ES8388_ADCCONTROL5, 0x02);
        
	// ALC
	// (optimized for voice)
	write_reg(ES8388_ADDR, ES8388_ADCCONTROL10, 0x00);   //by default not set ( see ALC method below)
	  //write_reg(ES8388_ADDR, ES8388_ADCCONTROL10, 0x22); 
	write_reg(ES8388_ADDR,  ES8388_ADCCONTROL11, 0xC0); 
	write_reg(ES8388_ADDR,  ES8388_ADCCONTROL12, 0x12); 
	write_reg(ES8388_ADDR,  ES8388_ADCCONTROL13, 0x06); 
	write_reg(ES8388_ADDR,  ES8388_ADCCONTROL14, 0xC3); 
	
	//  write_reg(ES8388_ADDR,  ES8388_CHIPPOWER, 0x55); 
        // set ADC volume 
        res &= write_reg(ES8388_ADDR, ES8388_ADCCONTROL8, 0x00);
        res &= write_reg(ES8388_ADDR, ES8388_ADCCONTROL9, 0x00);

        // set LOUT1 / ROUT1 volume: 0dB (unattenuated) 
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL24, 0x1e);
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL25, 0x1e);

        // set LOUT2 / ROUT2 volume: 0dB (unattenuated) 
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL26, 0x1e);
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL27, 0x1e);

        // power up and enable DAC; power up ADC (no MIC bias) 
        res &= write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x3c);
        res &= write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x00);
        res &= write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x00);
        
        // set up MCLK) 
//		PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
//       WRITE_PERI_REG(PIN_CTRL, 0xFFF0);
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

void ES8388::ALC(const bool valid)
{
    uint8_t val;
    val = (valid) ? 0xE2 : 0x00;
    write_reg(ES8388_ADDR, ES8388_ADCCONTROL10, val);  
    val = (valid) ? 0x77 : 0x88;
    write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, val); // +21/24db    
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
#endif
