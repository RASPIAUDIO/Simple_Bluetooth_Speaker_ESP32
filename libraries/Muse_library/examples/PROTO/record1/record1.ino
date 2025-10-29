// ***** Muse PROTO as a simple recorder*****
// -- records sound 5 sec after clicking on BUTTON_PROTO_ZERO (close to SD)
// -- plays it
///////////////////////////////////////////////


#include "museWrover.h"
#include "ESP_I2S.h"
#include "driver/gpio.h"

// Initialize your objects and variables
I2SClass i2s;


  uint8_t *wav_buffer;
  size_t wav_size;

void setup() {
  Serial.begin(115200);
  
//////I2S init
  i2s.setPins(I2S_BCLK, I2S_LRCK, I2S_SDOUT, I2S_SDIN, I2S_MCLK);

  if (!i2s.begin(I2S_MODE_STD, 16000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_RIGHT)) {
    printf("Failed to initialize I2S!\n");
    while (1); // do nothing
  }
   
////// GPIOs init  
  // power enable
  gpio_reset_pin(GPIO_PA_EN);
  gpio_set_direction(GPIO_PA_EN, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_PA_EN, HIGH);
  
  // gain => 12dB
  gpio_reset_pin(GPIO_PROTO_GAIN);
  gpio_set_direction(GPIO_PROTO_GAIN, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_PROTO_GAIN, LOW);
  
  // BUTTON
  gpio_reset_pin(BUTTON_PROTO_ZERO);
  gpio_set_direction(BUTTON_PROTO_ZERO, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_PROTO_ZERO, GPIO_PULLUP_ONLY);
  

  
  printf("initialization OK\n");
  delay(1000);
}

void loop() {
  
    if (gpio_get_level(BUTTON_PROTO_ZERO) == 0)
  {
    while(gpio_get_level(BUTTON_PROTO_ZERO) == 0) delay(10);
    printf("recording...\n");
// Record 5 seconds of audio data
    wav_buffer = i2s.recordWAV(5, &wav_size);
    delay(2000);
    printf("playing...  %d\n", wav_size);  
    i2s.playWAV(wav_buffer, wav_size);
  }
  
    delay(100);
  }
