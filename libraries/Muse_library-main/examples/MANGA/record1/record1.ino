// ***** Muse MANGA as a simple recorder*****
// -- records sound 5 sec after clicking on BUTTON_MANGA_CLICK
// -- plays it
///////////////////////////////////////////////


#include "museWrover.h"
#include "ESP_I2S.h"


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
  
  // BUTTON
  gpio_reset_pin(BUTTON_MANGA_CLICK);
  gpio_set_direction(BUTTON_MANGA_CLICK, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_MANGA_CLICK, GPIO_PULLUP_ONLY);
  

  
  printf("initialization OK\n");
  delay(1000);
}

void loop() {
  
    if (gpio_get_level(BUTTON_MANGA_CLICK) == 0)
  {
    while(gpio_get_level(BUTTON_MANGA_CLICK) == 0) delay(10);
    printf("recording...\n");
// Record 5 seconds of audio data
    wav_buffer = i2s.recordWAV(5, &wav_size);
    delay(2000);
    printf("playing...  %d\n", wav_size);  
    i2s.playWAV(wav_buffer, wav_size);
  }
  
    delay(100);
  }
