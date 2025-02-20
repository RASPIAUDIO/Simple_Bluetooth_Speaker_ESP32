// ***** Muse RADIO as a simple recorder*****
// -- records sound (3 sec after clicking on button CLICK1)
// -- plays it
///////////////////////////////////////////////


#include "museS3.h"
#include "ESP_I2S.h"


int volume = 100;                            // 0...100
int microVol = 90;                           // 0...96

// Initialize your objects and variables
I2SClass i2s;
MuseRadio radio;

ES8388 es;
  uint8_t *wav_buffer;
  size_t wav_size;

void setup() {
  Serial.begin(115200);
////// GPIOs init  
  // power enable
  gpio_reset_pin(GPIO_PA_EN);
  gpio_set_direction(GPIO_PA_EN, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_PA_EN, HIGH);

  // PAUSE
  gpio_reset_pin(CLICK1);
  gpio_set_direction(CLICK1, GPIO_MODE_INPUT);
  gpio_set_pull_mode(CLICK1, GPIO_PULLUP_ONLY);
 
  
//////I2S init
  i2s.setPins(I2S_BCLK, I2S_LRCK, I2S_SDOUT, I2S_SDIN, I2S_MCLK);

  if (!i2s.begin(I2S_MODE_STD, 8000, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
    Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }

  printf("Connect to ES8388 codec... ");
  while (not es.begin(IIC_DATA, IIC_CLK))
  {
      printf("Failed!\n");
      delay(1000);
  }
  printf("OK\n");

  es.volume(ES8388::ES_MAIN, volume);
  es.volume(ES8388::ES_OUT1, volume);
  es.mute(ES8388::ES_OUT1, false);
  es.mute(ES8388::ES_MAIN, false);
  es.microphone_volume(microVol);
  es.ALC(false);
  es.Amp_D(true);


}

void loop() {
  if (gpio_get_level(CLICK1) == 0)
    {
    while(gpio_get_level(CLICK1) == 0) delay(10);
    printf("recording...\n");
// Record 5 seconds of audio data
  wav_buffer = i2s.recordWAV(3, &wav_size);
  delay(2000);
  printf("playing...  %d\n", wav_size);  
  i2s.playWAV(wav_buffer, wav_size);
   }
    delay(100);
  }
