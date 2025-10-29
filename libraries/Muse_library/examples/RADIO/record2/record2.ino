// ***** Muse RADIO as a simple recorder*****
// -- records sound on a SD .wav file (button CLICK1 to start/stop recording)
// -- plays it
///////////////////////////////////////////////


#include "museS3.h"
#include "ESP_I2S.h"
#include "wav_header.h"
#include "driver/gpio.h"
#include "SPI.h"
#include "SD_MMC.h"
#include "FS.h"

int volume = 100;                            // 0...100
int microVol = 90;                           // 0...96
File f;

// Initialize your objects and variables
I2SClass i2s;
ES8388 es;
pcm_wav_header_t H = PCM_WAV_HEADER_DEFAULT(0, 16, 44100, 2);


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
  
  // SD init
   if (! SD_MMC.setPins(SD_MMC_clk, SD_MMC_cmd, SD_MMC_d0)) {
    printf("Pin change failed!\n");
  }
    printf ("SD_MMC init OK\n");
    
    if (!SD_MMC.begin("/sdcard", true)) {
    printf("Card Mount Failed\n");
    }

  
//////I2S init
  i2s.setPins(I2S_BCLK, I2S_LRCK, I2S_SDOUT, I2S_SDIN, I2S_MCLK);

  if (!i2s.begin(I2S_MODE_STD, 44100, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
    Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }
// codec ES8388 init
  Serial.printf("Connect to ES8388 codec... ");
  while (not es.begin(IIC_DATA, IIC_CLK))
  {
      Serial.printf("Failed!\n");
      while(1);
  }
  Serial.printf("OK\n");
// volumes
  es.volume(ES8388::ES_MAIN, volume);
  es.volume(ES8388::ES_OUT1, volume);
  es.mute(ES8388::ES_OUT1, false);
  es.mute(ES8388::ES_MAIN, false);
  es.microphone_volume(microVol);
  es.ALC(false);
  es.Amp_D(true);

 
}

void loop() {
#define T 1024
    size_t t; 
    char buf[T];
    int tb;
    int L;
    if (gpio_get_level(CLICK1) == 0)
    {
    while(gpio_get_level(CLICK1) == 0) delay(10);
    f = SD_MMC.open("/record.wav", "w");
    f.seek(PCM_WAV_HEADER_SIZE);   // leave room for the header
    
    printf("recording...\n");
    L = 0;
    while(gpio_get_level(CLICK1) == 1)
    {
      t = i2s.readBytes(buf, T);
      f.write((const uint8_t*)buf, (int) t);
      L += t;
    }
    // write the header
    f.seek(0);
    H.descriptor_chunk.chunk_size = L + PCM_WAV_HEADER_SIZE ;
    H.data_chunk.subchunk_size = L;
    f.write((uint8_t*)&H, PCM_WAV_HEADER_SIZE);
    
    f.seek(L + PCM_WAV_HEADER_SIZE);
    f.close();
    
    
    while(gpio_get_level(CLICK1) == 0) delay(10);
    printf("playing...\n");
    f = SD_MMC.open("/record.wav", "r");
    f.seek(PCM_WAV_HEADER_SIZE);    // skip the header
    do
    {
     tb = f.read((uint8_t*)buf, T);
     i2s.write((uint8_t*)buf, (size_t)tb);
    }while(tb > 0);
    f.close();
  }

    delay(100);
  }
