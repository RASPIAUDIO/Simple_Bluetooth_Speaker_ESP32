// ***** Muse Proto as a simple recorder*****
// -- records sound on a SD .wav file (BUTTON_PROTO_ZERO to start/stop recording)
// -- plays it
///////////////////////////////////////////////


#include "museWrover.h"
#include "ESP_I2S.h"
#include "wav_header.h"
#include "driver/gpio.h"
#include "SPI.h"
#include "SD.h"
#include "FS.h"

File f;

// Initialize your objects and variables
I2SClass i2s;
pcm_wav_header_t H = PCM_WAV_HEADER_DEFAULT(0, 16, 44100, 2);


void setup() {
  Serial.begin(115200);

//////I2S init
  i2s.setPins(I2S_BCLK, I2S_LRCK, I2S_SDOUT, I2S_SDIN, I2S_MCLK);

  if (!i2s.begin(I2S_MODE_STD, 44100, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_RIGHT)) {
    Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }

  if (!i2s.configureRX(44100, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO, I2S_RX_TRANSFORM_NONE) ){
    Serial.println("Failed to initialize RX Channel!");
    while (1); // do nothing
  }

 if (!i2s.configureTX(44100, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO)) {
    Serial.println("Failed to initialize TX Channel!");
    while (1); // do nothing
  }
  printf("I2S init OK\n");
  
////// GPIOs init  
  // power enable
  gpio_reset_pin(GPIO_PA_EN);
  gpio_set_direction(GPIO_PA_EN, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_PA_EN, HIGH);
 /* 
  // gain => 15dB
  gpio_reset_pin(GPIO_PROTO_GAIN);
  gpio_set_direction(GPIO_PROTO_GAIN, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(GPIO_PROTO_GAIN, GPIO_PULLDOWN_ONLY);
*/ 
  // PAUSE
  gpio_reset_pin(BUTTON_PROTO_ZERO);
  gpio_set_direction(BUTTON_PROTO_ZERO, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_PROTO_ZERO, GPIO_PULLUP_ONLY);
  
  // SD init
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI); 
  SPI.setFrequency(1000000);
  if( !SD.begin(SD_CS))
  {
    printf( "SD init failed\n");
    while(1); 
  }
  printf("SD init OK\n");
  

}

void loop() {
#define T 1024
    size_t t; 
    char buf[T/2];
    char bufb[T];
    int tb;
    int L;
    if (gpio_get_level(BUTTON_PROTO_ZERO) == 0)
    {
    while(gpio_get_level(BUTTON_PROTO_ZERO) == 0) delay(10);
    f = SD.open("/record.wav", "w");
    f.seek(PCM_WAV_HEADER_SIZE);   // leave room for the header
    
    printf("recording...\n");
    L = 0;
    uint8_t M, N;
    while(gpio_get_level(BUTTON_PROTO_ZERO) == 1)
    {
     t = i2s.readBytes(buf, T/2);
 // one channel ==> two channels   with amplification
 
     union
     {
      int16_t B;
      uint8_t l[2];
     }S;
    
     int amp = 3 ; // 0 => 0dB   1 => 6dB 2 => 12dB 3 => 18dB 4 => 24dB
    
     for(int i=0;i<t;i=i+2)
     {
    
 // PCM amp     
      S.l[0] = buf[i];
      S.l[1] = buf[i+1];
      S.B = S.B << amp;
      buf[i] = S.l[0];
      buf[i+1] = S.l[1];
//
    
      bufb[i+i] = buf[i];
      bufb[i+i+1] = buf[i+1];
      bufb[i+i+2] = buf[i];
      bufb[i+i+3] = buf[i+1];     
     }   
     f.write((const uint8_t*)bufb, t+t);    
     L = L + 2*t;   
    }

    f.seek(0);
 // to refresh size values   
    H.descriptor_chunk.chunk_size = L + PCM_WAV_HEADER_SIZE ;
    H.data_chunk.subchunk_size = L;
    
 // write the header   
    f.write((uint8_t*)&H, PCM_WAV_HEADER_SIZE);
    
    f.seek(L + PCM_WAV_HEADER_SIZE);
    f.close();
    
    
    while(gpio_get_level(BUTTON_PROTO_ZERO) == 0) delay(10);
    printf("playing...\n");
    f = SD.open("/record.wav", "r");
    f.seek(PCM_WAV_HEADER_SIZE);    // skip the header
    do
    {
     tb = f.read((uint8_t*)bufb, T);
     i2s.write((uint8_t*)bufb, (size_t)tb);
    }while(tb > 0);
    f.close();
  }

    delay(100);
  }
