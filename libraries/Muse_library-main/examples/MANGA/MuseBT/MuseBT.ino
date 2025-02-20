// ***** Muse Manga as a Bluetooth speaker *****
// BT speaker name ==> MANGA-xxxxxx
//
///////////////////////////////////////////////


#include "museWrover.h"
#include "BluetoothA2DPSink.h"  // https://github.com/pschatzmann/ESP32-A2DP
#include "ESP_I2S.h"
#include "driver/gpio.h"
#include "esp_mac.h"
#include <Wire.h>
#include <Adafruit_SH1106.h>    // https://github.com/davidperrenoud/Adafruit_SH1106
#include "ESP32Encoder.h"       // https://github.com/madhephaestus/ESP32Encoder/

bool BPause = false;
uint8_t volume = 100;                            // 0...127
#define volumeMAX 127

// Initialize your objects and variables
I2SClass i2s;
BluetoothA2DPSink a2dp_sink(i2s);
ESP32Encoder V;


Adafruit_SH1106 display(IIC_DATA, IIC_CLK);

uint8_t mac [6];
char macStr[20];
char dev_name [30];
char L[] = "RASPIAUDIO";

void setup() {
  Serial.begin(115200);
  display.begin(SH1106_SWITCHCAPVCC, 0x3C); 
  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(5,20);
  int i = 0;
  while(L[i] != 0) display.write(L[i++]);  
  display.display();

  
//////////////////////////////////////////////////
//Encoders init
//////////////////////////////////////////////////
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  V.attachHalfQuad(MANGA_ENC_A, MANGA_ENC_B);
  
//////I2S init
  i2s.setPins(I2S_BCLK, I2S_LRCK, I2S_SDOUT, I2S_SDIN, I2S_MCLK);
  if (!i2s.begin(I2S_MODE_STD, 44100, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
    Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }
  
// GPIOs init 
// PAUSE
  gpio_reset_pin(BUTTON_MANGA_CLICK);
  gpio_set_direction(BUTTON_MANGA_CLICK, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_MANGA_CLICK, GPIO_PULLUP_ONLY); 
////// Specific BT device name (using  MAC ref)

  esp_read_mac((uint8_t*)&mac, ESP_MAC_BT);
  snprintf(macStr, 19, "-%x%x%x", mac[3], mac[4], mac[5]);
  strcpy(dev_name, "MANGA");
  strcat(dev_name, macStr);
  printf("Device name : %s\n", dev_name);
  
////// Bluetooth A2DP sink init
  a2dp_sink.start(dev_name);
  Serial.println("Bluetooth A2DP Sink Initialized");
  V.setCount(volume);
  a2dp_sink.set_volume(volume);
}

void loop() {
  int n;
  if (a2dp_sink.get_audio_state() != ESP_A2D_AUDIO_STATE_STARTED) delay(1000);
  n = V.getCount();
  if(n != volume)
  {
     volume = n;
     if(volume > volumeMAX) volume = volumeMAX;
     if(volume < 0) volume = 0;
     a2dp_sink.set_volume(volume);    
  }

/////// Play/Pause button (using AVRC)  
  if (gpio_get_level(BUTTON_MANGA_CLICK) == 0)
  {
    while(gpio_get_level(BUTTON_MANGA_CLICK) == 0) delay(10);
    
    if (BPause == false)
    {
      a2dp_sink.pause();
      BPause = true;
    }
    else
    {   
      a2dp_sink.play();
      BPause = false;
    }
  }
  
    delay(100);
  }
