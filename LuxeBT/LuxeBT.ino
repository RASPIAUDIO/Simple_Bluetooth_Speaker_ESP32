// ***** Muse LUXE as a Bluetooth speaker *****
// BT speaker name ==> LUXE-xxxxxx
// Pause and +/- volume buttons managed
///////////////////////////////////////////////


#include "museWrover.h"
#include "BluetoothA2DPSink.h"      // https://github.com/pschatzmann/ESP32-A2DP
#include "A2DPVolumeControl.h"
#include "ESP_I2S.h"
#include "driver/gpio.h"
#include "esp_mac.h"
#include <Adafruit_NeoPixel.h>


bool BPause = false;
int volume = 60;                            // 0...100
#define maxVol 100
// Initialize your objects and variables
I2SClass i2s;
BluetoothA2DPSink a2dp_sink(i2s);


uint8_t mac [6];
char macStr[20];
char dev_name [30];
bool jack = false;
ES8388 es;

Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
void setup() {
  Serial.begin(115200);
////// GPIOs init  
  // power enable
  gpio_reset_pin(GPIO_PA_EN);
  gpio_set_direction(GPIO_PA_EN, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_PA_EN, HIGH);

  //BUTTON_VOL_PLUS
  gpio_reset_pin(BUTTON_VOL_PLUS);
  gpio_set_direction(BUTTON_VOL_PLUS, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_VOL_PLUS, GPIO_PULLUP_ONLY);


  //BUTTON_VOL_MINUS
  gpio_reset_pin(BUTTON_VOL_MINUS);
  gpio_set_direction(BUTTON_VOL_MINUS, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_VOL_MINUS, GPIO_PULLUP_ONLY);

  // PAUSE
  gpio_reset_pin(BUTTON_PAUSE);
  gpio_set_direction(BUTTON_PAUSE, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_PAUSE, GPIO_PULLUP_ONLY);

  // Jack_Detect
  gpio_reset_pin(Jack_Detect);
  gpio_set_direction(Jack_Detect, GPIO_MODE_INPUT);
  gpio_set_pull_mode(Jack_Detect, GPIO_PULLUP_ONLY);  

  
  
//////I2S init
  i2s.setPins(I2S_BCLK, I2S_LRCK, I2S_SDOUT, I2S_SDIN, I2S_MCLK);
  if (!i2s.begin(I2S_MODE_STD, 44100, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
    Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }

  Serial.printf("Connect to ES8388 codec... ");
  while (not es.begin(IIC_DATA, IIC_CLK))
  {
      Serial.printf("Failed!\n");
      delay(1000);
  }
  Serial.printf("OK\n");
  es.volume(ES8388::ES_MAIN, maxVol);
  es.volume(ES8388::ES_OUT1, volume);
//    es.volume(ES8388::ES_OUT2, volume);
  es.mute(ES8388::ES_OUT1, false);
//    es.mute(ES8388::ES_OUT2, false);
  es.mute(ES8388::ES_MAIN, false);

////// Specific BT device name (using  MAC ref)

  esp_read_mac((uint8_t*)&mac, ESP_MAC_BT);
  snprintf(macStr, 19, "-%x%x%x", mac[3], mac[4], mac[5]);
  strcpy(dev_name, "LUXE");
  strcat(dev_name, macStr);
  printf("Device name : %s\n", dev_name);
  
////// Bluetooth A2DP sink init
  a2dp_sink.start(dev_name);
  Serial.println("Bluetooth A2DP Sink Initialized");

/////task managing the battery
  xTaskCreatePinnedToCore(battery, "battery", 5000, NULL, 5, NULL,0);  
  
}

void loop() {
  if (a2dp_sink.get_audio_state() != ESP_A2D_AUDIO_STATE_STARTED) delay(1000);

/////// Play/Pause button (using AVRC)  
  if (gpio_get_level(BUTTON_PAUSE) == 0)
  {
    while(gpio_get_level(BUTTON_PAUSE) == 0) delay(10);
    
    if (BPause == false)
    {  
      es.mute(ES8388::ES_MAIN,true);      
      if(jack) 
      {
        es.volume(ES8388::ES_OUT2, 0);  
        es.mute(ES8388::ES_OUT2, true);
      }
      else 
      {
      es.volume(ES8388::ES_OUT1, 0); 
      es.mute(ES8388::ES_OUT1, true);     
      }
      a2dp_sink.pause();
      BPause = true;
    }
    else
    {
      es.mute(ES8388::ES_MAIN,false);
      if(jack) 
      {
      es.volume(ES8388::ES_OUT2, volume);  
      es.mute(ES8388::ES_OUT2, false);      
      }
      else 
      {
      es.volume(ES8388::ES_OUT1, volume);   
      es.mute(ES8388::ES_OUT1, false);
      }
      a2dp_sink.play();
      BPause = false;
    }
  }

////// Vol+ button  
  if (gpio_get_level(BUTTON_VOL_PLUS) == 0)
  {
    while(gpio_get_level(BUTTON_VOL_PLUS) == 0) delay(10);
    volume += 3;
    if(volume > maxVol) volume = maxVol;
    printf(" v = %d\n", volume);
    if(volume < 6)es.volume(ES8388::ES_MAIN, volume); else es.volume(ES8388::ES_MAIN, maxVol);    
    if(jack)   
    es.volume(ES8388::ES_OUT2, volume); 
    else
    es.volume(ES8388::ES_OUT1, volume); 
  }

////// Vol- button  
  if (gpio_get_level(BUTTON_VOL_MINUS) == 0)
  {
    while(gpio_get_level(BUTTON_VOL_MINUS) == 0) delay(10);    
    volume -= 3;
    if(volume < 0) volume = 0;
    printf(" v = %d\n", volume);   
    if(volume < 6)es.volume(ES8388::ES_MAIN, volume); else es.volume(ES8388::ES_MAIN, maxVol);          
    if(jack)   
    es.volume(ES8388::ES_OUT2, volume); 
    else
    es.volume(ES8388::ES_OUT1, volume);     
  }

  
///// Jack_Detect
  if(gpio_get_level(Jack_Detect) == 0)
  {
// jack ON => Rout2/Lout2  amp OFF 
    if(!jack)
    {
    es.volume(ES8388::ES_OUT2, volume); 
    es.select_out2(); 
    gpio_set_level(GPIO_PA_EN, 0); 
    }    
    jack = true;       
  }
  else
  {
// jack OFF => Rout1/Lout1  amp ON  
    if(jack) 
    {
    es.volume(ES8388::ES_OUT1, volume);
    es.select_out1();    
    gpio_set_level(GPIO_PA_EN, 1); 
    }    
    jack = false;     
  }

    vTaskDelay(100/portTICK_PERIOD_MS);
  }

  
//////////////////////////////////////////////////////////////////////////
// task for battery monitoring
//////////////////////////////////////////////////////////////////////////
#define NGREEN 2300
#define NYELLOW 1800
#define RED Color(255,0,0)
#define GREEN Color(0,255,0)
#define YELLOW Color(255,255,0)
#define ADC_battery 33
static void battery(void* pdata)
{
  int val;
  while(1)
  {
   val = analogRead(ADC_battery);
//   printf("Battery : %d\n");
   if(val < NYELLOW) pixels.setPixelColor(0, pixels.RED);       //RED
   else if(val > NGREEN) pixels.setPixelColor(0, pixels.GREEN); //GREEN
   else pixels.setPixelColor(0, pixels.YELLOW);                 //YELLOW
   pixels.show();   
   vTaskDelay(10000/portTICK_PERIOD_MS);
  }
}
  
