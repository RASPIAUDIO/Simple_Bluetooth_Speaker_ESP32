// ***** MUSE Luxe Factory test*****
// 
///////////////////////////////////////////////


#include "museWrover.h"
#include "ESP_I2S.h"
#include "driver/gpio.h"
#include "esp_mac.h"
#include <Adafruit_NeoPixel.h>
#include "SPIFFS.h"
#include <SPI.h>
#include <SD.h>
#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "esp_system.h"

bool BPause = false;
int volume = 60;                            // 0...100
#define maxVol 100



uint8_t mac [6];
char macStr[20];
char dev_name [30];
bool jack = false;

I2SClass i2s;
ES8388 es;
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
#define RED Color(255,0,0)
#define GREEN Color(0,255,0)
#define YELLOW Color(255,255,0)
#define BLUE Color(0,0,255)
#define WHITE Color(255,255,255)
uint8_t c[88*200];
bool testOK;
#define BLOCK_SIZE 128

/////////////////////////////////////////////////////////////////////////////////////////
//   FACTORY TEST
////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
//
// Task playing audio signal (mono, 8 bits, 44100)
//
////////////////////////////////////////////////////////////////////////////////////////
static void playAudio(void* data)
{ 
   int16_t s0,s1;
   //int8_t c[88*200];
   int l;  
   size_t t;
   uint16_t s16[64];

   int a = 1;
//  i2s_set_clk(I2SR,44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);

//  i2s_zero_dma_buffer(I2SR);
  for(int i = 0;i < 200*88; i++)
   {  
// sample value 8bits -> 16        
       s0 = (((int16_t)(c[i]&0xFF)) - 128) << 8 ;       
// attenuation  a 
       s0 = s0 >> a; 
// buffering -> s16                
       s16[i % 64] = (uint16_t)s0;      
      if(((i+1) % 64) ==  0)  
      {
//sending       
      i2s.write((uint8_t*)s16,(size_t)128);   
      }      
   }
// muting after playing   
   for(int i=0;i<64;i++)s16[i] = 0;

   i2s.write((uint8_t*)s16,(size_t)128);     
//   i2s_zero_dma_buffer(I2SR);

   printf ("Play End\n");
   vTaskDelete(NULL);
}
      
//////////////////////////////////////////////////////////////////////////////
// Task recording signal mono 16bits 44100
//
//////////////////////////////////////////////////////////////////////////////
static void recordAudio(void* data)
{
  static const int bytesToRead = 32000;
  uint8_t b[32000];
  size_t t;
  int16_t v[1024];
  int DC; 
  int i,j;
  int16_t max1, max2;
  int imax1, imax2;
  int16_t min1, min2;
  int imin1, imin2;
 // i2s_set_clk(I2SR,44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
  i = 0; 
  do
  {
     int n = 0;
        while(n == 0) n = (int)i2s.readBytes((char*)&b[i],(size_t)128 );
     i = i + n;
  }while(i < bytesToRead);

//selecting sample (1024)
j = 0;
for(i=16000;i<18048;i=i+2)
{
  v[j++] = (int16_t)(b[i+1] << 8) + (int16_t) b[i];
}

// DC component
DC = 0;
for(j=0;j<1024;j++)
{
  DC = DC + v[j];
}
DC = DC / 1024;
printf("DC = %d\n",DC);

// correction
for(j=0;j<1024;j++) 
{
  v[j] = v[j] - DC;
}
 max1 = max2 = 0;
 
//first max
 for(j=0;j<88;j++)
 {
  if(v[j] > max1)
  {
    max1 = v[j];
    imax1 = j;
  }
 }
// next max
 for(j=88;j<176;j++)
 {
  if(v[j] > max2)
  {
    max2 = v[j];
    imax2 = j;
  }
 }
 
//first min
 min1 = min2 = 0;
 for(j=0;j<88;j++)
 {
  if(v[j] < min1)
  {
    min1 = v[j];
    imin1 = j;
  }
 }
//next min
 for(j=88;j<176;j++)
 {
  if(v[j] < min2)
  {
    min2 = v[j];
    imin2 = j;
  }
 }
  printf("==================> %d %d  %d %d %d\n",max1,max2, imax1,imax2, imax2-imax1);
  printf("==================> %d %d  %d %d %d\n",min1,min2, imin1,imin2, imin2-imin1);
  
// test 88 samples = 2ms -> 500hz
 // if (((imax2-imax1) == 88) && ((imin2-imin1) == 88) && (abs(imax1-imin1) >= 40) && (abs(imax1-imin1)<=50)) testOK = true;
#define DT 4
#define LO (88 - DT)
#define HI (88 + DT)
#define LOH 40
#define HIH 52
  testOK = true;
  if((imax2-imax1) < LO) testOK = false;
  if((imax2-imax1) > HI) testOK = false;
  if((imin2-imin1) < LO) testOK = false;
  if((imin2-imin1) > HI) testOK = false;
  if(abs(imax1-imin1) <= LOH) testOK = false;
  if(abs(imax1-imin1) >= HIH) testOK = false; 
  printf("fin\n");
  vTaskDelete(NULL);
}



///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
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

 int test_phase;
  int i;
  
  printf("factory test...\n");
  pixels.setPixelColor(0,pixels.BLUE);
  pixels.show(); 


// ES8388 init
  printf("Connect to ES8388 codec...\n ");
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

// i2s init  
  i2s.setPins(I2S_BCLK, I2S_LRCK, I2S_SDOUT, I2S_SDIN, I2S_MCLK);
  if (!i2s.begin(I2S_MODE_STD, 44100, I2S_DATA_BIT_WIDTH_16BIT,
                 I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
    Serial.println("Failed to initialize I2S!");
    while (1);
  }

  
// computing a 500hz/mono/8bits signal (400 msec)    
double piX2 = 3.14159 * 2;
double v;     
int j;
for(int i=0;i<88*200;i++)
  {
      j = i % 88;
      v = sin(j * piX2 / 88) * 127;
        c[i] = (uint8_t) (v + 128);
   //   printf("%0 2x\n",c[i]);
  }

     
//
test_phase = 0;
//ROUT1 off LOUT1 on
es.write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x90);
es.write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x00);
printf("Left test\n");
//////////////////////////////////////////////////////////////
//up to  5 tests **left speaker + microphone**
/////////////////////////////////////////////////////////////
testOK = false;
i = 0;
while((testOK == false) && (i < 5))
{
//i2s_start(I2SR);  
i++;
// record task
xTaskCreatePinnedToCore(recordAudio, "recordAudio", 40000, NULL, 10, NULL,0);
delay(10);
// play task
xTaskCreatePinnedToCore(playAudio, "playAudio", 20000, NULL, 10, NULL,1);
delay(2000);
//i2s_stop(I2SR);  
}
if(testOK == true)
{
printf("Right test\n");  
test_phase = 1;
//ROUT1 on LOUT1 off
es.write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x00);
es.write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x90);
//////////////////////////////////////////////////////////////
//up to  5 tests **right speaker + microphone**
/////////////////////////////////////////////////////////////
testOK = false;
i = 0;
while((testOK == false) && (i < 5))
{
//i2s_start(I2SR);  
i++;
// record task
xTaskCreatePinnedToCore(recordAudio, "recordAudio", 40000, NULL, 10, NULL,0);
delay(10);
// play task
xTaskCreatePinnedToCore(playAudio, "playAudio", 20000, NULL, 10, NULL,1);
delay(2000);
//i2s_stop(I2SR);  
}
}
if(testOK == true)
{
 test_phase = 2;
//////////////////////////////////////////////////////////////////// 
//test write/read on SD
////////////////////////////////////////////////////////////////////  
 char b[15];

 SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
 delay(500);
 if(!SD.begin(SD_CS))
  {
    printf("init SD failed!\n");
    testOK = false;
  }  
 File f = SD.open("/record.wav", FILE_WRITE);
 f.write((uint8_t*)"MuseRosAndCO", 13);
 f.close(); 
 f = SD.open("/record.wav", FILE_READ);
 f.read ((uint8_t*)b, 13);
 f.close(); 
 if(strcmp(b, "MuseRosAndCO") != 0)
 {
 testOK = false;
 }
//
SD.remove("/record.wav");
}

////////////////////////////////////////////////////////////////////
// displaying  test result
//  led green => everything OK
//  led yellow => left speaker and/or microphone problem
//  led red => right speaker and/or microphone problem
//  led white => SD problem
//  
//////////////////////////////////////////////////////////////////

if(testOK == true)
{
  pixels.setPixelColor(0, pixels.GREEN);
  printf("Everything OK\n");
  SPIFFS.remove("/testToDo");
}
else
 {
switch(test_phase)
{
  case 0 : 
    pixels.setPixelColor(0, pixels.WHITE);
    printf("left speaker error\n");
    break;
  case 1 : 
    pixels.setPixelColor(0, pixels.RED);
    printf("right speaker error\n");
    break;
  case 2 : 
    pixels.setPixelColor(0, pixels.YELLOW);
    printf("SD error\n");
    break;        
}

 }

 pixels.show();
    const esp_partition_t* partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
    if (partition != NULL) {
      esp_ota_set_boot_partition(partition);
      delay(2000);
      esp_restart();
    }

 
 while(true) delay(1000);
  

}

void loop() {
 
    vTaskDelay(100/portTICK_PERIOD_MS);
  }

  


  
