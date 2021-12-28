                                                                                                                                                                                                                                                                                                                                                         
/*--------------------------------------------------------------------------------------------------
Introduction
This program is intended to be used with the ESP32 Muse speaker, a portable and affordable bluetooth speaker that is fully programmable. ESP32 Muse is a commercial product that can be purchase here: https://raspiaudio.com/espmuse

Features
ESP32 offers : Bluetooth, Wifi
line Input
SD card media reader
RGB leds
Microphone
Low battery sensor

To be done
Self test for the factory
Low battery warning
Issue with the volume in AUX mode
--------------------------------------------------------------------------------------------------*/

#include "Audio.h"

extern "C"
{
#include "hal_i2c.h"
#include "hal_i2s.h"
}

#include "Arduino.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "SPIFFS.h"
#include <NeoPixelBus.h>

#define muse 1





//////////////////////////////
// NeoPixel led control
/////////////////////////////
#define PixelCount 1
#define PixelPin 22
RgbColor RED(255, 0, 0);
RgbColor GREEN(0, 255, 0);
RgbColor BLUE(0, 0, 255);
RgbColor YELLOW(255, 128, 0);
RgbColor WHITE(255, 255, 255);
RgbColor BLACK(0, 0, 0);

RgbColor REDL(64, 0, 0);
RgbColor GREENL(0, 64, 0);
RgbColor BLUEL(0, 0, 64);
RgbColor WHITEL(64, 64, 64);
RgbColor BLACKL(0, 0, 0);
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);

//////////////////////////////////////////////////////////////
// init audio library
//////////////////////////////////////////////////////////////
  Audio audio;
////////////////////////////////////////////////////////////// 

////////////////////////////////
// Digital I/O used
///////////////////////////////
//SPI I2S I2C
#define SD_CS         13
#define SPI_MOSI      15
#define SPI_MISO      2
#define SPI_SCK       14
#define I2S_DOUT      26
#define I2S_BCLK      5
#define I2S_LRC       25
#define I2S_DIN       35
#define I2SN (i2s_port_t)0
#define SDA 18
#define SCL 23

//Contacts
#define PA      GPIO_NUM_21      // Amp power ON
#ifdef muse
#define AUXD    GPIO_NUM_27      // AUX In detect 27
#else
#define AUXD    GPIO_NUM_12
#endif
#define SDD     GPIO_NUM_34      // Sd detect

//Buttons

#ifdef muse
#define MU GPIO_NUM_12      // Pause/Play
#define VM GPIO_NUM_32      // Vol-
#define VP GPIO_NUM_19     // Vol+ 
#else
#define MU  GPIO_NUM_36    // Pause/Play
#define VM  GPIO_NUM_39   // Vol-
#define VP  GPIO_NUM_3   // Vol+ 
///////////// bidon en attendant.....
#define FW MU

#endif


#define maxMode 3
#define btM 0
#define sdM 1
#define auxM 2
#define maxVol 50

int b0 = -1,b1 = -1,b2 = -1;
bool mute = false;
uint8_t vfwd = 0;
bool dofwd = false;
bool dobck = false;
uint8_t  vauxd, vsdd;

int vol, oldVol;
int mode = btM;
bool sdON = false;
bool auxON = false;
bool beepON = false;
uint32_t sampleRate;
uint8_t c[88*200];

static uint32_t m_pkt_cnt = 0;
static esp_a2d_audio_state_t m_audio_state = ESP_A2D_AUDIO_STATE_STOPPED;

#define TAG "bt_sp"
// typedef int (*http_data_cb) (http_parser*, const char *at, size_t length);
// typedef int (*http_cb) (http_parser*);
void ES8388_Write_Reg(uint8_t reg, uint8_t val);
void factory_test(void);

static File root;
static File file;
static bool mp3ON;


bool testOK;
#define BLOCK_SIZE 128
#define I2SR (i2s_port_t)0


const i2s_config_t i2s_configR = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX), // Receive, transfer
      .sample_rate = 44100,                         // 
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // 
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, //
      .communication_format = I2S_COMM_FORMAT_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2,     // Interrupt level 1
      .dma_buf_count = 4,                           // number of buffers

      .dma_buf_len = BLOCK_SIZE                           // samples per buffer
  };
  
      i2s_pin_config_t pin_configR=
      {
      .bck_io_num = 5 ,    // BCKL
      .ws_io_num = 25 ,    // LRCL
      .data_out_num = 26,  // DOUT
      .data_in_num = 35    // DIN
      };
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
       int n = 0; 
//sending       
      while(n == 0) n = i2s_write_bytes(I2SR, (const char*)s16,128,portMAX_DELAY);   
      }      
   }
// muting after playing   
   for(int i=0;i<64;i++)s16[i] = 0;
     int n = 0; 
   while(n == 0) n = i2s_write_bytes(I2SR, (const char*)s16,128,portMAX_DELAY);     
   i2s_zero_dma_buffer(I2SR);

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
        while(n == 0)n = i2s_read_bytes(I2SR,(char*)&b[i],128 ,portMAX_DELAY);
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

/////////////////////////////////////////////////////////////////////////////////
//
// Factory test main function
/////////////////////////////////////////////////////////////////////////////////
void factory_test(void)
{
  int test_phase;
  int i;
  
  printf("factory test...\n");

  strip.Begin();  
  strip.SetPixelColor(0,BLUE);
  strip.Show(); 

// provides MCLK
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
  WRITE_PERI_REG(PIN_CTRL, READ_PERI_REG(PIN_CTRL)& 0xFFFFFFF0);

  // reset
  //ES8388_Write_Reg(0, 0x80);
  //ES8388_Write_Reg(0, 0x00);
  //  DAC mute
  ES8388_Write_Reg(25, 0x04);
  //powerup
  ES8388_Write_Reg(1, 0x50);
  ES8388_Write_Reg(2, 0x00);
  // slave mode
  ES8388_Write_Reg(8, 0x00);
  // DAC powerdown
  ES8388_Write_Reg(4, 0xC0);
  // vmidsel/500k ADC/DAC idem
  ES8388_Write_Reg(0, 0x12);

  // i2s 16 bits
  ES8388_Write_Reg(23, 0x18);
  // sample freq 256
  ES8388_Write_Reg(24, 0x02);
  //LIN1/RIN1
  ES8388_Write_Reg(38, 0x00);
  // left DAC to left mixer
  ES8388_Write_Reg(39, 0x90);
  // right DAC to right mixer
  ES8388_Write_Reg(42, 0x90);
  // DACLRC ADCLRC idem
  ES8388_Write_Reg(43, 0x80);
  ES8388_Write_Reg(45, 0x00);
  // DAC volume max
  ES8388_Write_Reg(27, 0x00);
  ES8388_Write_Reg(26, 0x00);  

   // ADC poweroff
   ES8388_Write_Reg(3,0xFF);
   // ADC amp 24dB
   ES8388_Write_Reg(9,0x88);   
   // LINPUT1/RINPUT1  
   ES8388_Write_Reg(10,0);
   // ADC mono left
   ES8388_Write_Reg(11,0x02);   
   //i2S 16b
   ES8388_Write_Reg(12,0x0C); 
   //MCLK 256
   ES8388_Write_Reg(13,0x02); 
   // ADC high pass filter
  // ES8388_Write_Reg(14,0x30);    
   // ADC Volume   
   ES8388_Write_Reg(16,0x00);
   ES8388_Write_Reg(17,0x00);
   // ALC OFF
  // ES8388_Write_Reg(18,0x3F);
   ES8388_Write_Reg(3,0x09);
   ES8388_Write_Reg(43,0x80);
  // Start
  ES8388_Write_Reg(2, 0xF0);
  delay(1);
  ES8388_Write_Reg(2, 0x00); 
  ES8388_Write_Reg(4, 0x30);
  ES8388_Write_Reg(3, 0x00);
  // DAC volume max                                                                                                                                  
  ES8388_Write_Reg(46, 0x1C);
  ES8388_Write_Reg(47,0x1C);
  // unmute
  ES8388_Write_Reg(25, 0x00);
  
  // amp validation
  gpio_set_level(PA, 1);

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

     
i2s_driver_uninstall(I2SR);
 //I2S port0 init:   TX, RX, mono , 16bits, 44100hz
i2s_driver_install(I2SR, &i2s_configR,0,NULL);
i2s_set_pin(I2SR, &pin_configR);
i2s_set_clk(I2SR,44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
i2s_stop(I2SR);  
//
test_phase = 0;

ES8388_Write_Reg(39, 0x90);
ES8388_Write_Reg(42, 0x00);

printf("Left test\n");
//////////////////////////////////////////////////////////////
//up to  5 tests **left speaker + microphone**
/////////////////////////////////////////////////////////////
testOK = false;
i = 0;
while((testOK == false) && (i < 5))
{
i2s_start(I2SR);  
i++;
// record task
xTaskCreatePinnedToCore(recordAudio, "recordAudio", 40000, NULL, 10, NULL,0);
delay(10);
// play task
xTaskCreatePinnedToCore(playAudio, "playAudio", 20000, NULL, 10, NULL,1);
delay(2000);
i2s_stop(I2SR);  
}


if(testOK == true)
{
printf("Right test\n");  
test_phase = 1;
//ROUT1 on LOUT1 off
ES8388_Write_Reg(39, 0x00);
ES8388_Write_Reg(42, 0x90);
//////////////////////////////////////////////////////////////
//up to  5 tests **right speaker + microphone**
/////////////////////////////////////////////////////////////
testOK = false;
i = 0;
while((testOK == false) && (i < 5))
{
i2s_start(I2SR);  
i++;
// record task
xTaskCreatePinnedToCore(recordAudio, "recordAudio", 40000, NULL, 10, NULL,0);
delay(10);
// play task
xTaskCreatePinnedToCore(playAudio, "playAudio", 20000, NULL, 10, NULL,1);
delay(2000);
i2s_stop(I2SR);  
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
  strip.SetPixelColor(0, GREEN);
  printf("Everything OK\n");
  SPIFFS.remove("/testToDo");
}
else
 {
switch(test_phase)
{
  case 0 : 
    strip.SetPixelColor(0, WHITE);
    printf("left speaker error\n");
    break;
  case 1 : 
    strip.SetPixelColor(0, RED);
    printf("right speaker error\n");
    break;
  case 2 : 
    strip.SetPixelColor(0, YELLOW);
    printf("SD error\n");
    break;        
}

 }

 strip.Show();
 while(true) delay(1000);
}

///////////////////////////////////////////////////////////////////////
//  
//
///////////////////////////////////////////////////////////////////////






/*
  void btc_avrc_ct_send_metadata_cmd()
  {
  uint32_t attr_list[] = {
          AVRC_MEDIA_ATTR_ID_TITLE,
          AVRC_MEDIA_ATTR_ID_ARTIST,
          AVRC_MEDIA_ATTR_ID_ALBUM,
          AVRC_MEDIA_ATTR_ID_TRACK_NUM,
          AVRC_MEDIA_ATTR_ID_NUM_TRACKS,
          AVRC_MEDIA_ATTR_ID_GENRE,
          AVRC_MEDIA_ATTR_ID_PLAYING_TIME
          };

  return get_element_attribute_cmd (AVRC_MAX_NUM_MEDIA_ATTR_ID, attr_list);

  }


  static bt_status_t get_element_attribute_cmd (uint8_t num_attribute, uint32_t *p_attr_ids)
  {
    tAVRC_STS status = BT_STATUS_UNSUPPORTED;
    int count  = 0;

    tAVRC_COMMAND avrc_cmd = {0};
    BT_HDR *p_msg = NULL;
    bt_status_t tran_status;

    avrc_cmd.get_elem_attrs.opcode = AVRC_OP_VENDOR;
    avrc_cmd.get_elem_attrs.status = AVRC_STS_NO_ERROR;
    avrc_cmd.get_elem_attrs.num_attr = num_attribute;
    avrc_cmd.get_elem_attrs.pdu = AVRC_PDU_GET_ELEMENT_ATTR;

    for (count = 0; count < num_attribute; count++)
    {
        avrc_cmd.get_elem_attrs.attrs[count] = p_attr_ids[count];
    }

    status = AVRC_BldCommand(&avrc_cmd, &p_msg);
    if (status == AVRC_STS_NO_ERROR)
    {
        BTA_AvMetaCmd(btc_rc_vb.rc_handle, 0, AVRC_CMD_STATUS, p_msg);
        status = BT_STATUS_SUCCESS;
    }

    //if (p_msg != NULL) osi_freebuf(p_msg);

    return status;
  }
*/
#define ES8388_ADDR 0x10
//////////////////////////////////////////////////////////////////////////
//
// ==> plays .wav records (in SPIFFS)
//    n : SPIFFS file name
//////////////////////////////////////////////////////////////////////////
void playWav(char* n)
{
  struct header
  {
    uint8_t a[16];
    uint8_t cksize[4];
    uint8_t wFormatTag[2];
    uint8_t nChannels[2];
    uint8_t nSamplesPerSec[4];
    uint8_t c[16];
  };
  uint32_t rate;
  uint8_t b[46];
  int l;
  bool mono;
  size_t t;
  File f = SPIFFS.open(n, FILE_READ);
  l = (int) f.read(b, sizeof(b));
  if (b[22] == 1) mono = true; else mono = false;
  rate =  (b[25] << 8) + b[24];
  printf(" rate = %d\n", rate);
  i2s_set_clk(I2SN, rate, (i2s_bits_per_sample_t)16, (i2s_channel_t)2);
  do
  {
    if (mono == true)
    {
      l = (int)f.read((uint8_t*)b, 2);
      b[2] = b[0]; b[3] = b[1];

    }
    else
      l = (int)f.read((uint8_t*)b, 4);

    i2s_write(I2SN, b, 4, &t, 1000);
  }
  while (l != 0);
  i2s_zero_dma_buffer((i2s_port_t)0);
  f.close();
}


/////////////////////////////////////////////////////////////////////
// beep....
/////////////////////////////////////////////////////////////////////
void beep(void)
{
#define volBeep 15
  ES8388vol_Set(volBeep);
  beepON = true;
  playWav("/Beep.wav");
  beepON = false;
  ES8388vol_Set(vol);
  i2s_set_clk(I2SN, sampleRate, (i2s_bits_per_sample_t)16, (i2s_channel_t)2);
}

//////////////////////////////////////////////////////////////////////
// annonce tiny messages (2-3 sec) (mode name)
//////////////////////////////////////////////////////////////////////
void modeCall(void)
{
#define volMode 50
  char*n[] = {"/bluetooth.wav", "/player.wav", "/jack.wav"};
  ES8388vol_Set(volMode);
  printf("%d  ====> %s\n", mode, n[mode]);
  beepON = true;
  playWav(n[mode]);
  beepON = false;
  ES8388vol_Set(vol);
  i2s_set_clk(I2SN, sampleRate, (i2s_bits_per_sample_t)16, (i2s_channel_t)2);
}

///////////////////////////////////////////////////////////////////////
// Write ES8388 register (using I2c)
///////////////////////////////////////////////////////////////////////
void ES8388_Write_Reg(uint8_t reg, uint8_t val)
{
  uint8_t buf[2];
  buf[0] = reg;
  buf[1] = val;
  hal_i2c_master_mem_write((i2c_port_t)0, ES8388_ADDR, buf[0], buf + 1, 1);
  // ES8388_REGVAL_TBL[reg]=val;
}

////////////////////////////////////////////////////////////////////////
// Read ES8388 register  (using I2c)
////////////////////////////////////////////////////////////////////////
uint8_t ES8388_Read_Reg( uint8_t reg_add)
{
  uint8_t val;
  hal_i2c_master_mem_read((i2c_port_t)0, ES8388_ADDR, reg_add, &val, 1);
  return val;
}


////////////////////////////////////////////////////////////////////////
//
// manages volume (via vol xOUT1, vol DAC, and vol xIN2)
//
////////////////////////////////////////////////////////////////////////
void ES8388vol_Set(uint8_t volx)
{
#define M maxVol-33

  printf("volume ==> %d\n", volx);
  ES8388_Write_Reg(25, 0x00);
  if (volx > maxVol) volx = maxVol;
  if (volx == 0)
  {
    ES8388_Write_Reg(25, 0x04);
    if (mode == auxM)
    {
      ES8388_Write_Reg(39, 0x80);
      ES8388_Write_Reg(42, 0x80);
    }
  }
  if (volx >= M)
  {
    ES8388_Write_Reg(46, volx - M);
    ES8388_Write_Reg(47, volx - M);
    ES8388_Write_Reg(26, 0x00);
    ES8388_Write_Reg(27, 0x00);
    if (mode == auxM)
    {
      uint8_t v = ((7 - (volx - M - 3) / 4) << 2) | 0xC0;
      ES8388_Write_Reg(39, v);
      ES8388_Write_Reg(42, v);
    }
  }
  else
  {
    ES8388_Write_Reg(46, 0x00);
    ES8388_Write_Reg(47, 0x00);
    ES8388_Write_Reg(26, (M - volx) * 3);
    ES8388_Write_Reg(27, (M - volx) * 3);
  }
}

//////////////////////////////////////////////////////////////////
//
// init CCODEC chip ES8388 (via I2C)
//
////////////////////////////////////////////////////////////////////
void ES8388_Init(void)
{
 // provides MCLK
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
  WRITE_PERI_REG(PIN_CTRL, READ_PERI_REG(PIN_CTRL)& 0xFFFFFFF0);

  // reset
  ES8388_Write_Reg(0, 0x80);
  ES8388_Write_Reg(0, 0x00);
  // mute
  ES8388_Write_Reg(25, 0x04);
  ES8388_Write_Reg(1, 0x50);
  //powerup
  ES8388_Write_Reg(2, 0x00);
  // slave mode
  ES8388_Write_Reg(8, 0x00);
  // DAC powerdown
  ES8388_Write_Reg(4, 0xC0);
  // vmidsel/500k ADC/DAC idem
  ES8388_Write_Reg(0, 0x12);

  ES8388_Write_Reg(1, 0x00);
  // i2s 16 bits
  ES8388_Write_Reg(23, 0x18);
  // sample freq 256
  ES8388_Write_Reg(24, 0x02);
  // LIN2/RIN2 for mixer
  ES8388_Write_Reg(38, 0x09);
  // left DAC to left mixer
  ES8388_Write_Reg(39, 0x90);
  // right DAC to right mixer
  ES8388_Write_Reg(42, 0x90);
  // DACLRC ADCLRC idem
  ES8388_Write_Reg(43, 0x80);
  ES8388_Write_Reg(45, 0x00);
  // DAC volume max
  ES8388_Write_Reg(27, 0x00);
  ES8388_Write_Reg(26, 0x00);

  ES8388_Write_Reg(2 , 0xF0);
  ES8388_Write_Reg(2 , 0x00);
  ES8388_Write_Reg(29, 0x1C);
  // DAC power-up LOUT1/ROUT1 enabled
  ES8388_Write_Reg(4, 0x30);
  // unmute
  ES8388_Write_Reg(25, 0x00);
  // amp validation
  gpio_set_level(PA, 1);
}



/////////////////////////////////////////////////////////////////////
//
// task managing analog input via AUXIn jack
//
/////////////////////////////////////////////////////////////////////
static void aux (void* data)
{
  delay(500);
  ES8388_Write_Reg(39, 0xC0);
  ES8388_Write_Reg(42, 0xC0);
  ES8388vol_Set(45);
  while (1)
  {
    delay(100);
    if (mode != auxM)
    {
      ES8388_Write_Reg(39, 0x90);
      ES8388_Write_Reg(42, 0x90);
      auxON = 0;
      vTaskDelete(NULL);
    }
  }
}




////////////////////////////////////////////////////////////////////
//
// task managing playlist on SD (SPI)
//
////////////////////////////////////////////////////////////////////
static void sd(void* pdata)
{
   static int N;
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    delay(500);
    if(!SD.begin(SD_CS))printf("init. SD failed !\n");
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    
    root = SD.open("/");
    file = root.openNextFile();

    Serial.println("nom du fichier");
    Serial.println(file.name());
    
    audio.connecttoSD(file.name());
    audio.setVolume(vol);               
    mp3ON = true;
    N = 0;
    while(1)
    {
//////////////////////////////////////////      
// next file  
/////////////////////////////    
       if(mp3ON == false)
       { 
           mp3ON = true;
           file = root.openNextFile(); 
           if(file)
           {
              N++;
              Serial.println(file.name());
              audio.connecttoSD(file.name());
           }
           else
           {
// playing list end 
// restart at the first song
              root.rewindDirectory();
              file = root.openNextFile();
              if(file)
              {
              N = 0;
              Serial.println(file.name());            
              audio.connecttoSD(file.name());   
              }  
              else
              {
// error SD mode stopped                
              sdON = 0;
              audio.stopSong();
              SPI.end();
              SD.end();
              vTaskDelete(NULL);        
              }
           }
        } 
/////////////////////////////////////
//Forward  
//////////////////////////////////      
        if(dofwd == true)
        {
          audio.stopSong();
          file = root.openNextFile();
          if(file)
           {
              N++;
              printf("%d\n",N);
              Serial.println(file.name());
              audio.connecttoSD(file.name());
           }
           else
           {
// playing list end 
// restart at the first song             
              root.rewindDirectory();
              file = root.openNextFile();
              if(file)
              {
              N = 0;
              Serial.println(file.name());            
              audio.connecttoSD(file.name());   
              }  
              else
              {
// error SD mode stopped                   
              sdON = 0;
              audio.stopSong();
              SPI.end();
              SD.end();
              vTaskDelete(NULL);        
              }       
           }
           dofwd = false;
        }
///////////////////////////////////////////////////
//Backward
/////////////////////////////////////
        if(dobck == true)
        {
          audio.stopSong();
          N--;
          if(N < 0) N = 0;
//rewind 
//going to song #N
          root.rewindDirectory();     
          for(int i=0;i<=N;i++)
          {
            file = root.openNextFile();   
          }
          Serial.println(file.name());            
          audio.connecttoSD(file.name());
          dobck = false   ;
        }


///////////////////////////////////////////        
//  file playing  
//////////////////////////////  
       if((mute == false) && (beepON == false)) audio.loop(); 
////////////////////////////////////       
// mode change  
/////////////////////////////     
       if(mode != sdM)
       {
              sdON = 0;
              audio.stopSong();
              SPI.end();
              SD.end();
              vTaskDelete(NULL);
       }      
    }
}






////////////////////////////////////////////////////////////////////////
//détection d'appui sur un bouton
// v valeur actuelle  0 => bouton appuyé  1 => bouton relevé
// l pointeur valeur précédente
////////////////////////////////////////////////////////////////////////
int inc(int v, uint8_t *l)
{
  if ((v == 0) && (v != *l))
  {
    *l = v;
    return 1;
  }
  *l = v;
  return 0;
}
///////////////////////////////////////////////////////////////////////
// task managing the speaker buttons
// normal or long press
/////////://///////////////////////////////////////////////////////////
static void keyb(void* pdata)
{
static int v0, v1, v2;
static int ec0=0, ec1=0, ec2=0;
  while(1)
  {
    if((gpio_get_level(VP) == 1) && (ec0 == 1)){b0 = v0; ec0 = 0;}
    if((gpio_get_level(VP) == 1) && (b0 == -1)) {v0 = 0;ec0 = 0;}
    if(gpio_get_level(VP) == 0) {v0++; ec0 = 1;}
   
    if((gpio_get_level(VM) == 1) && (ec1 == 1)){b1 = v1; ec1 = 0;}
    if((gpio_get_level(VM) == 1) && (b1 == -1)) {v1 = 0;ec1 = 0;}
    if(gpio_get_level(VM) == 0) {v1++; ec1 = 1;}
   
    if((gpio_get_level(MU) == 1) && (ec2 == 1)){b2 = v2; ec2 = 0;}
    if((gpio_get_level(MU) == 1) && (b2 == -1)) {v2 = 0; ec2 = 0;}
    if(gpio_get_level(MU) == 0) {v2++; ec2
    = 1;}
    
  //  printf("%d %d %d %d %d %d\n",b0,b1,b2,v0,v1,v2);
    delay(100);
  }
}

//////////////////////////////////////////////////////////////////////////
// task for battery monitoring
//////////////////////////////////////////////////////////////////////////
#define NGREEN 2300
#define NYELLOW 1800
static void battery(void* pdata)
{
  int val;
  while(1)
  {
   val = adc1_get_raw(ADC1_GPIO33_CHANNEL);
   printf("Battery : %d\n");
   if(val < NYELLOW) strip.SetPixelColor(0, RED);
   else if(val > NGREEN) strip.SetPixelColor(0, GREEN);
   else strip.SetPixelColor(0, YELLOW);
   strip.Show();   
   delay(10000);
  }
}




//////////////////////////////////////////////////////////////////////////
//
// bluetooth callbacks
///////////////////////////////////////////////////////////////////////////
void bt_app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param)
{
  esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(param);
  uint8_t *attr_text = (uint8_t *) malloc (rc->meta_rsp.attr_length + 1);
  memcpy(attr_text, rc->meta_rsp.attr_text, rc->meta_rsp.attr_length);
  attr_text[rc->meta_rsp.attr_length] = 0;
  printf("================> %s\n", (char*) attr_text);

  rc->meta_rsp.attr_text = attr_text;
}

void bt_app_rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
  printf("top\n");

  switch (event) {
    case ESP_AVRC_CT_METADATA_RSP_EVT:
      bt_app_alloc_meta_buffer(param);
      break;
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
      esp_avrc_ct_send_metadata_cmd(0, ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST);
      break;
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT: {
        //        bt_app_work_dispatch(bt_av_hdl_avrc_evt, event, param, sizeof(esp_avrc_ct_cb_param_t), NULL);
        break;
      }
    default:
      ESP_LOGE(BT_AV_TAG, "Invalid AVRC event: %d", event);
      break;
  }
}

void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len)
{
  if (mode != btM) return;
  if (beepON == true) return;
  i2s_write_bytes(I2SN, (const char *)data, len, portMAX_DELAY);
  if (++m_pkt_cnt % 100 == 0) {
    ESP_LOGE(BT_AV_TAG, "audio data pkt cnt %u", m_pkt_cnt);
  }
}

// callback for A2DP sink
void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *p_param)
{
  if (mode != btM) return;
  ESP_LOGD(BT_AV_TAG, "%s evt %d", __func__, event);
  esp_a2d_cb_param_t *a2d = NULL;
  switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "a2dp conn_state_cb, state %d", a2d->conn_stat.state);
        break;
      }
      a2d = (esp_a2d_cb_param_t *)(p_param);
      ESP_LOGI(BT_AV_TAG, "a2dp audio_state_cb state %d", a2d->audio_stat.state);
      m_audio_state = a2d->audio_stat.state;
      if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state) {
        m_pkt_cnt = 0;
      case ESP_A2D_AUDIO_STATE_EVT: {

        }
        break;
      }
    case ESP_A2D_AUDIO_CFG_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "a2dp audio_cfg_cb , codec type %d", a2d->audio_cfg.mcc.type);
        // for now only SBC stream is supported
        if (a2d->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
          int sample_rate = 16000;
          char oct0 = a2d->audio_cfg.mcc.cie.sbc[0];
          if (oct0 & (0x01 << 6)) {
            sample_rate = 32000;
          } else if (oct0 & (0x01 << 5)) {
            sample_rate = 44100;
          } else if (oct0 & (0x01 << 4)) {
            sample_rate = 48000;
          }
          sampleRate = sample_rate;
          i2s_set_clk(I2SN, sample_rate, (i2s_bits_per_sample_t)16, (i2s_channel_t)2);

          ESP_LOGI(BT_AV_TAG, "configure audio player %x-%x-%x-%x\n",
                   a2d->audio_cfg.mcc.cie.sbc[0],
                   a2d->audio_cfg.mcc.cie.sbc[1],
                   a2d->audio_cfg.mcc.cie.sbc[2],
                   a2d->audio_cfg.mcc.cie.sbc[3]);
          ESP_LOGI(BT_AV_TAG, "audio player configured, samplerate=%d", sample_rate);
        }
        break;
      }
    default:
      ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event);
      break;
  }
}








/////////////////////////////////////////////////////////////////////////////////////////
//
// APP init
/////////////////////////////////////////////////////////////////////////////////////////
void setup()
{

  Serial.begin(115200);
  hal_i2c_init(0, SDA, SCL);
  esp_err_t err;
  //err = nvs_flash_init();
 // if (err != ESP_OK) printf("nvs flash error...\n");

  if (!SPIFFS.begin())Serial.println("Erreur SPIFFS");
  /*
    printf("====> %d\n",(int)SPIFFS.totalBytes());
    printf("====> %d\n",(int)SPIFFS.usedBytes());
    SPIFFS.format();
  */
  printf(" SPIFFS used bytes  ====> %d of %d\n", (int)SPIFFS.usedBytes(), (int)SPIFFS.totalBytes());
  
                         
  vol = maxVol;
  ///////////////////////////////////////////
  // init GPIO pins
  /////////////////////////////////////////////
 
  // power enable
  gpio_reset_pin(PA);
  gpio_set_direction(PA, GPIO_MODE_OUTPUT);

 
  //VP
  gpio_reset_pin(VP);
  gpio_set_direction(VP, GPIO_MODE_INPUT);
  gpio_set_pull_mode(VP, GPIO_PULLUP_ONLY);


  //VM
  gpio_reset_pin(VM);
  gpio_set_direction(VM, GPIO_MODE_INPUT);
  gpio_set_pull_mode(VM, GPIO_PULLUP_ONLY);

  // AUX detect
  gpio_reset_pin(AUXD);
  gpio_set_direction(AUXD, GPIO_MODE_INPUT);
  // gpio_set_pull_mode(AUXD, GPIO_PULLUP_ONLY);

  // SD detect
  gpio_reset_pin(SDD);
  gpio_set_direction(SDD, GPIO_MODE_INPUT);
  gpio_set_pull_mode(SDD, GPIO_PULLUP_ONLY);

  // pause
  gpio_reset_pin(MU);
  gpio_set_direction(MU, GPIO_MODE_INPUT);
  gpio_set_pull_mode(MU, GPIO_PULLUP_ONLY);

  // Store SD detect and AUX detect initial values
  vsdd = gpio_get_level(SDD);
  vauxd = gpio_get_level(AUXD);
 


  /////////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////////////////////////////////////////////////
  // if all three buttons pressed ======>> factory test
  //
  /////////////////////////////////////////////////////////////////////////////////////////////////////

   if((gpio_get_level(VP) == 0) && (gpio_get_level(VM) == 0) && (gpio_get_level(MU) == 0)) factory_test();

   
 //////////////////////////////////////////////////////////////////////////////////////////////////////
 // if /testToDo exists ======>> factory test
 //////////////////////////////////////////////////////////////////////////////////////////////////////
   if(SPIFFS.exists("/testToDo") )factory_test();

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////   


  ////////////////////////////////////////////////////////////////
  // init led handle
  ///////////////////////////////////////////////////////////////
  strip.Begin();  

  
  ////////////////////////////////////////////////////////////////
  // init ADC interface for battery survey
  /////////////////////////////////////////////////////////////////
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_GPIO33_CHANNEL, ADC_ATTEN_DB_11);
  ////////////////////////////////////////////////////////////////

  /////////////////////////////////////////////////////////////////
  // init  codec ES8388
  //
  ES8388_Init();
  /////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////
  // init i2s
  ////////////////////////////////////////////////////////////////
  hal_i2s_init(I2SN, I2S_DOUT, I2S_LRC, I2S_BCLK, I2S_DIN, 2);
  // init i2s default rates
  i2s_set_clk(I2SN, 44100, (i2s_bits_per_sample_t)16, (i2s_channel_t)2);
  sampleRate = 44100;
  ////////////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////////////////////
  // init bluetooth
  //
  /////////////////////////////////////////////////////////////////////////
  btStart();

  if (esp_bluedroid_init() != ESP_OK) {
    Serial.println("initialize bluedroid failed!");
    return;
  }

  if (esp_bluedroid_enable() != ESP_OK) {
    Serial.println("enable bluedroid failed!");
    return;
  }

 // Specific BT device name (using  MAC ref)
  uint8_t mac [6];
  char macStr[20];
  char dev_name [30];
  esp_read_mac((uint8_t*)&mac, ESP_MAC_WIFI_STA);
  snprintf(macStr, 19, "-%x%x%x", mac[3], mac[4], mac[5]);
  strcpy(dev_name, "MUSE_SPEAKER");
  strcat(dev_name, macStr);
 // printf("devName ====> %s\n",dev_name);
  esp_bt_dev_set_device_name(dev_name);

  //initialize A2DP sink
  esp_a2d_register_callback(&bt_app_a2d_cb);
  esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);
  esp_a2d_sink_init();
  //initialize AVRC controller
  esp_avrc_ct_init();
  esp_avrc_ct_register_callback(bt_app_rc_ct_cb);
  //set discoverable and connectable mode, wait to be connected
  esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
  esp_avrc_ct_send_metadata_cmd(0, ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST);
  ///////////////////////////////////////////////////////////////////////////////////////
  
  ES8388vol_Set(vol);
  mode = btM;
  //////////////////////////////////////////////////////////////////////
  modeCall();
  //task managing buttons
  xTaskCreate(keyb, "keyb", 5000, NULL, 5, NULL);
  //task managing the battery
  xTaskCreate(battery, "battery", 5000, NULL, 1, NULL);  
}



void loop() {

#define longK 8  
  delay(200);
  // forward     (for SD)
  if ((b0 > longK) && (mode == sdM)) {
    beep();
    b0 = -1;
    dofwd = true;
  }
  // backward     (for SD)
  if ((b1 > longK) && (mode == sdM)) {
    beep();
    b1 = -1;
    dobck = true;
  }


  // volume change
  //

  oldVol = vol;
  if((b0 > 0) && (b0 < longK)) {vol = vol + 4 ;b0 = -1;}
  if((b1 > 0) && (b1 < longK)) {vol = vol - 4 ;b1 = -1;}
  
  if (vol > maxVol) vol = maxVol;
  if (vol < 0) vol = 0;
  if (vol != oldVol) {
    beep();
    ES8388vol_Set(vol); 
  }

  // mute / unmute (pause/run for SD)
  //
 
  if (b2 > 0)
  {
    if (mute == false)
    {
      beep();
      mute = true;
      ES8388_Write_Reg(25, 0x04);
      if (mode == auxM)
      {
        ES8388_Write_Reg(39, 0x00);
        ES8388_Write_Reg(42, 0x00);
      }
    }
    else
    {
      mute = false;
      ES8388_Write_Reg(25, 0x00);
      if (mode == auxM)
      {
        ES8388_Write_Reg(39, 0x40);
        ES8388_Write_Reg(42, 0x40);
      }
      beep();
    }
    b2 = -1;
  }
  

  // AUX in detect
  if (inc(gpio_get_level(AUXD), &vauxd) == 1)
   {
    mode = auxM;
    if ((mode == auxM) && (auxON == false)) {
      modeCall();
      xTaskCreate(aux, "jack", 5000, NULL, 1, NULL);
      auxON = true;
    }
  }
  vauxd = gpio_get_level(AUXD);
  if ((vauxd == 1) && (auxON == true)) {
    mode = btM;
    modeCall();
  }

  // SD detect
  if (inc(gpio_get_level(SDD), &vsdd) == 1)
  {
    mode = sdM;
    if ((mode == sdM) && (sdON == false)) {
      modeCall();
      xTaskCreate(sd, "playlist", 5000, NULL, 1, NULL);
      sdON = true;
    }
  }
  vsdd = gpio_get_level(SDD);
  if ((vsdd == 1) && (sdON == true)) {
    audio.stopSong();
    mode = btM;
    modeCall();
  }

  b0=b1=b2= -1;


}



// optional
void audio_info(const char *info) {
  Serial.print("info        "); Serial.println(info);
  if (strstr(info, "SampleRate=") > 0)
  {
    sscanf(info, "SampleRate=%d", &sampleRate);
    printf("==================>>>>>>>>>>%d\n", sampleRate);
  }
}
void audio_id3data(const char *info) { //id3 metadata
  Serial.print("id3data     "); Serial.println(info);
}
void audio_eof_mp3(const char *info) { //end of file
  Serial.print("eof_mp3     "); Serial.println(info); mp3ON = false;
}
void audio_showstation(const char *info) {
  Serial.print("station     "); Serial.println(info);
}
void audio_showstreaminfo(const char *info) {
  Serial.print("streaminfo  "); Serial.println(info);
  Serial.println("top");
}
void audio_showstreamtitle(const char *info) {
  Serial.print("streamtitle "); Serial.println(info);
}
void audio_bitrate(const char *info) {
  Serial.print("bitrate     "); Serial.println(info);

}
void audio_commercial(const char *info) { //duration in sec
  Serial.print("commercial  "); Serial.println(info);
}
void audio_icyurl(const char *info) { //homepage
  Serial.print("icyurl      "); Serial.println(info);
}
void audio_lasthost(const char *info) { //stream URL played
  Serial.print("lasthost    "); Serial.println(info);
}
void audio_eof_speech(const char *info) {
  Serial.print("eof_speech  "); Serial.println(info);
}
