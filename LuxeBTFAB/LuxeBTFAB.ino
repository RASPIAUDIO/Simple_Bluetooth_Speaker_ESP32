/***************************************************************************
 *  MUSE LUXE / PROTO – Bluetooth-speaker firmware                         *
 *  Commands: ##BT=<name>  ##LED=r,g,b  ##WAVB64=<len>                     *
 *  Startup WAV: 22 050 Hz · 16 bit · mono, ≤ 2 s                          *
 ***************************************************************************/

// ─── Select board ───────────────────────────────────────────────────────
#define BOARD_PROTO              // GPIO-23 gain, no codec
// #define BOARD_LUXE            // ES8388 codec + jack detect
// ────────────────────────────────────────────────────────────────────────

#if defined(BOARD_LUXE) && defined(BOARD_PROTO)
#error "Select either BOARD_LUXE or BOARD_PROTO – not both."
#endif
#if !defined(BOARD_LUXE) && !defined(BOARD_PROTO)
#error "Please define BOARD_LUXE or BOARD_PROTO."
#endif

#include "museWrover.h"
#include "BluetoothA2DPSink.h"
#include "ESP_I2S.h"
#include <Preferences.h>
#include <Adafruit_NeoPixel.h>
#include <LittleFS.h>
#include "esp_mac.h"
#include "esp_task_wdt.h"
#include "mbedtls/base64.h"

#if defined(BOARD_PROTO)
  #define GAIN_PIN 23
#endif

/* -------- constants -------------------------------------------------- */
#define MAX_WAV_B64 120000            // 2 s @ 22.05 kHz mono 16-bit
#define WAV_SR      22050
#define ADC_BAT     33
#define NYELLOW     1800
#define NGREEN      2300

/* -------- globals ---------------------------------------------------- */
Preferences prefs;
char  bt_base[24];   bool customName=false;
uint8_t userCol[3] = {0,255,0};

I2SClass          i2s;
BluetoothA2DPSink a2dp_sink(i2s);

uint8_t mac[6];  char macStr[20];  char dev_name[32];
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

/* -------------------------------------------------------------------- */
void applyUserCol(){
  pixels.setPixelColor(0,pixels.Color(userCol[0],userCol[1],userCol[2]));
  pixels.show();
}
void buildDevName(){
  esp_read_mac(mac,ESP_MAC_BT);
  snprintf(macStr,sizeof(macStr),"-%02X%02X%02X",mac[3],mac[4],mac[5]);
  snprintf(dev_name,sizeof(dev_name),"%s%s",bt_base, customName?"":macStr);
}

/* ---------------- battery task -------------------------------------- */
static void batteryTask(void*){
  uint32_t t0=0; bool off=false;
  for(;;){
    int v=analogRead(ADC_BAT);
    if(v<NYELLOW){ pixels.setPixelColor(0,pixels.Color(255,0,0)); pixels.show(); off=false; }
    else if(v>NGREEN){ applyUserCol(); off=false; }
    else{
      uint32_t now=millis();
      if(!off && now-t0>=2000){ pixels.setPixelColor(0,0); pixels.show(); off=true; t0=now; }
      else if(off && now-t0>=250){ applyUserCol(); off=false; }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/* ---------------- WAV upload state ---------------------------------- */
static bool   wavMode=false;
static size_t wavTarget=0;
static size_t wavCount =0;
static String wavB64;

bool saveWavFromB64(const String& b64){
  size_t binLen=0;
  if(b64.length()>MAX_WAV_B64) return false;
  if(mbedtls_base64_decode(nullptr,0,&binLen,
        reinterpret_cast<const unsigned char*>(b64.c_str()),
        b64.length())!=MBEDTLS_ERR_BASE64_BUFFER_TOO_SMALL) return false;

  uint8_t* buf=(uint8_t*)ps_malloc(binLen);
  if(!buf) return false;

  if(mbedtls_base64_decode(buf,binLen,&binLen,
        reinterpret_cast<const unsigned char*>(b64.c_str()),
        b64.length())!=0){ free(buf); return false; }

  File f=LittleFS.open("/bluetooth.wav",FILE_WRITE);
  if(!f){ free(buf); return false; }
  f.write(buf,binLen); f.close(); free(buf); return true;
}

/* ---------------- serial command parser ----------------------------- */
void handleLine(const String& s){
  if(wavMode) return;
  if(s.startsWith("##BT=")){
    String n=s.substring(5); n.trim();
    if(n.length()==0||n.length()>23){Serial.println("ERR"); return;}
    strlcpy(bt_base,n.c_str(),sizeof(bt_base)); prefs.putString("bt",bt_base);
    customName=true; prefs.putBool("custom",true);
    Serial.println("OK"); Serial.flush(); delay(100); esp_restart();
  }
  else if(s.startsWith("##LED=")){
    int r,g,b;
    if(sscanf(s.c_str()+6,"%d,%d,%d",&r,&g,&b)!=3||r<0||r>255||g<0||g>255||b<0||b>255){
      Serial.println("ERR"); return;
    }
    userCol[0]=r; userCol[1]=g; userCol[2]=b;
    prefs.putBytes("led",userCol,3); Serial.println("OK"); applyUserCol();
  }
  else if(s.startsWith("##WAVB64=")){
    wavTarget=atoi(s.c_str()+9);
    if(!wavTarget||wavTarget>MAX_WAV_B64){Serial.println("ERR");return;}
    wavMode=true; wavCount=0; wavB64.reserve(wavTarget); Serial.println("RDY");
  }
  else Serial.println("ERR");
}

/* ---------------- setup --------------------------------------------- */
void setup(){
  Serial.begin(115200);

#if defined(BOARD_LUXE)
  const char* defName="LUXE";
#else
  const char* defName="PROTO";
#endif
  prefs.begin("muse",false);
  customName=prefs.getBool("custom",false);
  if(!prefs.getString("bt",bt_base,sizeof(bt_base)))
      strlcpy(bt_base,defName,sizeof(bt_base));
  if(prefs.isKey("led")) prefs.getBytes("led",userCol,3);
  buildDevName(); Serial.printf("Device : %s\n",dev_name);

#if defined(BOARD_PROTO)
  pinMode(GAIN_PIN,OUTPUT); digitalWrite(GAIN_PIN,LOW);
#endif
  gpio_reset_pin(GPIO_PA_EN);
  gpio_set_direction(GPIO_PA_EN,GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_PA_EN,HIGH);

#if defined(BOARD_LUXE)
  while(!es.begin(IIC_DATA,IIC_CLK)){Serial.println("ES8388 fail");delay(500);}
  es.volume(ES8388::ES_MAIN,100); es.mute(ES8388::ES_MAIN,false);
#endif

  /* I²S pass 1 – 22 050 Hz */
  i2s.setPins(I2S_BCLK,I2S_LRCK,I2S_SDOUT,I2S_SDIN,I2S_MCLK);
  i2s.begin(I2S_MODE_STD,WAV_SR,I2S_DATA_BIT_WIDTH_16BIT,
#if defined(BOARD_LUXE)
            I2S_SLOT_MODE_STEREO,
#else
            I2S_SLOT_MODE_MONO,
#endif
            I2S_STD_SLOT_BOTH);

  a2dp_sink.start(dev_name);

  if(LittleFS.begin()){
    File f=LittleFS.open("/bluetooth.wav");
    if(f){ size_t len=f.size(); uint8_t* buf=(uint8_t*)ps_malloc(len);
           f.read(buf,len); f.close(); i2s.playWAV(buf,len); free(buf); }
  }

  /* I²S pass 2 – 44.1 kHz stereo for A2DP */
  i2s.end();
  i2s.begin(I2S_MODE_STD,44100,I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO,I2S_STD_SLOT_BOTH);

  applyUserCol();
  xTaskCreatePinnedToCore(batteryTask,"bat",4096,nullptr,5,nullptr,0);
}

/* ---------------- loop ---------------------------------------------- */
void loop(){
  while(Serial.available()){
    char c=Serial.read();

    if(wavMode){
      if(c=='\n'||c=='\r') continue;
      wavB64+=c; ++wavCount;
      if(wavCount==wavTarget){
        bool ok=saveWavFromB64(wavB64);
        Serial.println(ok?"DONE":"ERR");
        wavMode=false; wavB64=""; wavCount=0;
        if(ok){ Serial.flush(); delay(100); esp_restart(); }
      }
      continue;
    }

    static String ln;
    if(c=='\n'||c=='\r'){ if(ln.length()) handleLine(ln); ln=""; }
    else ln+=c;
  }
  vTaskDelay(pdMS_TO_TICKS(20));
}
