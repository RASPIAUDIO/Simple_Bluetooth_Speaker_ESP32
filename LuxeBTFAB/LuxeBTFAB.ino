/***************************************************************************
 *  MUSE LUXE / PROTO – Bluetooth-speaker firmware                         *
 *  Serial command  ##BT=<name>  (persistent)                              *
 ***************************************************************************/

//──────────── SELECT YOUR BOARD ────────────
#define BOARD_PROTO              // GPIO-23 gain, no codec
// #define BOARD_LUXE            // ES8388 codec + jack detect
//────────────────────────────────────────────

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

#if defined(BOARD_PROTO)
  #define GAIN_PIN 23
#endif

#define MAX_VOL   100
#define ADC_BAT   33
#define NYELLOW   1800
#define NGREEN    2300

Preferences prefs;
char bt_base[24];
bool customName = false;

I2SClass          i2s;
BluetoothA2DPSink a2dp_sink(i2s);

uint8_t mac[6];
char    macStr[20];
char    dev_name[32];

Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

/* ───────── Battery task ───────── */
static void batteryTask(void*) {
  for (;;) {
    int v = analogRead(ADC_BAT);
    uint32_t c = (v < NYELLOW) ? pixels.Color(255,0,0)
               : (v > NGREEN)  ? pixels.Color(0,255,0)
                               : pixels.Color(255,255,0);
    pixels.setPixelColor(0, c);  pixels.show();
    Serial.printf("Battery : %d\n", v);
    vTaskDelay(pdMS_TO_TICKS(10'000));
  }
}

/* ───────── Serial command ───────── */
void processCmd(const String& s) {
  if (!s.startsWith("##")) return;
  if (s.startsWith("##BT=")) {
    String n = s.substring(5); n.trim();
    if (n.isEmpty() || n.length() > 23) { Serial.println("ERR"); return; }
    strlcpy(bt_base, n.c_str(), sizeof(bt_base));
    prefs.putString("bt", bt_base);
    prefs.putBool  ("custom", true);
    Serial.println("OK");
    Serial.flush(); delay(100); esp_restart();
  } else Serial.println("ERR");
}

/* ───────── Helpers ───────── */
void makeDevName() {
  esp_read_mac(mac, ESP_MAC_BT);
  snprintf(macStr, sizeof(macStr), "-%02X%02X%02X", mac[3],mac[4],mac[5]);
  if (customName) snprintf(dev_name,sizeof(dev_name),"%s", bt_base);
  else             snprintf(dev_name,sizeof(dev_name),"%s%s",bt_base,macStr);
}

/* ───────── Setup ───────── */
void setup() {
  Serial.begin(115200);

#if defined(BOARD_LUXE)
  const char* defName="LUXE";
#else
  const char* defName="PROTO";
#endif
  prefs.begin("muse",false);
  customName = prefs.getBool("custom",false);
  if (!prefs.getString("bt",bt_base,sizeof(bt_base)))
      strlcpy(bt_base,defName,sizeof(bt_base));
  makeDevName();

  /* watchdog 60 s */
  esp_task_wdt_config_t cfg={
    .timeout_ms=60*1000,.idle_core_mask=(1<<portNUM_PROCESSORS)-1,.trigger_panic=true};
  esp_task_wdt_init(&cfg);

#if defined(BOARD_PROTO)
  pinMode(GAIN_PIN,OUTPUT); digitalWrite(GAIN_PIN,LOW);
#endif
  gpio_reset_pin(GPIO_PA_EN);
  gpio_set_direction(GPIO_PA_EN,GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_PA_EN,HIGH);

#if defined(BOARD_LUXE)
  while(!es.begin(IIC_DATA,IIC_CLK)){Serial.println("ES8388 fail");delay(500);}
  es.volume(ES8388::ES_MAIN,MAX_VOL); es.mute(ES8388::ES_MAIN,false);
#endif

  /* ── FIRST I²S BEGIN: wav playback ── */
  i2s.setPins(I2S_BCLK,I2S_LRCK,I2S_SDOUT,I2S_SDIN,I2S_MCLK);
  i2s.begin(I2S_MODE_STD,44100,I2S_DATA_BIT_WIDTH_16BIT,
#if defined(BOARD_LUXE)
            I2S_SLOT_MODE_STEREO,
#else
            I2S_SLOT_MODE_MONO,
#endif
            I2S_STD_SLOT_BOTH);

  /* A2DP sink after first begin */
  Serial.printf("Device : %s\n",dev_name);
  a2dp_sink.start(dev_name);

  /* optional startup WAV */
  if(LittleFS.begin()){
    File f=LittleFS.open("/bluetooth.wav");
    if(f){
      size_t len=f.size(); uint8_t* buf=(uint8_t*)ps_malloc(len);
      f.read(buf,len); f.close(); i2s.playWAV(buf,len); free(buf);
    }
  }

  /* ── SECOND I²S BEGIN: A2DP streaming ── */
  i2s.end();
  i2s.begin(I2S_MODE_STD,44100,I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH);

  xTaskCreatePinnedToCore(batteryTask,"battery",4096,nullptr,5,nullptr,0);
}

/* ───────── Loop ───────── */
void loop() {
  static String rx;
  while(Serial.available()){
    char c=Serial.read();
    if(c=='\n'||c=='\r'){ if(rx.length())processCmd(rx); rx=""; }
    else rx+=c;
  }
  vTaskDelay(pdMS_TO_TICKS(50));
}
