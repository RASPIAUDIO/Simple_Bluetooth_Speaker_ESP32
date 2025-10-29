

#include "Arduino.h"
#include "WiFi.h"
#include "SPI.h"
#include "SD.h"
#include "FS.h"
#include "Wire.h"
#include "museWrover.h"
#include "Audio.h"


char ssid[] =     "xhkap";
char password[] = "12345678";

#define maxVol 100
int volume = 80;                            // 0...100
bool jack = false;
bool BPause = false;
ES8388 es;
Audio audio;

//----------------------------------------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  Serial.println("\r\nReset");

  
///////just needed for SD card
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  SPI.setFrequency(1000000);
  SD.begin(SD_CS);

///////WiFi init
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while(WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(100);
  }
  Serial.printf_P(PSTR("Connected\r\nRSSI: "));
  Serial.print(WiFi.RSSI());
  Serial.print(" IP: ");
  Serial.println(WiFi.localIP());     


  
  if (!SPIFFS.begin()) {
    Serial.println("LittleFS initialisation failed!");
    while (1) for (;;);
  }

  Serial.printf("Connect to ES8388 codec... ");
  while (not es.begin(IIC_DATA, IIC_CLK))
  {
    Serial.printf("Failed!\n");
    delay(1000);
  }
  Serial.printf("OK\n");

  
/////// Volumes init
  es.volume(ES8388::ES_MAIN, 100);
  es.volume(ES8388::ES_OUT1, volume);
  //    es.volume(ES8388::ES_OUT2, volume);
  es.mute(ES8388::ES_OUT1, false);
  //    es.mute(ES8388::ES_OUT2, false);
  es.mute(ES8388::ES_MAIN, false);

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
  

  audio.setPinout(I2S_BCLK, I2S_LRCK, I2S_SDOUT, I2S_MCLK);
  audio.setVolume(21); // 0...21

   audio.connecttohost("http://direct.fipradio.fr/live/fip-midfi.mp3");
 //   audio.connecttoFS(SD, "truc.wav"); //SD card
 // audio.connecttoFS(SPIFFS, "/test.wav"); // SPIFFS internal flash upload first your file using ESP32 sketch data uploader
  //  audio.connecttospeech("Hello Raspiaudio, this text was genrated using google speech API", "en"); //uses google TTS
  //  audio.openai_speech(OPENAI_API_KEY, "tts-1", result, "shimmer", "mp3", "1"); //uses openai TTS (needs billable api key)

}
//----------------------------------------------------------------------------------------------------------------------
void loop()
{
  audio.loop();

  if (gpio_get_level(BUTTON_PAUSE) == 0)
  {
    while (gpio_get_level(BUTTON_PAUSE) == 0) delay(10);
    if (BPause == false)
    {
      es.mute(ES8388::ES_MAIN, true);
      if (jack)
      {
        es.volume(ES8388::ES_OUT2, 0);
        es.mute(ES8388::ES_OUT2, true);
      }
      else
      {
        es.volume(ES8388::ES_OUT1, 0);
        es.mute(ES8388::ES_OUT1, true);
      }
      BPause = true;
    }
    else
    {
      es.mute(ES8388::ES_MAIN, false);
      if (jack)
      {
        es.volume(ES8388::ES_OUT2, volume);
        es.mute(ES8388::ES_OUT2, false);
      }
      else
      {
        es.volume(ES8388::ES_OUT1, volume);
        es.mute(ES8388::ES_OUT1, false);
      }
      BPause = false;
    }
  }
  
  ////// Vol+ button
  if (gpio_get_level(BUTTON_VOL_PLUS) == 0)
  {
    while (gpio_get_level(BUTTON_VOL_PLUS) == 0) delay(10);
    volume += 3;
    if (volume > maxVol) volume = maxVol;
    printf(" v = %d\n", volume);
    if (volume < 6)es.volume(ES8388::ES_MAIN, volume); else es.volume(ES8388::ES_MAIN, maxVol);
    if (jack)
      es.volume(ES8388::ES_OUT2, volume);
    else
      es.volume(ES8388::ES_OUT1, volume);
  }

  ////// Vol- button
  if (gpio_get_level(BUTTON_VOL_MINUS) == 0)
  {
    while (gpio_get_level(BUTTON_VOL_MINUS) == 0) delay(10);
    volume -= 3;
    if (volume < 0) volume = 0;
    printf(" v = %d\n", volume);
    if (volume < 6)es.volume(ES8388::ES_MAIN, volume); else es.volume(ES8388::ES_MAIN, maxVol);
    if (jack)
      es.volume(ES8388::ES_OUT2, volume);
    else
      es.volume(ES8388::ES_OUT1, volume);
  }
 
  ///// Jack_Detect

  ///// ON
  if (gpio_get_level(Jack_Detect) == 0)
  {
/////// jack ON => Rout2/Lout2  amp OFF
    if (!jack)
    {
      es.select_out2();      
      es.volume(ES8388::ES_OUT2, volume);
      gpio_set_level(GPIO_PA_EN, LOW);           
      jack = true;
    }
  }
  ///// OFF
  if(gpio_get_level(Jack_Detect) == 1)
  {
/////// jack OFF => Rout1/Lout1  amp ON
    if (jack)
    {
      es.select_out1();
      es.volume(ES8388::ES_OUT1, volume);
      gpio_set_level(GPIO_PA_EN, HIGH);
      jack = false;
    }
  }  
  vTaskDelay(100 / portTICK_PERIOD_MS);  
}
//----------------------------------------------------------------------------------------------------------------------

// optional
void audio_info(const char *info) {
  Serial.print("info        "); Serial.println(info);
}
void audio_id3data(const char *info) { //id3 metadata
  Serial.print("id3data     "); Serial.println(info);
}
void audio_eof_mp3(const char *info) { //end of file
  Serial.print("eof_mp3     "); Serial.println(info);
}
void audio_showstation(const char *info) {
  Serial.print("station     "); Serial.println(info);
}
void audio_showstreaminfo(const char *info) {
  Serial.print("streaminfo  "); Serial.println(info);
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
