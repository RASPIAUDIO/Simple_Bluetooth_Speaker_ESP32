

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





Audio audio;

//----------------------------------------------------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  Serial.println("\r\nReset");

  
//just needed for SD card
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  SPI.setFrequency(1000000);
  SD.begin(SD_CS);


//WiFi init
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

// SPIFFS init  
  if (!SPIFFS.begin()) {
    Serial.println("SPIFFS initialisation failed!");
   SPIFFS.format();
    while (1) for (;;);
  }

 // Enable amplifier
  pinMode(GPIO_PA_EN, OUTPUT);
  digitalWrite(GPIO_PA_EN, HIGH);

  audio.setPinout(I2S_BCLK, I2S_LRCK, I2S_SDOUT, I2S_MCLK);
  audio.setVolume(21); // 0...21

  audio.connecttohost("http://direct.fipradio.fr/live/fip-midfi.mp3");
 //  audio.connecttoFS(SD, "brubeck.wav"); //SD card
 // audio.connecttoFS(SPIFFS, "/test.wav"); // SPIFFS internal flash upload first your file using ESP32 sketch data uploader
  //  audio.connecttospeech("Hello Raspiaudio, this text was genrated using google speech API", "en"); //uses google TTS
  //  audio.openai_speech(OPENAI_API_KEY, "tts-1", result, "shimmer", "mp3", "1"); //uses openai TTS (needs billable api key)

}
//----------------------------------------------------------------------------------------------------------------------
void loop()
{
  audio.loop();
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
