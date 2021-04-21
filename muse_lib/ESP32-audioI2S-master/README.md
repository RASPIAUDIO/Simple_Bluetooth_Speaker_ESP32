# ESP32-audioI2S
Play mp3 files from SD card via I2S with external hardware.
HELIX-mp3 decoder is included.
Works with MAX98357A (3 Watt amplifier with DAC), connected three lines (DOUT, BLCK, LRC) to I2S.
For stereo are two MAX98357A necessary. AudioI2S works with UDA1334A (Adafruit I2S Stereo Decoder Breakout Board) and PCM1502A.
Other HW may work but not tested. Plays also icy-streams (mp3 only) and GoogleTTS. Can compiled with Arduino IDE.

```` c++
#include "Arduino.h"
#include "WiFi.h"
#include "src/audioI2S/Audio.h"
#include "SD.h"
#include "FS.h"

// Digital I/O used
#define SD_CS          5
#define SPI_MOSI      23
#define SPI_MISO      19
#define SPI_SCK       18
#define I2S_DOUT      25
#define I2S_BCLK      27
#define I2S_LRC       26

Audio audio;

String ssid =     "Wolles-FRITZBOX";
String password = "*******";

void setup() {
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    Serial.begin(115200);
    SD.begin(SD_CS);
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());
    while (WiFi.status() != WL_CONNECTED) delay(1500);
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(21); // 0...21

    //audio.connecttoSD("/320k_test.mp3");
    //audio.connecttohost("www.wdr.de/wdrlive/media/einslive.m3u");
    audio.connecttohost("dg-ais-eco-http-fra-eco-cdn.cast.addradio.de/hellwegradio/west/mp3/high");
    //audio.connecttohost("fischkopp.stream.laut.fm/fischkopp");
    //audio.connecttospeech("Wenn die Hunde schlafen, kann der Wolf gut Schafe stehlen.", "de");
}

void loop()
{
    audio.loop();
}

// optional
void audio_info(const char *info){
    Serial.print("info        "); Serial.println(info);
}
void audio_id3data(const char *info){  //id3 metadata
    Serial.print("id3data     ");Serial.println(info);
}
void audio_eof_mp3(const char *info){  //end of file
    Serial.print("eof_mp3     ");Serial.println(info);
}
void audio_showstation(const char *info){
    Serial.print("station     ");Serial.println(info);
}
void audio_showstreaminfo(const char *info){
    Serial.print("streaminfo  ");Serial.println(info);
}
void audio_showstreamtitle(const char *info){
    Serial.print("streamtitle ");Serial.println(info);
}
void audio_bitrate(const char *info){
    Serial.print("bitrate     ");Serial.println(info);
}
void audio_commercial(const char *info){  //duration in sec
    Serial.print("commercial  ");Serial.println(info);
}
void audio_icyurl(const char *info){  //homepage
    Serial.print("icyurl      ");Serial.println(info);
}
void audio_lasthost(const char *info){  //stream URL played
    Serial.print("lasthost    ");Serial.println(info);
}
void audio_eof_speech(const char *info){
    Serial.print("eof_speech  ");Serial.println(info);
}

````
Breadboard
![Breadboard](https://github.com/schreibfaul1/ESP32-audioI2S/blob/master/additional_info/Breadboard.jpg)
Wiring
![Wiring](https://github.com/schreibfaul1/ESP32-audioI2S/blob/master/additional_info/ESP32_I2S_PCM5102A.JPG)
Impulse diagram
![Impulse diagram](https://github.com/schreibfaul1/ESP32-audioI2S/blob/master/additional_info/Impulsdiagramm.jpg)
