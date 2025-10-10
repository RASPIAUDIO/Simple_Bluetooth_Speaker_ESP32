// ***** Muse LUXE as a Bluetooth speaker + SD player*****
// BT speaker name ==> LUXE-xxxxxx
// Play/Pause and volume (+/-) buttons managed
///////////////////////////////////////////////

#include "museWrover.h"
#include "ESP_I2S.h"
#include "driver/gpio.h"
#include "LittleFS.h"
#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "SPI.h"
#include "SD.h"
#include "FS.h"
#include <Adafruit_NeoPixel.h>
#include "Audio.h"   // https://github.com/schreibfaul1/ESP32-audioI2S
#include "esp_task_wdt.h"
#include "esp_timer.h"

#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);


// Define your SD card detect pin (adjust the pin number as needed)
#define SD_DET_PIN GPIO_NUM_34

bool BPause = false;         // Playback pause state: false = playing, true = paused
int volume = 75;             // Volume level (0...100)
#define maxVol 100          // Maximum volume

// Initialize I2S and Bluetooth A2DP Sink objects
I2SClass i2s;

bool jack = false;   
bool mp3ON = false;        // Headphone jack detection state
ES8388 es;                   // ES8388 audio codec instance
uint8_t *b;                  // Pointer for WAV file buffer

Audio audio;
// Flag to ensure SD files are read only once per card insertion
bool sdInserted = false;

void setup() {
  Serial.begin(115200);
/*
  // Initialize the Task Watchdog Timer (TWDT)(for ESD event)
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 60 * 1000,                // 60 seconds timeout
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  
    .trigger_panic = true                  
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL); 
*/
  // Initialize SD card SPI interface
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  SPI.setFrequency(1000000);
 
  ////// GPIO initialization
  gpio_reset_pin(GPIO_PA_EN);
  gpio_set_direction(GPIO_PA_EN, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_PA_EN, HIGH);

  gpio_reset_pin(BUTTON_VOL_PLUS);
  gpio_set_direction(BUTTON_VOL_PLUS, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_VOL_PLUS, GPIO_PULLUP_ONLY);

  gpio_reset_pin(BUTTON_VOL_MINUS);
  gpio_set_direction(BUTTON_VOL_MINUS, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_VOL_MINUS, GPIO_PULLUP_ONLY);

  gpio_reset_pin(BUTTON_PAUSE);
  gpio_set_direction(BUTTON_PAUSE, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BUTTON_PAUSE, GPIO_PULLUP_ONLY);

  gpio_reset_pin(Jack_Detect);
  gpio_set_direction(Jack_Detect, GPIO_MODE_INPUT);

  gpio_reset_pin(SD_DET_PIN);
  gpio_set_direction(SD_DET_PIN, GPIO_MODE_INPUT);
 // gpio_set_pull_mode(SD_DET_PIN, GPIO_PULLUP_ONLY);
  
  
//////////////////////////////////////////////////////////////
// 3 buttons pushed ===> factory test
//////////////////////////////////////////////////////////////
  if((gpio_get_level(BUTTON_PAUSE) == LOW) && (gpio_get_level(BUTTON_VOL_PLUS) == LOW) && (gpio_get_level(BUTTON_VOL_MINUS) == LOW))
//if(gpio_get_level(BUTTON_PAUSE) == LOW)
 {
  const esp_partition_t* partition = esp_partition_find_first(
                                         ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);
    if (partition != NULL) {
      esp_ota_set_boot_partition(partition);
      esp_restart();
    }
 }
//////////////////////////////////////////////////////////////
  
  

  // Initialize the ES8388 audio codec
  Serial.printf("Connect to ES8388 codec... ");
  while (!es.begin(IIC_DATA, IIC_CLK)) {
    Serial.printf("Failed!\n");
    delay(1000);
  }
  Serial.println("OK");
  es.volume(ES8388::ES_MAIN, maxVol);
  es.volume(ES8388::ES_OUT1, volume);
  es.mute(ES8388::ES_MAIN, false);
  es.mute(ES8388::ES_OUT1, false);


  // Initialize LittleFS and play the startup sound
  if (!LittleFS.begin()) {
    Serial.println("LittleFS initialisation failed!");
 //  while (1) { }
  }

  xTaskCreatePinnedToCore(battery, "battery", 2048, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(sound, "sound", 2048, NULL, 5, NULL, 1);

  sdInserted = (digitalRead(SD_DET_PIN) == LOW) ? true : false;
  Serial.println("SD Card inserted. Reading all files:");

   i2s.setPins(I2S_BCLK, I2S_LRCK, I2S_SDOUT, I2S_SDIN, I2S_MCLK);
  if (!i2s.begin(I2S_MODE_STD, 44100, I2S_DATA_BIT_WIDTH_16BIT,
               I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) 
   {
  Serial.println("Failed to initialize I2S!");
  while (1);
   }  

  b = (uint8_t *)ps_malloc(160000);
  File ln = LittleFS.open("/player.wav", FILE_READ);
  ln.read(b, 115000);
  ln.close();
  i2s.playWAV(b, 115000);  // Play the startup sound
  free(b);
  i2s.end();

  audio.setPinout(I2S_BCLK, I2S_LRCK, I2S_SDOUT, I2S_MCLK);
  audio.setVolume(21); // 0...21
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card initialization failed!");
  } else {
    Serial.println("SD Card initialized.");
  } 
  File root = SD.open("/");
  if (!root) {
    Serial.println("Failed to open root directory");
    return;
  }
  delay(500);
  while (true) {
    File entry = root.openNextFile();
    if (!entry) {
      // No more files: ensure Audio releases I2S before returning to BT
      es.mute(ES8388::ES_OUT1, true);
      es.mute(ES8388::ES_OUT2, true);
      ESP.restart();
    }
    if (!entry.isDirectory()) {
      Serial.print("Playing file: ");
      Serial.println(entry.name());
      Serial.println(entry.size());
      delay(500);
      audio.connecttoFS(SD, entry.name());
      
      // Play file until it finishes
      while (audio.isRunning() && sdInserted) {  
        sdInserted = (digitalRead(SD_DET_PIN) == LOW) ? true : false; 
        audio.loop();
        esp_task_wdt_reset(); 
        delay(10);
      }
    }
    entry.close();
    audio.stopSong();
  }
  root.close();
}

void loop() {
  esp_task_wdt_reset(); 
  delay(100);
}

//////////////////////////////////////////////////////////////////////////
// sound monitoring
//////////////////////////////////////////////////////////////////////////
static void sound(void* pdata)
{
  int val;
  while(true)
  {
    
    if (gpio_get_level(SD_DET_PIN) == 1 ) {
    es.mute(ES8388::ES_MAIN, true);
    es.mute(ES8388::ES_OUT1, true);     
    const esp_partition_t* partition = esp_partition_find_first(
                                         ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
    if (partition != NULL) {
      esp_ota_set_boot_partition(partition);
      esp_restart();
    }
  }
  
  // Play/Pause Button Handling
    if (gpio_get_level(BUTTON_PAUSE) == 0) {
      while (gpio_get_level(BUTTON_PAUSE) == 0) delay(10);
      if (!BPause) {
        es.mute(ES8388::ES_MAIN, true);
        if (jack) {
          es.volume(ES8388::ES_OUT2, 0);
          es.mute(ES8388::ES_OUT2, true);
        } else {
          es.volume(ES8388::ES_OUT1, 0);
          es.mute(ES8388::ES_OUT1, true);
        }
        BPause = true;
      }
      else {
        es.mute(ES8388::ES_MAIN, false);
        if (jack) {
          es.volume(ES8388::ES_OUT2, volume);
          es.mute(ES8388::ES_OUT2, false);
        } else {
          es.volume(ES8388::ES_OUT1, volume);
          es.mute(ES8388::ES_OUT1, false);
        }
        BPause = false;
      }
    }

    // Volume Up Button Handling
    if (gpio_get_level(BUTTON_VOL_PLUS) == 0) {
      while (gpio_get_level(BUTTON_VOL_PLUS) == 0) delay(10);
      volume += 3;
      if (volume > maxVol) volume = maxVol;
      printf(" v = %d\n", volume);
      if (volume < 6)
        es.volume(ES8388::ES_MAIN, volume);
      else
        es.volume(ES8388::ES_MAIN, maxVol);
      if (jack)
        es.volume(ES8388::ES_OUT2, volume);
      else
        es.volume(ES8388::ES_OUT1, volume);
    }

    // Volume Down Button Handling
    if (gpio_get_level(BUTTON_VOL_MINUS) == 0) {
      while (gpio_get_level(BUTTON_VOL_MINUS) == 0) delay(10);
      volume -= 3;
      if (volume < 0) volume = 0;  
      printf(" v = %d\n", volume);
      if (volume < 6)
        es.volume(ES8388::ES_MAIN, volume);
      else
        es.volume(ES8388::ES_MAIN, maxVol);
      if (jack)
        es.volume(ES8388::ES_OUT2, volume);
      else
        es.volume(ES8388::ES_OUT1, volume);
    }

    // Headphone Jack Detection
    if (gpio_get_level(Jack_Detect) == 0) {
      if (!jack) {
        es.volume(ES8388::ES_OUT2, volume);
        es.select_out2();
        gpio_set_level(GPIO_PA_EN, 0);
      }
      jack = true;
    }
    else {
      if (jack) {
        es.volume(ES8388::ES_OUT1, volume);
        es.select_out1();
        gpio_set_level(GPIO_PA_EN, 1);
      }
      jack = false;
    }

delay(100);
}
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
  while (1)
  {
    val = analogRead(ADC_battery);
    printf("Battery : %d\n");
    if (val < NYELLOW) pixels.setPixelColor(0, pixels.RED);      //RED
    else if (val > NGREEN) pixels.setPixelColor(0, pixels.GREEN); //GREEN
    else pixels.setPixelColor(0, pixels.YELLOW);                 //YELLOW
    pixels.show();
    taskYIELD(); // Cède la main aux autres tâches

    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}



////////////////////////////////////////////////
void audio_info(const char *info){
   /* int sampleRate;
    Serial.print("info        "); Serial.println(info);
    if(strstr(info, "SampleRate=") != nullptr) 
    {
    sscanf(info,"SampleRate=%d",&sampleRate);
    printf("==================>>>>>>>>>>%d\n", sampleRate);
    }
   */
} 
void audio_id3data(const char *info){  //id3 metadata
  //  Serial.print("id3data     ");Serial.println(info);
}
void audio_eof_mp3(const char *info){  //end of file
    Serial.print("eof_mp3     ");Serial.println(info);mp3ON = false;
}
void audio_showstation(const char *info){
   // Serial.print("station     ");Serial.println(info);
}
void audio_showstreaminfo(const char *info){
  //  Serial.print("streaminfo  ");Serial.println(info);
   // Serial.println("top");
}
void audio_showstreamtitle(const char *info){
 //   Serial.print("streamtitle ");Serial.println(info);
}
void audio_bitrate(const char *info){
 //   Serial.print("bitrate     ");Serial.println(info);
    
}
void audio_commercial(const char *info){  //duration in sec
 //   Serial.print("commercial  ");Serial.println(info);
}
void audio_icyurl(const char *info){  //homepage
 //   Serial.print("icyurl      ");Serial.println(info);
}
void audio_lasthost(const char *info){  //stream URL played
 //   Serial.print("lasthost    ");Serial.println(info);
}
void audio_eof_speech(const char *info){  
 //   Serial.print("eof_speech  ");Serial.println(info);
}
