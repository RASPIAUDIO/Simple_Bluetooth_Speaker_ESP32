// ***** Muse LUXE as a Bluetooth speaker *****
// BT speaker name ==> LUXE-xxxxxx
// Play/Pause and volume (+/-) buttons managed
///////////////////////////////////////////////

#include "museWrover.h"
#include "BluetoothA2DPSink.h"      // Bluetooth A2DP sink library: https://github.com/pschatzmann/ESP32-A2DP
#include "A2DPVolumeControl.h"
#include "ESP_I2S.h"
#include "driver/gpio.h"
#include "esp_mac.h"
#include <Adafruit_NeoPixel.h>
#include "LittleFS.h"
#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "SPI.h"
#include "SD.h"
#include "FS.h"
#include "Audio.h"   // https://github.com/schreibfaul1/ESP32-audioI2S

// Define your SD card detect pin (adjust the pin number as needed)
#define SD_DET_PIN 34

bool BPause = false;         // Playback pause state: false = playing, true = paused
int volume = 75;             // Volume level (0...100)
#define maxVol 100          // Maximum volume

// Initialize I2S and Bluetooth A2DP Sink objects
I2SClass i2s;
BluetoothA2DPSink a2dp_sink(i2s);

uint8_t mac[6];              // Array to store MAC address
char macStr[20];             // String to store formatted MAC
char dev_name[30];           // Bluetooth device name
bool jack = false;           // Headphone jack detection state
ES8388 es;                   // ES8388 audio codec instance
uint8_t *b;                  // Pointer for WAV file buffer
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);  // NeoPixel for battery status

Audio audio;

// Flag to ensure SD files are read only once per card insertion
bool sdCardPlayed = false;

// Function to enumerate and play all files on the SD card
void playAllFilesFromSD() {
  Serial.println("SD Card inserted. Reading all files:");
  File root = SD.open("/");
  if (!root) {
    Serial.println("Failed to open root directory");
    return;
  }
  while (true) {
    File entry = root.openNextFile();
    if (!entry) {
      // No more files
      break;
    }
    if (!entry.isDirectory()) {
      Serial.print("Playing file: ");
      Serial.println(entry.name());
      audio.connecttoFS(SD, entry.name());
      // Play file until it finishes
      while (audio.isRunning()) {
        audio.loop();
        delay(10);
      }
    }
    entry.close();
  }
  root.close();
}

void setup() {
  Serial.begin(115200);

  // Initialize the Task Watchdog Timer (TWDT)
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 60 * 1000,                // 60 seconds timeout
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  
    .trigger_panic = true                  
  };
  esp_task_wdt_init(&wdt_config);

  // Initialize SD card SPI interface
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  SPI.setFrequency(1000000);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD Card initialization failed!");
  } else {
    Serial.println("SD Card initialized.");
  }
  
  // Initialize SD card detect pin
  pinMode(SD_DET_PIN, INPUT_PULLUP);

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
  gpio_set_pull_mode(Jack_Detect, GPIO_PULLUP_ONLY);

  // Factory Test Mode:
  if (gpio_get_level(BUTTON_VOL_PLUS) == 0 &&
      gpio_get_level(BUTTON_VOL_MINUS) == 0 &&
      gpio_get_level(BUTTON_PAUSE) == 0) {
    Serial.println("Factory test mode triggered");
    const esp_partition_t* partition = esp_partition_find_first(
                                         ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);
    if (partition != NULL) {
      esp_ota_set_boot_partition(partition);
      esp_restart();
    }
  }

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

  // Construct the Bluetooth device name using the MAC address
  esp_read_mac(mac, ESP_MAC_BT);
  snprintf(macStr, sizeof(macStr), "-%x%x%x", mac[3], mac[4], mac[5]);
  strcpy(dev_name, "LUXE");
  strcat(dev_name, macStr);
  Serial.print("Device name : ");
  Serial.println(dev_name);

  a2dp_sink.start(dev_name);
  Serial.println("Bluetooth A2DP Sink Initialized");

  // Initialize LittleFS and play the startup sound
  if (!LittleFS.begin()) {
    Serial.println("LittleFS initialisation failed!");
    while (1) { }
  }
  i2s.setPins(I2S_BCLK, I2S_LRCK, I2S_SDOUT, I2S_SDIN, I2S_MCLK);
  if (!i2s.begin(I2S_MODE_STD, 44100, I2S_DATA_BIT_WIDTH_16BIT,
                 I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
    Serial.println("Failed to initialize I2S!");
    while (1);
  }
  b = (uint8_t *)ps_malloc(160000);
  File ln = LittleFS.open("/bluetooth.wav", FILE_READ);
  ln.read(b, 115000);
  ln.close();
  i2s.playWAV(b, 115000);  // Play the startup sound
  free(b);
  i2s.end();
  i2s.begin(I2S_MODE_STD, 44100, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH);

  // Start the battery monitoring task on core 0
  xTaskCreatePinnedToCore(battery, "battery", 4096, NULL, 5, NULL, 0);
}

void loop() {
  // Check SD card insertion (active low detection)
  bool sdInserted = (digitalRead(SD_DET_PIN) == LOW);

  if (sdInserted) {
    // If the SD card is inserted, read and play all files (only once per insertion)
    if (!sdCardPlayed) {
      playAllFilesFromSD();
      sdCardPlayed = true;
    }
  } else {
    // SD card not inserted, reset flag and run Bluetooth streaming & button handling
    sdCardPlayed = false;
    
    if (a2dp_sink.get_audio_state() != ESP_A2D_AUDIO_STATE_STARTED) delay(1000);

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
        a2dp_sink.pause();
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
        a2dp_sink.play();
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
  }
}

//////////////////////////////////////////////////////////////////////////
// Battery Monitoring Task
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
    printf("Battery : %d\n", val);
    if (val < NYELLOW)
      pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    else if (val > NGREEN)
      pixels.setPixelColor(0, pixels.Color(0, 255, 0));
    else
      pixels.setPixelColor(0, pixels.Color(255, 255, 0));
    pixels.show();
    taskYIELD();
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

void audio_info(const char *info) {
  Serial.print("info        "); Serial.println(info);
}
void audio_id3data(const char *info) {
  Serial.print("id3data     "); Serial.println(info);
}
void audio_eof_mp3(const char *info) {
  Serial.print("eof_mp3     "); Serial.println(info);
}
