// ***** Muse LUXE as a Bluetooth speaker + SD player*****
// BT speaker name ==> LUXE-xxxxxx
// Play/Pause and volume (+/-) buttons managed
///////////////////////////////////////////////

#include "museWrover.h"
#include "ESP_I2S.h"
#include "driver/gpio.h"
#include <Adafruit_NeoPixel.h>
#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "SPI.h"
#include "SD.h"
#include "FS.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_err.h"

#define NUMPIXELS 1

// Define your SD card detect pin (adjust the pin number as needed)
#define SD_DET_PIN GPIO_NUM_34

int volume = 75;             // Volume level (0...100)
#define maxVol 100          // Maximum volume
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
I2SClass i2s;     
ES8388 es;                   // ES8388 audio codec instance


void setup() {
  Serial.begin(115200);

  // Initialize the Task Watchdog Timer (TWDT)(for ESD event)
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 5 * 1000,                // 5 seconds timeout
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,  
    .trigger_panic = true                  
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL); 
  
// prépare le redémarrage  classique
  const esp_partition_t* partition = esp_partition_find_first(
                                         ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
    if (partition != NULL) {
      esp_ota_set_boot_partition(partition);
    }
 
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
  pixels.begin();
  pixels.setBrightness(50);
  pixels.setPixelColor(0, pixels.Color(255,255,255));
  pixels.show();
  while((gpio_get_level(BUTTON_PAUSE) != LOW) || (gpio_get_level(BUTTON_VOL_PLUS) != LOW) || (gpio_get_level(BUTTON_VOL_MINUS) != LOW))
  {
    delay(500);
  }





  xTaskCreatePinnedToCore(factoryTest, "factoryTest", 8192, NULL, 5, NULL, 0);
}

void loop() {
  esp_task_wdt_reset();
  delay(100);
}

////////////////////////////////////////////////////////////////////////////////
// FACTORY TEST
////////////////////////////////////////////////////////////////////////////////
// Pour factory test via FFT et beep :
// FFT in-place sur 1024 échantillons (canal unique)
// Pour factory test via FFT et beep :
#define SAMPLE_RATE     16000      
#define BEEP_FREQ       500.0       // 500 Hz beep
#define RECORD_TIME_SEC 1           // Durée d'enregistrement en secondes
#define TOTAL_SAMPLES   (SAMPLE_RATE * RECORD_TIME_SEC)
#define TOTAL_BYTES     (TOTAL_SAMPLES * 4) // 32-bit par échantillon
#define I2S_CHUNK_SIZE  256         // Taille d'un chunk en frames
#define TOLERANCE_HZ    10.0f       // ±50 Hz tolerance (10% de 500Hz)

////////////////////////////////////////////////////////////////////////////////
// FACTORY TEST
////////////////////////////////////////////////////////////////////////////////
// Pour factory test via FFT et beep :
// FFT in-place sur 1024 échantillons (canal unique)

static void fft1024(float* real, float* imag) {
  const int N = 1024;
  // Réordonnancement par inversion de bits
  int j = 0;
  for (int i = 0; i < N; i++) {
    if (i < j) {
      float tempR = real[i];
      real[i] = real[j];
      real[j] = tempR;
      float tempI = imag[i];
      imag[i] = imag[j];
      imag[j] = tempI;
    }
    int m = N >> 1;
    while (m && (j >= m)) {
      j -= m;
      m >>= 1;  
    }
    j += m;
  }
  // Algorithme de Danielson-Lanczos
  for (int len = 2; len <= N; len <<= 1) {
    float theta = -2.0f * M_PI / (float)len;
    float wpr = cosf(theta) - 1.0f;
    float wpi = sinf(theta);
    for (int group = 0; group < N; group += len) {
      float wr = 1.0f, wi = 0.0f;
      for (int pair = 0; pair < (len >> 1); pair++) {
        int i1 = group + pair;
        int i2 = i1 + (len >> 1);
        float tmpR = wr * real[i2] - wi * imag[i2];
        float tmpI = wr * imag[i2] + wi * real[i2];
        real[i2] = real[i1] - tmpR;
        imag[i2] = imag[i1] - tmpI;
        real[i1] += tmpR;
        imag[i1] += tmpI;
        float wtemp = wr;
        wr += wtemp * wpr - wi * wpi;
        wi += wi * wpr + wtemp * wpi;
      }
    }
  }
}

// Initialise l'I2S en mode full-duplex (stéréo) sur I2S_NUM_0
static void i2sInitFullDuplex() {
  i2s.setPins(I2S_BCLK, I2S_LRCK, I2S_SDOUT, I2S_SDIN, I2S_MCLK);
  if (!i2s.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
    Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }
}

// Génère un bip de 500 Hz sur le canal choisi et enregistre depuis le micro (canal gauche)
// beepOnLeft = true => bip sur canal gauche, false => bip sur canal droit
void loopBeepAndRecord(bool beepOnLeft, uint8_t* micBuffer) {
  double phaseIncrement = (2.0 * M_PI * BEEP_FREQ) / SAMPLE_RATE;
  double phase = 0.0;
  int32_t txChunk[2 * I2S_CHUNK_SIZE];
  int32_t rxChunk[2 * I2S_CHUNK_SIZE];
 
  int samplesCaptured = 0;
  
  while (samplesCaptured < TOTAL_SAMPLES) {
    // Génère un chunk de données stéréo
    for (int i = 0; i < I2S_CHUNK_SIZE; i++) {
      float sampleF = sinf(phase) * 100.0f; // amplitude
      phase += phaseIncrement;
      int32_t val = (int32_t)(sampleF * 8388607.0f);
      if (beepOnLeft) {
        txChunk[2 * i + 0] = val;  // canal gauche : bip
        txChunk[2 * i + 1] = 0;    // canal droit : silence
      } else {
        txChunk[2 * i + 0] = 0;    // canal gauche : silence
        txChunk[2 * i + 1] = val;  // canal droit : bip
      }
    }
    size_t bytesWritten = 0;
    bytesWritten = i2s.write((uint8_t*)txChunk, sizeof(txChunk));
    size_t bytesRead = 0;
    bytesRead = i2s.readBytes((char*)rxChunk, sizeof(rxChunk));
    int framesRead = bytesRead / (2 * sizeof(int32_t));
    for (int f = 0; f < framesRead; f++) {
      // On suppose ici que le micro capte sur le canal gauche (index 0)
      ((int32_t*)micBuffer)[samplesCaptured] = rxChunk[2 * f + 0];
      samplesCaptured++;
      if (samplesCaptured >= TOTAL_SAMPLES) break;
    }
  }
 // i2s.end();
}

// Analyse FFT sur 1024 échantillons extraits de micBuffer et vérifie que le pic est autour de 500 Hz ± TOLERANCE_HZ.
bool analyzeFFT(const uint8_t* micBuffer) {
  int startSample = TOTAL_SAMPLES / 2 - 512;
  if (startSample < 0) startSample = 0;
  float* vReal = (float*) heap_caps_malloc(1024 * sizeof(float), MALLOC_CAP_SPIRAM);
  float* vImag = (float*) heap_caps_malloc(1024 * sizeof(float), MALLOC_CAP_SPIRAM);
  if (!vReal || !vImag) {
    Serial.println("Allocation FFT échouée");  pixels.setPixelColor(0, pixels.Color(255,255,255));
  pixels.show();
    if(vReal) free(vReal);
    if(vImag) free(vImag);
    return false;
  }
  for (int i = 0; i < 1024; i++) {
    int32_t raw = ((int32_t*)micBuffer)[startSample + i];
    vReal[i] = (float)raw;
    vImag[i] = 0.0f;
  }
  // Supprime DC
  float sum = 0.0f;
  for (int i = 0; i < 1024; i++) sum += vReal[i];
  float dc = sum / 1024.0f;
  for (int i = 0; i < 1024; i++) vReal[i] -= dc;
  fft1024(vReal, vImag);
  float maxMag = 0.0f;
  int binMax = 0;
  for (int b = 0; b < 512; b++) {
    float mag = vReal[b]*vReal[b] + vImag[b]*vImag[b];
    if (mag > maxMag) {
      maxMag = mag;
      binMax = b;
    }
  }
  free(vReal);
  free(vImag);
  float binRes = (float)SAMPLE_RATE / 1024.0f;
  float freq = binMax * binRes ;
  Serial.printf("FFT pic: bin=%d => ~%.1f Hz\n", binMax, freq);
  bool pass = (freq >= 500.0 - TOLERANCE_HZ && freq <= 500.0 + TOLERANCE_HZ);
  Serial.printf("Test => %s\n", pass ? "PASS" : "FAIL");
  return pass;
}

// Clignote le NeoPixel en bleu un certain nombre de fois
static void blinkBlue(int times) {
  for (int i = 0; i < times; i++) {
    pixels.setPixelColor(0, pixels.Color(0, 0, 255));
    pixels.show();
    delay(300);
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();
    delay(300);
  }
}

// La fonction factoryTest() lancée si les 3 boutons sont enfoncés au démarrage.
static void factoryTest(void* p) {
  Serial.println("=== MODE FACTORY TEST ===");
  

  // Réinitialise l'I2S en mode full-duplex pour test
  
  i2sInitFullDuplex();
  delay(1000);
  // Allume le NeoPixel en blanc pour indiquer que le test est lancé
 // pixels.begin();
//  pixels.setBrightness(50);
  pixels.setPixelColor(0, pixels.Color(255,255,255));
  pixels.show();

  delay(1000);
  blinkBlue(4);
  delay(500);
  pixels.setPixelColor(0, pixels.Color(0, 0, 255));
  pixels.show();

  // Allocation du buffer pour les échantillons du micro
  uint8_t* micBuffer = (uint8_t*) ps_malloc(TOTAL_BYTES);
  if (!micBuffer) {
    Serial.println("Allocation de micBuffer échouée!");
    while(1) delay(1000);
  }

  Serial.println("=== Test LEFT Speaker -> Left Mic ===");
  loopBeepAndRecord(true, micBuffer);
  bool leftOK = analyzeFFT(micBuffer);

  Serial.println("=== Test RIGHT Speaker -> Left Mic ===");
  loopBeepAndRecord(false, micBuffer);
  bool rightOK = analyzeFFT(micBuffer);

  free(micBuffer);
  i2s.end();
  Serial.println("===== RÉSULTATS FINALS =====");

  if (!leftOK && !rightOK) {
    Serial.println("Les deux tests échouent.");
    // LED rouge
    pixels.setPixelColor(0, pixels.Color(255,0,0));
  } else if (leftOK && !rightOK) {
    Serial.println("Test gauche PASS, test droit FAIL.");
    // LED orange
    pixels.setPixelColor(0, pixels.Color(255,128,0));
  } else if(!leftOK && rightOK) {
    Serial.println("Test gauche FAIL, test droit PASS.");
    // LED violet
    pixels.setPixelColor(0, pixels.Color(128,0,128));
  }
  pixels.show();
  delay(3000);
  if(!leftOK || !rightOK)
  {
      // LED rouge
    pixels.setPixelColor(0, pixels.Color(255,0,0));
    pixels.show();
    delay(2000);
    esp_restart();   
  }   
  blinkBlue(4);
  // Initialize SD card SPI interface
  bool SdOK = true;
  uint8_t b[15];
  if(gpio_get_level(SD_DET_PIN) == LOW)
  {
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  SPI.setFrequency(1000000);
  if (!SD.begin(SD_CS)) SdOK = false;
  File f = SD.open("/test.txt", FILE_WRITE);
  if(!f)SdOK = false;
  
  f.write((uint8_t*)"RASPIAUDIO", 11);
  f.close();
  f = SD.open("/test.txt", FILE_READ);
   if(!f) SdOK = false;
  f.read(b, 11);
  if(strcmp("RASPIAUDIO", (char*)b) != 0) SdOK = false;
  }
  else SdOK = false;

  if(SdOK) pixels.setPixelColor(0, pixels.Color(0,255,0));
  else  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();
  Serial.println("=== Factory Test terminé ===");
  delay(1000);
  for(;;)delay(10);
  //esp_restart();
}


//////////////////////////////////////////////////////////////////////////////////////////////
