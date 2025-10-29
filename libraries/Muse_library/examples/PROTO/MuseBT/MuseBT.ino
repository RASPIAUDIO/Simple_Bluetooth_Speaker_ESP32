// ***** Muse LUXE as a Bluetooth speaker *****
// BT speaker name ==> PROTO-xxxxxx
//
///////////////////////////////////////////////


#include "museWrover.h"
#include "BluetoothA2DPSink.h"   //https://github.com/pschatzmann/ESP32-A2DP
#include "ESP_I2S.h"
#include "driver/gpio.h"
#include "esp_mac.h"

bool BPause = false;
int volume = 100;                            // 0...100

// Initialize your objects and variables
I2SClass i2s;
BluetoothA2DPSink a2dp_sink(i2s);

uint8_t mac [6];
char macStr[20];
char dev_name [30];


void setup() {
  Serial.begin(115200);
////// GPIOs init  
  // power enable
  gpio_reset_pin(GPIO_PA_EN);
  gpio_set_direction(GPIO_PA_EN, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_PA_EN, HIGH);
 
  
//////I2S init
  i2s.setPins(I2S_BCLK, I2S_LRCK, I2S_SDOUT, I2S_SDIN, I2S_MCLK);
  if (!i2s.begin(I2S_MODE_STD, 44100, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH)) {
    Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }



////// Specific BT device name (using  MAC ref)

  esp_read_mac((uint8_t*)&mac, ESP_MAC_BT);
  snprintf(macStr, 19, "-%x%x%x", mac[3], mac[4], mac[5]);
  strcpy(dev_name, "PROTO");
  strcat(dev_name, macStr);
  printf("Device name : %s\n", dev_name);
  
////// Bluetooth A2DP sink init
  a2dp_sink.start(dev_name);
  Serial.println("Bluetooth A2DP Sink Initialized");
}

void loop() {
    delay(100);
  }
