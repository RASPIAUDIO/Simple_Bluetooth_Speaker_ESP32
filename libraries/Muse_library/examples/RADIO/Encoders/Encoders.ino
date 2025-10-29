

#include "Arduino.h"
#include "WiFi.h"
#include "museS3.h"  
#include <TFT_eSPI.h>
#include "ESP32Encoder.h"

TFT_eSPI tft = TFT_eSPI();
ESP32Encoder volEncoder;
ESP32Encoder staEncoder;
//----------------------------------------------------------------------------------------------------------------------

void setup()
{
  char buf[16];
    Serial.begin(115200);
    Serial.println("\r\nReset");
  //////////////////////////////////////////////////////
  //Screen init
  //////////////////////////////////////////////////////
  printf("screen init...\n");
  tft.init();
  tft.setRotation(1);
  tft.setTextColor(TFT_RED);
  tft.setTextDatum(TC_DATUM);
  tft.fillScreen(TFT_NAVY);
  tft.fillRect(130, 100, 70, 70, TFT_WHITE);
  tft.fillRect(132, 102, 66, 66, TFT_NAVY); 

  //////////////////////////////////////////////////
  //Encoders init
  //////////////////////////////////////////////////
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  volEncoder.attachHalfQuad(ENC_A1, ENC_B1);
  staEncoder.attachHalfQuad(ENC_A2, ENC_B2);
  // Initialize Encoder button pins
  pinMode(CLICK1, INPUT_PULLUP);
  pinMode(CLICK2, INPUT_PULLUP);  

  volEncoder.setCount(0);
  tft.setTextColor(TFT_RED);
  tft.setTextDatum(TC_DATUM);
  while (gpio_get_level(CLICK1) == 1)
  {
     sprintf(buf, "%d\n", volEncoder.getCount());
    tft.fillRect(150, 110, 35, 35, TFT_NAVY);
    tft.drawString(buf, 170, 125, 4);
    delay(100);   
  }
  tft.fillRect(150, 110, 35, 35, TFT_NAVY);  
  tft.drawString("OK", 170, 125, 4);
  staEncoder.setCount(0);
  tft.setTextColor(TFT_YELLOW);
  tft.setTextDatum(TC_DATUM);
  while (gpio_get_level(CLICK2) == 1)
  {
     sprintf(buf, "%d\n", staEncoder.getCount());
    tft.fillRect(150, 110, 35, 35, TFT_NAVY);
    tft.drawString(buf, 170, 125, 4);
    delay(100);   
  }
  tft.fillRect(150, 110, 35, 35, TFT_NAVY);  
  tft.drawString("OK", 170, 125, 4); 
}
//----------------------------------------------------------------------------------------------------------------------
void loop()
{
      delay(10);
    
    
}
//----------------------------------------------------------------------------------------------------------------------
