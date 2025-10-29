

#include "Arduino.h"
#include "WiFi.h"
#include "museS3.h"  
#include <TFT_eSPI.h>
MuseRadio radio;
TFT_eSPI tft = TFT_eSPI();

//----------------------------------------------------------------------------------------------------------------------

void setup()
{
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

}
//----------------------------------------------------------------------------------------------------------------------
void loop()
{
  char buf[8];
  int i;
    for (i=0; i<4 ; i++)
    {
      if(radio.button_get_level(i)  == 0) 
      {
        while(radio.button_get_level(i) == 0) delay(10);
        printf("%d\n", i);
        sprintf(buf, "SW%d\n", i + 1);
        tft.fillRect(132, 102, 66, 66, TFT_NAVY);
        tft.drawString(buf, 170, 125, 4);
      }
      delay(10);
    }
    
}
//----------------------------------------------------------------------------------------------------------------------
