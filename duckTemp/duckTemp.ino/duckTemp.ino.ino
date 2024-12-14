#include <WiFi.h>
#include <BMP180I2C.h>
#include <ThingSpeak.h>


#define I2C_ADDRESS 0x77
#define uS_TO_S_FACTOR 1000000ULL       /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 5               /* Time ESP32 will go to sleep (in seconds) */

char* ssid = "still_waters";
const char* password = "33turkeys511";
const char* writeAPIKey = "SAHC91JSCH9HBCGS";  //write channel
long writeChannelId = 1056342;
//create an BMP180 object using the I2C interface
BMP180I2C bmp180(I2C_ADDRESS);
WiFiClient client;

void setup() {
  	Serial.begin(115200);
  // put your setup code here, to run once:
	Wire.begin();
  	bmp180.resetToDefaults();
    	bmp180.setSamplingMode(BMP180MI::MODE_UHR);
    connectWiFi();
}

void loop() {
  // put your main code here, to run repeatedly:
	if (!bmp180.measureTemperature())
	{
		Serial.println("could not start temperature measurement, is a measurement already running?");
		return;
	}
	do
	{
		delay(100);
	} while (!bmp180.hasValue());

  	Serial.print("Temperature: "); 
    float cTemp=bmp180.getTemperature();
    float fTemp=9/5*cTemp+32;
  	Serial.print(fTemp); 
	  Serial.println(" degC");

    ThingSpeak.setField(1, fTemp);
    ThingSpeak.writeFields(writeChannelId, writeAPIKey);

      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

      Serial.println("Going to sleep now");
      delay(1000);
      esp_deep_sleep_start();
}


void connectWiFi() {
  int tryCounter = 0;

  while ((WiFi.status() != WL_CONNECTED)) {
    WiFi.begin(ssid, password);
    if (tryCounter > 5) {
   //   connectedBool = false;
    }

    delay(7500);
    Serial.println("Connecting to WiFi");
    tryCounter++;
  }
  Serial.println("Connected");
  //connectedBool = true;
  ThingSpeak.begin(client);
  tryCounter = 0;
//  blinkX(5, 100);
}

