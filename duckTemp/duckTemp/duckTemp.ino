#include <WiFi.h>
#include <BMP180I2C.h>
#include <ThingSpeak.h>

#define I2C_ADDRESS 0x77
#define uS_TO_S_FACTOR 1000000ULL       /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 1000               /* Time ESP32 will go to sleep (in seconds) */

char* ssid = "still_waters";
const char* password = "33turkeys511";
const char* writeAPIKey = "SAHC91JSCH9HBCGS";  //write channel
long writeChannelId = 1056342;
//create an BMP180 object using the I2C interface
BMP180I2C bmp180(I2C_ADDRESS);
WiFiClient client;

void setup() {
  	Serial.begin(115200);
    Serial.println("Start");
      connectWiFi();
  // put your setup code here, to run once:
	   Wire.begin();


    // Try to initialize BMP180 sensor
    if (!bmp180.begin()) {
        Serial.println("BMP180 initialization failed. Check interface and address.");
        ThingSpeak.setStatus("Sensor Error: BMP180 not detected");
        ThingSpeak.writeFields(writeChannelId, writeAPIKey);

        // Optionally, retry a few times before giving up
        for (int i = 0; i < 3; i++) {
            delay(1000); // Wait 1 second before retrying
            if (bmp180.begin()) {
                Serial.println("BMP180 successfully initialized after retry.");
                ThingSpeak.setStatus("Sensor recovered after retry.");
                ThingSpeak.writeFields(writeChannelId, writeAPIKey);
                break;
            }
        }

        if (!bmp180.begin()) {
            Serial.println("BMP180 initialization failed after retries. Continuing without sensor.");
            ThingSpeak.setStatus("Sensor permanently unavailable.");
            ThingSpeak.writeFields(writeChannelId, writeAPIKey);
        }
      Serial.println("Going to sleep now");
      delay(1000);
      esp_deep_sleep_start();
    }

  	  bmp180.resetToDefaults();
    	bmp180.setSamplingMode(BMP180MI::MODE_UHR);
      client.setConnectionTimeout(30*1000);
   
     
}

void loop() {
delay(1000);
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
    float cTemp = bmp180.getTemperature();
      Serial.println(String(cTemp) + " degC");
    float fTemp= 1.8 * cTemp + 32;
  	Serial.print(fTemp); 
	  Serial.println(" degF");
    	//wait for the measurement to finish. proceed as soon as hasValue() returned true. 
	do
	{
		delay(100);
	} while (!bmp180.hasValue());

	Serial.print("Pressure: "); 
  float myPres = bmp180.getPressure();
	Serial.print(myPres);
	Serial.println(" Pa");


    ThingSpeak.setField(1, fTemp);
       ThingSpeak.setField(2, myPres);
          ThingSpeak.setField(3, getStrength(3));
    ThingSpeak.writeFields(writeChannelId, writeAPIKey);

      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
      Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

      Serial.println("Going to sleep now");
      delay(1000);
      esp_deep_sleep_start();
      delay(10000);//never get here
      
}


void connectWiFi() {
  int tryCounter = 0;

  while ((WiFi.status() != WL_CONNECTED)) {
    WiFi.begin(ssid, password);
    if (tryCounter > 8) {
   //   connectedBool = false;
   //reset
   ESP.restart();
    }

    delay(1500+tryCounter*100);
    Serial.println("Connecting to WiFi");
    tryCounter++;
  }
  Serial.println("Connected");
  //connectedBool = true;
  ThingSpeak.begin(client);
  tryCounter = 0;
//  blinkX(5, 100);
}

// Take measurements of the Wi-Fi strength and return the average result.
int getStrength(int points) {
  long rssi = 0;
  long averageRSSI = 0;
  for (int i = 0; i < points; i++) {
    rssi += WiFi.RSSI();
    delay(20);
  }
  averageRSSI = rssi / points;
  return averageRSSI;
}

