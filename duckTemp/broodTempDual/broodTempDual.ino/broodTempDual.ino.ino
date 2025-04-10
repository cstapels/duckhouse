// Example testing sketch for various DHT humidity/temperature sensors written by ladyada
// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include "DHT.h"
#include <WiFi.h>
#include <ThingSpeak.h>

#define DHTPIN 4     // Digital pin connected to the DHT sensor
#define DHTPINSECOND 2
#define uS_TO_S_FACTOR 1000000ULL       /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 60              /* Time ESP32 will go to sleep (in seconds) */

// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

DHT dht(DHTPIN, DHTTYPE);
DHT dht2(DHTPINSECOND, DHTTYPE);
char* ssid = "still_waters";
const char* password = "33turkeys511";
const char* writeAPIKey = "50SLPV0KB2EIL4ZC";  //write channel
long writeChannelId = 1102903;


WiFiClient client;

void setup() {
  pinMode(DHTPIN, INPUT_PULLUP);
  pinMode(DHTPINSECOND, INPUT_PULLUP);
  Serial.begin(115200);
  Serial.println(F("startypants"));
     connectWiFi();
  dht.begin();
    dht2.begin();
}

void loop() {
if(WiFi.status() != WL_CONNECTED){
    
    connectWiFi();  
    }

  // Wait a few seconds between measurements.
  delay(20);

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h2 = dht2.readHumidity();
  // Read temperature as Celsius (the default)
  float t2 = dht2.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f2 = dht2.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h2) || isnan(t2) || isnan(f2)) {
    Serial.println(F("Failed to read from DHT sensor 2!"));
    return;
  }
delay(2000);

  Serial.print(F("Humidity: "));
  Serial.print(h);
    Serial.print(" , ");
      Serial.print(h2);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
      Serial.print(" , ");
      Serial.print(t2);
  Serial.print(F("°C "));
  Serial.print(f);    Serial.print(" , ");
      Serial.print(f2);



    ThingSpeak.setField(1, f);
       ThingSpeak.setField(2, f2);
          ThingSpeak.setField(3, h);
       ThingSpeak.setField(4, h2);
          ThingSpeak.setField(8, getStrength(3));
    ThingSpeak.writeFields(writeChannelId, writeAPIKey);

toSleep();
      delay(10000);//never get here


}

void toSleep(){
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
    delay(8500);
    if (tryCounter > 5) {
    toSleep();
    }

    
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