#include "../include/duckHouseConstants.h"
#include <WiFi.h>
#include <Arduino.h>
#include "../include/globals.h"

#include "../include/globals.h"
#include "../include/communication.h"
#include "../include/utilities.h"
#include <ThingSpeak.h>

bool connectedBool = false;

WiFiClient client;

bool connectWiFi() {
  const uint8_t maxAttempts = 5;
  const uint32_t connectTimeoutMs = 10000UL;
  const uint32_t retryDelayMs = 5000UL;

  for (uint8_t attempt = 1; attempt <= maxAttempts; ++attempt) {
    WiFi.disconnect(true);
    delay(100);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    unsigned long startMs = millis();
    Serial.print("Connecting to WiFi attempt ");
    Serial.print(attempt);
    Serial.print("/");
    Serial.println(maxAttempts);

    while ((WiFi.status() != WL_CONNECTED) && ((millis() - startMs) < connectTimeoutMs)) {
      delay(500);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println();
      Serial.println("Connected");
      connectedBool = true;
      ThingSpeak.begin(client);
      blinkX(5, 100);
      return true;
    }

    Serial.println();
    Serial.println("WiFi connect failed");

    if (attempt < maxAttempts) {
      delay(retryDelayMs);
    }
  }

  connectedBool = false;
  Serial.println("WiFi failed after retries. Sleeping for a few seconds and trying again.");
  WiFi.disconnect(true);
  esp_sleep_enable_timer_wakeup(5ULL * 1000000ULL);
  esp_deep_sleep_start();
  return false;
}


int getThingSpeakData() {
  //get the relevant data from ThingSpeak and process it
  //perhaps retun action for the door?
  if (!connectWiFi()) {
    runStatus = 0;
    return 0;
  }

  ThingSpeak.readMultipleFields(readChannelId, readAPIKey);
  int doorCommand = ThingSpeak.getFieldAsInt(1);  //open close (-1,0, 1)
  //getRSSI
  int rssi = getStrength(3);
  int doorState = 0;
  if (checkLimit(1)) { doorState = 1; }
  if (checkLimit(0)) { doorState = -1; }
  float lightValue = analogRead(LIGHT_IN_PIN);
  ThingSpeak.setField(1, 0);
  ThingSpeak.setField(2, rssi);
  ThingSpeak.setField(3, doorState);
  ThingSpeak.setField(4, lightValue);
  ThingSpeak.writeFields(writeChannelId, writeAPIKey);
  if (doorCommand == 1) { startDoor(1); }
  if (doorCommand == -1) { startDoor(0); }
      if (doorCommand == 2) { startLift(1); }
  if (doorCommand == -2) { startLift(0); }
  runStatus = 0;
  return 1;
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