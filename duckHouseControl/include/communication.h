#pragma once 

bool connectWiFi();
void initializeWiFi();
void sendToThingSpeak();
void readFromThingSpeak();
int getThingSpeakData();
int getStrength(int points);