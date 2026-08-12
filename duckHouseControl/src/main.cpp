//#include <analogWrite.h>
#include <Arduino.h>
#include "utilities.h"
#include "communication.h"
#include "../include/globals.h"
#include "../include/duckHouseConstants.h"
#include <ThingSpeak.h>


/*
NOTE:
======
Only RTC IO can be used as a source for external wake
source. They are pins: 0,2,4,12-15,25-27,32-39.
*/

RTC_DATA_ATTR int bootCount = 0;
int wakeReason = 0;  //1 for sleep, 2 for pin
 const char* ssid  = "still_waters";
 const char* password = "33turkeys511";
 const char* writeAPIKey = "DI6OLF96SEXYI8M6";
 const char* readAPIKey = "HP7M4ROAO0DFZ6YZ";
 const long readChannelId = 1415485L;
 const long writeChannelId = 1415485L;


const long TimeToSleepSeconds = 150;

long wakeTime=millis();

//just main 
int ledChannel = 0;
int ledChannelL = 1;
int liftLedChannel = 2;
int liftLedChannelL = 3;
int doorMoving = 0;  //1 for open, 2 for close
int liftMoving = 0;  //1 for open, 2 for close
int liftTimeLimit = 3000;
int doorTimeLimit = 3000;
int buttonPushed = 0;  //1 open door, 2 close door, 3 open lift, 4 close lift
int runStatus = 0;  //0 for idle, 1 for door open, 2 for door close, 3 for lift open, 4 for lift close, 5 for wifi check
unsigned long buttonTime = 0;
unsigned long lastButtonTime = millis();
unsigned int debounceTime = 200;
long doorTime = millis();
long liftTime = millis();

String myStatus="";

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void IRAM_ATTR openDoorButtonInt() {
  buttonTime = millis();
  if (buttonTime - lastButtonTime > debounceTime) {
    buttonPushed = 1;
    lastButtonTime = buttonTime;
  }
}
void IRAM_ATTR closeDoorButtonInt() {
  buttonTime = millis();
  if (buttonTime - lastButtonTime > debounceTime) {
    buttonPushed = 2;
    lastButtonTime = buttonTime;
  }
}
void IRAM_ATTR openLiftButtonInt() {
  buttonTime = millis();
   // Serial.println("0NT!");
  if (buttonTime - lastButtonTime > debounceTime) {
    buttonPushed = 3;
    lastButtonTime = buttonTime;
  }
}
void IRAM_ATTR closeLiftButtonInt() {
  buttonTime = millis();
  //Serial.println("INT!");
  if (buttonTime - lastButtonTime > debounceTime) {
    buttonPushed = 4;
    lastButtonTime = buttonTime;
  }
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup caused by external signal using RTC_IO");
      wakeReason = 4;
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      Serial.println("Wakeup caused by external signal using RTC_CNTL");
      wakeReason = 1;
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup caused by timer");
      wakeReason = 2;
      runStatus = 5;
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP: Serial.println("Wakeup caused by ULP program"); break;
    default: Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

int print_GPIO_wake_up() {
  uint64_t GPIO_reason = esp_sleep_get_ext1_wakeup_status();
  Serial.print("GPIO that triggered the wake up: GPIO ");
  Serial.println((log(GPIO_reason)) / log(2), 0);
  int pinValue = int((log(GPIO_reason)) / log(2));
  return pinValue;
}


void setup() {
  Serial.begin(115200);
  delay(100);  //Take some time to open up the Serial Monitor
  pinMode(LEDPin, OUTPUT);
  blinkX(8, 20);

  pinMode(OPEN_LIMIT_POWER, OUTPUT);
  pinMode(CLOSE_LIMIT_POWER, OUTPUT);
  digitalWrite(OPEN_LIMIT_POWER, LOW);
  digitalWrite(CLOSE_LIMIT_POWER, LOW);

  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  digitalWrite(R_EN, HIGH);
  digitalWrite(L_EN, HIGH);  
  int freq = 5000;
  int resolution = 8;

  pinMode(OPEN_DOOR_BUTTON, INPUT_PULLDOWN);
  pinMode(CLOSE_DOOR_BUTTON, INPUT_PULLDOWN);
  pinMode(OPEN_LIFT_BUTTON, INPUT_PULLDOWN);
  pinMode(CLOSE_LIFT_BUTTON, INPUT_PULLDOWN);
  pinMode(OPEN_LIMIT_PIN, INPUT_PULLDOWN);
  pinMode(CLOSE_LIMIT_PIN, INPUT_PULLDOWN);

  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(R_PWM, ledChannel);
  ledcSetup(ledChannelL, freq, resolution);
  ledcAttachPin(L_PWM, ledChannelL);

  pinMode(LIFT_POWER, OUTPUT);
  pinMode(LIFT_UP, OUTPUT);
  pinMode(LIFT_DOWN, OUTPUT);

  pinMode(LIGHT_IN_PIN, INPUT);
  pinMode(LIFT_TIME_PIN, INPUT_PULLUP);
  pinMode(DOOR_TIME_PIN, INPUT_PULLUP);
  pinMode(TIME_POWER_PIN, OUTPUT);

  digitalWrite(LIFT_POWER, LOW);

  //Increment boot number and print it every reboot
  // ++bootCount;
  // Serial.println("Boot number: " + String(bootCount));

  getTimeLimits();

  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  Serial.println("waker reason " + String(wakeReason));

  attachInterrupt(OPEN_DOOR_BUTTON, openDoorButtonInt, RISING);
  attachInterrupt(CLOSE_DOOR_BUTTON, closeDoorButtonInt, RISING);
  attachInterrupt(OPEN_LIFT_BUTTON, openLiftButtonInt, RISING);
  attachInterrupt(CLOSE_LIFT_BUTTON, closeLiftButtonInt, RISING);

//  buttonPushed = 0;
  //Serial.println("Run Status " + String(runStatus));

  doorTime = millis();
 liftTime = millis();
}

void loop() {
  //
  //Check wakeup reason
  //if pin, call start open door or open lift
  //else check wifi for door open command
  //process wifi commands
  //check limit sensors
  //check timer
  //Serial.println("Status " + String(runStatus));
  if (wakeReason == 1) {
    int myGPIO = print_GPIO_wake_up();
    Serial.println(String(int(myGPIO)));
    if (myGPIO == OPEN_DOOR_BUTTON) {
      Serial.println("open door wake int " + String(myGPIO));
      blinkX(2, 20);
      startDoor(1);
    }
    if (myGPIO == CLOSE_DOOR_BUTTON) {
      Serial.println("close door wake int " + String(myGPIO));
      blinkX(3, 20);
      startDoor(0);
    }
    if (myGPIO == OPEN_LIFT_BUTTON) {
      Serial.println("open lift wake int " + String(myGPIO));
      blinkX(4, 20);
      startLift(1);
    }
    if (myGPIO == CLOSE_LIFT_BUTTON) {
      Serial.println("close lift wake int " + String(myGPIO));
      blinkX(5, 20);
      startLift(0);
    }
    wakeReason = 0;
      //attaching innterrupts later didnt help
 
  }

//delay(3000);

  if (doorMoving > 0) {
    blinkX(2,46);
    if (millis() - doorTime > doorTimeLimit) {
      Serial.println("door time expired ");
      stopDoor();
      runStatus = 0;
    }
  
  }

  if (liftMoving > 0) {
    blinkX(3,15);
    if (millis() - liftTime > liftTimeLimit) {
      Serial.println("lift up time expired ");
      stopLift();  //lift opening check timer
      runStatus = 0;
    }
  }

  if (wakeReason == 2) {
    Serial.println("read write ThingSpeak");
    getThingSpeakData();  //check internet for commands
  }


  if (buttonPushed == 1) {
    blinkX(2, 25);
    Serial.println("buttonEvent " + String(buttonPushed));
      buttonPushed = 0;
      if (!doorMoving){
    startDoor(1);
      }
      else{
      stopDoor();
      }
      wakeTime=millis();
  }
  if (buttonPushed == 2) {
    blinkX(3, 25);
        Serial.println("buttonEvent " + String(buttonPushed));
      buttonPushed = 0;
         if (!doorMoving){
    startDoor(0);
         }
         else{
           stopDoor();
         }
         wakeTime=millis();
  }
  if (buttonPushed == 3) {
    blinkX(4, 25);
      Serial.println("buttonEvent " + String(buttonPushed));
      buttonPushed = 0;
   if (!liftMoving){
    startLift(1);
   }else{
     stopLift();
       }
       wakeTime=millis();
  }

  if (buttonPushed == 4) {
    blinkX(5, 25);
       Serial.println("buttonEvent " + String(buttonPushed));
      buttonPushed = 0;
         if (!liftMoving){
    startLift(0);
         }else{
           stopLift();
         } 
         wakeTime=millis();
  }

  delay(5);
  if (millis()-wakeTime>AWAKE_TIME){
  if (!(doorMoving || liftMoving)) {
    //delay(2250);
    //go back to sleep
    //Go to sleep now
    //power down everything just in case
   stopLift();
   stopDoor();
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
    esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
    Serial.println("Going to sleep now");
    delay(1000);
    esp_deep_sleep_start();
  }
  }
}