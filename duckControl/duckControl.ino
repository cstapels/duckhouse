//#include <analogWrite.h>
#include <WiFi.h>
#include <ThingSpeak.h>

/*
NOTE:
======
Only RTC IO can be used as a source for external wake
source. They are pins: 0,2,4,12-15,25-27,32-39.
*/

#define BUTTON_PIN_BITMASK 0xF00000000  //0xF00000000 // 2^32 + 2^33 + 2^34 + 2^35 in hex
#define uS_TO_S_FACTOR 1000000ULL       /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 150               /* Time ESP32 will go to sleep (in seconds) */
#define AWAKE_TIME 18000
#define OPEN_LIMIT_PIN 15
#define OPEN_LIMIT_POWER 18  //0
#define CLOSE_LIMIT_PIN 21
#define CLOSE_LIMIT_POWER 04  //was 04
#define LIFT_TIME_LIMIT 60 * 1000
#define DOOR_TIME_LIMIT 30 * 1000
#define LEDPin 2
#define DOOR_TIME_PIN 39
#define LIFT_TIME_PIN 27
#define TIME_POWER_PIN 26
#define LIGHT_POWER 5

#define LONG_BUTTON_TIME 200

#define R_EN 17
#define L_EN 5
#define R_PWM 16
#define L_PWM 19

#define LIFT_UP 14
#define LIFT_DOWN 12
#define LIFT_POWER 13

#define OPEN_DOOR_BUTTON 33
#define CLOSE_DOOR_BUTTON 32
#define OPEN_LIFT_BUTTON 35
#define CLOSE_LIFT_BUTTON 34

#define LIGHT_IN_PIN 36

RTC_DATA_ATTR int bootCount = 0;
int wakeReason = 0;  //1 for sleep, 2 for pin
long liftTime = millis();
long doorTime = millis();
int liftTimeLimit = 3000;
int doorTimeLimit = 3000;
long wakeTime = millis();
long longButtonTime = 0;
bool buttonTimerOn = false;

int drivePin;
int runStatus = 0;
int buttonPushed = 0;  //1 open door, 2 close door, 3 open lift, 4 close lift
int doorMoving = 0;
int liftMoving = 0;

WiFiClient client;
// Network information
char* ssid = "still_waters";
const char* password = "33turkeys511";
const char* writeAPIKey = "DI6OLF96SEXYI8M6";  //write channel
const char* readAPIKey = "HP7M4ROAO0DFZ6YZ";   //control channel
long readChannelId = 1415485;
long writeChannelId = 1415485;
unsigned long buttonTime = 0;
unsigned long lastButtonTime = millis();
unsigned int debounceTime = 500; //was 1500
bool connectedBool = false;
String myStatus = "";
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

void connectWiFi() {
  int tryCounter = 0;

  while ((WiFi.status() != WL_CONNECTED)) {
    WiFi.begin(ssid, password);
    if (tryCounter > 5) {
      connectedBool = false;
    }

    delay(7500);
    Serial.println("Connecting to WiFi");
    tryCounter++;
  }
  Serial.println("Connected");
  connectedBool = true;
  ThingSpeak.begin(client);
  tryCounter = 0;
  blinkX(5, 100);
}

void blinkX(int numTimes, int delayTime) {
  for (int g = 0; g < numTimes; g++) {

    // Turn the LED on and wait.
    digitalWrite(LEDPin, HIGH);
    delay(delayTime);

    // Turn the LED off and wait.
    digitalWrite(LEDPin, LOW);
    delay(delayTime);
  }
}

int startDoor(int openClose) {
  digitalWrite(LIFT_POWER, HIGH);  //same for both
  Serial.println("Start Door");

  digitalWrite(R_PWM, LOW);
  digitalWrite(L_PWM, LOW);
  doorTime = millis();

  if (openClose) {
    Serial.println("Start Door open");
    doorMoving = 1;
    digitalWrite(R_PWM, HIGH);
  }

  if (!openClose) {
    Serial.println("Start door close");
    doorMoving = 2;
    digitalWrite(L_PWM, HIGH);
  }

  return 1;
}

int stopDoor() {
  Serial.println("StopDoor");
  digitalWrite(R_PWM, LOW);
  digitalWrite(L_PWM, LOW);
  digitalWrite(LIFT_POWER, LOW);  ///same for both
  doorMoving = 0;
  return 1;
}

int startLift(int openClose) {
  digitalWrite(LIFT_POWER, HIGH);
  Serial.println("Start Lift");
  digitalWrite(LIFT_UP, LOW);
  digitalWrite(LIFT_DOWN, LOW);
  liftTime = millis();
  if (openClose) {
    Serial.println("Start Lift open");
    liftMoving = 1;
    digitalWrite(LIFT_UP, HIGH);
  }
  if (!openClose) {
    Serial.println("Start lift close");
    liftMoving = 2;
    digitalWrite(LIFT_DOWN, HIGH);
  }
  return 1;
}

int stopLift() {
  Serial.println("StopLift");
  digitalWrite(LIFT_UP, LOW);
  digitalWrite(LIFT_DOWN, LOW);
  digitalWrite(LIFT_POWER, LOW);
  liftMoving = 0;
  return 3;
}

int checkLimit(int direction) {  //direction ==1 for open, 0 for close
                                 // Serial.println("CheckLimit");
  int myValue = 0;
  //return(1);
  //write power pin high
  if (direction) {
    digitalWrite(OPEN_LIMIT_POWER, HIGH);
    delay(400);
    myValue = digitalRead(OPEN_LIMIT_PIN);
    //Serial.println("pin " + String(OPEN_LIMIT_PIN) + " read  " + String(myValue));
    digitalWrite(OPEN_LIMIT_POWER, LOW);
  } else {
    digitalWrite(CLOSE_LIMIT_POWER, HIGH);
    delay(400);
    myValue = digitalRead(CLOSE_LIMIT_PIN);
    //Serial.println("pin " + String(CLOSE_LIMIT_PIN) + " read " + String(myValue));
    digitalWrite(CLOSE_LIMIT_POWER, LOW);
  }
  return myValue;
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

int getThingSpeakData() {
  //get the relevant data from ThingSpeak and process it
  //perhaps retun action for the door?
  connectWiFi();
  if (connectedBool) {
    ThingSpeak.readMultipleFields(readChannelId, readAPIKey);
    int doorCommand = ThingSpeak.getFieldAsInt(1);  //open close (-1,0, 1)
    //getRSSI
    int rssi = getStrength(3);
    int doorState = 0;
    if (checkLimit(1)) { doorState = 1; }
    if (checkLimit(0)) { doorState = -1; }
    float lightValue = getLight();
    ThingSpeak.setField(1, 0);
    ThingSpeak.setField(2, rssi);
    ThingSpeak.setField(3, doorState);
    ThingSpeak.setField(4, lightValue);
    ThingSpeak.writeFields(writeChannelId, writeAPIKey);
    if (doorCommand == 1) { startDoor(1); }
    if (doorCommand == -1) { startDoor(0); }
    if (doorCommand == 2) { startLift(1); }
    if (doorCommand == -2) { startLift(0); }
  }
  runStatus = 0;
  return 1;
}

float getLight() {
  digitalWrite(LIGHT_POWER, HIGH);
  Serial.println("light power on");
  delay(1000);
  uint16_t lightVal = analogRead(LIGHT_IN_PIN);
  Serial.println("light is " + String(lightVal));
   digitalWrite(LIGHT_POWER, LOW);
  return lightVal/10;
}

void getTimeLimits() {
  //blinkX(8,50);//take otu
  digitalWrite(TIME_POWER_PIN, HIGH);
  delay(80);
  //blinkX(10,400); //take out
  int myDoorTime = analogRead(DOOR_TIME_PIN);
  delay(100);

  int myLiftTime = analogRead(LIFT_TIME_PIN);
  digitalWrite(TIME_POWER_PIN, LOW);
  liftTimeLimit = LIFT_TIME_LIMIT * myLiftTime / 512;  //out of 1024
    //blinkX(liftTimeLimit/5000,50);
    //delay(1000);
  doorTimeLimit = DOOR_TIME_LIMIT * myDoorTime / 512;
  //blinkX(doorTimeLimit/5000,50);
  // delay(1000);
  Serial.println("lift Time = " + String(liftTimeLimit));
  Serial.println("door Time = " + String(doorTimeLimit));
}

void setup() {
  Serial.begin(115200);
  delay(100);  //Take some time to open up the Serial Monitor
  pinMode(LEDPin, OUTPUT);
  blinkX(8, 20);
  int freq = 5000;
  int resolution = 8;
  

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

  pinMode(OPEN_DOOR_BUTTON, INPUT_PULLDOWN);
  pinMode(CLOSE_DOOR_BUTTON, INPUT_PULLDOWN);
  pinMode(OPEN_LIFT_BUTTON, INPUT_PULLDOWN);
  pinMode(CLOSE_LIFT_BUTTON, INPUT_PULLDOWN);
  pinMode(OPEN_LIMIT_PIN, INPUT_PULLDOWN);
  pinMode(CLOSE_LIMIT_PIN, INPUT_PULLDOWN);

  pinMode(LIGHT_IN_PIN, INPUT_PULLUP);
  pinMode(LIGHT_POWER, OUTPUT);

  pinMode(LIFT_POWER, OUTPUT);
  pinMode(LIFT_UP, OUTPUT);
  pinMode(LIFT_DOWN, OUTPUT);

  pinMode(LIGHT_IN_PIN, INPUT);
  pinMode(LIFT_TIME_PIN, INPUT_PULLDOWN);
  pinMode(DOOR_TIME_PIN, INPUT_PULLDOWN);
  pinMode(TIME_POWER_PIN, OUTPUT);

  digitalWrite(LIFT_POWER, LOW);
 // digitalWrite(LIGHT_POWER, LOW);
  getLight();

  getTimeLimits();

  //Print the wakeup reason for ESP32
  print_wakeup_reason();
  Serial.println("waker reason " + String(wakeReason));
  delay(200);
  attachInterrupt(OPEN_DOOR_BUTTON, openDoorButtonInt, RISING);
  attachInterrupt(CLOSE_DOOR_BUTTON, closeDoorButtonInt, RISING);
  attachInterrupt(OPEN_LIFT_BUTTON, openLiftButtonInt, RISING);
  attachInterrupt(CLOSE_LIFT_BUTTON, closeLiftButtonInt, RISING);

  //  buttonPushed = 0;
  //Serial.println("Run Status " + String(runStatus));
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
    buttonPushed=0;
  }


  if (doorMoving > 0) {
    blinkX(2, 46);
    if (millis() - doorTime > doorTimeLimit) {
      Serial.println("door time expired ");
      stopDoor();
      runStatus = 0;
    }
  }

  if (liftMoving > 0) {
    blinkX(3, 15);
    if (millis() - liftTime > liftTimeLimit) {
      Serial.println("lift up time expired ");
      stopLift();  //lift opening check timer
      runStatus = 0;
    }
  }

  if (wakeReason == 2) {
    Serial.println("read write ThingSpeak");
    getThingSpeakData();  //check internet for commands
    wakeReason = 0;
  }

  //delay(3000);
  //start a timer if button pushed
  //Then when timer expired rea the button again.
  if ((buttonPushed) > 0) {
    if (!buttonTimerOn) {
      longButtonTime = millis();
      buttonTimerOn = true;
    }

    if ((millis() - longButtonTime) > LONG_BUTTON_TIME) {
      longButtonTime = millis();
      Serial.println("long Event");
      if ((buttonPushed == 1) && (digitalRead(OPEN_DOOR_BUTTON) == 1)) {
        blinkX(2, 25);
        Serial.println("buttonEvent " + String(buttonPushed));
        buttonPushed = 0;
        if (!doorMoving) {
          startDoor(1);
        } else {
          stopDoor();
        }
        wakeTime = millis();
      }
      if ((buttonPushed == 2) && (digitalRead(CLOSE_DOOR_BUTTON) == 1)) {
        delay(100);  //maybe this will stop cris cross interrupst?
        blinkX(3, 25);
        Serial.println("buttonEvent " + String(buttonPushed));
        if (!doorMoving) {
          startDoor(0);
        } else {
          stopDoor();
        }
        wakeTime = millis();
      }
      if ((buttonPushed == 3) && (digitalRead(OPEN_LIFT_BUTTON) == 1)) {
        delay(100);  //maybe this will stop cris cross interrupst?
        blinkX(4, 25);
        Serial.println("buttonEvent " + String(buttonPushed));
        if (!liftMoving) {
          startLift(1);
        } else {
          stopLift();
        }
        wakeTime = millis();
      }

      if ((buttonPushed == 4) && (digitalRead(CLOSE_LIFT_BUTTON) == 1)) {
        delay(100);  //maybe this will stop cris cross interrupst?
        blinkX(5, 25);
        Serial.println("buttonEvent " + String(buttonPushed));
        if (!liftMoving) {
          startLift(0);
        } else {
          stopLift();
        }
        wakeTime = millis();
      }
      buttonPushed = 0;  //can probs take thee out of the others.  This is in the loog where the time has expired
      buttonTimerOn = false;
      //Serial.println("No dice on long press");
      Serial.println(digitalRead(OPEN_DOOR_BUTTON));
      Serial.println(digitalRead(CLOSE_DOOR_BUTTON));
      Serial.println(digitalRead(OPEN_LIFT_BUTTON));
      Serial.println(digitalRead(CLOSE_LIFT_BUTTON));
    }
  }
  delay(5);
  if (millis() - wakeTime > AWAKE_TIME) {
    if (!(doorMoving || liftMoving || buttonTimerOn)) {
      //delay(2250);
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