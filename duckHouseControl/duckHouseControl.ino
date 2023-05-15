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
#define AWAKE_TIME 8000
#define OPEN_LIMIT_PIN 15
#define OPEN_LIMIT_POWER 0
#define CLOSE_LIMIT_PIN 21
#define CLOSE_LIMIT_POWER 04  //was 04
#define LIFT_TIME_LIMIT 30 * 1000
#define DOOR_TIME_LIMIT 30 * 1000
#define LEDPin 2
#define DOOR_TIME_PIN 39
#define LIFT_TIME_PIN 27
#define TIME_POWER_PIN 26


#define R_EN 17
#define L_EN 5
#define R_PWM 16
#define L_PWM 19

//#define UP_PIN 14
//#define NOT_DOWN_PIN 12
//#define LIFT_POWER 13

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
long wakeTime=millis();

int ledChannel = 0;
int ledChannelL = 1;
int liftLedChannel = 2;
int liftLedChannelL = 3;
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
unsigned int debounceTime = 200;
bool connectedBool = false;
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

int startDoor(int openClose) {  //open one close 0
ledcWrite(0, 0); //make sure they are off for an interrupt change
ledcWrite(1, 0);

  doorTime = millis();
  if (openClose) {
    Serial.println("Start Door open");
    doorMoving = 1;
  }
  //return status

  int motorSpeed = 255;
  drivePin = openClose;

  ledcWrite(drivePin, motorSpeed);
  if (!openClose) {
    Serial.println("Start Door close");
    doorMoving = 2;
  }

  return 1;
}

int stopDoor() {
  int motorSpeed = 0;
  int driveChannel = 0;
  ledcWrite(driveChannel, motorSpeed);
  driveChannel = 1;
  ledcWrite(driveChannel, motorSpeed);

  doorMoving = 0;
  //return status
  return 1;
}

int startLift(int openClose) {
  digitalWrite(LIFT_POWER, HIGH);
  //all three high is open,
  //just 2 high is close
  Serial.println("Start Lift");

digitalWrite(LIFT_UP,LOW);
digitalWrite(LIFT_DOWN,LOW);
liftTime = millis();

  if (openClose) {
    Serial.println("Start Lift open");
    liftMoving = 1;
    digitalWrite(LIFT_UP,HIGH);
  }
  
  if (!openClose) {
    Serial.println("Start lift close");
    liftMoving = 2;
    digitalWrite(LIFT_DOWN,HIGH);
  }

 

//       //blinkX(10,100);
//   digitalWrite(LIFT_POWER, LOW);
  
//   delay(1000);
//      digitalWrite(UP_PIN, LOW);
// delay(1000);
//   digitalWrite(NOT_DOWN_PIN, LOW);

  //  delay(1000);
 //cases door int for power, not down,up
 //causes mor int for not down, power, up
 //worst is  nor down, up , power
  //if (openClose == 1) {
//  digitalWrite(NOT_DOWN_PIN, HIGH);
//  delay(100);
//     digitalWrite(UP_PIN, HIGH);
//     delay(100);
//        Serial.println("up");
//   }
//   if (openClose == 0) {
//     Serial.println("down");
//    delay(10);
//     digitalWrite(NOT_DOWN_PIN, LOW);
//     delay(100);
//     digitalWrite(UP_PIN, LOW);

//     //bad thigns happen with order=up then not down also changed above
//   }

//   delay(100);
//   Serial.println("Power on");
//   digitalWrite(LIFT_POWER, HIGH);

  

  //digitalWrite(controlPin, HIGH);
  //return status
  //turn on lift power
  //turn on open solenoid

  return 1;
}

int stopLift() {
  Serial.println("StopLift");
  digitalWrite(LIFT_UP,LOW);
digitalWrite(LIFT_UP,LOW);
digitalWrite(LIFT_DOWN,LOW);
  digitalWrite(LIFT_POWER, LOW);
  // digitalWrite(LIFT_POWER, LOW);
  // digitalWrite(UP_PIN, LOW);
  // digitalWrite(NOT_DOWN_PIN, LOW);
  //return status
  //turn on close solenoid
  //turn off power solenoid
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
  }
  runStatus = 0;
  return 1;
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

  // ledcSetup(liftLedChannel, freq, resolution);
  // ledcAttachPin(LIFT_UP, liftLedChannel);
  // ledcSetup(liftLedChannelL, freq, resolution);
  // ledcAttachPin(LIFT_DOWN, liftLedChannelL);

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
    // if (doorMoving == 1) {
    //   // Serial.println("fooglabiddle  ");
    //   int readLimit = checkLimit(0);
    //   Serial.println("readlimit " + String(readLimit));
    //   if (readLimit > 0) {
    //     Serial.println("open limit reached ");
    //     stopDoor();  //door opening check limit and timer
    //     runStatus = 0;
    //   }
    // }
    // if (doorMoving == 2) {
    //   int readLimit = checkLimit(1);
    //   //Serial.println("readlimit " + String(readLimit));
    //   if (readLimit > 0) {
    //     Serial.println("close limit reached ");
    //     stopDoor();
    //     runStatus = 0;
    //   }
    // }
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

  // loop until status ==0;
  // use && to combine status
  //consider write door status to thingspeak probably only on timer wake
  //consider check button status or at least interrupt here
  // if (buttonPushed > 0) {  //cuz the interrupt is stupid
  //   if (wakeReason == 1) {
  //     buttonPushed = 0;
  //     wakeReason = 0;
  //   }
  // }

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