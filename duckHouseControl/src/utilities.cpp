#include "../include/duckHouseConstants.h"
#include <Arduino.h>
#include "../include/globals.h"

int drivePin;

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
    return 1;
}


  int stopLift() {
  Serial.println("StopLift");

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