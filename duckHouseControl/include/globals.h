#pragma once

extern int drivePin;
extern int runStatus;
extern int buttonPushed;  //1 open door, 2 close door, 3 open lift, 4 close lift
extern int doorMoving ;
extern int liftMoving ;
extern long doorTime ;
extern long liftTime ;
extern int liftTimeLimit;
extern int doorTimeLimit;

  extern const char* ssid;
  extern const char* password;
  extern const char* writeAPIKey;
  extern const char* readAPIKey;
  extern const long readChannelId;
  extern const long writeChannelId;

  extern   const long TimeToSleepSeconds;