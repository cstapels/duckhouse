#include <stdint.h>

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

// WiFi configuration

// #define ssid  "still_waters"
// #define password  "33turkeys511"
// #define writeAPIKey  "DI6OLF96SEXYI8M6"
// #define readAPIKey  "HP7M4ROAO0DFZ6YZ"
// #define readChannelId  1415485L
// #define writeChannelId  1415485L


  // Sleep and timing defaults
  const uint64_t ButtonPinBitmask = 0xF00000000ULL;

  #define MicrosecondsToSecondsFactor  1000000ULL


 // const uint32_t AwakeTimeMs = 8000;
  #define LiftTimeLimitMs  30 * 1000;
  #define DoorTimeLimitMs  30 * 1000;

  // Board pins
  #define LedPin = 2
  #define DoorTimePin = 39
  #define LiftTimePin = 27
  #define TimePowerPin = 26
  #define OpenLimitPin = 15;
  #define CloseLimitPin = 21
  #define LiftPowerPin = 13
  #define LiftUpPin = 14
  #define LiftDownPin = 12
  #define OPEN_DOOR_BUTTON 33
  #define CLOSE_DOOR_BUTTON 32
  #define OPEN_LIFT_BUTTON 35
  #define CLOSE_LIFT_BUTTON 34
  #define LIGHT_IN_PIN 36
  #define LIFT_UP 14
  #define LIFT_DOWN 12
  #define LIFT_POWER 13