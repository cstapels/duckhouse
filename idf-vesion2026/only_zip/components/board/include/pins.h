#pragma once
/* GPIO map from Hardware Specification Rev 1.8 (section 20) and
 * Software Specification section 3. */

/* Outputs */
#define PIN_LIMIT_PWR       2   /* HIGH only while limits are read; strapping pin */
#define PIN_STATUS_LED      4   /* active HIGH */
#define PIN_DOOR_EN         5   /* L_EN + R_EN tied; strapping pin */
#define PIN_LIFT_LPWM       12  /* lift down / close; strapping pin */
#define PIN_LIFT_EN         13
#define PIN_LIFT_RPWM       14  /* lift up / open */
#define PIN_DOOR_LPWM       16  /* door close */
#define PIN_MOTOR_LOGIC_PWR 17  /* HIGH = BTS7960 5 V logic supply on */
#define PIN_DOOR_RPWM       19  /* door open */
#define PIN_PUMP_CTRL       23  /* active HIGH */

/* Inputs */
#define PIN_LIM_DOOR_OPEN   15  /* HIGH = switch closed; strapping pin */
#define PIN_LIM_DOOR_CLOSE  21
#define PIN_LIM_LIFT_CLOSE  25
#define PIN_BTN_DOOR_CLOSE  18  /* active HIGH, external pulldown */
#define PIN_BTN_DOOR_OPEN   22
#define PIN_BTN_LIFT_CLOSE  34  /* input-only */
#define PIN_BTN_LIFT_OPEN   35  /* input-only */
#define PIN_ONEWIRE         26
#define PIN_TANK_TOUCH      27  /* touch pad 7 (T7) */

/* Analog, ADC1 only */
#define PIN_ADC_VBAT        32
#define PIN_ADC_POT_LIFT    33
#define PIN_ADC_PHOTO       36
#define PIN_ADC_POT_DOOR    39
