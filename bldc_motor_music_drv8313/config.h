// BLDC Motor Music - motor firmware for SimpleFOCMini (DRV8313)
// https://github.com/nstb-robotics/bldc-motor-music  (MIT License)
//
// config.h: the only file you normally edit.
// Set MOTOR_NUMBER before flashing each board. It sets the USB name
// ("Motor 1" / "Motor 2") and the default stereo role.
// One motor only: keep 1 and type "role 0; save" once.

#pragma once

#define MOTOR_NUMBER 1

#ifdef __cplusplus

// Motor (Segway Drift W1 hub motor)
const int   POLE_PAIRS = 14;
const float R_EFF      = 0.34f;   // effective phase resistance [ohm], used by the current model

// Teensy 4.0 pins
const int PIN_A = 4, PIN_B = 5, PIN_C = 6;   // PWM, same FlexPWM module
const int PIN_EN = 8, PIN_FAULT = 9;        // PIN_FAULT optional: SimpleFOCMini nFT (not wired in the diagram)
const int PIN_HA = 14, PIN_HB = 15, PIN_HC = 16;
const int PIN_LED = 13;

// Driver limit (DRV8313: 2.5 A peak)
const float IMAX_LIMIT = 2.5f;

#endif
