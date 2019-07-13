#pragma once

/*********************************************************************
  Ninho Bicho Controller
*********************************************************************/

/////////////////////Board specs
#include <Arduino.h>
#include <Chrono.h>

#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/sleep.h>

#include "Debug.h"
#include "ProjectSettings.h"

#define STRINGIZE2(s) #s
#define TOSTRING(s) STRINGIZE2(s)


////////////////////////////////////////////////////////////////////////////////////////////////////
//General definitions
//
typedef enum{TRAY_IDLE,TRAY_DETECTED,TRAY_OPEN,TRAY_CLOSE}trayPulseState;
typedef enum{SENSOR_IDLE,SENSOR_OBJECT_NEAR,SENSOR_OBJECT_MEDIUM,SENSOR_OBJECT_FAR}sensorState;

enum{SLOW,MEDIUM,FAST,STOPPED};


////////////////////////////////////////////////////////////////////////////////////////////////////
//Motor
//
#define ENA_PIN 5
#define IN1_PIN 7
#define IN2_PIN 6

uint8_t trayDirection = 0;
uint16_t motorPower = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////
//Open/Closed Sensors
//
#define CLOSED_TRAY_SWITCH_PIN 10
#define OPENED_TRAY_SWITCH_PIN 9

////////////////////////////////////////////////////////////////////////////////////////////////////
//Distance Sensors
//
#define SENSOR_ANALOG_PIN A0

////////////////////////////////////////////////////////////////////////////////////////////////////
//LED + Button
//
#define LED_PIN 11
#define BUTTON_PIN 12
