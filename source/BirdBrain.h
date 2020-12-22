#ifndef BIRDBRAIN_H
#define BIRDBRAIN_H

#include "MicroBit.h"
#include "MicroBitUARTService.h"

extern MicroBit uBit;
extern MicroBitUARTService *bleuart;
extern char initials_name[3]; // Holds our fancy name initials
extern uint8_t whatAmI; // Holds whether the device is currently in standalone micro:bit, Finch, or Hummingbird mode 
extern bool bleConnected; // Holds if connected over BLE
extern bool notifyOn; // Holds if notifications are on (we are regularly sending sensor packets back)

#include "SpiControl.h"
#include "Naming.h"
#include "BLESerial.h"

#define RESET_PIN      1 // Pin to reset/turn off Finch - micro:bit pin 1

// Codes for which type of device we are - a micro:bit, Finch, or Hummingbird
#define A_MB           0
#define A_HB           1
#define A_FINCH        2

// disabling the event service as we don't need it
//#ifndef MICROBIT_BLE_EVENT_SERVICE
//#define MICROBIT_BLE_EVENT_SERVICE              0
//#endif

#endif

