#ifndef BIRDBRAIN_H
#define BIRDBRAIN_H

#include "MicroBit.h"
#include "MicroBitUARTService.h"

extern MicroBit uBit;
extern MicroBitUARTService *bleuart;
extern char initials_name[3]; // Holds our fancy name initials
extern uint8_t whatAmI; // Holds whether the device is currently in standalone micro:bit, Finch, or Hummingbird mode 

#include "SpiControl.h"
#include "Naming.h"

#define RESET_PIN      1 // Pin to reset/turn off Finch - micro:bit pin 1
#define A_MB           0
#define A_HB           1
#define A_FINCH        2


#endif

