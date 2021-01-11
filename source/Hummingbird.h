#ifndef HUMMINGBIRD_H
#define HUMMINGBIRD_H

#define HB_SETALL_LENGTH 19

// Initializes the Hummingbird, mostly setting the edge connector pins as we want
void initHB();

// Sends the stop command to the Hummingbird
void stopHB();

// Sets all Hummingbird outputs in one go
void setAllHB(uint8_t commands[], uint8_t length);

#endif