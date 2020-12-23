#ifndef BBMICROBIT_H
#define BBMICROBIT_H

// Function that decodes the display command
void BBMicroBitInit();
void decodeAndSetDisplay(uint8_t displayCommands[], uint8_t commandLength);
// This function sets the edge connector pins or internal buzzer
void decodeAndSetPins(uint8_t displayCommands[]);

#endif