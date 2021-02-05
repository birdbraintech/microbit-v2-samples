#ifndef BBMICROBIT_H
#define BBMICROBIT_H

#include "BLESerial.h"

// Function that decodes the display command
void BBMicroBitInit();
void decodeAndSetDisplay(uint8_t displayCommands[], uint8_t commandLength);
// This function sets the edge connector pins or internal buzzer
void decodeAndSetPins(uint8_t displayCommands[]);

// Gets the edge connector pins as analog inputs, if they have been set as an analog inputs
void getEdgeConnectorVals(uint8_t (&sensor_vals)[V2_SENSOR_SEND_LENGTH]);

// Get and convert the accelerometer values to 8 bit format, and check if the shake bit should be set
void getAccelerometerVals(uint8_t (&sensor_vals)[V2_SENSOR_SEND_LENGTH]);

// Get and convert the magnetometer values to a 16 bit format
void getMagnetometerVals(uint8_t (&sensor_vals)[V2_SENSOR_SEND_LENGTH]);

// Get the state of the buttons
void getButtonVals(uint8_t (&sensor_vals)[V2_SENSOR_SEND_LENGTH], bool V2Notification);

// Get and convert the accelerometer values to 8 bit format, and check if the shake bit should be set
void getAccelerometerValsFinch(uint8_t (&sensor_vals)[FINCH_SENSOR_SEND_LENGTH]);

// Get and convert the magnetometer values to a 16 bit format
void getMagnetometerValsFinch(uint8_t (&sensor_vals)[FINCH_SENSOR_SEND_LENGTH]);

// Get the state of the buttons
void getButtonValsFinch(uint8_t (&sensor_vals)[FINCH_SENSOR_SEND_LENGTH], bool V2Notification);


// Turns off edge connector, buzzer, LED screen
void stopMB();

// Function used for setting the buzzer for Hummingbird and Finch
void setBuzzer(uint16_t period, uint16_t duration);

#endif