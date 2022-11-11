#ifndef HATCHLING_H
#define HATCHLING_H


#define HATCHLING_SPI_LENGTH 19
#define HATCHLING_SETALL_LENGTH 20

// Initializes the Hatchling, mostly setting the edge connector pins as we want
void initHatchling();

// Sends the stop command to the Hatchling
void stopHatchling();

// Sets all onboard Hatchling LEDs
void setOnboardHatchlingLEDs(uint8_t commands[], uint8_t length);

// Sets all Hatchling GP Ports. Returns the number of bytes that were used to set the outputs
void setAllHatchlingPorts(uint8_t commands[], uint8_t length);

// Sets Hatchling GP Port states (analog, servo, digital, etc) - this is not used in bluetooth, only in SPI
void setHatchlingPortStates(uint8_t commands[], uint8_t length);

// Sets Hatchling Neopixel LED Strip on one port
void setHatchlingExternalNeopixelStrip(uint8_t commands[], uint8_t length);

// Turns off the Hatchling - in case we haven't received anything over BLE for 10 minutes
void turnOffHatchling();

// arranges the encoder data and other data from SPI to prepare it to send over BLE
void arrangeHatchlingSensors(uint8_t (&spi_sensors_only)[HATCHLING_SPI_SENSOR_LENGTH], uint8_t (&sensor_vals)[HATCHLING_SENSOR_SEND_LENGTH]);


#endif