#ifndef HATCHLING_H
#define HATCHLING_H


#define HATCHLING_SPI_LENGTH 19
#define HATCHLING_SETALL_LENGTH 19
#define HATCHLING_NEOPXL_SET_LENGTH 14
#define TOTAL_POSSIBLE_ACCESSORIES 26
#define GP_PORT_TOTAL 6
#define HATCHLING_ONBOARD_LED_CMD_LENGTH  GP_PORT_TOTAL*3+1
#define MAX_BRIGHTNESS 0x40 // Defines the maximum brightness of onboard LEDs
#define MIN_BRIGHTNESS 0x0A // Defines a dim LED

static const uint8_t id_values[TOTAL_POSSIBLE_ACCESSORIES] = {0, 254, 243, 234, 224, 214, 204, 195, 183, 171, 160, 149, 139, 127, 115, 105, 95, 86, 77, 68, 60, 51, 42, 34, 26, 13};


// Arrays for some pre-set colors (in order: red, orange, yellow, green, teal, blue, purple, white) - used for making the LED color code
static const uint8_t ledcolors[8][3] = {{MAX_BRIGHTNESS, 0, 0}, {MAX_BRIGHTNESS, MAX_BRIGHTNESS/3, 0}, {MAX_BRIGHTNESS, MAX_BRIGHTNESS, 0}, {0, MAX_BRIGHTNESS, 0},
                                        {0, MAX_BRIGHTNESS, MAX_BRIGHTNESS}, {0, 0, MAX_BRIGHTNESS}, {MAX_BRIGHTNESS, 0, MAX_BRIGHTNESS}, {MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS}};

// If the port is on, set to Blue/Yellow/Teal/Magenta/White/Orange in order from A-F
static const uint8_t PortOnLEDColors[6][3] = {{0, 0, MAX_BRIGHTNESS}, {MAX_BRIGHTNESS, MAX_BRIGHTNESS, 0}, {0, MAX_BRIGHTNESS, MAX_BRIGHTNESS/2}, {MAX_BRIGHTNESS, 0, MAX_BRIGHTNESS},
                                        {MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS}, {MAX_BRIGHTNESS, MAX_BRIGHTNESS/2, 0}};

// If the port is off, set to dim versions of Blue/Yellow/Teal/Magenta/White/Orange in order from A-F
static const uint8_t PortOffLEDColors[6][3] = {{0, 0, MIN_BRIGHTNESS}, {MIN_BRIGHTNESS, MIN_BRIGHTNESS, 0}, {0, MIN_BRIGHTNESS, MIN_BRIGHTNESS/2}, {MIN_BRIGHTNESS, 0, MIN_BRIGHTNESS},
                                        {MIN_BRIGHTNESS, MIN_BRIGHTNESS, MIN_BRIGHTNESS}, {MIN_BRIGHTNESS, MIN_BRIGHTNESS, 0}};


// Initializes the Hatchling, mostly setting the edge connector pins as we want
void initHatchling();

// Sends the stop command to the Hatchling
void stopHatchling();

// Sets all onboard Hatchling LEDs
void setOnboardHatchlingLEDs(uint8_t commands[], uint8_t length);

// Sets hatchling LEDs to initial connection level
void displayConnectedLEDs();

// Sets hatchling LEDs to a color code based on mac address
void showLEDCode();

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