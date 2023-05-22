#ifndef HATCHLING_H
#define HATCHLING_H


#define HATCHLING_SPI_LENGTH 21
#define HATCHLING_SETALL_LENGTH 19
#define HATCHLING_NEOPXL_SET_LENGTH 14
#define TOTAL_POSSIBLE_ACCESSORIES 26
#define GP_PORT_TOTAL 6
#define HATCHLING_ONBOARD_LED_CMD_LENGTH  GP_PORT_TOTAL*3+1
#define MAX_BRIGHTNESS 0x40 // Defines the maximum brightness of onboard LEDs
#define MIN_BRIGHTNESS 0x0A // Defines a dim LED

//GP Port States
#define ROTATION_SERVO					 1             //Rotation servo signals
#define POSITION_SERVO					 6             //Position servo signals
#define NEOPXL_SINGLE                    2             //Single Neopixel
#define NEOPXL_STRIP                     3             //Strip of Neopixel
#define ANALOG_SENSOR                    4             //Analog input
#define DIGITAL_OUT                      5             //Software PWM Enabled Digital Output
#define PORTOFF                          0             //Configured as digital input, start up state

static const uint8_t id_values[TOTAL_POSSIBLE_ACCESSORIES] = {255, 10, 21, 32, 44, 56, 66, 77, 87, 97, 107, 118, 128, 138, 147, 156, 165, 174, 183, 192, 201, 210, 219, 228, 236, 245};


// Arrays for some pre-set colors (in order: red, orange, yellow, green, teal, blue, purple, white) - used for making the LED color code
static const uint8_t ledcolors[8][3] = {{MAX_BRIGHTNESS, 0, 0}, {MAX_BRIGHTNESS, MAX_BRIGHTNESS/3, 0}, {MAX_BRIGHTNESS, MAX_BRIGHTNESS, 0}, {0, MAX_BRIGHTNESS, 0},
                                        {0, MAX_BRIGHTNESS, MAX_BRIGHTNESS}, {0, 0, MAX_BRIGHTNESS}, {MAX_BRIGHTNESS, 0, MAX_BRIGHTNESS}, {MAX_BRIGHTNESS, MAX_BRIGHTNESS, MAX_BRIGHTNESS}};

// If the port is on, set to Blue/Yellow/Teal/Magenta/White/Orange in order from A-F
static const uint8_t PortOnLEDColors[6][3] = {{0, 0, MAX_BRIGHTNESS*3/2}, {MAX_BRIGHTNESS*2/3, MAX_BRIGHTNESS*2/3, 0}, {0, MAX_BRIGHTNESS, MAX_BRIGHTNESS/4}, {MAX_BRIGHTNESS, 0, MAX_BRIGHTNESS},
                                        {MAX_BRIGHTNESS/2, MAX_BRIGHTNESS/2, MAX_BRIGHTNESS/2}, {MAX_BRIGHTNESS, MAX_BRIGHTNESS/4, 0}};

// If the port is off, set to dim versions of Blue/Yellow/Teal/Magenta/White/Orange in order from A-F
static const uint8_t PortOffLEDColors[6][3] = {{0, 0, MIN_BRIGHTNESS}, {MIN_BRIGHTNESS, MIN_BRIGHTNESS, 0}, {0, MIN_BRIGHTNESS, MIN_BRIGHTNESS/2}, {MIN_BRIGHTNESS, 0, MIN_BRIGHTNESS},
                                        {MIN_BRIGHTNESS, MIN_BRIGHTNESS, MIN_BRIGHTNESS}, {MIN_BRIGHTNESS, MIN_BRIGHTNESS/4, 0}};


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