#include "MicroBit.h"
#include "BirdBrain.h"
#include "SPIControl.h"
#include "BBMicroBit.h"
#include "Hatchling.h"
#include "BLESerial.h"


uint8_t prevHatchlingSetAllLEDs[HATCHLING_SETALL_LENGTH];

// Initializes the Hatchling, mostly setting the edge connector pins as we want
void initHatchling()
{
    // Setting these pins to analog inputs so we can read the ID values
    //MicroBitPin P0(MICROBIT_ID_IO_P0, MICROBIT_PIN_P0, PIN_CAPABILITY_BOTH); 
    //MicroBitPin P1(MICROBIT_ID_IO_P1, MICROBIT_PIN_P1, PIN_CAPABILITY_BOTH); 
    //MicroBitPin P2(MICROBIT_ID_IO_P2, MICROBIT_PIN_P2, PIN_CAPABILITY_BOTH); 
    stopHatchling();
    // Init the previous Hatchling LED command array
    memset(prevHatchlingSetAllLEDs, 0, HATCHLING_SETALL_LENGTH);
}

// Sends the stop command to the Hatchling
void stopHatchling()
{
    uint8_t stopCommand[HATCHLING_SPI_LENGTH];
    
    memset(stopCommand, 0xFF, HATCHLING_SPI_LENGTH);
    stopCommand[0] = FINCH_HATCHLING_STOPALL;
    spiWrite(stopCommand, HATCHLING_SPI_LENGTH);
    // Init the previous Hatchling LED command array to all 0s
    memset(prevHatchlingSetAllLEDs, 0, HATCHLING_SETALL_LENGTH);
}

// Sets all Hatchling LEDs
void setOnboardHatchlingLEDs(uint8_t commands[], uint8_t length)
{
    // Double check that the command contains enough data for us to proceed and that it isn't
    // an identical command from one sent previously
    if(length >= HATCHLING_SETALL_LENGTH)
    {    
         // setting the Hatchling LEDs
        spiWrite(commands, HATCHLING_SPI_LENGTH);
    }
}

// Sets all Hatchling ports if they are configured as outputs.
void setAllHatchlingPorts(uint8_t commands[], uint8_t length)
{
    // Making sure we have enough data and that the data is fresh
    if(length >= HATCHLING_SETALL_LENGTH)
    {
        spiWrite(commands, HATCHLING_SPI_LENGTH);
    }
}

// Sets Hatchling GP Port states (analog, servo, digital, etc) - this is not used in bluetooth, only in SPI
void setHatchlingPortStates(uint8_t commands[], uint8_t length)
{
    // Making sure we have enough data and that the data is fresh
    if(length >= HATCHLING_SETALL_LENGTH)
    {
        spiWrite(commands, HATCHLING_SPI_LENGTH);
    }
}

// Sets Hatchling Neopixel LED Strip on one port
void setHatchlingExternalNeopixelStrip(uint8_t commands[], uint8_t length)
{
    // Making sure we have enough data and that the data is fresh
    if(length >= HATCHLING_SETALL_LENGTH)
    {
        spiWrite(commands, HATCHLING_SPI_LENGTH);
    }
}

// Turns off the Finch - in case we haven't received anything over BLE for 10 minutes
void turnOffHatchling()
{
    uint8_t turnOffCommand[HATCHLING_SPI_LENGTH];
    
    memset(turnOffCommand, 0xFF, HATCHLING_SPI_LENGTH);
    turnOffCommand[0] = FINCH_HATCHLING_POWEROFF_SAMD;

    spiWrite(turnOffCommand, HATCHLING_SPI_LENGTH);       
}

/************************************************************************/
//Function which updates the Hatchling sensor values
/************************************************************************/
void arrangeHatchlingSensors(uint8_t (&spi_sensors_only)[HATCHLING_SPI_SENSOR_LENGTH], uint8_t (&sensor_vals)[HATCHLING_SENSOR_SEND_LENGTH])
{
	uint8_t i = 0;
	
	for(i=14;i<HATCHLING_SENSOR_SEND_LENGTH;i++)
	{
		sensor_vals[i] = spi_sensors_only[(i-14)*2+3];
	}

    // temporary hack displaying the raw analog values of the IDs for ports A-D. Eventually all six ID vals will need to fit into 4 bytes
    for(i=10;i<13;i++)
    {
        sensor_vals[i] = spi_sensors_only[i+4];
    }
    sensor_vals[13] = uBit.io.P0.getAnalogValue()>>2; // shift the 10 bit value to 8 bits

    // get the battery value for further processing
    sensor_vals[2] = spi_sensors_only[17];

}

