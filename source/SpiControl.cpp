#include "MicroBit.h"
#include "BirdBrain.h"
#include "SpiControl.h"

SPI spi(MOSI, MISO, SCK);
uint8_t whatAmI = 0;

// Initializing SPI, putting the SS pin high
void spiInit()
{
    uBit.io.P16.setDigitalValue(1);
    spi.format(8,0);
    spi.frequency(1000000);
    uBit.sleep(10);
}

void spiWrite(uint8_t* writeBuffer, uint8_t length)
{
    uBit.io.P16.setDigitalValue(0);
    NRFX_DELAY_US(SS_WAIT);
    for(int i = 0; i < length-1; i++)
    {
        spi.write(writeBuffer[i]);
        NRFX_DELAY_US(WAIT_BETWEEN_BYTES);
    }
    spi.write(writeBuffer[length-1]);
    NRFX_DELAY_US(SS_WAIT);
    uBit.io.P16.setDigitalValue(1);
}

uint8_t* spiReadHB()
{
    static uint8_t readBuffer[HB_SENSOR_LENGTH];
    uBit.io.P16.setDigitalValue(0);
    NRFX_DELAY_US(SS_WAIT);
    for(int i = 0; i < HB_SENSOR_LENGTH-1; i++)
    {
        readBuffer[i] = spi.write(0xCC);
        NRFX_DELAY_US(WAIT_BETWEEN_BYTES);
    }
    readBuffer[HB_SENSOR_LENGTH-1] = spi.write(0xCC);
    NRFX_DELAY_US(WAIT_BETWEEN_BYTES);
    readBuffer[0] = spi.write(0xCC);
    NRFX_DELAY_US(WAIT_BETWEEN_BYTES);
    readBuffer[1] = spi.write(0xCC);
    NRFX_DELAY_US(SS_WAIT);
    uBit.io.P16.setDigitalValue(1);

    return readBuffer;
}

uint8_t* spiReadFinch()
{
    static uint8_t readBuffer[FINCH_SENSOR_LENGTH];
    uBit.io.P16.setDigitalValue(0);
    NRFX_DELAY_US(SS_WAIT);
    for(int i = 0; i < FINCH_SENSOR_LENGTH-1; i++)
    {
        readBuffer[i] = spi.write(0xCC);
        NRFX_DELAY_US(WAIT_BETWEEN_BYTES);
    }
    readBuffer[FINCH_SENSOR_LENGTH-1] = spi.write(0xCC);
    NRFX_DELAY_US(SS_WAIT);
    uBit.io.P16.setDigitalValue(1);

    return readBuffer;
}

ManagedString whichDevice()
{
    ManagedString devicePrefix = "";

    // Reading the firmware version, doing this twice as the first time it seems like values are junk    
    readFirmwareVersion();
    uint8_t device = readFirmwareVersion();
   
	switch(device)
	{
		case MICROBIT_SAMD_ID:
			devicePrefix="MB";
            whatAmI = A_MB;
			break;
		case FINCH_SAMD_ID:
			devicePrefix="FN";
            whatAmI = A_FINCH;
			break;
		case HUMMINGBIT_SAMD_ID:
			devicePrefix="BB";
            whatAmI = A_HB;
			break;
        // If it was none of the above, including not 0, try one more time to get a value
		default:
            device = readFirmwareVersion();
            switch(device)
            {
                case MICROBIT_SAMD_ID:
                    devicePrefix="MB";
			        devicePrefix="MB";
                    break;
                case FINCH_SAMD_ID:
                    devicePrefix="FN";
                    whatAmI = A_FINCH;
                    break;
                case HUMMINGBIT_SAMD_ID:
                    devicePrefix="BB";
                    whatAmI = A_HB;
                    break;   
                // If the value is still junk, call it a standalone micro:bit 
                default:            
                    devicePrefix="MB";
			        devicePrefix="MB";
        			break;
            }
            break;
	}
    return devicePrefix;
}

uint8_t readFirmwareVersion()
{
    uBit.io.P16.setDigitalValue(0);
    NRFX_DELAY_US(SS_WAIT);
    uint8_t readBuffer[4];
    readBuffer[0] = spi.write(0x8C); // Special command to read firmware/hardware version
    NRFX_DELAY_US(WAIT_BETWEEN_BYTES);
    for(int i = 1; i < 3; i++)
    {
        readBuffer[i] = spi.write(0xFF);
        NRFX_DELAY_US(WAIT_BETWEEN_BYTES);
    }
    readBuffer[3] = spi.write(0xFF);
    NRFX_DELAY_US(SS_WAIT);
    uBit.io.P16.setDigitalValue(1);
    NRFX_DELAY_MS(1); // wait after reading firmware

    if(readBuffer[0] == FINCH_SAMD_ID)
        return FINCH_SAMD_ID;
    else if(readBuffer[3] == HUMMINGBIT_SAMD_ID)
        return HUMMINGBIT_SAMD_ID;
    else if((readBuffer[0] + readBuffer[1] + readBuffer[2] + readBuffer[3]) == 0) // Bit hokey, but if all bytes are 0, it's a micro:bit since SPI isn't responding
        return MICROBIT_SAMD_ID;
    else   
        return UNIDENTIFIED_DEV; // can be any number that isn't the FINCH and HUMMINGBIT IDs 
}