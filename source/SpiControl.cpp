#include "MicroBit.h"
#include "BirdBrain.h"
#include "SpiControl.h"
#include "Hummingbird.h"
#include "Finch.h"
#include "BLESerial.h"

SPI spi(MOSI, MISO, SCK);
uint8_t whatAmI = 0;
bool spiActive; // ensures we do not accidentally interleave SPI commands called from different fibers

// Initializing SPI, putting the SS pin high
void spiInit()
{
    uBit.io.P16.setDigitalValue(1);
    spi.format(8,0);
    spi.frequency(1000000);
    fiber_sleep(10);
    spiActive = false;
}

void spiWrite(uint8_t* writeBuffer, uint8_t length)
{
    // Wait up to 5 ms for another SPI command to complete
    uint8_t timeOut = 0;
    while(spiActive && timeOut < 5) {
        fiber_sleep(1);
        timeOut++;
    }
    if(!spiActive)
    {
        spiActive = true;

        uBit.io.P16.setDigitalValue(0);
        //NRFX_DELAY_US(SS_WAIT);
        for(int i = 0; i < length-1; i++)
        {
            spi.write(writeBuffer[i]);
        //    NRFX_DELAY_US(WAIT_BETWEEN_BYTES);
        }
        spi.write(writeBuffer[length-1]);
        //NRFX_DELAY_US(SS_WAIT);
        uBit.io.P16.setDigitalValue(1);
        NRFX_DELAY_US(50); // Ensures we don't hammer the Finch or Hummingbird with SPI packets
        spiActive = false;
    }
}

void spiReadHB(uint8_t (&readBuffer)[V2_SENSOR_SEND_LENGTH])
{
    // Wait up to 5 ms for another SPI command to complete
    uint8_t timeOut = 0;
    while(spiActive && timeOut < 5) {
        fiber_sleep(1);
        timeOut++;
    }
    if(!spiActive)
    {
        spiActive = true;
    
        uBit.io.P16.setDigitalValue(0);

        // send four nonsense bytes
        readBuffer[0] = spi.write(0xAA);
        readBuffer[1] = spi.write(0xBB);
        readBuffer[2] = spi.write(0xCC);
        readBuffer[3] = spi.write(0xDD);

        uBit.io.P16.setDigitalValue(1);
        spiActive = false;
    }
}

void spiReadFinch(uint8_t (&readBuffer)[FINCH_SPI_SENSOR_LENGTH])
{
    // Wait up to 5 ms for another SPI command to complete
    uint8_t timeOut = 0;
    while(spiActive && timeOut < 5) {
        fiber_sleep(1);
        timeOut++;
    }
    if(!spiActive)
    {
        spiActive = true;

        uBit.io.P16.setDigitalValue(0);
        //NRFX_DELAY_US(SS_WAIT);
        readBuffer[0] = spi.write(0xDE);
        //NRFX_DELAY_US(WAIT_BETWEEN_BYTES);
        for(int i = 1; i < FINCH_SPI_SENSOR_LENGTH; i++)
        {
            readBuffer[i] = spi.write(0xFF);
        //    NRFX_DELAY_US(WAIT_BETWEEN_BYTES);
        }
        //NRFX_DELAY_US(SS_WAIT);
        uBit.io.P16.setDigitalValue(1);
        spiActive = false;
    }
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
            //initFinch();
			break;
		case HUMMINGBIT_SAMD_ID:
			devicePrefix="BB";
            whatAmI = A_HB;
            initHB();
			break;
        // If it was none of the above, including not 0, try one more time to get a value
		default:
            device = readFirmwareVersion();
            switch(device)
            {
                case MICROBIT_SAMD_ID:
                    devicePrefix="MB";
                    whatAmI = A_MB;
                    break;
                case FINCH_SAMD_ID:
                    devicePrefix="FN";
                    whatAmI = A_FINCH;
                    //initFinch();
                    break;
                case HUMMINGBIT_SAMD_ID:
                    devicePrefix="BB";
                    whatAmI = A_HB;
                    initHB();
                    break;   
                // If the value is still junk, call it a standalone micro:bit 
                default:            
                    devicePrefix="MB";
                    whatAmI = A_MB;
        			break;
            }
            break;
	}
    
    return devicePrefix;
}

uint8_t readFirmwareVersion()
{
    // Wait up to 5 ms for another SPI command to complete
    uint8_t timeOut = 0;
    while(spiActive && timeOut < 5) {
        fiber_sleep(1);
        timeOut++;
    }
    if(!spiActive)
    {
        spiActive = true;

        uBit.io.P16.setDigitalValue(0);
        NRFX_DELAY_US(SS_WAIT);
        uint8_t readBuffer[4];
        readBuffer[0] = spi.write(0x8C); // Special command to read firmware/hardware version for both Finch and HB
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

        spiActive = false;
        
        if(readBuffer[0] == FINCH_SAMD_ID)
            return FINCH_SAMD_ID;
        else if((readBuffer[3] == HUMMINGBIT_SAMD_ID) || (readBuffer[3] == (HUMMINGBIT_SAMD_ID-1)) || (readBuffer[3] == (HUMMINGBIT_SAMD_ID-2)))
            return HUMMINGBIT_SAMD_ID;
        else if((readBuffer[0] + readBuffer[1] + readBuffer[2] + readBuffer[3]) == 0) // Bit hokey, but if all bytes are 0, it's a micro:bit since SPI isn't responding
            return MICROBIT_SAMD_ID;
        else   
            return UNIDENTIFIED_DEV; // can be any number that isn't the FINCH and HUMMINGBIT IDs 
    }
    return UNIDENTIFIED_DEV;
}

// Function for debugging use only
void printFirmwareResponse()
{
    if(uBit.buttonA.isPressed())
    {
        bleConnected = true;
        
        uBit.io.P16.setDigitalValue(0);
        NRFX_DELAY_US(SS_WAIT);
        uint8_t readBuffer[4];
        readBuffer[0] = spi.write(0x8C); // Special command to read firmware/hardware version for both Finch and HB
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

        for(int i = 0; i < 4; i++) {
            uBit.display.printAsync(readBuffer[i]);
            fiber_sleep(1400);
            uBit.display.clear();
            fiber_sleep(200);
        }
        bleConnected = false;
        
        }
}