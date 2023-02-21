#include "MicroBit.h"
#include "BirdBrain.h"
#include "SPIControl.h"
#include "BBMicroBit.h"
#include "Hatchling.h"
#include "BLESerial.h"
#include "ble_gap.h" // needed to get the mac address


uint8_t HatchlingOnBoardLEDs[HATCHLING_ONBOARD_LED_CMD_LENGTH];  

uint8_t GP_ID_vals[GP_PORT_TOTAL] = {31, 31, 31, 31, 31, 31};
uint16_t Filtered_ID_Vals[GP_PORT_TOTAL] = {0, 0, 0, 0, 0, 0};
uint8_t stabilizeCounter = 0;
uint8_t analogReadCounter = 0;
uint8_t analogPortStabilityTracker[3] = {0,0,0};
uint8_t previousAnalogVals[3] = {0,0,0};

// Setting these pins to analog inputs so we can read the ID values
MicroBitPin P0(MICROBIT_ID_IO_P0, MICROBIT_PIN_P0, PIN_CAPABILITY_ANALOG);
MicroBitPin P1(MICROBIT_ID_IO_P1, MICROBIT_PIN_P1, PIN_CAPABILITY_ANALOG);
MicroBitPin P2(MICROBIT_ID_IO_P2, MICROBIT_PIN_P2, PIN_CAPABILITY_ANALOG);

// Initializes the Hatchling, mostly setting the edge connector pins as we want
void initHatchling()
{ 
    stopHatchling();
    // Init the previous Hatchling LED command array
    showLEDCode();
}

// Sends the stop command to the Hatchling
void stopHatchling()
{
    uint8_t stopCommand[HATCHLING_SPI_LENGTH];
    
    memset(stopCommand, 0xFF, HATCHLING_SPI_LENGTH);
    stopCommand[0] = FINCH_HATCHLING_STOPALL;
    spiWrite(stopCommand, HATCHLING_SPI_LENGTH);
}

// Sets all Hatchling LEDs
void setOnboardHatchlingLEDs(uint8_t commands[], uint8_t length)
{
    // Double check that the command contains enough data for us to proceed and that it isn't
    // an identical command from one sent previously
    

    if(length >= HATCHLING_SPI_LENGTH)
    {    
         // setting the Hatchling LEDs
        spiWrite(commands, HATCHLING_SPI_LENGTH);
    }
}

// Sets hatchling LEDs to a color code based on mac address
void showLEDCode()
{
    ble_gap_addr_t mac;
    uint8_t mac_address_hex_vals[6];
    uint8_t i;
    sd_ble_gap_addr_get(&mac);

    // First get the 4 bit hex vals of the mac address
    mac_address_hex_vals[0] = mac.addr[2]&0x0F;
    mac_address_hex_vals[1] = (mac.addr[1]&0xF0)>>4;
    mac_address_hex_vals[2] = mac.addr[1]&0x0F;
    mac_address_hex_vals[3] = (mac.addr[0]&0xF0)>>4;
    mac_address_hex_vals[4] = mac.addr[0]&0x0F;

    // We have six LEDs, so let's derive the sixth LED color from the sum of the other five
    mac_address_hex_vals[5] = 0;
    for(i=0; i<5; i++)
    {
        mac_address_hex_vals[5] += mac_address_hex_vals[i];
    }

    mac_address_hex_vals[5] = mac_address_hex_vals[5]%16;

    for(i=0; i<GP_PORT_TOTAL;i++)
    {
        HatchlingOnBoardLEDs[i*3+1] = ledcolors[mac_address_hex_vals[i]/2][0];
        HatchlingOnBoardLEDs[i*3+2] = ledcolors[mac_address_hex_vals[i]/2][1];
        HatchlingOnBoardLEDs[i*3+3] = ledcolors[mac_address_hex_vals[i]/2][2];
    }

    HatchlingOnBoardLEDs[0] = HATCHLING_SET_ONBOARD_LEDS;
    // Haven't done the Mac address part yet, just makes something colorful
    //uint8_t LEDCommands[HATCHLING_ONBOARD_LED_CMD_LENGTH] = {0xE0, 0x40, 0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x40, 0x40, 0x00, 0x40, 0x40, 0x40, 0x40};
    setOnboardHatchlingLEDs(HatchlingOnBoardLEDs, HATCHLING_SPI_LENGTH);
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
    if(length >= HATCHLING_SPI_LENGTH)
    {
     /* Debugging only
        for(uint8_t i = 0; i < HATCHLING_SPI_LENGTH; i++)
        {
            uBit.serial.sendChar(commands[i]);
        }*/
        spiWrite(commands, HATCHLING_SPI_LENGTH);
    }
}

// Sets Hatchling Neopixel LED Strip on one port
void setHatchlingExternalNeopixelStrip(uint8_t commands[], uint8_t length)
{
    // Making sure we have enough data and that the data is fresh
    if(length >= HATCHLING_SPI_LENGTH)
    {
        spiWrite(commands, HATCHLING_SPI_LENGTH);
    }
}

// Turns off the Hatchling - in case we haven't received anything over BLE for 10 minutes
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
	uint8_t j = 0;
	
    uint8_t GP_ID_vals_new[6] = {31, 31, 31, 31, 31, 31}; // Initialize each value to 31

	for(i=14;i<HATCHLING_SENSOR_SEND_LENGTH;i++)
	{
		sensor_vals[i] = spi_sensors_only[(i-14)*2+3];
	}



    for(i=14;i<17;i++)
    {   
        Filtered_ID_Vals[i-14] = (Filtered_ID_Vals[i-14]*2+spi_sensors_only[i])/3; // Infinite time filter to reduce noise
        if(Filtered_ID_Vals[i-14] < 5)
        {
            GP_ID_vals_new[i-14] = 0;
        }
        else if(Filtered_ID_Vals[i-14] > 249)
        {
            GP_ID_vals_new[i-14] = 1;
        }   
        else {
            for(j = 2; j < TOTAL_POSSIBLE_ACCESSORIES; j++)
            {
                if((Filtered_ID_Vals[i-14] < (id_values[j] + 5)) && (Filtered_ID_Vals[i-14] > (id_values[j] -5)))
                {
                    GP_ID_vals_new[i-14] = j;
                    break; // break out of inner loop only
                }
            }
        }
    }

    uint16_t tempSensor;
    // filtering to reduce noise, and reading the micro:bit analog values less frequently
    // The analog input ports on the micro:bit seem to produce occasionally spurious values, so we need to filter them out
    if(analogReadCounter == 0)
    {
        tempSensor = P2.getAnalogValue()*0.343; // Convert from 0-1023 to the same range of values that the SAMD will report
        // In case you plug in the rotation servo and it is reading a bit high (could be 256 or 257)
        if(tempSensor > 255)
            tempSensor = 255;

        // Check if your current reading is close to your previous values
        if((tempSensor < (previousAnalogVals[0] + 5)) && (tempSensor > (previousAnalogVals[0] -5)))
        {
            analogPortStabilityTracker[0]++; 

            // Only update the port value if values seem to be stable - 4 out of 4 readings in the same range
            if(analogPortStabilityTracker[0] > 3)
            {
                // Apply a small IIT filter to the final value
                Filtered_ID_Vals[3] = (Filtered_ID_Vals[3]*2 + tempSensor)/3; ;
            //    uBit.serial.sendChar(Filtered_ID_Vals[3]);// Debug only
            //    uBit.serial.sendChar(tempSensor);
            }
        }
        // If even one value is not similar to the prior one, reset the stability counter
        else
        {
            analogPortStabilityTracker[0] = 0;
        }
        previousAnalogVals[0] = tempSensor;
   }
    
    if(analogReadCounter == 1)
    {
        tempSensor = P1.getAnalogValue()*0.343;
        if(tempSensor > 255)
            tempSensor = 255;

        if((tempSensor < (previousAnalogVals[1] + 5)) && (tempSensor > (previousAnalogVals[1] -5)))
        {
            analogPortStabilityTracker[1]++;
            if(analogPortStabilityTracker[1] > 3)
            {
                Filtered_ID_Vals[4] = (Filtered_ID_Vals[4]*2 + tempSensor)/3; 
             //   uBit.serial.sendChar(Filtered_ID_Vals[4]); //Debug only
             //   uBit.serial.sendChar(tempSensor);
            }
        }
        else
        {
            analogPortStabilityTracker[1] = 0;
        }
        previousAnalogVals[1] = tempSensor;
    }
    
    if(analogReadCounter == 2)
    {
        tempSensor = P0.getAnalogValue()*0.343;
        if(tempSensor > 255)
            tempSensor = 255;

        if((tempSensor < (previousAnalogVals[2] + 5)) && (tempSensor > (previousAnalogVals[2] -5)))
        {
            analogPortStabilityTracker[2]++;
            if(analogPortStabilityTracker[2] > 3)
            {
                Filtered_ID_Vals[5] = (Filtered_ID_Vals[5]*2 + tempSensor)/3; 
            //    uBit.serial.sendChar(Filtered_ID_Vals[5]); //Debug only
            //    uBit.serial.sendChar(tempSensor);
                
            //    uBit.serial.sendChar('\r');
            //    uBit.serial.sendChar('\n');
            }
        }
        else
        {
            analogPortStabilityTracker[2] = 0;
        }
        previousAnalogVals[2] = tempSensor;
        analogReadCounter = 0;
    }
    else 
    {
        analogReadCounter++;
    }

    for(i = 3; i < GP_PORT_TOTAL; i++)
    {
        if(Filtered_ID_Vals[i] < 5)
        {
            GP_ID_vals_new[i] = 0;
        }
        else if(Filtered_ID_Vals[i] > 249)
        {
            GP_ID_vals_new[i] = 1;
        }   
        else { 
            for(j = 2; j < TOTAL_POSSIBLE_ACCESSORIES; j++)
            {
                if((Filtered_ID_Vals[i] < (id_values[j] + 5)) && (Filtered_ID_Vals[i] > (id_values[j] -5)))
                {
                    GP_ID_vals_new[i] = j;
                    break; 
                }
            }
        }
    }
    // If any of the ports have changed, we'll need to tell the SAMD microcontroller to update their state
    // Future consideration - do we want debouncing here so things don't update too fast?
    for(i = 0; i < GP_PORT_TOTAL; i++)
    {
        if(GP_ID_vals_new[i]!=GP_ID_vals[i])
        {
            stabilizeCounter = 0;
        }
        GP_ID_vals[i] = GP_ID_vals_new[i];
    }

    if(stabilizeCounter < 3)
    {
        stabilizeCounter++;
    }
    // If you've waited three cycles for values to stabilize, time to update the Hatchling and Hatchling LEDs
    else if(stabilizeCounter == 3)
    {   
        stabilizeCounter++;
        
        uint8_t commands[HATCHLING_SPI_LENGTH];
        
        memset(commands, 0, HATCHLING_SPI_LENGTH);
        commands[0] = HATCHLING_SET_PORT_STATES;
        for(i=0; i < GP_PORT_TOTAL; i++)
        {
            commands[i+1] = GP_ID_vals[i];
            if((GP_ID_vals[i] > 26) || (GP_ID_vals[i] == 0))
            {
                HatchlingOnBoardLEDs[i*3+1] = PortOffLEDColors[i][0];
                HatchlingOnBoardLEDs[i*3+2] = PortOffLEDColors[i][1];
                HatchlingOnBoardLEDs[i*3+3] = PortOffLEDColors[i][2];
            }
            else
            {
                HatchlingOnBoardLEDs[i*3+1] = PortOnLEDColors[i][0];
                HatchlingOnBoardLEDs[i*3+2] = PortOnLEDColors[i][1];
                HatchlingOnBoardLEDs[i*3+3] = PortOnLEDColors[i][2];
            }
            NRFX_DELAY_US(25); //Need a delay here or else the SPI seems to fail
        }
        setHatchlingPortStates(commands, HATCHLING_SPI_LENGTH);
        //NRFX_DELAY_US(250); 
        // This is now done on the SAMD side
        //HatchlingOnBoardLEDs[0] = HATCHLING_SET_ONBOARD_LEDS;
        //setOnboardHatchlingLEDs(HatchlingOnBoardLEDs, HATCHLING_SPI_LENGTH);
    }
    

    // Now pack the 6 bytes of GP_ID_vals into bytes 10-13 of sensor vals
    // Lowest 5 bits of each byte correspond to GP_ID_vals of ports A to D
    // Port E's value uses top 3 bits of byte 0 and top 3 bits of byte 1
    // Port F's value uses top 3 bits of byte 2 and top 3 bits of byte 3
    // First, do ports A to D
    for(i = 0; i < 4; i++)
    {
        sensor_vals[i+10] = GP_ID_vals[i];
    }
    // Now do port E
    uint8_t TopByte = (GP_ID_vals[4]<<2) & 0xE0; // mask all but top three bits
    uint8_t BottomByte = (GP_ID_vals[4]<<5) & 0xE0; // mask all but top three bits
    sensor_vals[10] = sensor_vals[10] | TopByte;
    sensor_vals[11] = sensor_vals[11] | BottomByte;
    // Now do port F
    TopByte = (GP_ID_vals[5]<<2) & 0xE0; // mask all but top three bits
    BottomByte = (GP_ID_vals[5]<<5) & 0xE0; // mask all but top three bits
    sensor_vals[12] = sensor_vals[12] | TopByte;
    sensor_vals[13] = sensor_vals[13] | BottomByte;

    // get the battery value for further processing
    sensor_vals[2] = spi_sensors_only[17];

    // Just to debug port values
	/*for(i=14;i<17;i++)
	{
		sensor_vals[i] = Filtered_ID_Vals[i-11];
	}*/

}
