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

// Initializes the Hatchling, mostly setting the edge connector pins as we want
void initHatchling()
{
    // Setting these pins to analog inputs so we can read the ID values
    //MicroBitPin P0(MICROBIT_ID_IO_P0, MICROBIT_PIN_P0, PIN_CAPABILITY_BOTH); 
    //MicroBitPin P1(MICROBIT_ID_IO_P1, MICROBIT_PIN_P1, PIN_CAPABILITY_BOTH); 
    //MicroBitPin P2(MICROBIT_ID_IO_P2, MICROBIT_PIN_P2, PIN_CAPABILITY_BOTH); 
    stopHatchling();
    // Init the previous Hatchling LED command array
    memset(HatchlingOnBoardLEDs, 0x06, HATCHLING_ONBOARD_LED_CMD_LENGTH);
    HatchlingOnBoardLEDs[0] = HATCHLING_SET_ONBOARD_LEDS;
}

// Sends the stop command to the Hatchling
void stopHatchling()
{
    uint8_t stopCommand[HATCHLING_SPI_LENGTH];
    
    memset(stopCommand, 0xFF, HATCHLING_SPI_LENGTH);
    stopCommand[0] = FINCH_HATCHLING_STOPALL;
    spiWrite(stopCommand, HATCHLING_SPI_LENGTH);
    // Init the previous Hatchling LED command array to all 0s
    memset(HatchlingOnBoardLEDs, 0, HATCHLING_ONBOARD_LED_CMD_LENGTH);
    HatchlingOnBoardLEDs[0] = HATCHLING_SET_ONBOARD_LEDS;
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

// Sets hatchling LEDs to initial connection level
void displayConnectedLEDs()
{
    // Right now just shows all white, update to make it display what is plugged in as well

    memset(HatchlingOnBoardLEDs, 0x06, HATCHLING_ONBOARD_LED_CMD_LENGTH);
    HatchlingOnBoardLEDs[0] = HATCHLING_SET_ONBOARD_LEDS;
    setOnboardHatchlingLEDs(HatchlingOnBoardLEDs, HATCHLING_SPI_LENGTH);
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
    if(length >= GP_PORT_TOTAL+1)
    {
        spiWrite(commands, GP_PORT_TOTAL+1);
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
                if((Filtered_ID_Vals[i-14] < (id_values[j] + 4)) && (Filtered_ID_Vals[i-14] > (id_values[j] -4)))
                {
                    GP_ID_vals_new[i-14] = j;
                    break; // break out of inner loop only
                }
            }
        }
    }

    // Infinite time filtering to reduce noise
    Filtered_ID_Vals[3] = 0;//(Filtered_ID_Vals[3]*2 + (uint8_t)(uBit.io.P2.getAnalogValue()*0.37))/3; // multiply by 0.37 to make it the same as the SAMD readings (multiplier is 1.48, but SAMD is 8 bit and ubit is 10 bit)
    Filtered_ID_Vals[4] = 0;//(Filtered_ID_Vals[4]*2 + (uint8_t)(uBit.io.P1.getAnalogValue()*0.37))/3; // multiply by 0.37 to make it the same as the SAMD readings (multiplier is 1.48, but SAMD is 8 bit and ubit is 10 bit)
    Filtered_ID_Vals[5] = 0;//(Filtered_ID_Vals[5]*2 + (uint8_t)(uBit.io.P0.getAnalogValue()*0.37))/3; // multiply by 0.37 to make it the same as the SAMD readings (multiplier is 1.48, but SAMD is 8 bit and ubit is 10 bit)

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
                if((Filtered_ID_Vals[i] < (id_values[j] + 4)) && (Filtered_ID_Vals[i] > (id_values[j] -4)))
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
            
        //    uBit.serial.sendChar(GP_ID_vals_new[i]);
        //    uBit.serial.sendChar(i);
        //    uBit.serial.sendChar(Filtered_ID_Vals[i]);
       /*     if(GP_ID_vals_new[i] == 0 || GP_ID_vals_new[i] == 31) {
                HatchlingOnBoardLEDs[i*3+1] = 0x06;
                HatchlingOnBoardLEDs[i*3+2] = 0x06;
                HatchlingOnBoardLEDs[i*3+3] = 0x06;
            }
            else if(GP_ID_vals_new[i] < 6) {
                HatchlingOnBoardLEDs[i*3+1] = 0x40;
                HatchlingOnBoardLEDs[i*3+2] = 0x00;
                HatchlingOnBoardLEDs[i*3+3] = 0x40;
            }
            else {
                HatchlingOnBoardLEDs[i*3+1] = 0x80;
                HatchlingOnBoardLEDs[i*3+2] = 0x40;
                HatchlingOnBoardLEDs[i*3+3] = 0x00;
            }*/

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
        
        uint8_t commands[GP_PORT_TOTAL+1];
        commands[0] = HATCHLING_SET_PORT_STATES;
        for(i=0; i < GP_PORT_TOTAL; i++)
        {
            commands[i+1] = GP_ID_vals[i];
            if((GP_ID_vals[i] > 26) || (GP_ID_vals[i] == 0))
            {
                HatchlingOnBoardLEDs[i*3+1] = 0x06;
                HatchlingOnBoardLEDs[i*3+2] = 0x06;
                HatchlingOnBoardLEDs[i*3+3] = 0x06;
            }
            else
            {
                HatchlingOnBoardLEDs[i*3+1] = ledcolors[GP_ID_vals[i]%8][0];
                HatchlingOnBoardLEDs[i*3+2] = ledcolors[GP_ID_vals[i]%8][1];
                HatchlingOnBoardLEDs[i*3+3] = ledcolors[GP_ID_vals[i]%8][2];
            }
            NRFX_DELAY_US(25); //Need a delay here or else the SPI seems to fail
        }
        setHatchlingPortStates(commands, GP_PORT_TOTAL+1);
        //NRFX_DELAY_US(250); 
        HatchlingOnBoardLEDs[0] = HATCHLING_SET_ONBOARD_LEDS;
       /* for(i = 0; i < 19; i++)
            uBit.serial.sendChar(HatchlingOnBoardLEDs[i]);*/
        setOnboardHatchlingLEDs(HatchlingOnBoardLEDs, HATCHLING_SPI_LENGTH);
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

}
