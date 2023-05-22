#include "MicroBit.h"
#include "BirdBrain.h"
#include "SPIControl.h"
#include "BBMicroBit.h"
#include "Hatchling.h"
#include "BLESerial.h"
#include "ble_gap.h" // needed to get the mac address


uint8_t HatchlingOnBoardLEDs[HATCHLING_ONBOARD_LED_CMD_LENGTH];  

uint8_t GP_ID_vals[GP_PORT_TOTAL] = {31, 31, 31, 31, 31, 31};
uint8_t GP_ID_vals_compare[GP_PORT_TOTAL] = {31, 31, 31, 31, 31, 31};
uint16_t Filtered_ID_Vals[GP_PORT_TOTAL] = {0, 0, 0, 0, 0, 0};
bool Port_lock[6] = {false, false, false, false, false, false}; // If we've identified that a component is attached, lock that port so it only changes if it first goes through an "unplugged" situation

uint8_t stabilizeCounter = 0;
uint8_t analogReadCounter = 0;
uint8_t analogPortStabilityTracker[3] = {0,0,0};
uint8_t previousAnalogVals[3] = {0,0,0};


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
    memset(GP_ID_vals_compare, 31, GP_PORT_TOTAL); // Resetting the port identifiers to force an onboard LED update when we reconnect
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



    for(i=14;i<20;i++)
    {   
        Filtered_ID_Vals[i-14] = (Filtered_ID_Vals[i-14]*2+spi_sensors_only[i])/3; // Infinite time filter to reduce noise
        // If nothing is plugged in, the value should be 255
        if(Filtered_ID_Vals[i-14] > 250)
        {
            GP_ID_vals_new[i-14] = 0;
        } 
        else {
            for(j = 1; j < TOTAL_POSSIBLE_ACCESSORIES; j++)
            {
                if((Filtered_ID_Vals[i-14] < (id_values[j] + 5)) && (Filtered_ID_Vals[i-14] > (id_values[j] -5)))
                {
                    GP_ID_vals_new[i-14] = j;
                    break; // break out of inner loop only
                }
            }
        }
    }


    // If any of the ports have changed, we'll need to tell the SAMD microcontroller to update their state
    for(i = 0; i < GP_PORT_TOTAL; i++)
    {
        if(GP_ID_vals_new[i]!=GP_ID_vals_compare[i])
        {
            stabilizeCounter = 0;
        /*    uBit.serial.sendChar(GP_ID_vals_compare[i]); // Debugging only
            uBit.serial.sendChar(GP_ID_vals_new[i]);
            uBit.serial.sendChar(0xFF);*/
        }
        GP_ID_vals_compare[i] = GP_ID_vals_new[i];

        // for debugging, print the port states
        //uBit.serial.sendChar(GP_ID_vals[i]);
    }

    if(stabilizeCounter < 9)
    {
        stabilizeCounter++;
        // for debugging, print the counter
        //uBit.serial.sendChar(stabilizeCounter);
    }
    // If you've waited nine cycles for values to stabilize, time to update the Hatchling port states
    else if(stabilizeCounter == 9)
    {   
        stabilizeCounter++;
        
        uint8_t commands[HATCHLING_SPI_LENGTH];
        
        memset(commands, 0, HATCHLING_SPI_LENGTH);
        commands[0] = HATCHLING_SET_PORT_STATES;
        bool sendUpdateCommand = false;
        for(i=0; i < GP_PORT_TOTAL; i++)
        {
            // Only update the state if the port is not locked
            // To unlock a port the value needs to go to 0 (nothing plugged in). 
            if(GP_ID_vals_compare[i] == 0)
            {
                Port_lock[i] = false; // Unlock the port if nothing is plugged in
                GP_ID_vals[i] = GP_ID_vals_compare[i]; // Update the value sent to the tablet/device
                sendUpdateCommand = true; // Make sure to update the Hatchling daughter controller so it knows to update the port to a "port off" state
            }
            // If the port is unlocked, you can update it to the new component that is plugged into it
            else if(Port_lock[i] == false)
            {
                GP_ID_vals[i] = GP_ID_vals_compare[i];
                sendUpdateCommand = true;
                Port_lock[i] = true; // Lock the port since we are setting it
            }
            // Update the state of the port
            if((GP_ID_vals[i] ==1) || (GP_ID_vals[i] == 2))
            {
                commands[i+1] = ROTATION_SERVO;
            }
            else if((GP_ID_vals[i] > 2) && (GP_ID_vals[i] < 7))
            {
                commands[i+1] = POSITION_SERVO;
            }
            else if((GP_ID_vals[i] == 7) || (GP_ID_vals[i] == 9))
            {
                commands[i+1] = NEOPXL_SINGLE;
            }
            else if(GP_ID_vals[i] == 10)
            {
                commands[i+1] = NEOPXL_STRIP;
            }
            else if((GP_ID_vals[i] > 10) && (GP_ID_vals[i] < 22))
            {
                commands[i+1] = ANALOG_SENSOR;
            }
            else if((GP_ID_vals[i] == 8) || ((GP_ID_vals[i] > 21) && (GP_ID_vals[i] < 26)))
            {
                commands[i+1] = DIGITAL_OUT;
            }
            else
            {
                commands[i+1] = PORTOFF;
            }
        }


        // Only update the settings if at least one port required an update
        if(sendUpdateCommand)
        {
            bool settingGood = false;
            while(!settingGood)
            {
                // Send the update command
                setHatchlingPortStates(commands, HATCHLING_SPI_LENGTH);
                fiber_sleep(5); // It takes 5 ms for the sensor packet to update
                uint8_t spi_sensors_only[HATCHLING_SPI_SENSOR_LENGTH];
                memset(sensor_vals, 0, HATCHLING_SENSOR_SEND_LENGTH);    
                
                // Check that the Hatchling has successfully received the port settings by reading the sensors
                spiReadHatchling(spi_sensors_only);
                settingGood = true;
                for(i = 0; i < GP_PORT_TOTAL; i++)
                {
                    // If at least one of the commands doesn't agree, we need to set the port states again, then rerun this loop
                    if(commands[i+1] != spi_sensors_only[2*i+2])
                    {
                        settingGood = false;
                        //uBit.serial.sendChar(0x43); // For debugging
                    }
                }
            }
        }
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
    sensor_vals[2] = spi_sensors_only[20];

    // Just to debug port values
	/*for(i=14;i<17;i++)
	{
		sensor_vals[i] = Filtered_ID_Vals[i-11];
	}*/

}
