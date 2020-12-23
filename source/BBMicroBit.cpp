// Implements specialized micro:bit functions for use with BirdBrain's Bluetooth tethered apps
#include "MicroBit.h"
#include "BirdBrain.h"
#include "BBMicroBit.h"

char messageFlash[18]; // 18 is the maximum message length
uint8_t messageLength; 
bool flashOn;  // Allows us to cancel flashing a message if we're asked to print a symbol before we're done
bool pinsInputs[3]; // Determines if pins 0, 1, and 2 are set as inputs

uint16_t buzzPeriod;
uint16_t buzzDuration;
bool newBuzz; // Flag to show if a new buzzer value needs to update the buzzer

// Flash the message 
void flashMessage(MicroBitEvent)
{
    uint8_t i = 0;
    uint8_t state = 0;
    flashOn = true;
    // Checks if a different message or symbol has come by to override the current message
    // Checks every 200 ms
    while(flashOn && (i < messageLength))
    {
        if(state == 0)
        {
            uBit.display.printAsync(messageFlash[i]);
        }
        if(state == 2)
        {
            uBit.display.clear(); // clear the display to provide a "flash"
            i++;
            state = -1; // reset our state counter (effectively to 0 since we increment it next)
        }
        state++;
        fiber_sleep(200);
    }
    if(i == messageLength)
        flashOn = false;
}

// Run the buzzer
// This mostly seems to be working, but it's possible that it is missing some notes when you rapidly
// send notes with no delay in between
void mbBuzz(MicroBitEvent)
{
    uint16_t elapsed = 0;
    uint16_t localDuration = 0;
    localDuration = buzzDuration;
    // Make a sound if the tone is below 20 KHz and longer than 10 ms
    if(buzzPeriod > 50 && localDuration > 10) {
        while(elapsed < (localDuration/10))
        {
            // If we've updated the buzzer while it's playing, update to the new values
            if(newBuzz && buzzDuration > 0) {
                uBit.io.speaker.setAnalogValue(512);
                uBit.io.speaker.setAnalogPeriodUs(buzzPeriod);
                elapsed = 0; //resetting elapsed time
                localDuration = buzzDuration; //resetting the buzz duration
                newBuzz = false; //making sure we don't update again until a real new value comes in over BLE
            }
            fiber_sleep(10); 
            elapsed++;
        }
        uBit.io.speaker.setAnalogValue(0);
    }
    // Else just set the buzzer to off
    else {
    //    uBit.io.speaker.setAnalogValue(0);
    } 
    newBuzz = false;     
}


void BBMicroBitInit()
{
    // Set up a listener for flashing messages    
    uBit.messageBus.listen(BB_ID, FLASH_MSG_EVT, flashMessage);
    uBit.messageBus.listen(BB_ID, MB_BUZZ_EVT, mbBuzz);

    flashOn = false;
    memset(pinsInputs, false, 3);
    messageLength = 0; 
    buzzPeriod = 0;
    buzzDuration = 0;

}

// Set the display based on the bluetooth command
void decodeAndSetDisplay(uint8_t displayCommands[], uint8_t commandLength)
{
    if (displayCommands[1] & SYMBOL) //In this case we are going to display a symbol 
    {
        flashOn = false; // Turn of flashing the message if printing a symbol now
        MicroBitImage bleImage(5,5);
        
        uint32_t imageVals = 0; // puts all 25 bits into a single value
        uint8_t currentByte = 0;
        for(int i = 2; i<6;i++)
        {
            currentByte = displayCommands[i];
            imageVals   = imageVals | currentByte<<(24-8*(i-2));
        }
        for(int row = 0; row < 5; row++)
        {
            for(int col = 0; col < 5; col++)
            {
                if(imageVals & 0x01<<(col*5+row)) 
                {
                    bleImage.setPixelValue(row, col, 255);
                }
                else
                {
                    bleImage.setPixelValue(row, col, 0);
                }
            }
        }
        uBit.display.clear();
        uBit.display.printAsync(bleImage);
    }
    else if(displayCommands[1] & SCROLL) // In this we will scroll a message
    {
        uBit.display.clear();
        messageLength= (displayCommands[1] & 0x1F);// - SCROLL; // This gets us the length of the message to scroll

        // flash the characters - necessary for now since scrolling is slower and messes with
        // printing multiple messages. Could fix the timing in the apps later for V2
        for(int i = 0; i < messageLength; i++)
        {
            messageFlash[i] = displayCommands[i+2];
         /*   uBit.display.printAsync(scrollVals[i]);
            fiber_sleep(400);
            uBit.display.clear();
            fiber_sleep(200);*/
        }
        MicroBitEvent evt(BB_ID, FLASH_MSG_EVT); 
        //ManagedString scrollMsg(scrollVals, scroll_length);
        //uBit.display.printAsync(scrollMsg);
        //}
    }
    else // if neither of these, clear the display
    {
        uBit.display.clear();
    }
}

// This function sets the edge connector pins or internal buzzer
// I still need to test that the edge connector pins are being set properly
void decodeAndSetPins(uint8_t displayCommands[])
{
    uint16_t pwmVal = 0; //variable to set the output pin to 
    // Setting pin 0
    // Buzzer mode
    if((displayCommands[4] & 0x30) == 0x20)
    {
        pinsInputs[0] = false;
        buzzPeriod = (displayCommands[1]<<8) + displayCommands[2];
        buzzDuration = (displayCommands[3]<<8) + displayCommands[5];
        newBuzz = true; //Turning this on to update the loop in case the event is already active
        MicroBitEvent evt(BB_ID, MB_BUZZ_EVT); 
    }
    // input mode
    else if((displayCommands[4] & 0x30) == 0x10)
    {
        // Tells the sensor data function to treat this pin as an analog input
        pinsInputs[0] = true;
    }
    // set PWM mode
    else
    {
        pinsInputs[0] = false;
        pwmVal = 4*displayCommands[5]; 
        uBit.io.P0.setAnalogValue(pwmVal);
    }

    // Setting pin 1
    if((displayCommands[4] & 0x0C) == 0x04)
    {
        // Tells the sensor data function to treat this pin as an analog input
        pinsInputs[1] = true;
    }
    // set PWM mode
    else
    {
        pinsInputs[1] = false;
        pwmVal = 4*displayCommands[6]; 
        uBit.io.P1.setAnalogValue(pwmVal);
    }

    // Setting pin 2
    if((displayCommands[4] & 0x03) == 0x01)
    {
        // Tells the sensor data function to treat this pin as an analog input
        pinsInputs[2] = true;
    }
    // set PWM mode
    else
    {
        pinsInputs[2] = false;
        pwmVal = 4*displayCommands[7]; 
        uBit.io.P2.setAnalogValue(pwmVal);
    }
    
}
