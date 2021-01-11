#include "MicroBit.h"
#include "BirdBrain.h"
#include "SPIControl.h"
#include "BBMicroBit.h"
#include "Finch.h"
#include "BLESerial.h"

int32_t leftEncoder = 0; //Holds the running value of the left encoder
int32_t rightEncoder = 0; //Holds the running value of the right encoder

//Helps in knowing which motor is in motion and what direction it is moving
bool leftMotorMove     = false ;
bool leftMotorForwardDirection = false ;
bool rightMotorMove     = false ;
bool rightMotorForwardDirection = false ;


// Helper function to record how we're setting the motors
void moveMotor(uint8_t* currentCommand);

// Initializes the Finch, mostly setting the edge connector pins as we want
void initFinch()
{
    stopFinch();
    resetEncoders();
}

// Sends the stop command to the Finch
void stopFinch()
{
    uint8_t stopCommand[FINCH_SPI_LENGTH];
    
    memset(stopCommand, 0x00, FINCH_SPI_LENGTH);
    stopCommand[0] = FINCH_STOPALL;
    spiWrite(stopCommand, FINCH_SPI_LENGTH);
}

// Sets all Finch LEDs + buzzer in one go
void setAllFinchLEDs(uint8_t commands[], uint8_t length)
{
    // Double check that the command contains enough data for us to proceed
    if(length >= FINCH_SETALL_LENGTH)
    {    
        // setting the buzzer
        uint16_t buzzPeriod = (commands[16]<<8) + commands[17];
        uint16_t buzzDuration = (commands[18]<<8) + commands[19];
        setBuzzer(buzzPeriod, buzzDuration);

        // setting the Finch LEDs
        spiWrite(commands, FINCH_SPI_LENGTH);
    }
}

// Sets all Finch motors + the micro:bit LED array
void setAllFinchMotorsAndLEDArray(uint8_t commands[], uint8_t length)
{
    uint8_t mode;
    // Making sure we have enough data
    if(length >= 2)
    {
        // Use only the top 3 bits to determine mode
        mode = (commands[1]>>5) & LED_MOTOR_MODE_MASK;

        switch(mode)
        {
            case PRINT:
                commands[1] = SCROLL + (length-2); // Changing the command to one decodeAndSetDisplay understands
                decodeAndSetDisplay(commands, length);
                break;
            case FINCH_SYMBOL:
                // checking that we have enough data to set the screen
                if(length >= 6)
                {
                    commands[1] = SYMBOL; // Changing the command to one decodeAndSetDisplay understands
                    decodeAndSetDisplay(commands, length);
                }
                break;
            case MOTORS:
                // Checking that we have enough data to set the motor
                if(length >= 10)
                {
                    moveMotor(commands);
                }
                break;
            case MOTORS_SYMBOL:
                // Checking that we have enough data to set the motor and LED screen
                if(length >= 14)
                {
                    uint8_t symbolCommands[6];
                    // creating the symbol command set
                    symbolCommands[0] = commands[0];
                    symbolCommands[1] = SYMBOL;
                    for(int i = 0; i < 4; i++)
                    {
                        symbolCommands[i+2] = commands[i+10];
                    }
                    decodeAndSetDisplay(symbolCommands, 6);
                    
                    moveMotor(commands); // safe to send the symbol commands too, they get overwritten with zeros
                }
                break;
            case MOTORS_PRINT:
                
                uint8_t print_length = commands[1] & 0x0F; // Finding out how long the message to print is
                // Checking that we have enough data
                if(length >= print_length + 10)
                {
                    // creating the print command set
                    uint8_t printCommands[print_length+2]; // The length of the array needs to be 2 bytes longer than the length of the message
                    printCommands[0] = commands[0];
                    printCommands[1] = SCROLL + print_length;
                    for(int i = 0; i < print_length; i++)
                    {
                        printCommands[i+2] = commands[i+10];
                    }
                    decodeAndSetDisplay(printCommands, print_length+2);
                    
                    moveMotor(commands); // safe to send the print commands too, they get overwritten with zeros
                }
                break;
        }
    }

}

// Resets the Finch encoders
void resetEncoders()
{
    leftEncoder = 0;
    rightEncoder = 0;
    // Do we also need to reset the encoder on the Finch SAMD chip?
}

// Turns off the Finch - in case we haven't received anything over BLE for 10 minutes
void turnOffFinch()
{
    uint8_t turnOffCommand[FINCH_SPI_LENGTH];
    
    memset(turnOffCommand, 0x00, FINCH_SPI_LENGTH);
    turnOffCommand[0] = FINCH_POWEROFF_SAMD;

    spiWrite(turnOffCommand, FINCH_SPI_LENGTH);       
}

/************************************************************************/
//Function which updates the Encoder count value
//Increases in one direction , decreases in another, based on the direction of the motor
//Only active when the motors are moving
/************************************************************************/
void arrangeFinchSensors(uint8_t (&spi_sensors_only)[FINCH_SPI_SENSOR_LENGTH], uint8_t (&sensor_vals)[FINCH_SENSOR_SEND_LENGTH])
{
	uint8_t i = 0;
	
	static int32_t leftMotorChange      = 0;
    uint32_t currentLeftCounterValue = 0;
	static uint32_t prevLeftCounterValue = 0;
	
	static int32_t  rightMotorChange      = 0;
	uint32_t currentRightCounterValue = 0;
	static uint32_t prevRightCounterValue = 0;
	
	//Update the left counter based on the info got from SAMD
	currentLeftCounterValue  = ((uint32_t)spi_sensors_only[9]<<16);
	currentLeftCounterValue |= (uint32_t)spi_sensors_only[10]<<8;
	currentLeftCounterValue |= (uint32_t)spi_sensors_only[11];
	//Update the right counter based on the info got from SAMD
	currentRightCounterValue =((uint32_t)spi_sensors_only[12]<<16);
	currentRightCounterValue |= (uint32_t)spi_sensors_only[13]<<8;
	currentRightCounterValue |= (uint32_t)spi_sensors_only[14];
	
	
	//Adust the encoders
	//When the motors are moving , start considering the value
	if(leftMotorMove == true)
	{
		//Get the change from previous value
		leftMotorChange           = currentLeftCounterValue - prevLeftCounterValue ;
		//Based on the direction of the motor add/substract the values
		if(leftMotorForwardDirection == true )
		{
			leftEncoder  = leftEncoder + leftMotorChange;
		}
		else
		{
			leftEncoder  = leftEncoder - leftMotorChange;
		}
	}
	prevLeftCounterValue = currentLeftCounterValue;
	
	if(rightMotorMove == true)
	{
		//Get the change from previous value
		rightMotorChange       = currentRightCounterValue - prevRightCounterValue ;
		//Based on the direction of the motor add/substract the values
		if(rightMotorForwardDirection == true )
		{
			rightEncoder  = rightEncoder + rightMotorChange ;
		}
		else
		{
			rightEncoder  = rightEncoder - rightMotorChange;
		}
	}
	prevRightCounterValue = currentRightCounterValue;
	
	
	for(i=0;i<7;i++)
	{
		sensor_vals[i] = spi_sensors_only[i+2];
	}
	
	
	//Left Encoder to sensor values
	sensor_vals[7]  = (leftEncoder  & 0xFF0000) >>16;
	sensor_vals[8] =  (leftEncoder & 0x00FF00) >>8;
	sensor_vals[9] =  leftEncoder & 0x0000FF;
	
	//Right Encoders
	sensor_vals[10] = (rightEncoder & 0xFF0000) >>16;
	sensor_vals[11] = (rightEncoder & 0x00FF00) >>8;
	sensor_vals[12] = rightEncoder & 0x0000FF;
}


/************************************************************************/
//Update movement flags as they are useful in getting a relative encoder tick count
//Convert 32 bit number into a 24 bit value and send it to SAMD
//Left motor
/************************************************************************/
void moveMotor(uint8_t* currentCommand)
{
    uint16_t leftMotorSpeed  = 0;
    uint32_t leftMotorTicks  = 0;
    uint16_t rightMotorSpeed = 0;
    uint32_t rightMotorTicks = 0;
    leftMotorSpeed  = currentCommand[2];
    rightMotorSpeed = currentCommand[6];
    //Left Motor ticks
    leftMotorTicks  =  (((uint32_t)currentCommand[3])<<16)&0x00FF0000;
    leftMotorTicks  |= (((uint32_t)currentCommand[4])<<8)&0x0000FF00;
    leftMotorTicks  |= (((uint32_t)currentCommand[5]))&0x000000FF;
    //Right motor ticks
    rightMotorTicks  = (((uint32_t)currentCommand[7])<<16)&0x00FF0000;
    rightMotorTicks  |= (((uint32_t)currentCommand[8])<<8) & 0x0000FF00;
    rightMotorTicks  |= (((uint32_t)currentCommand[9])) & 0x000000FF;

    //nrf_delay_ms(1); Not sure why we did this, but it was in the V1 firmware
    //
    if(((leftMotorTicks == 1) && (leftMotorSpeed == 0) && (rightMotorTicks == 1) && (rightMotorSpeed == 0)) == false )
    {
        //Direction 
        if(leftMotorSpeed >= 128)
        {
            leftMotorForwardDirection = true;
        }
        else
        {
            leftMotorForwardDirection = false;
        }
        if((leftMotorSpeed == 0) && (leftMotorTicks == 0))
        { 
            leftMotorMove = false;
        }
        else
        {
            leftMotorMove = true;
        }
        //Direction
        if(rightMotorSpeed  >= 128)
        {
            rightMotorForwardDirection = true;
        }
        else
        {
            rightMotorForwardDirection = false;
        }
        if((rightMotorSpeed == 0) && (rightMotorTicks == 0))
        {
            rightMotorMove = false;
        }
        else
        {
            rightMotorMove = true;
        }

        // pad the packet to be 16 bytes
        for(int i = 10; i < FINCH_SPI_LENGTH; i++)
            currentCommand[i] = 0x00;

        spiWrite(currentCommand,FINCH_SPI_LENGTH);
    }
}