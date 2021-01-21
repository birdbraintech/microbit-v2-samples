#include "MicroBit.h"
#include "BirdBrain.h"
#include "BLESerial.h"
#include "Hummingbird.h"
#include "Finch.h"
#include "SpiControl.h"

//uint8_t ble_read_buff[BLE__MAX_PACKET_LENGTH]; // incoming commands from the tablet/computer
//uint8_t sensor_vals[BLE__MAX_PACKET_LENGTH]; // sensor data to send to the tablet/computer
bool bleConnected = false; // Holds if connected over BLE
bool notifyOn = false; // Holds if notifications are being sent
bool calibrationSuccess = false;
MicroBitUARTService *bleuart;
bool processCommand = false; // Flag to hold off sending sensor data until we have first processed an inbound command

uint8_t sensorPacketCount = 0; // to send a sensor packet every 20 ms or so 
uint8_t sense_count = 0;
uint16_t sleepCounter = 0;

// For debugging
uint8_t bufferLength = 0;
uint8_t bufferReport = 0;
uint8_t bufferMem = 0;

// Convenience function to read the command packet, returns false if it timed out
bool getCommands(uint8_t commands[], uint8_t startIndex, uint8_t length);

// Convenience function to read a single byte, index is the position in the array where we want to put the byte
bool readOneByteWithTimeOut(uint8_t commands[], int index);

// Sends BLE sensor data every 30 ms
void send_ble_data()
{
    while(notifyOn) {
        assembleSensorData(); // assembles and sends a sensor packet
        fiber_sleep(30); // Change this to change sensor data frequency 
    }
    release_fiber();
}


void onConnected(MicroBitEvent)
{
    bleConnected = true;
    playConnectSound();
}

void onDisconnected(MicroBitEvent)
{
    bleConnected = false;
    notifyOn = false; // in case this was not reset by the computer/tablet
    flashOn = false; // Turning off any current message being printed to the screen
    stopMB(); // Stops the LED screen and buzzer, and if a MB sets edge connector pins to inputs
    playDisconnectSound();
    if(whatAmI == A_FINCH)
    {
        stopFinch();
    }
    if(whatAmI == A_HB)
    {
        stopHB();
    }
}

void sleepTimer()
{
    // Checks if we should turn off the Finch
    bool playDisconnectWhenTimedOut = true;
    while(1)
    {
        fiber_sleep(60000); // wait one minute
        if(whatAmI == A_FINCH)
        {
            sleepCounter++;
            if(sleepCounter > FINCH_INACTIVITY_TIMEOUT)
            {
                // To keep the disconnect sound from playing multiple times   
                if(playDisconnectWhenTimedOut) {
                    playDisconnectSound(); // play a sound to tell people we're turning off
                    playDisconnectWhenTimedOut = false;
                }
                turnOffFinch();
            }
        }
    }
}

void flashInitials()
{
    uint8_t count = 0;
    while(1)
    {
        if(!bleConnected) {
            // Print one of three initials
            uBit.display.printAsync(initials_name[count]);
            fiber_sleep(400);
            uBit.display.clear();
            fiber_sleep(200);
            count++;
            // If you're at 3, spend a longer time with display cleared so it's obvious which initial is first
            if(count ==3)
            {
                fiber_sleep(500);
                count = 0;
            }
        }
        else {
            fiber_sleep(1000);
            count = 0;
        }
    }
}

// Initializes the UART
void bleSerialInit(ManagedString devName) 
{
    bleuart = new MicroBitUARTService(*uBit.ble, 240, 32); //127, 127); // increasing the buffer size to catch more near simultaneous commands

    fiber_sleep(10); //Give the UART service a moment to register

    // Configure advertising with the UART service added, and with our prefix added
    uBit.ble->configAdvertising(devName);
    
    // Waiting for the BLE stack to stabilize
    fiber_sleep(10);
    // Error checking code - uncomment if you need to check this
    //err= uBit.ble->configAdvertising(devName);
    //uint32_t *err;
    //char buffer[9];    
    /*
    for(int i = 0; i < 4; i++) {
        sprintf(buffer,"%lX",*(err+i)); //buffer now contains sn as a null terminated string
        ManagedString serialNumberAsHex(buffer);
        uBit.display.scroll(serialNumberAsHex);
    }*/
    uBit.ble->setTransmitPower(7); 
    uBit.ble->advertise();
    
    uBit.messageBus.listen(MICROBIT_ID_BLE, MICROBIT_BLE_EVT_CONNECTED, onConnected);
    uBit.messageBus.listen(MICROBIT_ID_BLE, MICROBIT_BLE_EVT_DISCONNECTED, onDisconnected);
    create_fiber(flashInitials); // Start flashing since we're disconnected
    create_fiber(sleepTimer); 
}

// Checks what command (setAll, get firmware, etc) is coming over BLE, then acts as necessary

void bleSerialCommand()
{
    //uint8_t buff_length = bleuart->rxBufferedSize(); // checking how many bytes we have in the buffer

    uint8_t ble_read_buff[20]; // local buffer to hold our command packet
    uint8_t bytesUsed = 0;

    memset(ble_read_buff, 0, 20); // resetting the packet


    
    //ble_read_buff[0] = bleuart->getc(SYNC_SLEEP); // waits until something shows up

    // Run this loop if there is data in the buffer 
    // This allows multiple commands to execute sequentially since it just gets called over and over in the main while loop
    if(bleConnected && (bleuart->rxBufferedSize() > 0) && (processCommand == false))
    {
        bufferLength = bleuart->rxBufferedSize(); // for debugging only
        //memset(ble_read_buff, 0, 20); // resetting the packet
        // Check that the sensor packet isn't currently in the process of being set/sent
        /*uint8_t timeOut = 0;
        // give it a few tries before giving up on reading the command packet
        while(processCommand && timeOut < 5)
        {
           fiber_sleep(1);
           timeOut++;     
        }*/
        processCommand = true; // set a flag that tells the sensor packet function not to interrupt this
        ble_read_buff[0] = bleuart->getc(ASYNC); // reads immediately
        bytesUsed = 1; // we have now used at least one byte
        sleepCounter = 0; // reset the sleep counter since we have received a command
        // Switch on the first command byte
        switch(ble_read_buff[0])
        {
            case SET_LEDARRAY:
                if(whatAmI == A_MB) {
                    // try to read the second byte, give it five tries before giving up
                    if(readOneByteWithTimeOut(ble_read_buff, 1))
                    {
                        // If our command is to print a symbol, then we're using 6 bytes
                        if(ble_read_buff[1] == SYMBOL)
                        {
                            bytesUsed = 6;  // 2 bytes for commands, 4 for the symbol
                        }
                        else if(ble_read_buff[1] & SCROLL)
                        {
                            bytesUsed =  (ble_read_buff[1] & 0x1F) + 2; // We use 2 bytes + the length of the message
                        }
                        else {
                            bytesUsed = 2; // If this happens, it is a clear screen command
                        }
                        // get the rest of the command packet, only do something with it if you have enough data
                        if(getCommands(ble_read_buff, 2, bytesUsed)) {
                            decodeAndSetDisplay(ble_read_buff, bytesUsed);
                        }
                    }
                }
                break;
            case SET_FIRMWARE:
            case FINCH_SET_FIRMWARE:            
                // if you can read three more bytes, then you can send the firmware data
                if(getCommands(ble_read_buff, 1, 4))
                {
                    returnFirmwareData();
                    bytesUsed = 4; // This command sends 0xCF or 0xD4 followed by 0xFF three times
                }
                break;                    
            case NOTIFICATIONS:
                if(readOneByteWithTimeOut(ble_read_buff, 1)) {
                    if(ble_read_buff[1] == START_NOTIFY) {
                        notifyOn = true;
                        //sensorPacketCount=0;
                        create_fiber(send_ble_data); // Sends sensor data every 30 ms
                    }
                    else if(ble_read_buff[1] == STOP_NOTIFY) {
                        notifyOn = false;
                    }
                    bytesUsed = 2; // Uses two bytes
                }
                break;
            case MICRO_IO:
                if(whatAmI == A_MB) {
                    if(getCommands(ble_read_buff, 1, 8))
                    {
                            decodeAndSetPins(ble_read_buff);
                            bytesUsed = 8; // This command always uses 8 bytes
                    }
                }
                break;
            case STOP_ALL:
                // If you receive three more bytes
                if(getCommands(ble_read_buff, 1, 4))
                {
                    stopMB(); // Stops the LED screen and buzzer, and if a MB sets edge connector pins to inputs
                    if(whatAmI == A_HB)
                    {
                        stopHB(); // stops servos and LEDs on the HB
                    }
                    bytesUsed = 4; // This command sends 0xCB followed by 0xFF three times
                }
                break;
            case SET_CALIBRATE:
                // If you receive three more bytes
                if(getCommands(ble_read_buff, 1, 4))
                {
                    // Turn off notifications while calibrating
                    notifyOn = false;
                    //sensorPacketCount = 0;
                    uBit.compass.calibrate();
                    calibrationSuccess = true; // = uBit.compass.isCalibrated(); // probably not necessary
                    notifyOn = true; // restart notifications
                    create_fiber(send_ble_data);
                    bytesUsed = 4; // This command sends 0xCE followed by 0xFF three times
                }
                break;
            // Sets the Hummingbird outputs and, in some cases, the micro:bit's buzzer
            case SETALL_SPI:
                if(whatAmI == A_HB)
                {
                    if(getCommands(ble_read_buff, 1, HB_SETALL_LENGTH)) {
                        bytesUsed = HB_SETALL_LENGTH; // 19 bytes right now
                        setAllHB(ble_read_buff, bytesUsed);
                    }
                }
                if(whatAmI == A_MB)
                {
                    if(getCommands(ble_read_buff, 1, HB_SETALL_LENGTH)) {
                        // setting the buzzer
                        uint16_t buzzPeriod = (ble_read_buff[15]<<8) + ble_read_buff[16];
                        uint16_t buzzDuration = (ble_read_buff[17]<<8) + ble_read_buff[18];
                        setBuzzer(buzzPeriod, buzzDuration);                    
                        bytesUsed = HB_SETALL_LENGTH; // 19 bytes right now
                    }    
                }
                break;
            // Sets the Finch LEDs + buzzer
            case FINCH_SETALL_LED:
                if(whatAmI == A_FINCH)
                {
                    if(getCommands(ble_read_buff, 1, FINCH_SETALL_LENGTH)) {
                        bytesUsed = FINCH_SETALL_LENGTH; // 20 bytes
                        setAllFinchLEDs(ble_read_buff, bytesUsed);
                    }
                }
                break;
            // Sets the Finch motors + LED screen, depending on mode
            case FINCH_SETALL_MOTORS_MLED:
                if(whatAmI == A_FINCH)
                {
                    // getting the mode byte
                    if(readOneByteWithTimeOut(ble_read_buff, 1)) 
                    {
                        // Use only the top 3 bits to determine mode
                        uint8_t mode = (ble_read_buff[1]>>5) & LED_MOTOR_MODE_MASK;
                        bool sendCommand = false;
                        switch(mode) {
                            case PRINT:
                                bytesUsed = (ble_read_buff[1] & 0x0F)+2; // We're using two command bytes + the message to print
                                sendCommand = getCommands(ble_read_buff, 2, bytesUsed); // get the rest of the commands
                                break;
                            case FINCH_SYMBOL:
                                bytesUsed = 6; // using two commands bytes + 4 for the symbol
                                sendCommand = getCommands(ble_read_buff, 2, bytesUsed); // get the rest of the commands
                                break;
                            case MOTORS:
                                bytesUsed = 10; // 2 command bytes + 8 to set motors
                                sendCommand = getCommands(ble_read_buff, 2, bytesUsed); // get the rest of the commands
                                break;
                            case MOTORS_SYMBOL:
                                bytesUsed = 14; // 2 command bytes + 8 for motors + 4 for symbol
                                sendCommand = getCommands(ble_read_buff, 2, bytesUsed); // get the rest of the commands
                                break;
                            case MOTORS_PRINT:
                                bytesUsed = (ble_read_buff[1] & 0x0F)+10; // We're using two command bytes + 8 for motor + the message to print
                                sendCommand = getCommands(ble_read_buff, 2, bytesUsed); // get the rest of the commands
                                break;
                        }
                        if(sendCommand)
                        {
                            setAllFinchMotorsAndLEDArray(ble_read_buff, bytesUsed);
                        }
                    }
                }
                break;    
            // Finch Stop command
            case FINCH_STOPALL:
                stopMB(); // turn off LED array and buzzer
                if(whatAmI == A_FINCH) {
                    stopFinch(); // Stop the Finch moving and LEDs
                }
                bytesUsed = 1; // 0xDF
                break;
            case FINCH_RESET_ENCODERS:
                if(whatAmI == A_FINCH) {
                    resetEncoders();
                }
                bytesUsed = 1; // 0xD5
                break;
            default:
                break;
        }
        processCommand = false; // we are done processing commands, so now we should allow sensor packets to go out
    }
    else {
        fiber_sleep(1);
        sensorPacketCount++;
    }
    // Seems happier if this is done asynchronously in a different fiber
    /*if(notifyOn && sensorPacketCount > 20)
    {
        assembleSensorData();
        sensorPacketCount = 0;
    }*/

}

// Collects the notification data and sends it to the computer/tablet
void assembleSensorData()
{
    if(bleConnected && notifyOn)
    {
        uint8_t timeOut = 0;
        // give it a few tries before giving up on sending a sensor packet

        while(processCommand && timeOut < 5)
        {
           fiber_sleep(1);
           timeOut++;     
        }

        processCommand = true;

        if(whatAmI == A_FINCH)
        {
            uint8_t sensor_vals[FINCH_SENSOR_SEND_LENGTH];
            uint8_t spi_sensors_only[FINCH_SPI_SENSOR_LENGTH];
            memset(sensor_vals, 0, FINCH_SENSOR_SEND_LENGTH);    
            
            spiReadFinch(spi_sensors_only);
            arrangeFinchSensors(spi_sensors_only, sensor_vals);

            getAccelerometerValsFinch(sensor_vals);
            getMagnetometerValsFinch(sensor_vals);
            getButtonValsFinch(sensor_vals);
            
            // Overwriting right encoder for debugging purposes
            /*
            if(bufferLength > 0)
            {
                bufferReport = 0;
                bufferMem = bufferLength;
            }
            // Make sure we give it at least ten cycles of reporting our non zero buffer length
            if(bufferReport < 10) {
                int32_t temp = bufferMem*792;
                    //Right Encoders
                sensor_vals[10] = (temp & 0xFF0000) >>16;
                sensor_vals[11] = (temp & 0x00FF00) >>8;
                sensor_vals[12] = temp & 0x0000FF;
                bufferReport++;
            }
            else {
                sensor_vals[10] = 0;
                sensor_vals[11] = 0;
                sensor_vals[12] = 0;              
            }*/


            // Probably not necessary as we get feedback from the LED screen            
            if(calibrationSuccess)
            {
                sensor_vals[16] = sensor_vals[16] | 0x04;
            }

            bleuart->send(sensor_vals, sizeof(sensor_vals), ASYNC);
        }
        else
        {
            uint8_t sensor_vals[SENSOR_SEND_LENGTH];
            memset(sensor_vals, 0, SENSOR_SEND_LENGTH);
           
            if(whatAmI == A_MB)
            {
                getEdgeConnectorVals(sensor_vals);
                sensor_vals[3] = 0xFF; // no battery level reported
            }
            if(whatAmI == A_HB)
            {
                // reading Hummingbird sensors + battery level via SPI
                spiReadHB(sensor_vals);
            }
            getAccelerometerVals(sensor_vals);
            getMagnetometerVals(sensor_vals);
            getButtonVals(sensor_vals);
            
            // Probably not necessary as we get feedback from the LED screen            
            if(calibrationSuccess)
            {
                sensor_vals[7] = sensor_vals[7] | 0x04;
            }

            //send the data asynchronously
            bleuart->send(sensor_vals, sizeof(sensor_vals), ASYNC);
        }
        processCommand = false; // Allow others to interrupt
    }
}

// Get the rest of the command packet
// start index is where in the command packet we want to start reading
// length is the end index - so we are going to read length-startIndex bytes
bool getCommands(uint8_t commands[], uint8_t startIndex, uint8_t length)
{
    uint8_t timeOut = 0;
    for(int i = startIndex; i < length; i++)
    {
        timeOut=0;
        // try to read each character
        while(!bleuart->isReadable() && timeOut < 5)
        {
            fiber_sleep(1);
            timeOut++;     
        }
        if(timeOut < 5) {        
            commands[i] = bleuart->getc(ASYNC);
        }
        // If there's a character, read it
        /*if(bleuart->isReadable()) {        
            commands[i] = bleuart->getc(ASYNC);
        }*/
        // if we timed out, return failure
        else {
            return false;
        }
    }
    return true;
}

// Convenience function to read a single byte, index is the position in the array where we want to put the byte
bool readOneByteWithTimeOut(uint8_t commands[], int index)
{
    uint8_t timeOut=0;
        // try to read each character
    while(!bleuart->isReadable() && timeOut < 5)
    {
        fiber_sleep(1);
        timeOut++;     
    }
    if(timeOut < 5) {        
    //if(bleuart->isReadable()) {  
        commands[index] = bleuart->getc(ASYNC);
        return true;
    }
    // if we timed out, return failure
    else {
        return false;
    }    
}


void returnFirmwareData()
{
    // hardware version is 1 for NXP, 2 for LS - currently uses LS
    // second byte is micro_firmware_version - 0x02 on V1
    // third byte is SAMD firmware version - 0 for MB, 3 for HB, 7 or 44 for Finch
    // Hard coding this for now
    uint8_t return_buff[3];
    return_buff[0] = 2;
    return_buff[1] = 2;
    if(whatAmI == A_MB)
    {
        return_buff[2] = 0xFF;
    } 
    else if(whatAmI == A_HB)
    {
        return_buff[2] = 3;
    } 
    else if(whatAmI == A_FINCH)
    {
        return_buff[2] = 44;
    } 
    else
    {
        return_buff[2] = 0;
    }
    bleuart->send(return_buff, 3, ASYNC);
}


// Plays the connect sound
void playConnectSound()
{
    // Plays the BirdBrain connect song, over the built-in speaker if a micro:bit, or over Finch/HB buzzer if not 
    if(whatAmI == A_MB)
    {
        uBit.io.speaker.setAnalogValue(512);
        uBit.io.speaker.setAnalogPeriodUs(3039);
        fiber_sleep(100);
        uBit.io.speaker.setAnalogPeriodUs(1912);
        fiber_sleep(100);
        uBit.io.speaker.setAnalogPeriodUs(1703);
        fiber_sleep(100);
        uBit.io.speaker.setAnalogPeriodUs(1351);
        fiber_sleep(100);
        uBit.io.speaker.setAnalogValue(0);
    }
    else 
    {
        uBit.io.P0.setAnalogValue(512);
        uBit.io.P0.setAnalogPeriodUs(3039);
        fiber_sleep(100);
        uBit.io.P0.setAnalogPeriodUs(1912);
        fiber_sleep(100);
        uBit.io.P0.setAnalogPeriodUs(1703);
        fiber_sleep(100);
        uBit.io.P0.setAnalogPeriodUs(1351);
        fiber_sleep(100);
        uBit.io.P0.setAnalogValue(0);
    }    
}

// Plays the disconnect sound
void playDisconnectSound()
{
    // Plays the BirdBrain disconnect song
    if(whatAmI == A_MB)
    {
        uBit.io.speaker.setAnalogValue(512);
        uBit.io.speaker.setAnalogPeriodUs(1702);
        fiber_sleep(100);
        uBit.io.speaker.setAnalogPeriodUs(2024);
        fiber_sleep(100);
        uBit.io.speaker.setAnalogPeriodUs(2551);
        fiber_sleep(100);
        uBit.io.speaker.setAnalogPeriodUs(3816);
        fiber_sleep(100);
        uBit.io.speaker.setAnalogValue(0);
    }
    else
    {
        uBit.io.P0.setAnalogValue(512);
        uBit.io.P0.setAnalogPeriodUs(1702);
        fiber_sleep(100);
        uBit.io.P0.setAnalogPeriodUs(2024);
        fiber_sleep(100);
        uBit.io.P0.setAnalogPeriodUs(2551);
        fiber_sleep(100);
        uBit.io.P0.setAnalogPeriodUs(3816);
        fiber_sleep(100);
        uBit.io.P0.setAnalogValue(0);
    }    
}


// Checks what command (setAll, get firmware, etc) is coming over BLE, then acts as necessary
// Old version of this function, seemed glitchy
/*
void bleSerialCommand()
{
    uint8_t buff_length = bleuart->rxBufferedSize(); // checking how many bytes we have in the buffer

    uint8_t bytesUsed = 0;
    bool continueReading = true;
    // Execute a command if we have data in our buffer and we're connected to a device
    if(bleConnected && (buff_length > 0))
    {
        sleepCounter = 0; // reset the sleep counter since we have received a command
        processCommand = true;
        uint8_t ble_read_buff[buff_length];
        bleuart->read(ble_read_buff, buff_length, SYNC_SLEEP); // Other option is ASYNC
        // Loop for as long as there are unaddressed commands in the buffer
        while(continueReading)
        {
            // Switch on the command byte
            switch(ble_read_buff[0])
            {
                case SET_LEDARRAY:
                    if(buff_length >= 2) {
                        decodeAndSetDisplay(ble_read_buff, buff_length);
                        // If our command is to print a symbol, then we're using 6 bytes
                        if(ble_read_buff[1] == SYMBOL)
                        {
                            bytesUsed = 6;  // 2 bytes for commands, 4 for the symbol
                        }
                        else 
                        {
                            bytesUsed =  (ble_read_buff[1] & 0x1F) + 2; // We use 2 bytes + the length of the message
                        }
                    }
                    else {
                        bytesUsed = 1; // Invalid command
                    }
                    break;
                case SET_FIRMWARE:
                    returnFirmwareData();
                    bytesUsed = 4; // This command sends 0xCF followed by 0xFF three times
                    break;                    
                case FINCH_SET_FIRMWARE:
                    returnFirmwareData();
                    bytesUsed = 4; // This command sends 0xD4 followed by 0xFF three times
                    break;
                case NOTIFICATIONS:
                    if(buff_length > 1) {
                        if(ble_read_buff[1] == START_NOTIFY) {
                            notifyOn = true;
                            create_fiber(send_ble_data); // Sends sensor data every 30 ms
                        }
                        else if(ble_read_buff[1] == STOP_NOTIFY) {
                            notifyOn = false;
                        }
                        bytesUsed = 2; // Uses two bytes
                    }
                    else {
                        bytesUsed = 1; // this shouldn't happen
                    }
                    break;
                case MICRO_IO:
                    decodeAndSetPins(ble_read_buff);
                    bytesUsed = 8; // This command always uses 8 bytes
                    break;
                case STOP_ALL:
                    stopMB(); // Stops the LED screen and buzzer, and if a MB sets edge connector pins to inputs
                    if(whatAmI == A_HB)
                    {
                        stopHB();
                    }
                    bytesUsed = 4; // This command sends 0xCB followed by 0xFF three times
                    break;
                case SET_CALIBRATE:
                    // Turn off notifications while calibrating
                    notifyOn = false;
                    uBit.compass.calibrate();
                    calibrationSuccess = true; // = uBit.compass.isCalibrated(); // probably not necessary
                    notifyOn = true; // restart notifications
                    create_fiber(send_ble_data);
                    bytesUsed = 4; // This command sends 0xCE followed by 0xFF three times
                    break;
                // Sets the Hummingbird outputs
                case SETALL_SPI:
                    if(whatAmI == A_HB)
                    {
                        setAllHB(ble_read_buff, buff_length);
                    }
                    bytesUsed = HB_SETALL_LENGTH; // 19 bytes right now
                    break;
                // Sets the Finch LEDs + buzzer
                case FINCH_SETALL_LED:
                    if(whatAmI == A_FINCH)
                    {
                        setAllFinchLEDs(ble_read_buff, buff_length);
                    }
                    bytesUsed = FINCH_SETALL_LENGTH; // 20 bytes
                    break;
                // Sets the Finch motors + LED screen, depending on mode
                case FINCH_SETALL_MOTORS_MLED:
                    if(whatAmI == A_FINCH)
                    {
                        bytesUsed = setAllFinchMotorsAndLEDArray(ble_read_buff, buff_length); // bytes used is variable
                    }
                    else {
                        bytesUsed = 1; // this case should not happen
                    }
                    break;    
                // Finch Stop command
                case FINCH_STOPALL:
                    stopMB(); // turn off LED array and buzzer
                    stopFinch();
                    bytesUsed = 1; // 0xDF
                    break;
                case FINCH_RESET_ENCODERS:
                    resetEncoders();
                    bytesUsed = 1; // 0xD5
                    break;
                default:
                    bytesUsed = 1; // We still used one byte, even if it's an invalid command
                    break;
            }
        
            // Check if our buffer has another command in it - if so, attempt to execute it as well by going back through the loop
            
            if(buff_length > bytesUsed)
            {

                buff_length = buff_length - bytesUsed; // shorten the buffer length by what we've used
                bufferLength = buff_length;
                // Overwrite the buffer with just what we haven't used yet
                uint8_t ble_buff_temp[buff_length];
                for(int i = 0; i < buff_length; i++)
                {
                    ble_buff_temp[i] = ble_read_buff[bytesUsed+i];
                }
                // Update the buffer array
                memcpy(ble_read_buff, ble_buff_temp, buff_length);

                bytesUsed = 1; // reset bytes used to 1 byte (we always use 1 byte) and run us through the switch statement again
            }
            // If not, stop the loop
            else
            {
               bufferLength = 0;
               continueReading = false;
            }
        }
        processCommand = false; // we are done processing commands, so now we should allow sensor packets to go out
    }

}
*/