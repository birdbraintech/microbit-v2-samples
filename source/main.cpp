#include "MicroBit.h"
#include "MicroBitUARTService.h"

#include "BirdBrain.h"
#include "SpiControl.h"
#include "Naming.h"

#include <cstdio>


MicroBit uBit;
MicroBitUARTService *bleuart;
static NRF52PWM *speaker = NULL;
static Mixer2 *mixer = NULL;


// disabling the event service as we don't need it
#ifndef MICROBIT_BLE_EVENT_SERVICE
#define MICROBIT_BLE_EVENT_SERVICE              0
#endif

// BIRDBRAIN CHANGE
#define UART_SERVICE_ID 0x0001 // Nordic service ID


bool connected = false;
bool justConnected = false;
bool justDisconnected = false;

void microbit_ble_configureAdvertising( bool connectable, bool discoverable, bool whitelist,
                                               uint16_t interval_ms, int timeout_seconds);

void onConnected(MicroBitEvent)
{
    connected = true;
    justConnected = true;

}

void onDisconnected(MicroBitEvent)
{
    connected = false;
    justDisconnected = true;
}

int 
main()
{
    //bool advertising = true;
    uint8_t read_buff[32];
    //uint8_t *sensor_vals;
    //uint8_t spi_write_buff[20];


    uBit.init(); // Initializes everything 
    spiInit();   // Turn on SPI

    // Toggle the reset pin on the Finch, then hold it low
    // This happens even for HB and standalone micro:bit, as it needs to happen before we can determine device type
    uBit.io.pin[RESET_PIN].setDigitalValue(1);
    fiber_sleep(200);
    uBit.io.pin[RESET_PIN].setDigitalValue(0);
    
    // Wait for the SAMD bootloader
    fiber_sleep(1850);

    ManagedString bbDevName = whichDevice(); // Get our name prefix - BB, FN, or MB - depending on what we are attached to

    //uBit.display.scroll("u");
    // Figure out what we are called, start flashing our initials
    getInitials_fancyName();

    fiber_sleep(10); // Wait for the stack to stabilize before registering the UART service
    // Start up a UART service
    bleuart = new MicroBitUARTService(*uBit.ble, 32, 32);

    fiber_sleep(10); //Give the UART service a moment to register
    uint32_t *err;
    char buffer[9];    
    // Configure advertising with the UART service added, and with our prefix added
    err= uBit.ble->configAdvertising(bbDevName);
    /*
    for(int i = 0; i < 4; i++) {
        sprintf(buffer,"%lX",*(err+i)); //buffer now contains sn as a null terminated string
        ManagedString serialNumberAsHex(buffer);
        uBit.display.scroll(serialNumberAsHex);
    }*/
    uBit.ble->setTransmitPower(7); 
    uBit.ble->advertise();

    // Initialize speaker - this is not working
   /* speaker = new NRF52PWM(NRF_PWM1, *mixer, 44100);
    mixer = new Mixer2();
    mixer->setSampleRange(speaker->getSampleRange());
    mixer->setOrMask(0x8000);

    speaker->setDecoderMode(PWM_DECODER_LOAD_Common);
    speaker->connectPin(uBit.io.P0, 0);
    speaker->connectPin(uBit.io.speaker, 1);
*/
    uBit.messageBus.listen(MICROBIT_ID_BLE, MICROBIT_BLE_EVT_CONNECTED, onConnected);
    uBit.messageBus.listen(MICROBIT_ID_BLE, MICROBIT_BLE_EVT_DISCONNECTED, onDisconnected);



    while(1) {
        //bleuart->read(read_buff, 1, ASYNC);
        // Print our initials while not connected over Bluetooth
        if(connected == false) {
            printInitials();
        } 
        fiber_sleep(50);
        // Example code that changes the name
        if (uBit.buttonA.isPressed()) {
           uBit.ble->stopAdvertising();
           err= uBit.ble->configAdvertising("MB");
           
           for(int i = 0; i < 4; i++) {
                sprintf(buffer,"%lX",*(err+i)); //buffer now contains sn as a null terminated string
                ManagedString serialNumberAsHex(buffer);
                uBit.display.scroll(serialNumberAsHex);
           }
            uBit.ble->setTransmitPower(7); 
            uBit.ble->advertise(); 
        }
        if(justConnected)
        {
            justConnected = false;  
            // Plays the BirdBrain connect song 
            if(whatAmI == A_MB)
            {
                uBit.io.speaker.setAnalogValue(512);
                uBit.io.speaker.setAnalogPeriodUs(3039);
                uBit.sleep(100);
                uBit.io.speaker.setAnalogPeriodUs(1912);
                uBit.sleep(100);
                uBit.io.speaker.setAnalogPeriodUs(1703);
                uBit.sleep(100);
                uBit.io.speaker.setAnalogPeriodUs(1351);
                uBit.sleep(100);
                uBit.io.speaker.setAnalogValue(0);
            }
            else 
            {
                uBit.io.P0.setAnalogValue(512);
                uBit.io.P0.setAnalogPeriodUs(3039);
                uBit.sleep(100);
                uBit.io.P0.setAnalogPeriodUs(1912);
                uBit.sleep(100);
                uBit.io.P0.setAnalogPeriodUs(1703);
                uBit.sleep(100);
                uBit.io.P0.setAnalogPeriodUs(1351);
                uBit.sleep(100);
                uBit.io.P0.setAnalogValue(0);
            }
        }

        if(justDisconnected)
        {
            justDisconnected = false;
            // Plays the BirdBrain disconnect song
            if(whatAmI == A_MB)
            {
                uBit.io.speaker.setAnalogValue(512);
                uBit.io.speaker.setAnalogPeriodUs(1702);
                uBit.sleep(100);
                uBit.io.speaker.setAnalogPeriodUs(2024);
                uBit.sleep(100);
                uBit.io.speaker.setAnalogPeriodUs(2551);
                uBit.sleep(100);
                uBit.io.speaker.setAnalogPeriodUs(3816);
                uBit.sleep(100);
                uBit.io.speaker.setAnalogValue(0);
            }
            else
            {
                uBit.io.P0.setAnalogValue(512);
                uBit.io.P0.setAnalogPeriodUs(1702);
                uBit.sleep(100);
                uBit.io.P0.setAnalogPeriodUs(2024);
                uBit.sleep(100);
                uBit.io.P0.setAnalogPeriodUs(2551);
                uBit.sleep(100);
                uBit.io.P0.setAnalogPeriodUs(3816);
                uBit.sleep(100);
                uBit.io.P0.setAnalogValue(0);
            }
            
        }

        //uBit.display.scroll(read_buff[0]);
/*
        if (uBit.buttonA.isPressed()) {
            printInitials();

            
            sensor_vals = spiReadHB();
            for(int i = 0; i < 4; i ++) {
                uBit.display.scroll(sensor_vals[i]);
                uBit.sleep(1500);
                uBit.display.print(" ");
                uBit.sleep(500);
            }

        } 
        if (uBit.buttonB.isPressed()) {
            // Get the serial number
            uint32_t sn = microbit_serial_number();

            // Convert the serial number to a string
            char buffer[9];
            sprintf(buffer,"%lX",sn); //buffer now contains sn as a null terminated string
            ManagedString serialNumberAsHex(buffer);
            uBit.display.scroll(serialNumberAsHex);


            bleuart->read(read_buff, 1, ASYNC);
            uBit.display.print("B");
            uint8_t spi_write_buff[4];
            spi_write_buff[0] = 0xC4;
            spi_write_buff[1] = read_buff[0];
            spi_write_buff[2] = 0x00;
            spi_write_buff[3] = 255-read_buff[0];
                
            spiWrite(spi_write_buff, 4);
        }*/
    }
}

