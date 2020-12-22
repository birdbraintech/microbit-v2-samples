#include "MicroBit.h"
#include "MicroBitUARTService.h"

#include "BirdBrain.h"
#include "SpiControl.h"
#include "Naming.h"

#include <cstdio>


MicroBit uBit;



int 
main()
{
    //bool advertising = true;
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

    bleSerialInit(bbDevName);     // Start up a UART service and start advertising

    while(1) {
        //bleuart->read(read_buff, 1, ASYNC);
        // Print our initials while not connected over Bluetooth
        /*if(connected == false) {
            printInitials();
        } */

        bleSerialCommand();
        assembleSensorData();
        fiber_sleep(10);
/*
        if(uBit.buttonA.isPressed()) {
            assembleSensorData();
        }
        if(uBit.buttonB.isPressed()) {
            returnFirmwareData();
        }

        fiber_sleep(300);*/
        // Example code that changes the name
        /*
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
        }*/

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

