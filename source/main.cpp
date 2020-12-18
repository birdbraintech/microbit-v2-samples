#include "MicroBit.h"
#include "MicroBitUARTService.h"

#include "BirdBrain.h"
#include "SpiControl.h"
#include "Naming.h"

#include <cstdio>

MicroBit uBit;
MicroBitUARTService *bleuart;


// disabling the event service as we don't need it
#ifndef MICROBIT_BLE_EVENT_SERVICE
#define MICROBIT_BLE_EVENT_SERVICE              0
#endif


int connected = 0;


void onConnected(MicroBitEvent)
{
    connected = 1;

}

void onDisconnected(MicroBitEvent)
{
    connected = 0;
}

int 
main()
{
    //bool advertising = true;
    //uint8_t read_buff[32];
    //uint8_t *sensor_vals;
    //uint8_t spi_write_buff[20];


    NRFX_DELAY_MS(2050);
    uBit.init(); // Initializes everything but BLE and SPI
    spiInit();   // Turn on SPI
    ManagedString bbDevName = whichDevice(); // Get our name prefix - BB, FN, or MB - depending on what we are attached to
    
    uBit.initBLE(bbDevName);
    // Figure out what we are called, start flashing our initials
    getInitials_fancyName();

    // Start up a UART service
    //bleuart = new MicroBitUARTService(*uBit.ble, 32, 32);
    uBit.ble->setTransmitPower(7); 
    /*uBit.ble->stopAdvertising();
    uBit.ble->configAdvertising();
    uBit.ble->advertise();*/
    
    uBit.messageBus.listen(MICROBIT_ID_BLE, MICROBIT_BLE_EVT_CONNECTED, onConnected);
    uBit.messageBus.listen(MICROBIT_ID_BLE, MICROBIT_BLE_EVT_DISCONNECTED, onDisconnected);


    while(1) {
        //bleuart->read(read_buff, 1, ASYNC);
        // Print our initials while not connected over Bluetooth
        if(connected == 0) {
            printInitials();
        }
        fiber_sleep(10);
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
