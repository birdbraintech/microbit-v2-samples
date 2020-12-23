#include "MicroBit.h"
#include "MicroBitUARTService.h"

#include "BirdBrain.h"
#include "SpiControl.h"
#include "Naming.h"

#include <cstdio>


MicroBit uBit;

// Check for BLE data, execute appropriate commands, and send sensor data
void ble_mgmt_loop() {
    while(1) { // loop for ever
        bleSerialCommand();   // reads the serial command and then executes on that command
        assembleSensorData(); // assembles and sends a sensor packet
        fiber_sleep(10);
    }
}

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

    BBMicroBitInit();     // Setting up an event listener for flashing messages

    create_fiber(ble_mgmt_loop);
    release_fiber();
    
}

