#include "MicroBit.h"

MicroBit uBit;
BLEDevice BirdBrainBLE;
NVMController mem_controller;
MicroBitStorage storeMe(mem_controller, -1);


int 
main()
{
    bool advertising = true;
    uBit.init();

    BirdBrainBLE.init("MB9C99D", uBit.getSerial(), uBit.messageBus, storeMe, true);
    BirdBrainBLE.advertise();
    BirdBrainBLE.setTransmitPower(7);

    while(1) {
        if (uBit.buttonA.isPressed()) {
            if(advertising) {
                advertising = false;
                BirdBrainBLE.stopAdvertising();
                uBit.display.image.print('X');
                uBit.sleep(200);
            }
            else {
                advertising = true;
                BirdBrainBLE.advertise();
                uBit.display.image.print('A');
                uBit.sleep(200);
            }
        }
    }
}