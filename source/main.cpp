#include "MicroBit.h"
#include "MicroBitUARTService.h"

MicroBit uBit;
//BLEDevice BirdBrainBLE; 


NVMController mem_controller; // Need to create a micro:bit storage controller for Bluetooth
MicroBitStorage storeMe(mem_controller, -1); //Hopefully putting the storage of the data at the end with the -1 flag
MicroBitUARTService *bleuart;
//const uint8_t bs_service_uuid[ 16] =
//{ 0x6e,0x40,0x00,0x01,0xb5,0xa3,0xf3,0x93,0xe0,0xa9,0xe5,0x0e,0x24,0xdc,0xca,0x9e };
int connected = 0;

void onConnected(MicroBitEvent)
{
    uBit.display.print("connect");
    connected = 1;

}

void onDisconnected(MicroBitEvent)
{
    uBit.display.print("disconnect");
    connected = 0;
}

int 
main()
{
    bool advertising = true;
    uint8_t read_buff[32];
   // uint8_t write_buff[32];
    uBit.sleep(100);

    uBit.init();

 /*   BirdBrainBLE.init("MB9C99D", uBit.getSerial(), uBit.messageBus, storeMe, false);

    BirdBrainBLE.setTransmitPower(7);
    BirdBrainBLE.advertise();
    bleuart = new MicroBitUARTService(*uBit.ble, 32, 32);*/

    uBit.ble->init("MB9C99D", uBit.getSerial(), uBit.messageBus, storeMe, false);
    bleuart = new MicroBitUARTService(*uBit.ble, 32, 32);
    uBit.ble->setTransmitPower(7);
    uBit.ble->advertise();


    uBit.messageBus.listen(MICROBIT_ID_BLE, MICROBIT_BLE_EVT_CONNECTED, onConnected);
    uBit.messageBus.listen(MICROBIT_ID_BLE, MICROBIT_BLE_EVT_DISCONNECTED, onDisconnected);

    while(1) {
        bleuart->read(read_buff, 1, ASYNC);

        uBit.display.image.print(read_buff[0]);

        if (uBit.buttonA.isPressed()) {
            if(advertising) {
                advertising = false;
                //uBit.ble->stopAdvertising();
                uBit.display.image.print('X');
                uBit.sleep(200);
            }
            else {
                advertising = true;
               // uBit.ble->advertise();
                uBit.display.image.print('A');
                uBit.sleep(200);
            }
        }
    }
}