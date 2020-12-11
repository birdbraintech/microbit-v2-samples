#include "MicroBit.h"
#include "MicroBitUARTService.h"

#include "ble_gap.h"

MicroBit uBit;
//BLEDevice BirdBrainBLE; 


//NVMController mem_controller; // Need to create a micro:bit storage controller for Bluetooth
//MicroBitStorage storeMe(mem_controller, -1); //Hopefully putting the storage of the data at the end with the -1 flag
MicroBitUARTService *bleuart;

int connected = 0;

char DEVICE_NAME[8];
ManagedString deviceName ="";

char convert_ascii(uint8_t input);
void set_devicename_array();

void onConnected(MicroBitEvent)
{
    uBit.display.print("C");
    connected = 1;

}

void onDisconnected(MicroBitEvent)
{
    uBit.display.print("D");
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

    set_devicename_array();
    for(int i = 0; i < 8; i++) {
        uBit.display.image.print(DEVICE_NAME[i]);
        uBit.sleep(1000);
    }

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

/************************************************************************/
/**@brief 		Function for setting the device name  it either starts with HB or MB 
 *            the next five characters are taken from the last five hex values of MAC address
 *						followed by Null.
 */
/************************************************************************/
void set_devicename_array()
{
	ble_gap_addr_t 					mac;
    sd_ble_gap_addr_get(&mac);
		
	DEVICE_NAME[0] = 'M';
	DEVICE_NAME[1] =  'B';//INITIAL_NAME[1];
	DEVICE_NAME[2] = convert_ascii((mac.addr[2]&0x0F));
	
	
	DEVICE_NAME[3] = (mac.addr[1]&0xF0);
	DEVICE_NAME[3] = convert_ascii(DEVICE_NAME[3]>>4);
		
	
	DEVICE_NAME[4] = convert_ascii(mac.addr[1]&0x0F);
	DEVICE_NAME[5] = (mac.addr[0]& 0xF0);
	DEVICE_NAME[5] = convert_ascii(DEVICE_NAME[5]>>4);
		
	DEVICE_NAME[6] = convert_ascii(mac.addr[0]&0x0F);
	DEVICE_NAME[7] = '\0';
}

/************************************************************************/
/**@brief 		Function for converting Hex values to ASCII values
 * @param[in] Input in hex to be converted to ASCII
 * @return 		character which is an ascii value of the input
 */
 /************************************************************************/
char convert_ascii(uint8_t input)
{
	char output;
	if(input <=9)
	{
		output = input + 0x30;
	}
	else
	{
		output = input + 0x37;
	}
	return output;
}
