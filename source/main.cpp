#include "MicroBit.h"
#include "MicroBitUARTService.h"

//#include "mbed.h"
#include <cstdio>

#define		DEVICE_MASK        							0xE0
//To know what device is connected look for these
#define MICROBIT_SAMD_ID                                7
#define FINCH_SAMD_ID									1
#define HUMMINGBIT_SAMD_ID								0

MicroBit uBit;
SPI spi(MOSI, MISO, SCK);


/*NVMController mem_controller; // Need to create a micro:bit storage controller for Bluetooth
MicroBitStorage storeMe(mem_controller, -1); //Hopefully putting the storage of the data at the end with the -1 flag*/
MicroBitUARTService *bleuart;

int connected = 0;
/*
char DEVICE_NAME[8];
ManagedString deviceName ="";

char convert_ascii(uint8_t input);
void set_devicename_array();
*/

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
    //bool advertising = true;
    uint8_t read_buff[32];

    uint8_t spi_write_buff[20];
    uint8_t spi_read_buff[20];
    

    uBit.io.P16.setDigitalValue(1); // Set the SS pin high to start

    spi_write_buff[0] = 0x8C;
    spi_write_buff[1] = 0xFF;
    spi_write_buff[2] = 0xFF;
    spi_write_buff[3] = 0xFF;


    spi_read_buff[0] = 0xFF;
    spi_read_buff[1] = 0xFF;
    spi_read_buff[2] = 0xFF;
    spi_read_buff[3] = 0xFF;

    NRFX_DELAY_MS(2050);
    
    uBit.io.P16.setDigitalValue(0);
    NRFX_DELAY_US(100);
    //spi.transfer(spi_write_buff, 4, spi_read_buff, 4);
    for(int i = 0; i < 4; i++)
    {
    //    spi_read_buff[i] = spi.write(spi_write_buff[i]);
        NRFX_DELAY_US(4);
    }

    NRFX_DELAY_US(100);
    uBit.io.P16.setDigitalValue(1);
    uBit.sleep(10);
    uint8_t samd_firmware_version = spi_read_buff[3];
    uint8_t device = (samd_firmware_version & DEVICE_MASK)>>5;
    ManagedString bbDevName("");
	switch(device)
	{
		case MICROBIT_SAMD_ID:
			bbDevName="MB";
			break;
		case FINCH_SAMD_ID:
			bbDevName="FN";
			break;
		case HUMMINGBIT_SAMD_ID:
			bbDevName="BB";
			break;
		default:
			break;
	}

    uBit.init(bbDevName);

    spi.format(8,0);
    spi.frequency(1000000);

    bleuart = new MicroBitUARTService(*uBit.ble, 32, 32);
    uBit.ble->setTransmitPower(7);
    uBit.ble->advertise();
    
    uBit.messageBus.listen(MICROBIT_ID_BLE, MICROBIT_BLE_EVT_CONNECTED, onConnected);
    uBit.messageBus.listen(MICROBIT_ID_BLE, MICROBIT_BLE_EVT_DISCONNECTED, onDisconnected);

    // Leaving this code for now as it'll help with fancy name choosing later
    uint32_t sn = microbit_serial_number();
    char buffer[16];
    sprintf(buffer,"%lX",sn); //buffer now contains sn as a null terminated string, len contains the length of the string.
   /*
    for(int i = 0; i < 4; i ++) {
        uBit.display.print(spi_read_buff[i]);
        uBit.sleep(1500);
        uBit.display.print(" ");
        uBit.sleep(500);
    }*/
    
    while(1) {
        //bleuart->read(read_buff, 1, ASYNC);

        if (uBit.buttonA.isPressed()) {
            uBit.display.print("A");

            spi_write_buff[0] = 0xCC;
            spi_write_buff[1] = 0x66;
            spi_write_buff[2] = 0x77;
            spi_write_buff[3] = 0x88;
                
            uBit.io.P16.setDigitalValue(0);
            NRFX_DELAY_US(4);
            //spi.transfer(spi_write_buff, 4, spi_read_buff, 4);
            for(int i = 0; i < 3; i++)
            {
                spi_read_buff[i] = spi.write(spi_write_buff[i]);
                NRFX_DELAY_US(100);
            }
            spi_read_buff[3] = spi.write(spi_write_buff[3]);
            // clearing the buffer
            NRFX_DELAY_US(4); 
            spi_read_buff[0] = spi.write(0x55);
            NRFX_DELAY_US(4);
            spi_read_buff[1] = spi.write(0x66);
            NRFX_DELAY_US(4);
            uBit.io.P16.setDigitalValue(1);
            for(int i = 0; i < 4; i ++) {
                uBit.display.print(spi_read_buff[i]);
                uBit.sleep(1500);
                uBit.display.print(" ");
                uBit.sleep(500);
            }
        }
            /*
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
            }*/
        
        if (uBit.buttonB.isPressed()) {

            bleuart->read(read_buff, 1, ASYNC);
            uBit.display.print("B");
            spi_write_buff[0] = 0xC4;
            spi_write_buff[1] = read_buff[0];
            spi_write_buff[2] = 0x00;
            spi_write_buff[3] = 255-read_buff[0];
                
            uBit.io.P16.setDigitalValue(0);
            NRFX_DELAY_US(4);
            //spi.transfer(spi_write_buff, 4, spi_read_buff, 4);
            for(int i = 0; i < 3; i++)
            {
                spi_read_buff[i] = spi.write(spi_write_buff[i]);
            }
            spi_read_buff[3] = spi.write(spi_write_buff[3]);
            NRFX_DELAY_US(4);
            uBit.io.P16.setDigitalValue(1);
            /*for(int i = 0; i < 4; i ++) {
                uBit.display.print(spi_read_buff[i]);
                uBit.sleep(1500);
                uBit.display.print(" ");
                uBit.sleep(500);
            }*/
        }
    }
}

/************************************************************************/
/**@brief 		Function for setting the device name  it either starts with HB or MB 
 *            the next five characters are taken from the last five hex values of MAC address
 *						followed by Null.
 */
/************************************************************************/
/*void set_devicename_array()
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
}*/

/************************************************************************/
/**@brief 		Function for converting Hex values to ASCII values
 * @param[in] Input in hex to be converted to ASCII
 * @return 		character which is an ascii value of the input
 */
 /************************************************************************/
/*char convert_ascii(uint8_t input)
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
*/