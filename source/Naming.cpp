#include "MicroBit.h"
#include "BirdBrain.h"
#include "Naming.h"
#include "ble_gap.h" // needed to get the mac address
#include <cstdio>

char initials_name[10] = {'E', 'R', 'R', 'F', 'N', '0', '0', '0', '0', '0'}; // Holds our name initials

char convert_to_ascii(uint8_t input); // convenience function

/************************************************************************/
// Check if the three letter word matches the rude words and if yes change the word
/************************************************************************/
bool rude_word_check()
{
	for(unsigned int i=0;i<sizeof(first_letter);i++)
	{
		if(initials_name[0] == first_letter[i] )
		{
			if(initials_name[1] == second_letter[i])
			{
				if(initials_name[2] == third_letter[i] )
				{
					return true;
				}
			}
		}
	}
	return false;
}

/************************************************************************/
// Use the micro:bit serial number to find the first three letters of the fancy name to print on the LED screen
// Check if the three letter word matches the rude words and if yes change the word
/************************************************************************/
void getInitials_fancyName()
{
	bool rude_word = true;

    // Get the last five hex digits of the serial number
    /*uint32_t temp = (microbit_serial_number()&0x000FFFFF);

	uint8_t mod16 = 0;
    uint8_t top8  = 0;
    uint8_t bot6  = 0;
    uint8_t mid6  = 0;
    uint8_t tempCount = 0;*/
    // Use the Mac address to get the name
    ble_gap_addr_t mac;
    sd_ble_gap_addr_get(&mac);
    volatile uint32_t temp = 0;
	uint8_t mod16 = 0;
    uint8_t top8  = 0;
    uint8_t bot6  = 0;
    uint8_t mid6  = 0;
    uint8_t tempCount = 0;
    //Get the Mac Address into a 32 32 bit variable 	
    temp  |= (uint32_t)(mac.addr[2]&0x0F) << 16;	
	temp  |= (uint32_t)mac.addr[1] << 8;
	temp  |= (uint32_t)mac.addr[0];

	  //Divide 5 bytes into 4 regions 
	mod16  =  temp%16;
	top8   =  temp%256;
	mid6   =  (temp/256)%64;
    bot6   =  (temp/256)/64;
		
		//Use these 4 regions to form 3 letters
	initials_name[0] = name_first[top8 + mod16];
	initials_name[1] = name_second[mid6 + mod16];
	initials_name[2] = name_third[bot6 + mod16];

    ManagedString bbDevName = whichDevice(); 

    initials_name[3] = bbDevName.charAt(0);
    initials_name[4] = bbDevName.charAt(1);

	initials_name[5] = convert_to_ascii((mac.addr[2]&0x0F));
	
	
	initials_name[6] = (mac.addr[1]&0xF0);
	initials_name[6] = convert_to_ascii(initials_name[6]>>4);
		
	
	initials_name[7] = convert_to_ascii(mac.addr[1]&0x0F);
	initials_name[8] = (mac.addr[0]& 0xF0);
	initials_name[8] = convert_to_ascii(initials_name[8]>>4);
		
	initials_name[9] = convert_to_ascii(mac.addr[0]&0x0F);
		
	//Check if the word generated could be an English rude words
	while(rude_word == true)
	{
        //check if the existing three letter word is a rude word
        rude_word = rude_word_check();
        tempCount++;
        //update the middle letter and see if it is still a rude word - eventually we will get to a non-rude word
        if(rude_word == true)
        {
            initials_name[0] = name_first[top8 + mod16];
            initials_name[1] = name_second[(mid6 + mod16 + tempCount)%512];
            initials_name[2] = name_third[bot6 + mod16];
        }
    }
}

/************************************************************************/
// Print the initials
/************************************************************************/
void printInitials()
{
    uBit.display.print(initials_name[0]);
    fiber_sleep(400);
    uBit.display.clear();
    fiber_sleep(200);
    uBit.display.print(initials_name[1]);
    fiber_sleep(400);
    uBit.display.clear();
    fiber_sleep(200);
    uBit.display.print(initials_name[2]);
    fiber_sleep(400);
    uBit.display.clear();
    fiber_sleep(800);
}

/************************************************************************/
/**@brief 		Function for converting Hex values to ASCII values
 * @param[in] Input in hex to be converted to ASCII
 * @return 		character which is an ascii value of the input
 */
 /************************************************************************/
char convert_to_ascii(uint8_t input)
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