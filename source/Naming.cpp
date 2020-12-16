#include "MicroBit.h"
#include "BirdBrain.h"
#include "Naming.h"
#include <cstdio>

char initials_name[3] = {'E', 'R', 'R'}; // Holds our fancy name initials


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
    uint32_t temp = (microbit_serial_number()&0x000FFFFF);

	uint8_t mod16 = 0;
    uint8_t top8  = 0;
    uint8_t bot6  = 0;
    uint8_t mid6  = 0;
    uint8_t tempCount = 0;

	  //Divide 5 bytes into 4 regions 
	mod16  =  temp%16;
	top8   =  temp%256;
	mid6   =  (temp/256)%64;
    bot6   =  (temp/256)/64;
		
		//Use these 4 regions to form 3 letters
	initials_name[0] = name_first[top8 + mod16];
	initials_name[1] = name_second[mid6 + mod16];
	initials_name[2] = name_third[bot6 + mod16];
		
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