////////
//
//      LCD 16x2 & 20x4 - Liquid Crystal Display for STM32
//      Adaptation - YH - ENSEA - September 2024
//		Source File - Version with 4-bits commands & I2C communication
//		Fully dynamic allocation of memory commands / strings
//			(even if dynamic allocation can be dangerous on microcontrollers...)
//
////////
#include "cnc_lcd_i2c.h"


//
// 		GLOBAL VARIABLES / STRUCTURES INITIALISATION
//
// 	Def of lcd_config + pointer to lcd_content
cnc_lcd_content_t *lcd_pContent;
cnc_lcd_config_t  lcd_cfg =
{
	0x00,				// I2C command storage before sending to the LCD
	100,				// Delay time in us - EN ON
	100,				// Delay time in us - OFF Between 2 sets
	300,				// Delay time in us - EN OFF
	0					// flag to check if the LCD is busy
};


//
//		FUNCTIONS
//
//
//	1st Function : Function to transmit the 8-bits command / data to the LCD
//
void cnc_lcd_i2c_transmit_cmd8(uint8_t nbcharIn, char *pCmdIn, uint32_t *pDelayIn)
{
	//
	//		Step 1 : Create a structure to store the values
	//
	// Dynamic Creation of a new structure NG (Next Gen)
	cnc_lcd_content_t *lcd_pContNG = (cnc_lcd_content_t*)malloc(sizeof(cnc_lcd_content_t));
	// Store the values
	lcd_pContNG->pData  	= pCmdIn;
	lcd_pContNG->pDelay 	= pDelayIn;
	lcd_pContNG->cmd_or_str = 0;
	lcd_pContNG->lcd_xbits  = 8;
	lcd_pContNG->lcd_nbchar = nbcharIn;
	lcd_pContNG->lcd_ichar  = 0;
	lcd_pContNG->lcd_istep  = 0;
	lcd_pContNG->lcd_next   = NULL;

	//
	//		Step 2 : Decide if we send it directly (lcd_content)
	//						or we keep it in memory for the next turn
	//
	// If the LCD is not busy
	if (lcd_cfg.lcd_busy == 0)
	{
		// Tells the LCD transmission is now busy, take the address of the structure
		lcd_cfg.lcd_busy = 1;
		lcd_pContent = lcd_pContNG;
		//	And call the sending_manager
		cnc_lcd_i2c_sending_manager();
	}
	// If the LCD is busy, keep the structure in the "next" stack
	else
	{
		// Store its address in the lcd_pContent->next->...->next
		cnc_lcd_content_t *last = lcd_pContent;
		while (last->lcd_next != NULL) { last = last->lcd_next; }
		last->lcd_next = lcd_pContNG;
	}
}


//
//	2nd Function : Function to transmit a string to the LCD
//
void cnc_lcd_i2c_transmit_string(uint8_t nbcharIn, char *pDataIn)
{
	//
	//		Step 1 : Create a structure to store the values
	//
	// Dynamic Creation of a new structure NG (Next Gen)
	cnc_lcd_content_t *lcd_pContNG = (cnc_lcd_content_t*)malloc(sizeof(cnc_lcd_content_t));
	// Store the values
	lcd_pContNG->pData  	= pDataIn;
	lcd_pContNG->pDelay 	= NULL;
	lcd_pContNG->cmd_or_str = 1;
	lcd_pContNG->lcd_xbits  = 8;
	lcd_pContNG->lcd_nbchar = nbcharIn;
	lcd_pContNG->lcd_ichar  = 0;
	lcd_pContNG->lcd_istep  = 0;
	lcd_pContNG->lcd_next   = NULL;

	//
	//		Step 2 : Decide if we send it directly (lcd_content)
	//						or we keep in in memory for the next turn
	//
	// If the LCD is not busy
	if (lcd_cfg.lcd_busy == 0)
	{
		// Tells the LCD transmission is now busy, take the address of the structure
		lcd_cfg.lcd_busy = 1;
		lcd_pContent = lcd_pContNG;
		//	And call the sending_manager
		cnc_lcd_i2c_sending_manager();
	}
	// If the LCD is busy, keep the structure in the "next" stack
	else
	{
		// Store its address in the lcd_pContent->next->...->next
		cnc_lcd_content_t *last = lcd_pContent;
		while (last->lcd_next != NULL) { last = last->lcd_next; }
		last->lcd_next = lcd_pContNG;
	}
}


//
//	3rd function : Managing the timer interrupts
//
void cnc_lcd_i2c_sending_manager()
{
	// Resetting the TIM7 before calling
	HAL_TIM_Base_Stop(&TIM_HANDLE);
	TIM_NAME->CNT = 0;

	// Formatting for the I2C sending
    // 8 bits = D7-D6-D5-D4-BT-EN-RW-RS
    // if (lcd_pCont->cmd_or_str == 0) BT = 1, EN = 1, RW = 0, RS = 0; => 12 = 0x0C
    // if (lcd_pCont->cmd_or_str == 1) BT = 1, EN = 1, RW = 0, RS = 1; => 14 = 0x0D

	if (lcd_pContent != NULL)
	{
		// If (nb of commands sent < total nb of commands)
		if (lcd_pContent->lcd_ichar < lcd_pContent->lcd_nbchar)
		{
			//
			//		Depending on the state of the transmission
			//
			// Load the MSB & Send the EN impulse, EN to 1
			if (lcd_pContent->lcd_istep == 0)
			{
				// Load the MSB of the character to be sent and format for the i2c com
                lcd_cfg.i2cCmd = lcd_pContent->pData[lcd_pContent->lcd_ichar] & 0xF0;
                lcd_cfg.i2cCmd |= ( (lcd_pContent->cmd_or_str == 0) ? 0x0C : 0x0D );
                HAL_I2C_Master_Transmit(&HI2C, DEVICE_ADDRESS, (uint8_t*)&(lcd_cfg.i2cCmd), 1, 100);

				// Common for all = Increment the event flag / set the waiting time / call the timer
				lcd_pContent->lcd_istep++;
				TIM_NAME->ARR = lcd_cfg.std_delay_on;
				HAL_TIM_Base_Start_IT(&TIM_HANDLE);
			}
			// Second step, the EN impulse back to 0, between 2 sets of 4-bits command
			else if (lcd_pContent->lcd_istep == 1)
			{
				// Ending the impulse, EN back to 0 with the mask 0xFB, and wait the delay...
                lcd_cfg.i2cCmd &= 0xFB;
                HAL_I2C_Master_Transmit(&HI2C, DEVICE_ADDRESS, (uint8_t*)&(lcd_cfg.i2cCmd), 1, 100);

				// Common for all = Increment the event flag / set the waiting time / call the timer
				lcd_pContent->lcd_istep++;
				TIM_NAME->ARR = lcd_cfg.std_delay_between;
				HAL_TIM_Base_Start_IT(&TIM_HANDLE);
			}
			// Third step, for the characters, the second part of the character
			else if (lcd_pContent->lcd_istep == 2)
			{
				// Load the LSB of the character to be sent and format for the i2c com
                lcd_cfg.i2cCmd = (lcd_pContent->pData[lcd_pContent->lcd_ichar] & 0x0F) << 4;
                lcd_cfg.i2cCmd |= ((lcd_pContent->cmd_or_str == 0) ? 0x0C : 0x0D);
                HAL_I2C_Master_Transmit(&HI2C, DEVICE_ADDRESS, (uint8_t*)&(lcd_cfg.i2cCmd), 1, 100);

				// Common for all = Increment the event flag / set the waiting time / call the timer
				lcd_pContent->lcd_istep++;
				TIM_NAME->ARR = lcd_cfg.std_delay_on;
				HAL_TIM_Base_Start_IT(&TIM_HANDLE);
			}
			// Fourth step, the EN impulse back to 0, waiting for the treatment of the command
			else if (lcd_pContent->lcd_istep == 3)
			{
				// Ending the impulse, EN back to 0 with the mask 0xFB, and wait the delay...
                lcd_cfg.i2cCmd &= 0xFB;
                HAL_I2C_Master_Transmit(&HI2C, DEVICE_ADDRESS, (uint8_t*)&(lcd_cfg.i2cCmd), 1, 100);

				// Common for all = Increment the event flag / set the waiting time / call the timer
				// Set the time of the timer - waiting for the treatment of the command by the LCD
				lcd_pContent->lcd_istep++;
				if (lcd_pContent->pDelay != NULL) { TIM_NAME->ARR = lcd_pContent->pDelay[lcd_pContent->lcd_ichar]; }
				else { TIM_NAME->ARR = lcd_cfg.std_delay_off; }
				HAL_TIM_Base_Start_IT(&TIM_HANDLE);
			}
			// End of the last impulse / character transmission
			else if (lcd_pContent->lcd_istep == 4)
			{
				// Only prepare next step
				lcd_pContent->lcd_istep = (lcd_pContent->lcd_xbits == 4) ? 2 : 0;
				lcd_pContent->lcd_ichar++ ;
				// And call directly itself, the sending_manager
				cnc_lcd_i2c_sending_manager();
			}
		}
		else
		{
			if (lcd_pContent->lcd_next != NULL)
			{
				// Take the next pointer as the new first content values
				cnc_lcd_content_t *pOld = lcd_pContent;
				lcd_pContent = lcd_pContent->lcd_next;

				// Free the memory (only command data/delay are dynamic)
				if ( (pOld->pData != NULL)  && (pOld->cmd_or_str == 0) ) { free(pOld->pData);  }
				if ( (pOld->pDelay != NULL) && (pOld->cmd_or_str == 0) ) { free(pOld->pDelay); }
				if (pOld != NULL) { free(pOld); }

				// And call the sending manager
				cnc_lcd_i2c_sending_manager();
			}
			// End of character list + no next structure => End of Transmission
			else
			{
				// Reset the Flag - End of transmission
				lcd_cfg.lcd_busy = 0;

				//	FREE all the dynamic memory (only command data/delay are dynamic)
				if ( (lcd_pContent->pData != NULL)  && (lcd_pContent->cmd_or_str == 0) ) { free(lcd_pContent->pData);  }
				if ( (lcd_pContent->pDelay != NULL) && (lcd_pContent->cmd_or_str == 0) ) { free(lcd_pContent->pDelay); }
				if (  lcd_pContent != NULL ) { free(lcd_pContent); }
			}
		}
	}
}


//
//	4th function : Initialisation of the LCD
//
// Details of the initialisation procedure :
// 1. Wait for the factory initialisation = minimum 40ms after power on if 2.7V
//				(only 10ms if 5V powered)
// 2. 3 times 0x03 to wait for initialisation (check on datasheet)
// 				(still on 8-bits - the 4 other pins are physically connected to GND)
// 3. Function Set - To say that we work with a 4-bit connection
// 				(Command D7D6D5D4/RS = 0x02 = 0010/0)
// 4. Same Function Set to 4-bits + end of instruction (nb of lines)
// 				(DL = DataLine = 1 (for 2-lines = 1000), or DL = 0 (for 1 line = 0000))
// 				(Command D7D6D5D4/RS = 0010/0 then 1000/0 = 8 bits 0x28)
// 5. Display ON/OFF - To switch ON/OFF the display / cursor / blinking
// 				(Command D7D6D5D4/RS = 0000/0 then 1111/0 = 8 bits 0x0F)
// 6. Entry Mode Set - To define the increment / Shift of the display
// 				(Command D7D6D5D4/RS = 0000/0 then 0110/0 = 8 bits 0x06)
// 7. Display Clear - To clear the display / Back to the beginning
// 				(Command D7D6D5D4/RS = 0000/0 then 0001/0 = 8 bits 0x01)
// End of Initialisation Function
//
void cnc_lcd_i2c_init()
{
	// Set the busy_flag to 1
	lcd_cfg.lcd_busy = 1;

	//
	//		Step 1 : When the LCD is still in 8-bits mode
	//
	int nb_cmd_Part1 		= 5;
	char pInitPart1[] 		= {  0x00,  0x03, 0x03, 0x03, 0x02};
	uint32_t pDelayPart1[] 	= {100000, 10000, 1000, 1000, 1000};

	// Dynamic Creation / Storage of the structure
	lcd_pContent = (cnc_lcd_content_t*)malloc(sizeof(cnc_lcd_content_t));
	lcd_pContent->cmd_or_str = 0;
	lcd_pContent->lcd_xbits  = 4;
	lcd_pContent->lcd_nbchar = nb_cmd_Part1;
	lcd_pContent->lcd_ichar  = 0;
	lcd_pContent->lcd_istep  = 2;
	lcd_pContent->lcd_next   = NULL;
	// Allocate the structures for data and delay
	lcd_pContent->pData  = (char*)malloc(nb_cmd_Part1 * sizeof(char));
	for (int i=0; i<nb_cmd_Part1; i++) { lcd_pContent->pData[i]  = pInitPart1[i]; }
	lcd_pContent->pDelay = (uint32_t*)malloc(nb_cmd_Part1 * sizeof(uint32_t));
	for (int i=0; i<nb_cmd_Part1; i++) { lcd_pContent->pDelay[i] = pDelayPart1[i]; }

	// And Send the command
	cnc_lcd_i2c_sending_manager();

	//
	//		Step 2 : The LCD is now in 4-bits mode
	//
	int nb_cmd_Part2  		= 4;
	char pInitPart2[] 		= {0x28, 0x0F, 0x06, 0x01};
	uint32_t pDelayPart2[] 	= {1000, 1000, 1000, 10000};

	// Dynamic Creation / Call of the transmit function
	char *pCmdIn = (char*)malloc(nb_cmd_Part2 * sizeof(char));
	for (int i=0; i<nb_cmd_Part2; i++) { pCmdIn[i]  = pInitPart2[i]; }
	uint32_t* pDelayIn = (uint32_t*)malloc(nb_cmd_Part2 * sizeof(uint32_t));
	for (int i=0; i<nb_cmd_Part2; i++) { pDelayIn[i] = pDelayPart2[i]; }

	// And transmit the 8-bits command in 2 sets of 4...
	cnc_lcd_i2c_transmit_cmd8(nb_cmd_Part2, pCmdIn, pDelayIn);

	//
	//		Little welcoming message
	//
	cnc_lcd_i2c_goto(1,1);
	cnc_lcd_i2c_transmit_string(11," Init OK ! ");
}


//
//	5th Function : Select the Cursor Position
//
//  Command to be sent =  1 AC AC AC - AC AC AC AC
//								(with AC = Address Counter)
//		=> 0x80 = First 1 for the position set command
//		=> 0x40 = Second bit = 0 for 1st row (0x80) / 1 for 2nd row (0x80+0x40)
//		=> 0x20 & 0x10 = Memory, not used for display...
//		=> 0x00 to 0x0F = 16 bits displayed on the screen
//
void cnc_lcd_i2c_goto(uint8_t row, uint8_t col)
{
	// 8-bits version - dynamic allocation of the commands
	char *pCmd8 = (char*)malloc(sizeof(char));
	// Version 2 lines
	// *pCmd8 = (row == 2) ? (0x80 + 0x40 + col - 1) : (0x80 + col - 1);
	// Version 4 lines
	switch (row)
	{
		case 1:
			*pCmd8 = (0x80 + col - 1);
			break;
		case 2:
			*pCmd8 = (0x80 + 0x40 + col - 1);
			break;
		case 3:
			*pCmd8 = (0x80 + 20 + col - 1);
			break;
		case 4:
			*pCmd8 = (0x80 + 0x40 + 20 + col - 1);
			break;
		default:
			// If other, print on the 1st line
			*pCmd8 = (0x80 + col - 1);
	}
	cnc_lcd_i2c_transmit_cmd8(1, pCmd8, NULL);
}


//
//	6th Function : Clear the Display	(Command = 0x01)
//
void cnc_lcd_i2c_clear_display()
{
	// 8-bits version - dynamic allocation
	char *pCmd8 = (char*)malloc(sizeof(char));
	*pCmd8 = 0x01;
	uint32_t *pDelay = (uint32_t*)malloc(sizeof(uint32_t));
	*pDelay = 10000;
	cnc_lcd_i2c_transmit_cmd8(1, pCmd8, pDelay);
}


//
//	7th Function : Return Home			(Command = 0x02)
//
void cnc_lcd_i2c_return_home()
{
	// 8-bits version - dynamic allocation
	char *pCmd8 = (char*)malloc(sizeof(char));
	*pCmd8 = 0x02;
	uint32_t *pDelay = (uint32_t*)malloc(sizeof(uint32_t));
	*pDelay = 1000;
	cnc_lcd_i2c_transmit_cmd8(1, pCmd8, pDelay);
}


//
//	8th Function : Switch ON/OFF Display / Cursor / Cursor Blink
//			(Command = 0b 0000 1DCB)
//			D = Display (1 for ON / 0 for OFF)
//			C = Cursor  (1 for ON / 0 for OFF)
//			B = Blink   (1 for ON / 0 for OFF)
//
void cnc_lcd_i2c_set_DCB(int D, int C, int B)
{
	// 8-bits version - dynamic allocation
	char *pCmd8 = (char*)malloc(sizeof(char));
	*pCmd8 = 0x08 + (D!=0)*4 + (C!=0)*2 + (B!=0);
	cnc_lcd_i2c_transmit_cmd8(1, pCmd8, NULL);
}
