
/* */
/* Microfox Arduino nano version. Converted from PIC C November 2019 */
/* Jerry Boyd WB8WFK */
/* This controller will replace the Albuquerque VHF and HF microfox pic based controller */
/* */
/* */

#include "defs.h"
#include "linkbus.h"
/*#include <EEPROM.h> */

#define MAX_PATTERN_TEXT_LENGTH 20

volatile int seconds    = 0;    /* Init timer to first second. Set in ISR checked by main. */
volatile int minutes    = 1;    /* Init timer to cycle 1. */
volatile int g_fox      = 0;    /* Sets Fox number not set by ISR. Set in startup and checked in main. */
volatile int active     = 0;    /* Disable active. set and clear in ISR. Checked in main. */
volatile int EOC_KEY    = 0;    /* Used for end of cycle flag. Set in main is cleared in ISR and checked in main. */
volatile int demo       = 0;    /* used to put us in ARDF cycle mode sw = 6. */
volatile int foxO       = 0;    /* used to put us in fox o mode switch 7. */

/***********************************************************************
 * Global Variables & String Constants
 *
 * Identify each global with a "g_" prefix
 * Whenever possible limit globals' scope to this file using "static"
 * Use "volatile" for globals shared between ISRs and foreground
 ************************************************************************/
static volatile EC g_last_error_code = ERROR_CODE_NO_ERROR;
static BOOL EEMEM ee_interface_eeprom_initialization_flag = EEPROM_UNINITIALIZED;
static char EEMEM ee_stationID_text[MAX_PATTERN_TEXT_LENGTH + 1];
static char EEMEM ee_pattern_text[MAX_PATTERN_TEXT_LENGTH + 1];
static uint8_t EEMEM ee_pattern_codespeed;
static uint8_t EEMEM ee_id_codespeed;
static uint16_t EEMEM ee_ID_time;
static uint16_t EEMEM ee_clock_calibration;
static uint8_t EEMEM ee_override_DIP_switches;
static uint8_t EEMEM ee_enable_LEDs;

static char g_messages_text[2][MAX_PATTERN_TEXT_LENGTH + 1] = { "\0", "\0" };
static volatile uint8_t g_id_codespeed = EEPROM_ID_CODE_SPEED_DEFAULT;
static volatile uint8_t g_pattern_codespeed = EEPROM_PATTERN_CODE_SPEED_DEFAULT;
static volatile uint16_t g_time_needed_for_ID = 0;
static volatile int16_t g_ID_period_seconds = EEPROM_ID_TIME_INTERVAL_DEFAULT;  /* amount of time between ID/callsign transmissions */
static volatile uint16_t g_clock_calibration = EEPROM_CLOCK_CALIBRATION_DEFAULT;
static volatile uint8_t g_override_DIP_switches = EEPROM_OVERRIDE_DIP_SW_DEFAULT;
static volatile uint8_t g_enable_LEDs;

static char g_tempStr[21] = { '\0' };

static volatile uint8_t g_start_override = FALSE;

/*
 * Function Prototypes
 */
void handleLinkBusMsgs(void);
void initializeEEPROMVars(void);
void saveAllEEPROM(void);
void transmitString(char msg[], int speed);
void word_space(int speed);
int codeBits(char alphaNum);
void (* resetFunc) (void) = 0;  /*declare reset function @ address 0 */

void setup()
{
	initializeEEPROMVars(); /* Initialize variables stored in EEPROM */
	linkbus_init(57600);    /* Start the Link Bus serial comms */
/*  Serial.begin(57600);  // debug - this might not work with the Link Bus enabled or vice versa */
	linkbus_send_text((char*)"\n\nStored Data:\n");
	sprintf(g_tempStr, "  ID: %s\n", g_messages_text[STATION_ID]);
	linkbus_send_text(g_tempStr);
	sprintf(g_tempStr, "  CAL: %u\n", g_clock_calibration);
	linkbus_send_text(g_tempStr);
	sprintf(g_tempStr, "  DIP: %u\n", g_override_DIP_switches);
	linkbus_send_text(g_tempStr);
	sprintf(g_tempStr, "  LED: %s\n", g_enable_LEDs == TRUE ? "ON" : "OFF");
	linkbus_send_text(g_tempStr);
/* put your setup code here, to run once:
 * set pins as outputs and inputs */
	pinMode(13, OUTPUT);    /* The nano amber LED
	                         * This led blinks when off cycle and
	                         * blinks with code when on cycle. */
	pinMode(2, OUTPUT);     /* This pin is used to control the KEY line to the transmittter
	                         * only active on cycle. */
/* set the pins that are inputs */
	pinMode(3, INPUT);      /* This pin is used as the sync line
	                         * NOTE: The original albq PIC controllers used the CPU reset line as the
	                         * sync line. We had issues with transmitters getting out of sync during transport.
	                         * later investigation found that ESD events during transport was resetting the
	                         * PIC. This code will read this I/O line for sync and after finding that the I/O line
	                         * has switched to the high state it never reads it again until a power cycle.
	                         * The Arduino reset line does not go off board. ESD event caused out of sync issue fixed. */
	pinMode(4, INPUT);      /* fox switch LSB */
	pinMode(5, INPUT);      /* fox switch middle bit */
	pinMode(6, INPUT);      /* fox switch MSB */


/* set up interrupts and serial */


/*
 * read the dip switches and compute the desired fox number
 * */
	if(g_override_DIP_switches)
	{
		g_fox = g_override_DIP_switches;
	}
	else
	{
		if(digitalRead(4) == HIGH ) /*Lsb */
		{
			g_fox++;
		}
		if(digitalRead(5) == HIGH ) /* middle bit */
		{
			g_fox += 2;
		}
		if(digitalRead(6) == HIGH ) /* MSB */
		{
			g_fox += 4;
		}
	}

	if(g_fox == 6 )         /* trap for invalid > 6 fox */
	{
		demo = 1;           /* force to demo mode */
		g_fox = 1;          /* start with MOE */
	}

	if(g_fox == 7 )         /* put us in fox O mode */
	{
		foxO = 1;           /* force to fox O mode */
	}

	cli();                  /*stop interrupts for setup */

	digitalWrite(13, LOW);  /* Turn off led sync switch is now open */

/*
 * get ready set go
 * time to start fox operation
 *
 * The timer came from the below source and I modified it
 * for use with microfox. it is the 1 second core timer
 * this replaced the PIC timer as the hardware is different.
 * ******************************************************
 * timer interrupts
 * by Amanda Ghassaei
 * June 2012 */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 */
/*
 * Modifications to the time for microfox done by Jerry Boyd WB8WFK November 2019
 * Removed the unused timers we only use timer 1.
 *
 * timer setup for timer1,
 * For Arduino uno or any board with ATMEL 328/168.. diecimila, duemilanove, lilypad, nano, mini...
 * this code will enable arduino timer 1 interrupts.
 * timer1 will interrupt at 1Hz
 * master variables
 * set timer1 interrupt at 1Hz */
	TCCR1A = 0; /* set entire TCCR1A register to 0 */
	TCCR1B = 0; /* same for TCCR1B */
	TCNT1 = 0;  /*initialize counter value to 0 */
/* Set compare match register for 1hz increments
 ************************************************************
 **
 **
 ** USE THIS TO FIX BOARD PROCESSOR CLOCK ERROR
 **
 **
 ************************************************************
 * first testing found bad drift relative to a gps stable clock (iphone timer) is was 10 seconds in about 50 minutes
 * Today I measured the Arduino nano 16 Mhz clock and it was 16.050 MHz. Yes 50 Khz high!!
 * OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)    //was 15624 for 16.000 MHz on center frequency clock
 * OCR1A = 15672;// = (16.043*10^6) / (1*1024) - 1 (must be <65536)   //15672 computed for + 16.050 MHz off frequency clock board 4
 * OCR1A = 15661;// = (16.0386*10^6) / (1*1024) - 1 (must be <65536)   //15661 computed for + 16.0386 MHz off frequency clock board 3 */
/*	OCR1A = 15629;/ * = (16.006*10^6) / (1*1024) - 1 (must be <65536)   //15629 computed for + 16.006 MHz off frequency clock board 3 * / */
	OCR1A = g_clock_calibration;

/* turn on CTC mode */
	TCCR1B |= (1 << WGM12);
/* Set CS11 bit for 1024 prescaler */
	TCCR1B |= (1 << CS12) | (1 << CS10);
/* enable timer compare interrupt */
	TIMSK1 |= (1 << OCIE1A);
	sei();  /*allow interrupts. Arm and run */

/* I found this on the web
 * note Timer0 - An 8 bit timer used by Arduino functions delay(), millis() and micros().
 *      Timer1 - A 16 bit timer used by the Servo() library
 *      Timer2 - An 8 bit timer used by the Tone() library */
/*
 * we now look for the sync line and then arm the timer interrupts
 * when the sync switch is released.
 * */

	linkbus_send_text((char*)"Waiting for sync.\n");
	linkbus_send_text((char*)"Type \"$GO;\"\n");

	while(( digitalRead(3) == LOW) && !g_start_override)
	{
		if(g_enable_LEDs)
		{
			digitalWrite(13, HIGH); /* arduino nano LED
		                             * turn on led to show sync switch is closed */
		}
		handleLinkBusMsgs();
	}

	g_start_override = TRUE;

#ifndef COMPILE_FOR_ARDUINO
		loop();
#endif
}/*end setup */


/***********************************************************************
 * USART Rx Interrupt ISR
 *
 * This ISR is responsible for reading characters from the USART
 * receive buffer to implement the Linkbus.
 *
 *      Message format:
 *              $id,f1,f2... fn;
 *              where
 *                      id = Linkbus MessageID
 *                      fn = variable length fields
 *                      ; = end of message flag
 *
 ************************************************************************/
ISR(USART_RX_vect)
{
	static LinkbusRxBuffer* buff = NULL;
	static uint8_t charIndex = 0;
	static uint8_t field_index = 0;
	static uint8_t field_len = 0;
	static uint32_t msg_ID = 0;
	static BOOL receiving_msg = FALSE;
	uint8_t rx_char;

	rx_char = UDR0;

	if(!buff)
	{
		buff = nextEmptyRxBuffer();
	}

	if(buff)
	{
		rx_char = toupper(rx_char);
		SMCR = 0x00;                                /* exit power-down mode */

		if((rx_char == '$') || (rx_char == '!'))    /* start of new message = $ */
		{
			charIndex = 0;
			buff->type = (rx_char == '!') ? LINKBUS_MSG_REPLY : LINKBUS_MSG_COMMAND;
			field_len = 0;
			msg_ID = LINKBUS_MSG_UNKNOWN;
			receiving_msg = TRUE;

			/* Empty the field buffers */
			for(field_index = 0; field_index < LINKBUS_MAX_MSG_NUMBER_OF_FIELDS; field_index++)
			{
				buff->fields[field_index][0] = '\0';
			}

			field_index = 0;
		}
		else if(receiving_msg)
		{
			if((rx_char == ',') || (rx_char == ';') || (rx_char == '?'))    /* new field = ,; end of message = ; */
			{
				/* if(field_index == 0) // message ID received */
				if(field_index > 0)
				{
					buff->fields[field_index - 1][field_len] = 0;
				}

				field_index++;
				field_len = 0;

				if(rx_char == ';')
				{
					if(charIndex >= LINKBUS_MIN_MSG_LENGTH)
					{
						buff->id = (LBMessageID)msg_ID;
					}
					receiving_msg = FALSE;
				}
				else if(rx_char == '?')
				{
					buff->type = LINKBUS_MSG_QUERY;
					if(charIndex > LINKBUS_MIN_MSG_LENGTH)
					{
						buff->id = (LBMessageID)msg_ID;
					}
					receiving_msg = FALSE;
				}

				if(!receiving_msg)
				{
					buff = 0;
				}
			}
			else
			{
				if(field_index == 0)    /* message ID received */
				{
					msg_ID = msg_ID * 10 + rx_char;
				}
				else
				{
					buff->fields[field_index - 1][field_len++] = rx_char;
				}
			}
		}
		else if(rx_char == 0x0D)    /* Handle carriage return */
		{
			buff->id = (LBMessageID)LINKBUS_MSG_UNKNOWN;
			charIndex = LINKBUS_MAX_MSG_LENGTH;
			field_len = 0;
			msg_ID = LINKBUS_MSG_UNKNOWN;
			field_index = 0;
			buff = NULL;
		}

		if(++charIndex >= LINKBUS_MAX_MSG_LENGTH)
		{
			receiving_msg = FALSE;
			charIndex = 0;
		}
	}
}


/***********************************************************************
 * USART Tx UDRE ISR
 *
 * This ISR is responsible for filling the USART transmit buffer. It
 * implements the transmit function of the Linkbus.
 *
 ************************************************************************/
ISR(USART_UDRE_vect)
{
	static LinkbusTxBuffer* buff = 0;
	static uint8_t charIndex = 0;

	if(!buff)
	{
		buff = nextFullTxBuffer();
	}

	if((*buff)[charIndex])
	{
		/* Put data into buffer, sends the data */
		UDR0 = (*buff)[charIndex++];
	}
	else
	{
		charIndex = 0;
		(*buff)[0] = '\0';
		buff = nextFullTxBuffer();
		if(!buff)
		{
			linkbus_end_tx();
		}
	}
}



/*
 *
 * here is our main ISR for the ARDF timer
 * modified from ISR example for microfox by Jerry Boyd WB8WFK
 * this runs once a second and generates the cycle and sets control flags for the main controller.
 * */

ISR(TIMER1_COMPA_vect)                  /*timer1 interrupt 1Hz */

{/*This is the ARDF core timer */
	++seconds;                          /* one second event - update seconds */

/*
 * Code to make home transmitter (MO) cycle work
 * this lets transmitter go off for a second and then restart
 * by override of the timer in the MO mode.
 * */
	if((g_fox == 0) && (minutes > 0))   /* check for override for home transmitter mode
	                                     * all other ARDF transmitters use 1,2,3,4,and 5 */
	{
		minutes = 0;                    /* override timer */
	}

/* normal fox mode (MOx cycle)
 * check seconds and update minutes */
	if(seconds == 59)
	{
		++minutes;      /* make minutes adjustment */
		EOC_KEY = 0;    /* flag to kick out of end cycle long tone keydown */
	}
	if(seconds == 60)
	{

		seconds = 0;    /* set seconds back to 0 */
	}
	/* adjust timer for cycle */
	if(minutes == 6)    /* adjust counter for ARDF cycle */
	{
		minutes = 1;    /* set for first transmitter */
	}

/******************************************************
 *
 * Code below enables ARDF cycle based on Fox number
 *
 ****************************************************** */

	if(g_fox == minutes)    /* test for active ARDF cycle */
	{
		active = 1;         /* active is a flag that the main state machine uses */
	}
	else
	{
		active = 0;
	}
}   /* end of timer ISR */



/****************************************************
**
** Function to send a CW message
**
**************************************************** */
void  Send_Message(int imessage, int speed)
{
	/* Morse Code timing
	 * Dit              = 1 time unit (reference)
	 * Dah              = 3 time units
	 * Symbol space     = 1 time units
	 * letter Space     = 3 time units
	 * Word space       = 7 time units
	 *{ */
	int time;

	if(imessage == ' ')
	{
		word_space(speed);
	}
	else
	{
		while(1)
		{

			if((imessage & 0X01 ) == 0x01)
			{
				if(g_enable_LEDs)
				{
					digitalWrite(13, HIGH); /*  nano LED */
				}
				digitalWrite(2, HIGH);      /* TX key line */
				time = speed * 3;
				delay(time);
				digitalWrite(13, LOW);      /* arduino nano LED */
				digitalWrite(2, LOW);       /* TX key line */

			}
			else
			{
				/*OUTPUT_BIT(CW_KEY, 1);  // Dit */
				if(g_enable_LEDs)
				{
					digitalWrite(13, HIGH); /* arduino nano LED */
				}
				digitalWrite(2, HIGH);      /* TX key line */
				delay(speed);

				digitalWrite(13, LOW);      /* arduino nano LED */
				digitalWrite(2, LOW);       /* TX key line */
			}

			imessage = imessage >> 1;       /* shift right one bit */
			if(imessage == 1)
			{
				break;                      /* break the loop */
			}
			/*Do a symbol space */
			delay(speed);                   /*Symbol space */
		}

		/* Do a letter space */
		time = speed * 3;                   /* */
		delay(time);                        /*inter character (letter) space */
	}
}



/******************************************
**
** Function to do a word space based on selected speed
**
****************************************** */
void word_space(int speed)

{
	int time;

	time =  speed * 7;
	delay(time);

	return;
}



/*
 *
 * here is the main microfox code controller by Jerry Boyd WB8WFK
 *
 * */
void loop()
{
	/* put your main code here, to run repeatedly: */

	int message, id;
	int speed;
	int fast_speed = 50;        /* for last part of cycle */
	int slow_speed = 80;        /* for first 30 seconds of cycle */
	int very_fast_speed = 45;   /* for CW ID */

/*
 *
 * F O X  O   MODE
 *
 * This will place us in fox O mode
 * FOX O mode */
	if(foxO == 1)
	{
		while(1)
		{
			id = 1;     /* id flag */
			speed = 55;
			/* do the CW ID for fox O */
			if(id == 1) /* turns on ID for US version */
			{
				/* send transmitter call sign
				 * call sign WB8WFK
				 * */
				transmitString(g_messages_text[STATION_ID], speed);
				id = 0; /* clear ID Flag */
			}           /* end of CW ID space */


			speed = 70;
			while(1)
			{

				message = 0x02; /* E */
				Send_Message(message, speed);
				word_space(speed);
				word_space(speed);
				if(seconds > 58)
				{
					id = 1;
					break;
				}
				handleLinkBusMsgs();

			}


		}

		/***********************************************************************
		 *  Handle arriving Linkbus messages
		 ************************************************************************/
	}                               /* end of the fox O loop */


/*
 * A R D F cycles MO and DEMO
 * */
	while(1)                        /* Start of the standard ARDF and demo mode loop */
	{
		/*get ready for first pass
		 * */
		id    = 1;                  /* set first time pass flag */
		speed = very_fast_speed;    /* super Fast ID CW speed */
		/* ID real fast to get past the ID ASAP
		 *
		 * ID the transmitter with the call sign
		 * */

		while(active)
		{
			/*Serial.println(seconds);
			 * Serial.println(minutes);
			 * Serial.println(active); */
			if(id == 1)                 /* turns on ID for US version */
			{
				/* send transmitter call sign
				 * call sign WB8WFK
				 * */
				transmitString(g_messages_text[STATION_ID], speed);
				id = 0;                 /* clear ID Flag */
			}                           /* end of CW ID space */

			/*************************************************************
			**
			**          now send MOX based on Fox number
			**
			************************************************************* */
			speed = slow_speed;         /* Set Slow CW speed for non hams // was 100
			                             * I.E. boy scouts, school groups
			                             * and Orinereering clubs.
			                             * */
			word_space(speed);          /* have a gap between CW ID and first Mox */
			/*
			 * Main MOX send loop
			 * */
			while( seconds < 50)
			{
				if(seconds > 30)        /* do we switch CW speed to high speed for hams */
				{                       /* at 1/2 cycle point */
					speed = fast_speed; /* Yes set fast CW speed for balance of cycle was 55 for pic */
				}                       /* for hams. */

				switch(g_fox)           /* select the FOX (MOX) to send */
				{
					case 0:             /* Fox number home */
					{
						/* Send MO */
						message = 0x07; /* M */
						Send_Message(message, speed);
						word_space(speed);
						message = 0x0F; /* O */
						Send_Message(message, speed);
						word_space(speed);
						handleLinkBusMsgs();
					}
					break;

					case 1:             /* Fox number 1 */
					{
						/* Send MOE */
						message = 0x07; /* M */
						Send_Message(message, speed);
						word_space(speed);
						message = 0x0F; /* O */
						Send_Message(message, speed);
						word_space(speed);
						message = 0x02; /* E */
						Send_Message(message, speed);
						word_space(speed);
						handleLinkBusMsgs();
					}
					break;

					case 2:             /* Fox number 2 */
					{
						/* Send MOI */
						message = 0x07; /* M */
						Send_Message(message, speed);
						word_space(speed);
						message = 0x0F; /* O */
						Send_Message(message, speed);
						word_space(speed);
						message = 0x04; /* I */
						Send_Message(message, speed);
						word_space(speed);
						handleLinkBusMsgs();
					}
					break;

					case 3:             /* Fox number 3 */
					{
						/* Send MOS */
						message = 0x07; /* M */
						Send_Message(message, speed);
						word_space(speed);
						message = 0x0F; /* O */
						Send_Message(message, speed);
						word_space(speed);
						message = 0x08; /* S */
						Send_Message(message, speed);
						word_space(speed);
						handleLinkBusMsgs();
					}
					break;

					case 4:             /* Fox number 4 */
					{
						/* Send MOH */
						message = 0x07; /* M */
						Send_Message(message, speed);
						word_space(speed);
						message = 0x0F; /* O */
						Send_Message(message, speed);
						word_space(speed);
						message = 0x10; /* H */
						Send_Message(message, speed);
						word_space(speed);
						handleLinkBusMsgs();
					}
					break;

					case 5:             /* Fox number 5 */
					{
						/* Send MO5 */
						message = 0x07; /* M */
						Send_Message(message, speed);
						word_space(speed);
						message = 0x0F; /* O */
						Send_Message(message, speed);
						word_space(speed);
						message = 0x20; /* 5 */
						Send_Message(message, speed);
						word_space(speed);
						handleLinkBusMsgs();
					}
					break;
				}                                   /* end of fox # case test */
			}                                       /* end of mox loop */

			if((seconds > 50) && (seconds < 55))    /* a window to enter */
			{
				/* Signal the end of the cycle
				 * send a letter C then do the end of cycle tone */
				message = 0x15;                     /*letter  C */
				Send_Message(message, speed);
				word_space(speed);
				EOC_KEY = 1;                        /* force a key down for end of cycle */
				/*
				 * To do. For the Arduino version its possable to add a low battery monitor
				 * that will turn off the long tone at the end of the cycle to
				 * aleart the event crew to swap out battery.
				 * */

				/*******************************************
				**
				**      send the long tone at the
				**      end of an ARDF cycle
				**
				******************************************* */
				if(g_enable_LEDs)
				{
					digitalWrite(13, HIGH); /*arduino turn on end of cycle LED */
				}
				digitalWrite(2, HIGH);      /*TX key line. Force a EOC key down */
				while(EOC_KEY)
				{
					/* key down transmitter for end of cycle tone after letter C
					 * KEYdown was set before entering this loop
					 * just spend some time here until done */

					handleLinkBusMsgs();
				}

				digitalWrite(13, LOW);  /* arduino turn off end of cycle LED */
				digitalWrite(2, LOW);   /* turn off TX key line */
			}                           /* end of > 50 second checker */
		}                               /* end of while active loop */

		/* a trap to walk us through demo mode by cycling fox number */
		if(demo == 1)
		{
			g_fox++;        /* cycle to next one */
			if(g_fox > 5)
			{
				g_fox = 1;  /*reset to 1 */
			}
		}

		/* below will flash LED when offcycle for a heartbeat indicator */
		if(g_enable_LEDs)
		{
			delay(1000);            /* delay between offf cycle flash events */
			digitalWrite(13, HIGH); /* arduino flash led when off cycle turn it on */
			delay(100);             /* led on time */
			digitalWrite(13, LOW);  /* arduino flash led when off cycle turn it off */
		}

		handleLinkBusMsgs();
	}                               /* end of while 1 loop */
}

int codeBits(char alphaNum)
{
	switch(alphaNum)
	{
		case ' ':
		{
			return( ' ');
		}

		case '0':   /* 0 -----         00111111   0x3F */
		{ return( 0x3F); }

		case '1':   /* 1 .----         00111110   0x3E */
		{ return( 0x3E); }

		case '2':   /* 2 ..---         00111100   0x3C */
		{ return( 0x3C); }

		case '3':   /* 3 ...--         00111000   0x38 */
		{ return( 0x38); }

		case '4':   /* 4 ....-         00110000   0x30 */
		{ return( 0x30); }

		case '5':   /* 5 .....         00100000   0x20 */
		{ return( 0x20); }

		case '6':   /* 6 -....         00100001   0x21 */
		{ return( 0x21); }

		case '7':   /* 7 --...         00100011   0x23 */
		{ return( 0x23); }

		case '8':   /* 8 ---..         00100111   0x27 */
		{ return( 0x27); }

		case '9':   /* 9 ----.         00101111   0x2F */
		{ return( 0x2F); }

		case 'A':   /* A .-            00000110   0x06 */
		{ return( 0x06); }

		case 'B':   /* B -...          00010001   0x11 */
		{ return( 0x11); }

		case 'C':   /* C -.-.          00010101   0x15 */
		{ return( 0x15); }

		case 'D':   /* D -..           00001001   0x09 */
		{ return( 0x09); }

		case 'E':   /* E .             00000010   0x02 */
		{ return( 0x02); }

		case 'F':   /* F ..-.          00010100   0x14 */
		{ return( 0x14); }

		case 'G':   /* G --.           00001011   0x0B */
		{ return( 0x0B); }

		case 'H':   /* H ....          00010000   0x10 */
		{ return( 0x10); }

		case 'I':   /* I ..            00000100   0x04 */
		{ return( 0x04); }

		case 'J':   /* J .---          00011110   0x1E */
		{ return( 0x1E); }

		case 'K':   /* K -.-           00001101   0x0D */
		{ return( 0x0D); }

		case 'L':   /* L .-..          00010010   0x12 */
		{ return( 0x12); }

		case 'M':   /* M --            00000111   0x07 */
		{ return( 0x07); }

		case 'N':   /* N -.            00000101   0x05 */
		{ return( 0x05); }

		case 'O':   /* O ---           00001111   0x0F */
		{ return( 0x0F); }

		case 'P':   /* P .--.          00010110   0x16 */
		{ return( 0x16); }

		case 'Q':   /* Q --.-          00011011   0x1B */
		{ return( 0x1B); }

		case 'R':   /* R .-.           00001010   0x0A */
		{ return( 0x0A); }

		case 'S':   /* S ...           00001000   0x08 */
		{ return( 0x08); }

		case 'T':   /* T -             00000011   0x03 */
		{ return( 0x03); }

		case 'U':   /* U ..-           00001100   0x0C */
		{ return( 0x0C); }

		case 'V':   /* V ...-          00011000   0x18 */
		{ return( 0x18); }

		case 'W':   /* W .--           00001110   0x0E */
		{ return( 0x0E); }

		case 'X':   /* X -..-          00011001   0x19 */
		{ return( 0x19); }

		case 'Y':   /* Y -.--          00011101   0x1D */
		{ return( 0x1D); }

		case 'Z':   /* Z --..          00010011   0x13 */
		{ return( 0x13); }

		case '?':   /* ? ..--..        01001100   0x49   Question mark */
		{ return( 0x49); }

		default:
			return( 0);
	}
}

/* The compiler does not seem to optimize large switch statements correctly */
void __attribute__((optimize("O0"))) handleLinkBusMsgs()
{
	LinkbusRxBuffer* lb_buff;
	BOOL send_ack = TRUE;

	while((lb_buff = nextFullRxBuffer()))
	{
		LBMessageID msg_id = lb_buff->id;

		switch(msg_id)
		{
			case MESSAGE_WIFI:
			{
				/* Your code goes here */
			}
			break;

			case MESSAGE_RESET:
			{
#ifdef USE_WATCHDOG
					wdt_init(WD_FORCE_RESET);
					while(1)
					{
						;
					}
#endif  /* USE_WATCHDOG */
				resetFunc();                                                                                                                                 /*call reset */
			}
			break;

			case MESSAGE_OVERRIDE_DIP:
			{
				if(lb_buff->fields[FIELD1][0])
				{
					int d = atoi(lb_buff->fields[FIELD1]);

					if((d >= 0) && (d <= 7))
					{
						g_override_DIP_switches = d;
					}

					saveAllEEPROM();
				}

				sprintf(g_tempStr, "DIP=%u\n", g_override_DIP_switches);
				linkbus_send_text(g_tempStr);
				send_ack = FALSE;
			}
			break;

			case MESSAGE_LEDS:
			{
				if(lb_buff->fields[FIELD1][0])
				{
					if((lb_buff->fields[FIELD1][1] == 'F') || (lb_buff->fields[FIELD1][0] == '0'))
					{
						g_enable_LEDs = FALSE;
					}
					else
					{
						g_enable_LEDs = TRUE;
					}

					saveAllEEPROM();
				}

				sprintf(g_tempStr, "LED:%s\n", g_enable_LEDs ? "ON" : "OFF");
				linkbus_send_text(g_tempStr);
				send_ack = FALSE;
			}
			break;

			case MESSAGE_TX_POWER:
			{
				/* Your code goes here */
			}
			break;

			case MESSAGE_PERM:
			{
				saveAllEEPROM();
			}
			break;

			case MESSAGE_GO:
			{
				if(g_start_override)
				{
					linkbus_send_text((char*)"Already synced!\n");
				}
				else
				{
					g_start_override = TRUE;
					linkbus_send_text((char*)"We're off!\n");
				}
				send_ack = FALSE;
			}
			break;

			case MESSAGE_FACTORY_RESET:
			{
				uint8_t flag = EEPROM_INITIALIZED_FLAG + 1;
				eeprom_write_byte(&ee_interface_eeprom_initialization_flag, flag);
				resetFunc();    /*call reset */
			}
			break;

			case MESSAGE_CLOCK_CAL:
			{
				if(lb_buff->fields[FIELD1][0])
				{
					uint16_t c = (uint16_t)atoi(lb_buff->fields[FIELD1]);

					if(c > 100)
					{
						g_clock_calibration = c;
						OCR1A = g_clock_calibration;
					}

					saveAllEEPROM();
				}

				sprintf(g_tempStr, "Cal=%u\n", g_clock_calibration);
				linkbus_send_text(g_tempStr);
				send_ack = FALSE;
			}
			break;

			case MESSAGE_SET_STATION_ID:
			{
				if(lb_buff->fields[FIELD1][0])
				{
					strncpy(g_messages_text[STATION_ID], lb_buff->fields[FIELD1], MAX_PATTERN_TEXT_LENGTH);

					if(g_messages_text[STATION_ID][0])
					{
/*            g_time_needed_for_ID = (500 + timeRequiredToSendStrAtWPM(g_messages_text[STATION_ID], g_id_codespeed)) / 1000; */
					}

					saveAllEEPROM();
				}

				sprintf(g_tempStr, "ID:%s\n", g_messages_text[STATION_ID]);
				linkbus_send_text(g_tempStr);
				send_ack = FALSE;
			}
			break;

			case MESSAGE_CODE_SPEED:
			{
				/* Your code goes here */
			}
			break;

			case MESSAGE_TIME_INTERVAL:
			{
				/* Your code goes here */
			}
			break;

			case MESSAGE_SET_PATTERN:
			{
				if(lb_buff->fields[FIELD1][0])
				{
					strncpy(g_messages_text[PATTERN_TEXT], lb_buff->fields[FIELD1], MAX_PATTERN_TEXT_LENGTH);
				}
			}
			break;

			case MESSAGE_BAND:
			{
				/* Your code goes here */
			}
			break;

			case MESSAGE_BAT:
			{
				/* Your code goes here */
			}
			break;

			case MESSAGE_TEMP:
			{
				/* Your code goes here */
			}
			break;

			default:
			{
				linkbus_reset_rx(); /* flush buffer */
				g_last_error_code = ERROR_CODE_ILLEGAL_COMMAND_RCVD;
			}
			break;
		}

		lb_buff->id = (LBMessageID)MESSAGE_EMPTY;
		if(send_ack)
		{
			linkbus_send_text((char*)MESSAGE_ACK);
		}
	}
}

void transmitString(char msg[], int speed)
{
	int len = strlen(msg);

	for(int i = 0; i < len; i++)
	{
		Send_Message(codeBits(msg[i]), speed);
		word_space(speed);
	}
}


/*
 * Set EEPROM to its default values
 */
void initializeEEPROMVars()
{
	uint8_t i;

	if(eeprom_read_byte(&ee_interface_eeprom_initialization_flag) == EEPROM_INITIALIZED_FLAG)
	{
		g_pattern_codespeed = eeprom_read_byte(&ee_pattern_codespeed);
		g_id_codespeed = eeprom_read_byte(&ee_id_codespeed);
		g_ID_period_seconds = eeprom_read_word(&ee_ID_time);
		g_clock_calibration = eeprom_read_word(&ee_clock_calibration);
		g_override_DIP_switches = eeprom_read_byte(&ee_override_DIP_switches);
		g_enable_LEDs = eeprom_read_byte(&ee_enable_LEDs);

		for(i = 0; i < 20; i++)
		{
			g_messages_text[STATION_ID][i] = (char)eeprom_read_byte((uint8_t*)(&ee_stationID_text[i]));
			if(!g_messages_text[STATION_ID][i])
			{
				break;
			}
		}

		for(i = 0; i < 20; i++)
		{
			g_messages_text[PATTERN_TEXT][i] = (char)eeprom_read_byte((uint8_t*)(&ee_pattern_text[i]));
			if(!g_messages_text[PATTERN_TEXT][i])
			{
				break;
			}
		}
	}
	else
	{
		g_id_codespeed = EEPROM_ID_CODE_SPEED_DEFAULT;
		g_pattern_codespeed = EEPROM_PATTERN_CODE_SPEED_DEFAULT;
		g_ID_period_seconds = EEPROM_ID_TIME_INTERVAL_DEFAULT;
		g_clock_calibration = EEPROM_CLOCK_CALIBRATION_DEFAULT;
		g_override_DIP_switches = EEPROM_OVERRIDE_DIP_SW_DEFAULT;
		g_enable_LEDs = EEPROM_ENABLE_LEDS_DEFAULT;
		strncpy(g_messages_text[STATION_ID], EEPROM_STATION_ID_DEFAULT, MAX_PATTERN_TEXT_LENGTH);
		strncpy(g_messages_text[PATTERN_TEXT], EEPROM_PATTERN_TEXT_DEFAULT, MAX_PATTERN_TEXT_LENGTH);
		saveAllEEPROM();
		eeprom_write_byte(&ee_interface_eeprom_initialization_flag, EEPROM_INITIALIZED_FLAG);
	}
}

/*
 * Save all non-volatile values to EEPROM
 */
void saveAllEEPROM()
{
	uint8_t i;

	eeprom_update_byte(&ee_id_codespeed, g_id_codespeed);
	eeprom_update_byte(&ee_pattern_codespeed, g_pattern_codespeed);
	eeprom_update_word(&ee_ID_time, g_ID_period_seconds);
	eeprom_update_word(&ee_clock_calibration, g_clock_calibration);
	eeprom_update_byte(&ee_override_DIP_switches, g_override_DIP_switches);
	eeprom_update_byte(&ee_enable_LEDs, g_enable_LEDs);

	for(i = 0; i < strlen(g_messages_text[STATION_ID]); i++)
	{
		eeprom_update_byte((uint8_t*)&ee_stationID_text[i], (uint8_t)g_messages_text[STATION_ID][i]);
	}

	eeprom_update_byte((uint8_t*)&ee_stationID_text[i], 0);

	for(i = 0; i < strlen(g_messages_text[PATTERN_TEXT]); i++)
	{
		eeprom_update_byte((uint8_t*)&ee_pattern_text[i], (uint8_t)g_messages_text[PATTERN_TEXT][i]);
	}

	eeprom_update_byte((uint8_t*)&ee_pattern_text[i], 0);
}

