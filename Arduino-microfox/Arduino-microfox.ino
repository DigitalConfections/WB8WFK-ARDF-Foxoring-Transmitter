/*
 * Microfox Arduino nano version. Converted from PIC C November 2019
 * Jerry Boyd WB8WFK
 * This controller will replace the Albuquerque VHF and HF microfox pic based controller
 *
 * */


#include "defs.h"
#include "linkbus.h"
#include "morse.h"

#ifdef COMPILE_FOR_ATMELSTUDIO7
#include <avr/io.h>
#include <avr/eeprom.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "delay.h"
#include "ardooweeno.h"
#endif  /* COMPILE_FOR_ATMELSTUDIO7 */


#define MAX_PATTERN_TEXT_LENGTH 20

volatile int g_seconds          = 0;            /* Init timer to first second. Set in ISR checked by main. */
volatile int g_minutes          = 1;            /* Init timer to cycle 1. */
volatile int32_t g_seconds_since_sync = 0;      /* Total elapsed time counter */
FoxType g_fox          = BEACON;                /* Sets Fox number not set by ISR. Set in startup and checked in main. */
volatile int g_active           = 0;            /* Disable active. set and clear in ISR. Checked in main. */

volatile int g_on_the_air               = 0;    /* Controls transmitter Morse activity */
volatile int g_code_throttle    = 0;            /* Adjusts Morse code speed */
const char g_morsePatterns[][6] = { "MO ", "MOE ", "MOI ", "MOS ", "MOH ", "MO5 ", "", "5" };
volatile BOOL g_callsign_sent = FALSE;

volatile BOOL g_blinky_time = FALSE;

#ifndef COMPILE_FOR_ATMELSTUDIO7
	FoxType& operator++ (FoxType & orig)
	{
		orig = static_cast < FoxType > (orig + 1);  /* static_cast required because enum + int -> int */
		if(orig > INVALID_FOX)
		{
			orig = INVALID_FOX;
		}
		return( orig);
	}

	FoxType operator++ (FoxType & orig, int)
	{
		FoxType rVal = orig;
		++orig;
		return( rVal);
	}

	FoxType& operator += (FoxType & a, int b)
	{
		return( a = static_cast < FoxType > (a + b));
	}
#endif  /* COMPILE_FOR_ATMELSTUDIO7 */


/***********************************************************************
 * Global Variables & String Constants
 *
 * Identify each global with a "g_" prefix
 * Whenever possible limit globals' scope to this file using "static"
 * Use "volatile" for globals shared between ISRs and foreground
 ************************************************************************/
static BOOL EEMEM ee_interface_eeprom_initialization_flag = EEPROM_UNINITIALIZED;
static char EEMEM ee_stationID_text[MAX_PATTERN_TEXT_LENGTH + 1];
static char EEMEM ee_pattern_text[MAX_PATTERN_TEXT_LENGTH + 1];
static uint8_t EEMEM ee_pattern_codespeed;
static uint8_t EEMEM ee_id_codespeed;
static uint16_t EEMEM ee_ID_time;
static uint16_t EEMEM ee_clock_calibration;
static uint8_t EEMEM ee_override_DIP_switches;
static uint8_t EEMEM ee_enable_LEDs;
static uint8_t EEMEM ee_enable_sync;
static int16_t EEMEM ee_temp_calibration;

static char g_messages_text[2][MAX_PATTERN_TEXT_LENGTH + 1] = { "\0", "\0" };
static volatile uint8_t g_id_codespeed = EEPROM_ID_CODE_SPEED_DEFAULT;
static volatile uint8_t g_pattern_codespeed = EEPROM_PATTERN_CODE_SPEED_DEFAULT;
static volatile uint16_t g_time_needed_for_ID = 0;
static volatile int16_t g_ID_period_seconds = EEPROM_ID_TIME_INTERVAL_DEFAULT;  /* amount of time between ID/callsign transmissions */
static volatile uint16_t g_clock_calibration = EEPROM_CLOCK_CALIBRATION_DEFAULT;
static volatile int16_t g_temp_calibration = EEPROM_TEMP_CALIBRATION_DEFAULT;
static volatile uint8_t g_override_DIP_switches = EEPROM_OVERRIDE_DIP_SW_DEFAULT;
static volatile uint8_t g_enable_LEDs;
static volatile uint8_t g_enable_sync;

static char g_tempStr[21] = { '\0' };

static volatile uint8_t g_start_override = FALSE;

/*
 * Function Prototypes
 */
void handleLinkBusMsgs(void);
void initializeEEPROMVars(void);
void saveAllEEPROM(void);
float getTemp(void);
uint16_t readADC();
void setUpTemp(void);

#ifndef USE_WATCHDOG
	void (* resetFunc)(void) = 0;   /*declare reset function @ address 0 */
#endif

#ifdef COMPILE_FOR_ATMELSTUDIO7
	void loop(void);
	int main(void)
#else
	void setup()
#endif  /* COMPILE_FOR_ATMELSTUDIO7 */
{
	initializeEEPROMVars();     /* Initialize variables stored in EEPROM */
	linkbus_init(BAUD);         /* Start the Link Bus serial comms */
	setUpTemp();

	/* set pins as outputs */
	pinMode(PIN_NANO_LED, OUTPUT);  /* The nano amber LED: This led blinks when off cycle and blinks with code when on cycle. */
	digitalWrite(PIN_NANO_LED, OFF);
	pinMode(PIN_NANO_KEY, OUTPUT);  /* This pin is used to control the KEY line to the transmittter only active on cycle. */
	digitalWrite(PIN_NANO_KEY, OFF);

/* set the pins that are inputs */
	pinMode(PIN_NANO_SYNC, INPUT);  /* This pin is used as the sync line
	                                 * NOTE: The original albq PIC controllers used the CPU reset line as the
	                                 * sync line. We had issues with transmitters getting out of sync during transport.
	                                 * later investigation found that ESD events during transport was resetting the
	                                 * PIC. This code will read this I/O line for sync and after finding that the I/O line
	                                 * has switched to the high state it never reads it again until a power cycle.
	                                 * The Arduino reset line does not go off board. ESD event caused out of sync issue fixed. */
	pinMode(PIN_NANO_DIP_0, INPUT); /* fox switch LSB */
	pinMode(PIN_NANO_DIP_1, INPUT); /* fox switch middle bit */
	pinMode(PIN_NANO_DIP_2, INPUT); /* fox switch MSB */

/*
 * read the dip switches and compute the desired fox number
 * */
	if(g_override_DIP_switches)
	{
		g_fox = (FoxType)g_override_DIP_switches;
	}
	else
	{
		if(digitalRead(PIN_NANO_DIP_0) == HIGH )    /*Lsb */
		{
			g_fox++;
		}
		if(digitalRead(PIN_NANO_DIP_1) == HIGH )    /* middle bit */
		{
			g_fox += 2;
		}
		if(digitalRead(PIN_NANO_DIP_2) == HIGH )    /* MSB */
		{
			g_fox += 4;
		}
	}

	cli();                              /*stop interrupts for setup */

	digitalWrite(PIN_NANO_LED, OFF);    /* Turn off led sync switch is now open */

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
 ** USE THIS TO FIX BOARD PROCESSOR CLOCK ERROR
 ************************************************************/
	/* first testing found bad drift relative to a gps stable clock (iphone timer) is was 10 seconds in about 50 minutes
	 * Today I measured the Arduino nano 16 Mhz clock and it was 16.050 MHz. Yes 50 Khz high!!
	 * OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)    //was 15624 for 16.000 MHz on center frequency clock
	 * OCR1A = 15672;// = (16.043*10^6) / (1*1024) - 1 (must be <65536)   //15672 computed for + 16.050 MHz off frequency clock board 4
	 * OCR1A = 15661;// = (16.0386*10^6) / (1*1024) - 1 (must be <65536)   //15661 computed for + 16.0386 MHz off frequency clock board 3 */
/*	OCR1A = 15629;/ * = (16.006*10^6) / (1*1024) - 1 (must be <65536)   //15629 computed for + 16.006 MHz off frequency clock board 3 * / */
	OCR1A = g_clock_calibration;

	/**
	 * TIMER2 is for periodic interrupts to drive Morse code generation */
	OCR2A = 0x0C;                                       /* set frequency to ~300 Hz (0x0c) */
	TCCR2A |= (1 << WGM01);                             /* set CTC with OCRA */
	TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);  /* 1024 Prescaler */
	TIMSK2 |= (1 << OCIE0B);                            /* enable compare interrupt */

/* turn on CTC mode */
	TCCR1B |= (1 << WGM12);
/* Set CS11 bit for 1024 prescaler */
	TCCR1B |= (1 << CS12) | (1 << CS10);
/* enable timer compare interrupt */
	TIMSK1 |= (1 << OCIE1A);
	sei();  /*allow interrupts. Arm and run */

	lb_send_string((char*)"\n\nStored Data:\n", TRUE);
	sprintf(g_tempStr, "  ID: %s\n", g_messages_text[STATION_ID]);
	lb_send_string(g_tempStr, TRUE);
	sprintf(g_tempStr, "  CAL: %u\n", g_clock_calibration);
	lb_send_string(g_tempStr, TRUE);
	sprintf(g_tempStr, "  DIP: %u\n", g_override_DIP_switches);
	lb_send_string(g_tempStr, TRUE);
	sprintf(g_tempStr, "  LED: %s\n", g_enable_LEDs == TRUE ? "ON" : "OFF");
	lb_send_string(g_tempStr, TRUE);
	sprintf(g_tempStr, "  SYN: %s\n", g_enable_sync == TRUE ? "ON" : "OFF");
	lb_send_string(g_tempStr, TRUE);
	lb_send_NewPrompt();

/* I found this on the web
 * note Timer0 - An 8 bit timer used by Arduino functions delay(), millis() and micros().
 *      Timer1 - A 16 bit timer used by the Servo() library
 *      Timer2 - An 8 bit timer used by the Tone() library */
/*
 * we now look for the sync line and then arm the timer interrupts
 * when the sync switch is released.
 * */

	if(g_enable_sync && (g_fox != FOXORING) && (g_fox != BEACON) && (g_fox != FOX_DEMO))
	{
		lb_send_string((char*)"Waiting for sync.\n", TRUE);
		lb_send_string((char*)"Type \"GO\"\n", TRUE);
		lb_send_NewPrompt();

		while(( digitalRead(PIN_NANO_SYNC) == LOW) && !g_start_override)
		{
			if(g_enable_LEDs)
			{
				digitalWrite(PIN_NANO_LED, HIGH);   /* arduino nano LED turn on led to show sync switch is closed */
			}

			handleLinkBusMsgs();
		}
	}
	else
	{
		lb_send_string((char*)"Tx is running!\n", TRUE);
		lb_send_NewPrompt();
	}

	TCNT1 = 0;  /* Initialize 1-second counter value to 0 */
	g_seconds = 0;
	g_minutes = 0;
	g_seconds_since_sync = 0;

	g_start_override = TRUE;

#ifdef COMPILE_FOR_ATMELSTUDIO7
		while(1)
		{
			loop();
		}
#endif  /* COMPILE_FOR_ATMELSTUDIO7 */
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
	static char textBuff[LINKBUS_MAX_MSG_FIELD_LENGTH];
	static LinkbusRxBuffer* buff = NULL;
	static uint8_t charIndex = 0;
	static uint8_t field_index = 0;
	static uint8_t field_len = 0;
	static int msg_ID = 0;
	static BOOL receiving_msg = FALSE;
	uint8_t rx_char;

	rx_char = UDR0;

	if(!buff)
	{
		buff = nextEmptyRxBuffer();
	}

	if(buff)
	{
		static uint8_t ignoreCount = 0;
		rx_char = toupper(rx_char);

		if(ignoreCount)
		{
			rx_char = '\0';
			ignoreCount--;
		}
		else if(rx_char == 0x1B)    /* ESC sequence start */
		{
			rx_char = '\0';

			if(charIndex < LINKBUS_MAX_MSG_FIELD_LENGTH)
			{
				rx_char = textBuff[charIndex];
			}

			ignoreCount = 2;    /* throw out the next two characters */
		}
		if(rx_char == 0x0D)     /* Handle carriage return */
		{
			if(receiving_msg)
			{
				if(charIndex > 0)
				{
					buff->type = LINKBUS_MSG_QUERY;
					buff->id = (LBMessageID)msg_ID;

					if(field_index > 0) /* terminate the last field */
					{
						buff->fields[field_index - 1][field_len] = 0;
					}

					textBuff[charIndex] = '\0'; /* terminate last-message buffer */
				}

				lb_send_NewLine();
			}
			else
			{
				buff->id = INVALID_MESSAGE; /* print help message */
			}

			charIndex = 0;
			field_len = 0;
			msg_ID = MESSAGE_EMPTY;

			field_index = 0;
			buff = NULL;

			receiving_msg = FALSE;
		}
		else if(rx_char)
		{
			textBuff[charIndex] = rx_char;  /* hold the characters for re-use */

			if(charIndex)
			{
				if(rx_char == 0x7F)         /* Handle backspace */
				{
					charIndex--;
					if(field_index == 0)
					{
						msg_ID -= textBuff[charIndex];
						msg_ID /= 10;
					}
					else if(field_len)
					{
						field_len--;
					}
					else
					{
						buff->fields[field_index][0] = '\0';
						field_index--;
					}
				}
				else
				{
					if(rx_char == ' ')
					{
						if(textBuff[charIndex - 1] == ' ')
						{
							rx_char = '\0';
						}
						else
						{
							if(field_index > 0)
							{
								buff->fields[field_index - 1][field_len] = 0;
							}

							field_index++;
							field_len = 0;
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

					charIndex = MIN(charIndex + 1, LINKBUS_MAX_MSG_FIELD_LENGTH);
				}
			}
			else
			{
				if((rx_char == 0x7F) || (rx_char == ' '))   /* Handle backspace and Space */
				{
					rx_char = '\0';
				}
				else                                        /* start of new message */
				{
					uint8_t i;
					field_index = 0;
					msg_ID = 0;

					msg_ID = msg_ID * 10 + rx_char;

					/* Empty the field buffers */
					for(i = 0; i < LINKBUS_MAX_MSG_NUMBER_OF_FIELDS; i++)
					{
						buff->fields[i][0] = '\0';
					}

					receiving_msg = TRUE;
					charIndex = MIN(charIndex + 1, LINKBUS_MAX_MSG_FIELD_LENGTH);
				}
			}

			if(rx_char)
			{
				lb_echo_char(rx_char);
			}
		}
	}
}   /* End of UART Rx ISR */


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
}   /* End of UART Tx ISR */

/***********************************************************************
 * Timer/Counter2 Compare Match B ISR
 *
 * Handles periodic tasks not requiring precise timing.
 ************************************************************************/
ISR( TIMER2_COMPB_vect )
{
	static uint16_t codeInc = 0;
	static int blink_counter = 100;
	static int blink_count_direction = -1;
	BOOL repeat = TRUE, finished = FALSE;

	if(blink_counter < -BLINK_LONG)
	{
		blink_count_direction = 1;
	}
	if(blink_counter > BLINK_SHORT)
	{
		blink_count_direction = -1;
	}
	blink_counter += blink_count_direction;

	if(blink_counter < 0)
	{
		g_blinky_time = TRUE;
	}
	else
	{
		g_blinky_time = FALSE;
	}

	static BOOL key = OFF;

	if(g_on_the_air > 0)
	{
		if(codeInc)
		{
			codeInc--;

			if(!codeInc)
			{
				key = makeMorse(NULL, &repeat, &finished);

				if(!repeat && finished) /* ID has completed, so resume pattern */
				{
/*					g_code_throttle = THROTTLE_VAL_FROM_WPM(g_pattern_codespeed);
 *					repeat = TRUE;
 *					makeMorse(g_messages_text[PATTERN_TEXT], &repeat, NULL);
 *					key = makeMorse(NULL, &repeat, &finished); */
					key = OFF;
					g_callsign_sent = TRUE;
				}

				if(key)
				{
					if(g_enable_LEDs)
					{
						digitalWrite(PIN_NANO_LED, HIGH);   /*  Nano LED */
					}

					digitalWrite(PIN_NANO_KEY, HIGH);       /* TX key line */
				}
			}
		}
		else
		{
			if(g_enable_LEDs)
			{
				digitalWrite(PIN_NANO_LED, key);    /*  nano LED */
			}

			digitalWrite(PIN_NANO_KEY, key);        /* TX key line */
			codeInc = g_code_throttle;
		}
	}
	else if(!g_on_the_air)
	{
		if(key)
		{
			key = OFF;
			digitalWrite(PIN_NANO_LED, LOW);    /*  nano LED */
			digitalWrite(PIN_NANO_KEY, LOW);    /* TX key line */
		}
	}
} /* End of Timer 2 ISR */



/***********************************************************************
 * Timer/Counter1 Compare Match A ISR
 *
 * Handles once-per-second tasks
 ************************************************************************/
/*
 * here is our main ISR for the ARDF 1-second timer
 * modified from ISR example for microfox by Jerry Boyd WB8WFK
 * this runs once a second and generates the cycle and sets control flags for the main controller.
 */
ISR(TIMER1_COMPA_vect)      /*timer1 interrupt 1Hz */
{
	g_seconds_since_sync++; /* Total elapsed time counter */
	g_seconds++;            /* Update seconds */

	if(g_seconds > 59)
	{
		g_seconds = 0;
		g_minutes++;

		if(g_minutes > 4)
		{
			g_minutes = 0;
		}
	}
}   /* end of Timer 1 ISR */



/*
 *
 * Here is the main loop for the Microfox Transmitter
 *
 * */
void loop()
{
	static int time_for_id = 99;
	static BOOL id_set = TRUE;
	static BOOL proceed = FALSE;

	handleLinkBusMsgs();

	if(!g_on_the_air || proceed)
	{
		proceed = FALSE;

		/* Choose the appropriate Morse pattern to be sent */
		if(g_fox == FOX_DEMO)
		{
			strcpy(g_messages_text[PATTERN_TEXT], g_morsePatterns[g_minutes + 1]);
		}
		else
		{
			strcpy(g_messages_text[PATTERN_TEXT], g_morsePatterns[g_fox]);
		}

		/* At the appropriate time set the pattern to be sent and start transmissions */
		if((g_fox == FOX_DEMO) || (g_fox == BEACON) || (g_fox == FOXORING) || (g_fox == (g_minutes + 1)))
		{
			BOOL repeat = TRUE;
			g_code_throttle = THROTTLE_VAL_FROM_WPM(g_pattern_codespeed);
			makeMorse(g_messages_text[PATTERN_TEXT], &repeat, NULL);
			time_for_id = 60 -  (500 + timeRequiredToSendStrAtWPM(g_messages_text[STATION_ID], g_id_codespeed)) / 1000;

			id_set = FALSE;
			g_on_the_air = TRUE;
			g_callsign_sent = FALSE;
		}
		else
		{
			if(g_enable_LEDs)   /* below will flash LED when off cycle for a heartbeat indicator */
			{
				if(g_blinky_time)
				{
					digitalWrite(PIN_NANO_LED, OFF);
				}
				else
				{
					digitalWrite(PIN_NANO_LED, ON);
				}
			}
		}
	}
	else
	{
		if(!id_set && (g_seconds == time_for_id))   /* Send the call sign at the right time */
		{
			g_code_throttle = THROTTLE_VAL_FROM_WPM(g_id_codespeed);
			BOOL repeat = FALSE;
			makeMorse(g_messages_text[STATION_ID], &repeat, NULL);
			id_set = TRUE;
			g_callsign_sent = FALSE;
		}
		else if(g_seconds == 30)    /* Speed up code after 30 seconds */
		{
			g_code_throttle = THROTTLE_VAL_FROM_WPM(g_pattern_codespeed * 2);
		}

		if((g_fox == FOX_DEMO))
		{
			if((g_callsign_sent) && (g_seconds == 0))   /* Ensure we've begun the next minute before proceeding */
			{
				proceed = TRUE;
			}
		}
		else if((g_fox == BEACON) || (g_fox == FOXORING))   /* Proceed as soon as the callsign has been sent */
		{
			if(g_callsign_sent)
			{
				proceed = TRUE;
			}
		}
		else if((g_fox != (g_minutes + 1)) && g_callsign_sent)  /* Turn off transmissions during minutes when this fox should be silent */
		{
			g_on_the_air = FALSE;
		}
	}
}   /* End of main loop() */

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
			case MESSAGE_RESET:
			{
#ifdef USE_WATCHDOG
					wdt_init(WD_FORCE_RESET);
					while(1)
					{
						;
					}
#else
					resetFunc();    /*call reset */
#endif /* USE_WATCHDOG */
			}
			break;

			case MESSAGE_OVERRIDE_DIP:
			{
				if(lb_buff->fields[FIELD1][0])
				{
					int d = atoi(lb_buff->fields[FIELD1]);

					if((d >= BEACON) && (d < INVALID_FOX))
					{
						g_override_DIP_switches = d;
						g_fox = (FoxType)d;
					}

					saveAllEEPROM();
				}

				sprintf(g_tempStr, "DIP=%u\n", g_override_DIP_switches);
				lb_send_string(g_tempStr, FALSE);
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
				lb_send_string(g_tempStr, FALSE);
			}
			break;

			case MESSAGE_SYNC_ENABLE:
			{
				if(lb_buff->fields[FIELD1][0])
				{
					if((lb_buff->fields[FIELD1][1] == 'F') || (lb_buff->fields[FIELD1][0] == '0'))
					{
						g_enable_sync = FALSE;
					}
					else
					{
						g_enable_sync = TRUE;
					}

					saveAllEEPROM();
				}

				sprintf(g_tempStr, "SYN:%s\n", g_enable_sync ? "ON" : "OFF");
				lb_send_string(g_tempStr, FALSE);
			}
			break;

			case MESSAGE_GO:
			{
				if(g_start_override)
				{
					lb_send_string((char*)"Already synced!\n", FALSE);
				}
				else
				{
					g_start_override = TRUE;
					lb_send_string((char*)"Running!\n", FALSE);
				}
			}
			break;

			case MESSAGE_FACTORY_RESET:
			{
				uint8_t flag = EEPROM_INITIALIZED_FLAG + 1;
				eeprom_write_byte(&ee_interface_eeprom_initialization_flag, flag);
#ifdef USE_WATCHDOG
					wdt_init(WD_FORCE_RESET);
					while(1)
					{
						;
					}
#else
					resetFunc();    /*call reset */
#endif /* USE_WATCHDOG */
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
				lb_send_string(g_tempStr, FALSE);
			}
			break;

			case MESSAGE_SET_STATION_ID:
			{
				if(lb_buff->fields[FIELD1][0])
				{
					strcpy(g_messages_text[STATION_ID], " ");   /* Space before ID gets sent */
					strcat(g_messages_text[STATION_ID], lb_buff->fields[FIELD1]);

					if(lb_buff->fields[FIELD2][0])
					{
						strcat(g_messages_text[STATION_ID], " ");
						strcat(g_messages_text[STATION_ID], lb_buff->fields[FIELD2]);
					}

					saveAllEEPROM();
				}

				if(g_messages_text[STATION_ID][0])
				{
					g_time_needed_for_ID = (500 + timeRequiredToSendStrAtWPM(g_messages_text[STATION_ID], g_id_codespeed)) / 1000;
				}

				sprintf(g_tempStr, "ID:%s\n", g_messages_text[STATION_ID]);
				lb_send_string(g_tempStr, TRUE);
			}
			break;

			case MESSAGE_CODE_SPEED:
			{
				uint8_t speed = g_pattern_codespeed;

				if(lb_buff->fields[FIELD1][0] == 'I')
				{
					if(lb_buff->fields[FIELD2][0])
					{
						speed = atol(lb_buff->fields[FIELD2]);
						g_id_codespeed = CLAMP(5, speed, 20);

						if(g_messages_text[STATION_ID][0])
						{
							g_time_needed_for_ID = (500 + timeRequiredToSendStrAtWPM(g_messages_text[STATION_ID], g_id_codespeed)) / 1000;
						}

						saveAllEEPROM();
					}
				}
				else if(lb_buff->fields[FIELD1][0] == 'P')
				{
					if(lb_buff->fields[FIELD2][0])
					{
						speed = atol(lb_buff->fields[FIELD2]);
						g_pattern_codespeed = CLAMP(5, speed, 20);
						g_code_throttle = THROTTLE_VAL_FROM_WPM(g_pattern_codespeed);

						saveAllEEPROM();
					}
				}

				sprintf(g_tempStr, "ID:  %d wpm\n", g_id_codespeed);
				lb_send_string(g_tempStr, FALSE);
				sprintf(g_tempStr, "Pat: %d wpm\n", g_pattern_codespeed);
				lb_send_string(g_tempStr, FALSE);
			}
			break;

			case MESSAGE_VERSION:
			{
				sprintf(g_tempStr, "SW Ver:%s\n", SW_REVISION);
				lb_send_string(g_tempStr, FALSE);
			}
			break;

			case MESSAGE_TEMP:
			{
				if(lb_buff->fields[FIELD1][0] == 'C')
				{
					if(lb_buff->fields[FIELD2][0])
					{
						int16_t v = atoi(lb_buff->fields[FIELD2]);

						if((v > -2000) && (v < 2000))
						{
							g_temp_calibration = v;
							saveAllEEPROM();
						}
					}

					sprintf(g_tempStr, "T Cal= %d\n", g_temp_calibration);
					lb_send_string(g_tempStr, FALSE);
				}

				float temp = getTemp();
				sprintf(g_tempStr, "Temp: %dC\n", (int)temp);
				lb_send_string(g_tempStr, FALSE);
			}
			break;

			default:
			{
				lb_send_Help();
			}
			break;
		}

		lb_buff->id = (LBMessageID)MESSAGE_EMPTY;
		if(send_ack)
		{
			lb_send_NewPrompt();
		}
	}
}


/*
 * Set non-volatile variables to their values stored in EEPROM
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
		g_temp_calibration = (int16_t)eeprom_read_word((uint16_t*)&ee_temp_calibration);
		g_override_DIP_switches = eeprom_read_byte(&ee_override_DIP_switches);
		g_enable_LEDs = eeprom_read_byte(&ee_enable_LEDs);
		g_enable_sync = eeprom_read_byte(&ee_enable_sync);

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
		g_temp_calibration = EEPROM_TEMP_CALIBRATION_DEFAULT;
		g_override_DIP_switches = EEPROM_OVERRIDE_DIP_SW_DEFAULT;
		g_enable_LEDs = EEPROM_ENABLE_LEDS_DEFAULT;
		g_enable_sync = EEPROM_ENABLE_SYNC_DEFAULT;
		strncpy(g_messages_text[STATION_ID], EEPROM_STATION_ID_DEFAULT, MAX_PATTERN_TEXT_LENGTH);
		strncpy(g_messages_text[PATTERN_TEXT], EEPROM_PATTERN_TEXT_DEFAULT, MAX_PATTERN_TEXT_LENGTH);
		saveAllEEPROM();
		eeprom_write_byte(&ee_interface_eeprom_initialization_flag, EEPROM_INITIALIZED_FLAG);
	}
}

/*
 * Save all changed non-volatile values to EEPROM
 */
void saveAllEEPROM()
{
	uint8_t i;

	eeprom_update_byte(&ee_id_codespeed, g_id_codespeed);
	eeprom_update_byte(&ee_pattern_codespeed, g_pattern_codespeed);
	eeprom_update_word(&ee_ID_time, g_ID_period_seconds);
	eeprom_update_word(&ee_clock_calibration, g_clock_calibration);
	eeprom_update_word((uint16_t*)&ee_temp_calibration, (uint16_t)g_temp_calibration);
	eeprom_update_byte(&ee_override_DIP_switches, g_override_DIP_switches);
	eeprom_update_byte(&ee_enable_LEDs, g_enable_LEDs);
	eeprom_update_byte(&ee_enable_sync, g_enable_sync);

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

/*
 * Set up registers for measuring processor temperature
 */
void setUpTemp(void)
{
	/* The internal temperature has to be used
	 * with the internal reference of 1.1V.
	 * Channel 8 can not be selected with
	 * the analogRead function yet. */
	/* Set the internal reference and mux. */
	ADMUX = ((1 << REFS1) | (1 << REFS0) | (1 << MUX3));

	/* Slow the ADC clock down to 125 KHz
	 * by dividing by 128. Assumes that the
	 * standard Arduino 16 MHz clock is in use. */
	ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

	ADCSRA |= (1 << ADEN);  /* enable the ADC */

	delay(200);             /* wait for voltages to become stable. */

	ADCSRA |= (1 << ADSC);  /* Start the ADC */

	readADC();
}

/*
 * Read the temperature ADC value
 */
uint16_t readADC()
{
	/* Make sure the most recent ADC read is complete. */
	while((ADCSRA & (1 << ADSC)))
	{
		;   /* Just wait for ADC to finish. */
	}
	uint16_t result = ADCW;
	/* Initiate another reading. */
	ADCSRA |= (1 << ADSC);
	return( result);
}

/*
 * Returns the most recent temperature reading
 */
float getTemp(void)
{
	float offset = (float)g_temp_calibration / 10.;

	/* The offset (first term) was determined empirically */
	readADC();  /* throw away first reading */
	return(offset + (readADC() - 324.31) / 1.22);
}

