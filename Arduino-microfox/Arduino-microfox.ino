/*
 *  MIT License
 *
 *  Copyright (c) 2020 DigitalConfections
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */
/*
 * Microfox Arduino nano version. Converted from PIC C November 2019
 * Jerry Boyd WB8WFK
 * This controller will replace the Albuquerque VHF and HF microfox pic based controller
 *
 * */


#include "defs.h"
#include "linkbus.h"
#include "morse.h"
#include <avr/wdt.h>

#if COMPILE_FOR_ATMELSTUDIO7
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
#define TEMP_STRING_LENGTH (MAX_PATTERN_TEXT_LENGTH + 10)

volatile int32_t g_seconds_since_sync = 0;  /* Total elapsed time counter */
volatile FoxType g_fox          = BEACON;   /* Sets Fox number not set by ISR. Set in startup and checked in main. */
volatile int g_active           = 0;        /* Disable active. set and clear in ISR. Checked in main. */

volatile int g_on_the_air       = 0;        /* Controls transmitter Morse activity */
volatile int g_code_throttle    = 0;        /* Adjusts Morse code speed */
const char g_morsePatterns[][6] = { "MO ", "MOE ", "MOI ", "MOS ", "MOH ", "MO5 ", "", "5", "S", "ME", "MI", "MS", "MH", "M5", "OE", "OI", "OS", "OH", "O5" };
volatile BOOL g_callsign_sent = FALSE;

volatile BOOL g_blinky_time = FALSE;

volatile int g_on_air_interval = 0;
volatile int g_fox_seconds_into_interval = 0;
volatile int g_fox_counter = 1;
volatile int g_number_of_foxes = 0;
volatile BOOL g_fox_transition = FALSE;
volatile int g_fox_id_offset = 0;
volatile int g_id_interval = 0;
volatile BOOL g_time_to_ID = FALSE;
volatile int g_startclock_interval = 60;
volatile int g_fox_tone_offset = 1;

volatile BOOL g_audio_tone_state = FALSE;
volatile uint8_t g_lastSeconds = 0x00;
volatile int16_t g_sync_pin_timer = 0;
volatile BOOL g_sync_pin_stable = FALSE;
volatile BOOL g_sync_enabled = TRUE;

#if !COMPILE_FOR_ATMELSTUDIO7
	FoxType operator++ (volatile FoxType &orig, int)
	{
		orig = static_cast < FoxType > (orig + 1);  /* static_cast required because enum + int -> int */
		if(orig > INVALID_FOX)
		{
			orig = INVALID_FOX;
		}
		return( orig);
	}

	FoxType operator += (volatile FoxType &a, int b)
	{
		a = static_cast < FoxType > (a + b);    /* static_cast required because enum + int -> int */
		return( a);
	}
 #endif  /* COMPILE_FOR_ATMELSTUDIO7 */


/***********************************************************************
 * Global Variables & String Constants
 *
 * Identify each global with a "g_" prefix
 * Whenever possible limit globals' scope to this file using "static"
 * Use "volatile" for globals shared between ISRs and foreground
 ************************************************************************/
static uint8_t EEMEM ee_interface_eeprom_initialization_flag = EEPROM_UNINITIALIZED;
static char EEMEM ee_stationID_text[MAX_PATTERN_TEXT_LENGTH + 1];
static uint8_t EEMEM ee_id_codespeed;
static uint16_t EEMEM ee_clock_calibration;
static uint8_t EEMEM ee_override_DIP_switches;
static uint8_t EEMEM ee_enable_LEDs;
static int16_t EEMEM ee_temp_calibration;
static uint8_t EEMEM ee_enable_start_timer;
static uint8_t EEMEM ee_enable_transmitter;

static char g_messages_text[2][MAX_PATTERN_TEXT_LENGTH + 1] = { "\0", "\0" };
static volatile uint8_t g_id_codespeed = EEPROM_ID_CODE_SPEED_DEFAULT;
static volatile uint8_t g_pattern_codespeed = EEPROM_PATTERN_CODE_SPEED_DEFAULT;
static volatile uint16_t g_time_needed_for_ID = 0;
static volatile uint16_t g_clock_calibration = EEPROM_CLOCK_CALIBRATION_DEFAULT;
static volatile int16_t g_temp_calibration = EEPROM_TEMP_CALIBRATION_DEFAULT;
static volatile uint8_t g_override_DIP_switches = EEPROM_OVERRIDE_DIP_SW_DEFAULT;
static volatile uint8_t g_enable_LEDs;
static volatile uint8_t g_enable_start_timer;
static volatile uint8_t g_enable_transmitter;

static char g_tempStr[TEMP_STRING_LENGTH] = { '\0' };
static volatile uint8_t g_LEDs_Timed_Out = FALSE;
/*
 * Function Prototypes
 */
void handleLinkBusMsgs(void);
void initializeEEPROMVars(BOOL resetAll);
float getTemp(void);
uint16_t readADC();
void setUpTemp(void);
void sendMorseTone(BOOL onOff);
void playStartingTone(uint8_t toneFreq);
void wdt_set(WDReset resetType);
void doSynchronization(void);
void setupForFox(void);
void softwareReset(void);

#if USE_WDT_RESET
uint8_t mcusr_copy __attribute__ ((section (".noinit")));
void disable_wdt(void) \
     __attribute__((naked)) \
     __attribute__((section(".init3")));
void disable_wdt(void) {
  mcusr_copy = MCUSR;
  MCUSR = 0x00;
  wdt_disable();
}
#endif // USE_WDT_RESET


#if COMPILE_FOR_ATMELSTUDIO7
	void loop(void);
	int main(void)
#else
	void setup()
#endif  /* COMPILE_FOR_ATMELSTUDIO7 */
{
	initializeEEPROMVars(FALSE);
	setUpTemp();

	cli();                          /*stop interrupts for setup */

	/* set pins as outputs */
	pinMode(PIN_LED, OUTPUT);       /* The amber LED: This led blinks when off cycle and blinks with code when on cycle. */
	digitalWrite(PIN_LED, OFF);
	pinMode(PIN_MORSE_KEY, OUTPUT); /* This pin is used to control the KEY line to the transmitter only active on cycle. */
	digitalWrite(PIN_MORSE_KEY, OFF);
	pinMode(PIN_AUDIO_OUT, OUTPUT);
	digitalWrite(PIN_AUDIO_OUT, OFF);

	/* Set unused pins as outputs pulled high */
	pinMode(PIN_UNUSED_7, INPUT_PULLUP);
	pinMode(PIN_UNUSED_8, INPUT_PULLUP);
	pinMode(PIN_UNUSED_10, INPUT_PULLUP);
	pinMode(PIN_UNUSED_12, INPUT_PULLUP);
	pinMode(A0, INPUT_PULLUP);
	pinMode(A1, INPUT_PULLUP);
	pinMode(A2, INPUT_PULLUP);
	pinMode(A3, INPUT_PULLUP);
#if !CAL_SIGNAL_ON_PD3
		pinMode(PIN_CAL_OUT, INPUT_PULLUP);
#endif

	/* set timer1 interrupt at 1Hz */
	TCCR1A = 0; /* set entire TCCR1A register to 0 */
	TCCR1B = 0; /* same for TCCR1B */
	TCNT1 = 0;  /*initialize counter value to 0 */

/* Set compare match register for 1hz increments
 ************************************************************
 ** USE THIS TO CALIBRATE FOR BOARD PROCESSOR CLOCK ERROR
 ************************************************************/
	/* first testing found bad drift relative to a gps stable clock (iphone timer) is was 10 seconds in about 50 minutes
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

	/**
	 * TIMER2 is for periodic interrupts to drive Morse code generation */
	/* Reset control registers */
	TCCR2A = 0;
	TCCR2B = 0;
	TCCR2A |= (1 << WGM21);                             /* set Clear Timer on Compare Match (CTC) mode with OCR2A setting the top */
#if CAL_SIGNAL_ON_PD3
		pinMode(PIN_CAL_OUT, OUTPUT);                   /* 601Hz Calibration Signal */
		TCCR2A |= (1 << COM2A0);                        /* Toggle OC2A (PB3) on compare match */
#endif /* CAL_SIGNAL_ON_PD3 */
	TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);  /* 1024 Prescaler */

	OCR2A = 0x0C;                                       /* set frequency to ~300 Hz (0x0c) */
	OCR2B = 0x00;
	/* Use system clock for Timer/Counter2 */
	ASSR &= ~(1 << AS2);
	/* Reset Timer/Counter2 Interrupt Mask Register */
	TIMSK2 = 0;
	TIMSK2 |= (1 << OCIE2B);    /* Output Compare Match B Interrupt Enable */

	/* Timer 0 is for audio Start tone generation and control
	 * Note: Do not use millis() or DELAY() after TIMER0 has been reconfigured here! */
	TCCR0A = 0x00;
	TCCR0A |= (1 << WGM01);     /* Set CTC mode */
	TCCR0B = 0x00;
	TCCR0B |= (1 << CS02);      /* Prescale 256 */
	OCR0A = DEFAULT_TONE_FREQUENCY;
	TIMSK0 = 0x00;
	TIMSK0 |= (1 << OCIE0A);

	/* Sync button pin change interrupt */
	PCMSK2 = 0x00;
	PCMSK2 = (1 << PCINT19);    /* Enable PCINT19 */
	PCICR = 0x00;
	PCICR = (1 << PCIE2);       /* Enable pin change interrupt 2 */

	sei();                      /* Enable interrupts */

	linkbus_init(BAUD);         /* Start the Link Bus serial comms */

	setupForFox();

	lb_send_string((char*)"\n\nStored Data:\n", TRUE);
	sprintf(g_tempStr, "  ID: %s\n", g_messages_text[STATION_ID]);
	lb_send_string(g_tempStr, TRUE);
	sprintf(g_tempStr, "  CAL: %u\n", g_clock_calibration);
	lb_send_string(g_tempStr, TRUE);
	sprintf(g_tempStr, "  DIP: %u\n", g_override_DIP_switches);
	lb_send_string(g_tempStr, TRUE);
	sprintf(g_tempStr, "  LED: %s\n", g_enable_LEDs ? "ON" : "OFF");
	lb_send_string(g_tempStr, TRUE);
	sprintf(g_tempStr, "  STA: %s\n", g_enable_start_timer ? "ON" : "OFF");
	lb_send_string(g_tempStr, TRUE);
	sprintf(g_tempStr, "  TXE: %s\n", g_enable_transmitter ? "ON" : "OFF");
	lb_send_string(g_tempStr, TRUE);
	lb_send_NewPrompt();

#if COMPILE_FOR_ATMELSTUDIO7
		while(1)
		{
			loop();
		}
#endif  /* COMPILE_FOR_ATMELSTUDIO7 */
}/*end setup */

/***********************************************************************
 * Pin Change Interrupt 2 ISR
 *
 * Handles SYNC pin operation
 *
 ************************************************************************/
ISR(PCINT2_vect)
{
	BOOL pinVal = digitalRead(PIN_SYNC);

	if(pinVal)  /* Sync is high */
	{
		if(g_sync_pin_stable)
		{
			doSynchronization();
		}
	}

	g_sync_pin_timer = 0;
}


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
	static char textBuff[LINKBUS_MAX_COMMANDLINE_LENGTH];
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
		else if(rx_char == 0x1B)    /* Ignore ESC sequences */
		{
			rx_char = '\0';

			if(charIndex < LINKBUS_MAX_MSG_FIELD_LENGTH)
			{
				rx_char = textBuff[charIndex];
			}

			ignoreCount = 2;        /* throw out the next two characters */
		}
		else if(rx_char == 0x0D)    /* Handle carriage return */
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
						buff->fields[field_index - 1][field_len] = '\0';
					}
					else if(textBuff[charIndex] == ' ')
					{
						field_index--;
						field_len = strlen(buff->fields[field_index]);
					}
					else
					{
						buff->fields[field_index][0] = '\0';
						field_index--;
					}

					textBuff[charIndex] = '\0'; /* replace deleted char with null */

					if(charIndex == 0)
					{
						receiving_msg = FALSE;
					}
				}
				else
				{
					if(rx_char == ' ')
					{
						if((textBuff[charIndex - 1] == ' ') || ((field_index + 1) >= LINKBUS_MAX_MSG_NUMBER_OF_FIELDS))
						{
							rx_char = '\0';
						}
						else
						{
							if(field_index > 0)
							{
								buff->fields[field_index - 1][field_len] = '\0';
							}

							field_index++;
							field_len = 0;
							charIndex = MIN(charIndex + 1, (LINKBUS_MAX_COMMANDLINE_LENGTH - 1));
						}
					}
					else if(field_len < LINKBUS_MAX_MSG_FIELD_LENGTH)
					{
						if(field_index == 0)    /* message ID received */
						{
							msg_ID = msg_ID * 10 + rx_char;
							field_len++;
						}
						else
						{
							buff->fields[field_index - 1][field_len++] = rx_char;
							buff->fields[field_index - 1][field_len] = '\0';
						}

						charIndex = MIN(charIndex + 1, (LINKBUS_MAX_COMMANDLINE_LENGTH - 1));
					}
					else
					{
						rx_char = '\0';
					}
				}
			}
			else
			{
				if(rx_char == 0x7F) /* Handle Backspace */
				{
					if(msg_ID <= 0)
					{
						rx_char = '\0';
					}

					msg_ID = 0;
				}
				else if(rx_char == ' ') /* Handle Space */
				{
					rx_char = '\0';
				}
				else                    /* start of new message */
				{
					uint8_t i;
					field_index = 0;
					msg_ID = rx_char;

					/* Empty the field buffers */
					for(i = 0; i < LINKBUS_MAX_MSG_NUMBER_OF_FIELDS; i++)
					{
						buff->fields[i][0] = '\0';
					}

					receiving_msg = TRUE;
					charIndex++;
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
	static uint8_t hold_last10sec = 0;
	static int starting_blip = 0;
	static int starting_boop = 0;
	static BOOL playMorse = TRUE;
	BOOL repeat = TRUE, finished = FALSE;

	if(g_sync_enabled)
	{
		if(digitalRead(PIN_SYNC) == LOW)
		{
			if(g_sync_pin_timer < TIMER2_SECONDS_3)
			{
				g_sync_pin_timer++;
			}

			if(g_sync_pin_timer > TIMER2_SECONDS_1)
			{
				g_sync_pin_stable = TRUE;
				digitalWrite(PIN_LED, HIGH);
			}
		}
	}

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

	if(g_enable_start_timer)
	{
		if(hold_last10sec != g_lastSeconds)
		{
			hold_last10sec = g_lastSeconds;

			if(hold_last10sec > 0)
			{
				playMorse = FALSE;
				starting_blip = BLINK_SHORT;
			}
			else if(hold_last10sec == 0)
			{
				starting_blip = 0;
				starting_boop = TIMER2_SECONDS_2;
			}
		}

		if(starting_blip)
		{
			starting_blip--;

			if(starting_blip)
			{
				if(g_lastSeconds > 5)
				{
					playStartingTone(TONE_500Hz);
				}
				else
				{
					playStartingTone(TONE_600Hz);
				}
			}
			else
			{
				playStartingTone(0);
			}

		}
		else if(starting_boop)
		{
			starting_boop--;

			if(!starting_boop)
			{
				playStartingTone(0);
				playMorse = TRUE;
			}
			else
			{
				playStartingTone(TONE_400Hz);
			}
		}
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
					key = OFF;
					g_callsign_sent = TRUE;
					if(playMorse)
					{
						sendMorseTone(OFF);
					}
				}

				if(key)
				{
					if(!g_LEDs_Timed_Out)
					{
						digitalWrite(PIN_LED, HIGH);    /*  LED */
					}

					if(g_enable_transmitter)
					{
						digitalWrite(PIN_MORSE_KEY, HIGH);  /* TX key line */
					}
				}

				if(playMorse)
				{
					sendMorseTone(key);
				}
			}
		}
		else
		{
			if(!g_LEDs_Timed_Out && !g_sync_pin_stable)
			{
				digitalWrite(PIN_LED, key); /*  LED */
			}

			if(g_enable_transmitter)
			{
				digitalWrite(PIN_MORSE_KEY, key);   /* TX key line */
			}

			codeInc = g_code_throttle;

			if(playMorse)
			{
				sendMorseTone(key);
			}
		}
	}
	else if(!g_on_the_air)
	{
		if(key)
		{
			key = OFF;
			if(!g_sync_pin_stable)
			{
				digitalWrite(PIN_LED, OFF);     /*  LED Off */
			}

			digitalWrite(PIN_MORSE_KEY, OFF);   /* TX key line */
		}

		if(playMorse)
		{
			sendMorseTone(OFF);
		}
	}
}   /* End of Timer 2 ISR */



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
ISR(TIMER1_COMPA_vect)              /*timer1 interrupt 1Hz */
{
	static int id_countdown = 0;

	if(g_seconds_since_sync == 0)   /* sync just occurred */
	{
		id_countdown = g_id_interval;
		g_fox_counter = 1;
		g_lastSeconds = 0;
	}

	g_seconds_since_sync++; /* Total elapsed time counter */
	g_fox_seconds_into_interval++;

	if(id_countdown)
	{
		id_countdown--;
	}

	if(g_number_of_foxes && ((g_seconds_since_sync % g_on_air_interval) == 0))
	{
		g_fox_counter++;

		if(g_fox_counter > g_number_of_foxes)
		{
			g_fox_counter = 1;

			if(g_sync_enabled)
			{
				PCMSK2 &= ~(1 << PCINT19);  /* Disable PCINT19 */
				PCICR &= ~(1 << PCIE2);     /* Disable pin change interrupt 2 */
				pinMode(PIN_SYNC, INPUT);
				pinMode(PIN_SYNC, OUTPUT);  /* Set sync pin as output low */
				g_sync_enabled = FALSE;
			}

			g_LEDs_Timed_Out = TRUE;
			digitalWrite(PIN_LED, OFF);
		}
		g_fox_transition = TRUE;
		g_fox_seconds_into_interval = 0;

		if(!id_countdown)
		{
			id_countdown = g_id_interval;
			g_time_to_ID = TRUE;
		}
	}

	if(g_enable_start_timer && ((g_seconds_since_sync + 11) % g_startclock_interval <= 10))
	{
		g_lastSeconds = (uint8_t)((g_seconds_since_sync + 11) % g_startclock_interval);
	}
	else
	{
		g_lastSeconds = 0;
	}
}   /* end of Timer1 ISR */

/* This interrupt generates an audio tone on the audio out pin. */
SIGNAL(TIMER0_COMPA_vect)
{
	static BOOL toggle = 0;

	toggle = !toggle;

	if(g_audio_tone_state)
	{
		if(toggle)
		{
			digitalWrite(PIN_AUDIO_OUT,ON);
		}
		else
		{
			digitalWrite(PIN_AUDIO_OUT,OFF);
		}
	}
	else
	{
		digitalWrite(PIN_AUDIO_OUT,OFF);
	}
}

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

	/**************************************
	* The watchdog must be petted periodically to keep it from barking
	**************************************/
	cli(); wdt_reset(); /* HW watchdog */ sei();

	handleLinkBusMsgs();

	if(!g_on_the_air || proceed)
	{
		proceed = FALSE;

		/* Choose the appropriate Morse pattern to be sent */
		if(g_fox == FOX_DEMO)
		{
			strcpy(g_messages_text[PATTERN_TEXT],g_morsePatterns[g_fox_counter]);
		}
		else if(g_fox == SPRINT_DEMO)
		{
			strcpy(g_messages_text[PATTERN_TEXT],g_morsePatterns[g_fox_counter + 8]);
		}
		else
		{
			strcpy(g_messages_text[PATTERN_TEXT],g_morsePatterns[g_fox]);
		}

		/* At the appropriate time set the pattern to be sent and start transmissions */
		if((g_fox == FOX_DEMO) || (g_fox == SPRINT_DEMO) || (g_fox == BEACON) || (g_fox == FOXORING) || (g_fox == SPECTATOR) || (g_fox == (g_fox_counter + g_fox_id_offset)))
		{
			BOOL repeat = TRUE;
			g_code_throttle = THROTTLE_VAL_FROM_WPM(g_pattern_codespeed);
			makeMorse(g_messages_text[PATTERN_TEXT],&repeat,NULL);

			if(g_time_to_ID || (g_id_interval <= g_on_air_interval))
			{
				time_for_id = g_on_air_interval - (500 + timeRequiredToSendStrAtWPM(g_messages_text[STATION_ID],g_id_codespeed)) / 1000;
				g_time_to_ID = FALSE;
			}
			else
			{
				time_for_id = g_on_air_interval + 99;   /* prevent sending ID */
			}

			id_set = FALSE;
			g_on_the_air = TRUE;
			g_callsign_sent = FALSE;
			g_fox_transition = FALSE;
			g_fox_tone_offset = g_fox_counter;
		}
		else
		{
			if(!g_LEDs_Timed_Out)   /* below will flash LED when off cycle for a heartbeat indicator */
			{
				if(g_blinky_time)
				{
					if(!g_sync_pin_stable)
					{
						digitalWrite(PIN_LED,OFF);
					}
				}
				else
				{
					digitalWrite(PIN_LED,ON);
				}
			}
		}
	}
	else
	{
		if(!id_set && (g_fox_seconds_into_interval == time_for_id)) /* Send the call sign at the right time */
		{
			g_code_throttle = THROTTLE_VAL_FROM_WPM(g_id_codespeed);
			BOOL repeat = FALSE;
			makeMorse(g_messages_text[STATION_ID],&repeat,NULL);
			id_set = TRUE;
			g_callsign_sent = FALSE;
		}
		else if((g_fox >= SPRINT_S1) && (g_fox <= SPRINT_DEMO))
		{
			if(g_fox_transition)
			{
				g_fox_transition = FALSE;
				g_on_the_air = FALSE;
				proceed = TRUE;
			}
		}

		if((g_fox == FOX_DEMO) || (g_fox == SPRINT_DEMO))
		{
			if((g_callsign_sent) && g_fox_transition)   /* Ensure we've begun the next minute before proceeding */
			{
				proceed = TRUE;
			}
		}
		else if((g_fox == BEACON) || (g_fox == FOXORING) || (g_fox == SPECTATOR))   /* Proceed as soon as the callsign has been sent */
		{
			if(g_callsign_sent)
			{
				proceed = TRUE;
			}
		}
		else if((g_fox >= SPRINT_S1) && (g_fox <= SPRINT_F5) && g_callsign_sent)
		{
			g_on_the_air = FALSE;
		}
		else if((g_fox != g_fox_counter) && g_callsign_sent)    /* Turn off transmissions during minutes when this fox should be silent */
		{
			g_on_the_air = FALSE;
		}
	}
}   /* End of main loop() */

void sendMorseTone(BOOL onOff)
{
	if(!g_lastSeconds)
	{
		OCR0A = DEFAULT_TONE_FREQUENCY - g_fox_tone_offset;
		g_audio_tone_state = onOff;
	}
	else
	{
		OCR0A = DEFAULT_TONE_FREQUENCY;
		g_audio_tone_state = OFF;
	}
}

void playStartingTone(uint8_t toneFreq)
{
	if(toneFreq > 0)
	{
		OCR0A = toneFreq;
		g_audio_tone_state = ON;
	}
	else
	{
		OCR0A = DEFAULT_TONE_FREQUENCY;
		g_audio_tone_state = OFF;
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
			case MESSAGE_RESET:
			{
				softwareReset();
			}
			break;

			case MESSAGE_OVERRIDE_DIP:
			{
				FoxType holdFox = (FoxType)g_override_DIP_switches;
				int c = (int)(lb_buff->fields[FIELD1][0]);

				if(c)
				{
					if(c == 'B')
					{
						c = BEACON;
					}
					else if(c == 'D')
					{
						char t = lb_buff->fields[FIELD2][0];

						if(t == 'S')
						{
							c = SPRINT_DEMO;
						}
						else
						{
							c = FOX_DEMO;
						}
					}
					else if(c == 'F')
					{
						c = FOXORING;
					}
					else if(c == 'C')
					{
						char t = lb_buff->fields[FIELD2][0];
						lb_buff->fields[FIELD2][1] = '\0';

						if(t == 'B')
						{
							t = '0';
						}

						if(isdigit(t))
						{
							c = CLAMP(BEACON,atoi(lb_buff->fields[FIELD2]),FOX_5);
						}
					}
					else if(c == 'S')
					{
						int x = 0;
						char t = lb_buff->fields[FIELD2][0];
						char u = lb_buff->fields[FIELD2][1];
						lb_buff->fields[FIELD2][2] = '\0';

						if(t == 'B')
						{
							x = BEACON;
						}
						else if(t == 'F')
						{
							if((u > '0') && (u < '6'))
							{
								x = SPRINT_F1 + (u - '1');
							}
						}
						else if(t == 'S')
						{
							if((u > '0') && (u < '6'))
							{
								x = SPRINT_S1 + (u - '1');
							}
							else
							{
								x = SPECTATOR;
							}
						}
						else if(u == 'F')
						{
							if((t > '0') && (t < '6'))
							{
								x = SPRINT_F1 + (t - '1');
							}
						}
						else if(u == 'S')
						{
							if((t > '0') && (t < '6'))
							{
								x = SPRINT_S1 + (t - '1');
							}
						}

						if(x != BEACON)
						{
							c = CLAMP(SPECTATOR,x,SPRINT_F5);
						}
					}
					else if(c == 'N')
					{
						char t = lb_buff->fields[FIELD2][0];

						if(t == '2')
						{
							c = NO_CODE_START_TONES_2M;
						}
						else if(t == '5')
						{
							c = NO_CODE_START_TONES_5M;
						}
						else
						{
							c = BEACON;
						}
					}
					else
					{
						c = atoi(lb_buff->fields[FIELD1]);
					}

					if((c >= BEACON) && (c < INVALID_FOX))
					{
						g_override_DIP_switches = c;
						g_fox = (FoxType)c;
						eeprom_update_byte(&ee_override_DIP_switches,g_override_DIP_switches);
						if(holdFox != g_fox)
						{
							doSynchronization();
						}
					}
				}

				sprintf(g_tempStr,"DIP=%u\n",g_override_DIP_switches);
				lb_send_string(g_tempStr,FALSE);
			}
			break;

			case MESSAGE_LEDS:
			{
				if(lb_buff->fields[FIELD1][0])
				{
					if((lb_buff->fields[FIELD1][1] == 'F') || (lb_buff->fields[FIELD1][0] == '0'))
					{
						g_enable_LEDs = FALSE;
						digitalWrite(PIN_LED,OFF);  /*  LED Off */
					}
					else
					{
						g_enable_LEDs = TRUE;
					}

					eeprom_update_byte(&ee_enable_LEDs,g_enable_LEDs);
					g_LEDs_Timed_Out = !g_enable_LEDs;
				}

				sprintf(g_tempStr,"LED:%s\n",g_enable_LEDs ? "ON" : "OFF");
				lb_send_string(g_tempStr,FALSE);
			}
			break;

			case MESSAGE_STARTTONES_ENABLE:
			{
				if(lb_buff->fields[FIELD1][0])
				{
					if((lb_buff->fields[FIELD1][1] == 'F') || (lb_buff->fields[FIELD1][0] == '0'))
					{
						g_enable_start_timer = FALSE;
					}
					else
					{
						g_enable_start_timer = TRUE;
					}

					eeprom_update_byte(&ee_enable_start_timer,g_enable_start_timer);
				}

				sprintf(g_tempStr,"STA:%s\n",g_enable_start_timer ? "ON" : "OFF");
				lb_send_string(g_tempStr,FALSE);
			}
			break;

			case MESSAGE_TRANSMITTER_ENABLE:
			{
				if(lb_buff->fields[FIELD1][0])
				{
					if((lb_buff->fields[FIELD1][1] == 'F') || (lb_buff->fields[FIELD1][0] == '0'))
					{
						cli();
						g_enable_transmitter = FALSE;
						digitalWrite(PIN_MORSE_KEY,OFF);
						sei();
					}
					else
					{
						g_enable_transmitter = TRUE;
					}

					eeprom_update_byte(&ee_enable_transmitter,g_enable_transmitter);
				}

				sprintf(g_tempStr,"TXE:%s\n",g_enable_transmitter ? "ON" : "OFF");
				lb_send_string(g_tempStr,FALSE);
			}
			break;

			case MESSAGE_GO:
			{
				doSynchronization();
				lb_send_string((char*)"Re-sync successful!\n",FALSE);
			}
			break;

			case MESSAGE_FACTORY_RESET:
			{
				initializeEEPROMVars(TRUE);
				softwareReset();
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
						eeprom_update_word(&ee_clock_calibration,g_clock_calibration);
					}
				}

				sprintf(g_tempStr,"Cal=%u\n",g_clock_calibration);
				lb_send_string(g_tempStr,FALSE);
			}
			break;

			case MESSAGE_SET_STATION_ID:
			{
				if(lb_buff->fields[FIELD1][0])
				{
					strcpy(g_tempStr," ");  /* Space before ID gets sent */
					strcat(g_tempStr,lb_buff->fields[FIELD1]);

					if(lb_buff->fields[FIELD2][0])
					{
						strcat(g_tempStr," ");
						strcat(g_tempStr,lb_buff->fields[FIELD2]);
					}

					if(strlen(g_tempStr) <= MAX_PATTERN_TEXT_LENGTH)
					{
						uint8_t i;
						strcpy(g_messages_text[STATION_ID],g_tempStr);

						for(i = 0; i < strlen(g_messages_text[STATION_ID]); i++)
						{
							eeprom_update_byte((uint8_t*)&ee_stationID_text[i],(uint8_t)g_messages_text[STATION_ID][i]);
						}

						eeprom_update_byte((uint8_t*)&ee_stationID_text[i],0);
					}
				}

				if(g_messages_text[STATION_ID][0])
				{
					g_time_needed_for_ID = (500 + timeRequiredToSendStrAtWPM(g_messages_text[STATION_ID],g_id_codespeed)) / 1000;
				}

				sprintf(g_tempStr,"ID:%s\n",g_messages_text[STATION_ID]);
				lb_send_string(g_tempStr,TRUE);
			}
			break;


			case MESSAGE_CODE_SPEED:
			{
				if(lb_buff->fields[FIELD1][0] == 'I')
				{
					if(lb_buff->fields[FIELD2][0])
					{
						uint8_t speed = atol(lb_buff->fields[FIELD2]);
						g_id_codespeed = CLAMP(MIN_CODE_SPEED_WPM,speed,MAX_CODE_SPEED_WPM);
						eeprom_update_byte(&ee_id_codespeed,g_id_codespeed);

						if(g_messages_text[STATION_ID][0])
						{
							g_time_needed_for_ID = (500 + timeRequiredToSendStrAtWPM(g_messages_text[STATION_ID],g_id_codespeed)) / 1000;
						}
					}
				}
				sprintf(g_tempStr,"ID:  %d wpm\n",g_id_codespeed);
				lb_send_string(g_tempStr,FALSE);
			}
			break;

			case MESSAGE_VERSION:
			{
				sprintf(g_tempStr,"SW Ver:%s\n",SW_REVISION);
				lb_send_string(g_tempStr,FALSE);
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
							eeprom_update_word((uint16_t*)&ee_temp_calibration,(uint16_t)g_temp_calibration);
						}
					}

					sprintf(g_tempStr,"T Cal= %d\n",g_temp_calibration);
					lb_send_string(g_tempStr,FALSE);
				}

				float temp = 10. * getTemp();
				sprintf(g_tempStr,"Temp: %d.%dC\n",(int)temp / 10,abs((int)temp % 10));
				lb_send_string(g_tempStr,FALSE);
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
void initializeEEPROMVars(BOOL resetAll)
{
	uint8_t i;

	uint8_t initialization_flag = eeprom_read_byte(&ee_interface_eeprom_initialization_flag);

	if(!resetAll && (initialization_flag == EEPROM_INITIALIZED_FLAG)) /* EEPROM is up to date */
	{
		g_id_codespeed = CLAMP(MIN_CODE_SPEED_WPM,eeprom_read_byte(&ee_id_codespeed),MAX_CODE_SPEED_WPM);
		g_clock_calibration = eeprom_read_word(&ee_clock_calibration);
		g_temp_calibration = (int16_t)eeprom_read_word((uint16_t*)&ee_temp_calibration);
		g_override_DIP_switches = eeprom_read_byte(&ee_override_DIP_switches);
		g_enable_LEDs = eeprom_read_byte(&ee_enable_LEDs);
		g_enable_start_timer = eeprom_read_byte(&ee_enable_start_timer);
		g_enable_transmitter = eeprom_read_byte(&ee_enable_transmitter);

		for(i = 0; i < MAX_PATTERN_TEXT_LENGTH; i++)
		{
			g_messages_text[STATION_ID][i] = (char)eeprom_read_byte((uint8_t*)(&ee_stationID_text[i]));
			if(!g_messages_text[STATION_ID][i])
			{
				break;
			}
		}
	}
	else /* EEPROM is missing some updates */
	{
		if(resetAll || (eeprom_read_byte(&ee_id_codespeed) == 0xFF))
		{
			g_id_codespeed = EEPROM_ID_CODE_SPEED_DEFAULT;
			eeprom_update_byte(&ee_id_codespeed,g_id_codespeed);
		}
		else
		{
			g_id_codespeed = CLAMP(MIN_CODE_SPEED_WPM,eeprom_read_byte(&ee_id_codespeed),MAX_CODE_SPEED_WPM);
		}

		if(resetAll || (eeprom_read_word(&ee_clock_calibration) == 0xFFFF))
		{
			g_clock_calibration = EEPROM_CLOCK_CALIBRATION_DEFAULT;
			eeprom_update_word(&ee_clock_calibration,g_clock_calibration); /* Calibration only gets set by a serial command */
		}
		else
		{
			g_clock_calibration = eeprom_read_word(&ee_clock_calibration);
		}

		if(resetAll || ((uint16_t)eeprom_read_word((uint16_t*)&ee_temp_calibration) == 0xFFFF))
		{
			g_temp_calibration = EEPROM_TEMP_CALIBRATION_DEFAULT;
			eeprom_update_word((uint16_t*)&ee_temp_calibration,(uint16_t)g_temp_calibration);
		}
		else
		{
			g_temp_calibration = (int16_t)eeprom_read_word((uint16_t*)&ee_temp_calibration);
		}

		if(resetAll || (eeprom_read_byte(&ee_override_DIP_switches) == 0xFF))
		{
			g_override_DIP_switches = EEPROM_OVERRIDE_DIP_SW_DEFAULT;
			eeprom_update_byte(&ee_override_DIP_switches,g_override_DIP_switches);  /* Only gets set by a serial command */
		}
		else
		{
			g_override_DIP_switches = eeprom_read_byte(&ee_override_DIP_switches);
		}

		if(resetAll || (eeprom_read_byte(&ee_enable_LEDs) == 0xFF))
		{
			g_enable_LEDs = EEPROM_ENABLE_LEDS_DEFAULT;
			eeprom_update_byte(&ee_enable_LEDs,g_enable_LEDs);  /* Only gets set by a serial command */
		}
		else
		{
			g_enable_LEDs = eeprom_read_byte(&ee_enable_LEDs);
		}

		if(resetAll || (eeprom_read_byte(&ee_enable_start_timer) == 0xFF))
		{
			g_enable_start_timer = EEPROM_ENABLE_STARTTIMER_DEFAULT;
			eeprom_update_byte(&ee_enable_start_timer,g_enable_start_timer);  /* Only gets set by a serial command */
		}
		else
		{
			g_enable_start_timer = eeprom_read_byte(&ee_enable_start_timer);
		}

		if(resetAll || (eeprom_read_byte(&ee_enable_transmitter) == 0xFF))
		{
			g_enable_transmitter = EEPROM_ENABLE_TRANSMITTER_DEFAULT;
			eeprom_update_byte(&ee_enable_transmitter,g_enable_transmitter);  /* Only gets set by a serial command */
		}
		else
		{
			g_enable_transmitter = eeprom_read_byte(&ee_enable_transmitter);
		}

		if(resetAll || (eeprom_read_byte((uint8_t*)(&ee_stationID_text[0])) == 0xFF))
		{
			uint16_t i;
			strncpy(g_messages_text[STATION_ID],EEPROM_STATION_ID_DEFAULT,MAX_PATTERN_TEXT_LENGTH);

			for(i = 0; i < strlen(g_messages_text[STATION_ID]); i++) /* Only gets set by a serial command */
			{
				eeprom_update_byte((uint8_t*)&ee_stationID_text[i],(uint8_t)g_messages_text[STATION_ID][i]);
			}
			eeprom_update_byte((uint8_t*)&ee_stationID_text[i],0);
		}
		else
		{
			for(i = 0; i < MAX_PATTERN_TEXT_LENGTH; i++)
			{
				g_messages_text[STATION_ID][i] = (char)eeprom_read_byte((uint8_t*)(&ee_stationID_text[i]));
				if(!g_messages_text[STATION_ID][i])
				{
					break;
				}
			}
		}

		eeprom_write_byte(&ee_interface_eeprom_initialization_flag,EEPROM_INITIALIZED_FLAG);
	}

	return;
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

//	_delay_ms(200);         /* wait for voltages to become stable. */

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
	float offset = CLAMP(-200.,(float)g_temp_calibration / 10.,200.);

	/* The offset (first term) was determined empirically */
	readADC();  /* throw away first reading */
	return(offset + (readADC() - 324.31) / 1.22);
}


void doSynchronization()
{
	setupForFox();
	cli();
	/* Sync button pin change interrupt */
	PCMSK2 = 0x00;
	PCMSK2 = (1 << PCINT19);    /* Enable PCINT19 */
	PCICR = 0x00;
	PCICR = (1 << PCIE2);       /* Enable pin change interrupt 2 */
	pinMode(PIN_SYNC,INPUT_PULLUP);
	g_sync_enabled = TRUE;

	TCNT1 = 0;                  /* Initialize 1-second counter value to 0 */
	g_seconds_since_sync = 0;
	g_fox_seconds_into_interval = 0;
	g_sync_pin_stable = FALSE;
	digitalWrite(PIN_LED,LOW);
	g_on_the_air = FALSE;
	g_fox_counter = 1;  /* Don't count on the 1-sec timer resetting this quickly enough */
	sei();
}

void setupForFox()
{
	cli();
	pinMode(PIN_SYNC,INPUT_PULLUP);     /* Sync button */
	pinMode(PIN_DIP_0,INPUT_PULLUP);    /* DIP switch LSB */
	pinMode(PIN_DIP_1,INPUT_PULLUP);    /* DIP switch middle bit */
	pinMode(PIN_DIP_2,INPUT_PULLUP);    /* DIP switch MSB */
	digitalWrite(PIN_LED,OFF);          /* Turn off led sync switch is now open */

	g_seconds_since_sync = 0;           /* Total elapsed time counter */
	g_on_the_air       = FALSE;         /* Controls transmitter Morse activity */
	g_code_throttle    = 0;             /* Adjusts Morse code speed */
	g_callsign_sent = FALSE;

	g_on_air_interval = 0;
	g_fox_seconds_into_interval = 0;
	g_number_of_foxes = 0;
	g_fox_transition = FALSE;
	g_fox_id_offset = 0;
	g_id_interval = 0;
	g_time_to_ID = FALSE;
	g_audio_tone_state = OFF;
	sei();

	g_LEDs_Timed_Out = !g_enable_LEDs;

	if(g_override_DIP_switches)
	{
		g_fox = CLAMP(BEACON,(FoxType)g_override_DIP_switches,INVALID_FOX);
		if(g_fox == INVALID_FOX)
		{
			g_fox = BEACON;
		}
	}
	else                                    /* Read DIP Switches */
	{
		g_fox = (FoxType)0;

		if(digitalRead(PIN_DIP_0) == LOW)   /* LSB */
		{
			g_fox += 1;
		}

		if(digitalRead(PIN_DIP_1) == LOW)   /* middle bit */
		{
			g_fox += 2;
		}

		if(digitalRead(PIN_DIP_2) == LOW)   /* MSB */
		{
			g_fox += 4;
		}
	}

#if !HARDWARE_EXTERNAL_DIP_PULLUPS_INSTALLED
		/* Disable pull-ups to save power */
		pinMode(PIN_DIP_0,INPUT);   /* fox switch LSB */
		pinMode(PIN_DIP_1,INPUT);   /* fox switch middle bit */
		pinMode(PIN_DIP_2,INPUT);   /* fox switch MSB */
		pinMode(PIN_DIP_0,OUTPUT);  /* Don't allow pin to float */
		pinMode(PIN_DIP_1,OUTPUT);  /* Don't allow pin to float */
		pinMode(PIN_DIP_2,OUTPUT);  /* Don't allow pin to float */
#endif  /* HARDWARE_EXTERNAL_DIP_PULLUPS_INSTALLED */

	switch(g_fox)
	{
		case NO_CODE_START_TONES_2M:
		{
			g_startclock_interval = 120;
			g_enable_start_timer = TRUE;
		}
		break;

		case NO_CODE_START_TONES_5M:
		{
			g_startclock_interval = 300;
			g_enable_start_timer = TRUE;
		}
		break;

		case FOX_1:
		case FOX_2:
		case FOX_3:
		case FOX_4:
		case FOX_5:
		case FOX_DEMO:
		{
			g_on_air_interval = 60;
			g_number_of_foxes = 5;
			g_fox_id_offset = 0;
			g_pattern_codespeed = 8;
			g_id_interval = 60;
			g_startclock_interval = 300;
		}
		break;

		case SPRINT_S1:
		case SPRINT_F5:
		case SPRINT_DEMO:
		{
			g_on_air_interval = 12;
			g_number_of_foxes = 5;
			g_pattern_codespeed = ((g_fox == SPRINT_DEMO) || (g_fox <= SPRINT_S5)) ? 10 : 15;
			g_fox_id_offset = g_fox <= SPRINT_S5 ? SPRINT_S1 - 1 : SPRINT_F1 - 1;
			g_id_interval = 600;
			g_startclock_interval = 120;
		}
		break;

/* case BEACON: */
/* case SPECTATOR: */
		default:
		{
			g_on_air_interval = 600;
			g_number_of_foxes = 1;
			g_pattern_codespeed = 8;
			g_id_interval = 600;
			g_startclock_interval = (g_fox == SPECTATOR) ? 120 : 300;
		}
		break;
	}
}

void softwareReset()
{
#if USE_WDT_RESET
	cli();
	wdt_enable(WDTO_1S);
	while(1)
	{
		;
	}
#else
	lb_send_string((char*)"Press RESET button now.\n",FALSE);
#endif // USE_WDT_RESET
}
