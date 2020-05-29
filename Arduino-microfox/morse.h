/*
 * morse.h
 *
 * Created: 3/19/2018 3:16:02 PM
 *  Author: charl
 */


#ifndef MORSE_H_
#define MORSE_H_

#include "defs.h"

#define PROCESSSOR_CLOCK_HZ			(16000000L)
#define WPM_TO_MS_PER_DOT(w)		(1200/(w))
#define THROTTLE_VAL_FROM_WPM(w)	(PROCESSSOR_CLOCK_HZ / 8000000L) * ((7042 / (w)) / 10)

/*
*/
typedef struct {
	uint8_t		pattern;
	uint8_t		lengthInSymbols;
	uint8_t		lengthInElements;
} MorseCharacter;

/**
Load a string to send by passing in a pointer to the string in the argument.
Call this function with a NULL argument at intervals of 1 element of time to generate Morse code.
Once loaded with a string each call to this function returns a BOOL indicating whether a CW carrier should be sent
 */
BOOL makeMorse(char* s, BOOL* repeating, BOOL* finished);

/**
Returns the number of milliseconds required to send the string pointed to by the first argument at the WPM code speed
passed in the second argument.
*/
uint16_t timeRequiredToSendStrAtWPM(char* str, uint16_t spd);

#endif /* MORSE_H_ */