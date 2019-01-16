/*
 * KJ6FOWSPR.cpp - JT65/JT9/WSPR/FSQ encoder library for Arduino
 *
 * Modified Version of JTEncode by Jason Milldrum. Enhanced to support WSPR type 2 (Comppound callsigns) and type 3 6 Digit Grids with hashed callsign
 Enhanced by Don Gibson KJ6FO.
 *
 * Copyright (C) Don Gibson 2018-2019
 *
 * Portions Copyright (C) 2015-2016 Jason Milldrum <milldrum@gmail.com>
 *
 * Based on the algorithms presented in the WSJT software suite.
 * Thanks to Andy Talbot G4JNT for the whitepaper on the WSPR encoding
 * process that helped me to understand all of this.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <KJ6FOWSPR.h>

#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
#include <avr/pgmspace.h>
#endif

#include "Arduino.h"

// Define an upper bound on the number of glyphs.  Defining it this
// way allows adding characters without having to update a hard-coded
// upper bound.
#define NGLYPHS         (sizeof(fsq_code_table)/sizeof(fsq_code_table[0]))

/* Public Class Members */

KJ6FOWSPR::KJ6FOWSPR(void)
{
// Nothing to do yet.
}

/*
 * jt65_encode(const char * msg, uint8_t * symbols)
 *
 * Takes an arbitrary message of up to 13 allowable characters and returns
 * a channel symbol table.
 *
 * message - Plaintext Type 6 message.
 * symbols - Array of channel symbols to transmit retunred by the method.
 *  Ensure that you pass a uint8_t array of size JT65_SYMBOL_COUNT to the method.
 *
 */
void KJ6FOWSPR::jt65_encode(const char * msg, uint8_t * symbols)
{
  char message[14];
  memset(message, 0, 14);
  strcpy(message, msg);

  // Ensure that the message text conforms to standards
  // --------------------------------------------------
  jt_message_prep(message);

  // Bit packing
  // -----------
  uint8_t c[12];
  jt65_bit_packing(message, c);

  // Reed-Solomon encoding
  // ---------------------
  uint8_t s[JT65_ENCODE_COUNT];
  rs_encode(c, s);

  // Interleaving
  // ------------
  jt65_interleave(s);

  // Gray Code
  // ---------
  jt_gray_code(s, JT65_ENCODE_COUNT);

  // Merge with sync vector
  // ----------------------
  jt65_merge_sync_vector(s, symbols);
}

/*
 * jt9_encode(const char * msg, uint8_t * symbols)
 *
 * Takes an arbitrary message of up to 13 allowable characters and returns
 * a channel symbol table.
 *
 * message - Plaintext Type 6 message.
 * symbols - Array of channel symbols to transmit retunred by the method.
 *  Ensure that you pass a uint8_t array of size JT9_SYMBOL_COUNT to the method.
 *
 */
void KJ6FOWSPR::jt9_encode(const char * msg, uint8_t * symbols)
{
  char message[14];
  memset(message, 0, 14);
  strcpy(message, msg);

  // Ensure that the message text conforms to standards
  // --------------------------------------------------
  jt_message_prep(message);

  // Bit packing
  // -----------
  uint8_t c[13];
  jt9_bit_packing(message, c);

  // Convolutional Encoding
  // ---------------------
  uint8_t s[JT9_BIT_COUNT];
  convolve(c, s, 13, JT9_BIT_COUNT);

  // Interleaving
  // ------------
  jt9_interleave(s);

  // Pack into 3-bit symbols
  // -----------------------
  uint8_t a[JT9_ENCODE_COUNT];
  jt9_packbits(s, a);

  // Gray Code
  // ---------
  jt_gray_code(a, JT9_ENCODE_COUNT);

  // Merge with sync vector
  // ----------------------
  jt9_merge_sync_vector(a, symbols);
}

/*
 * jt4_encode(const char * msg, uint8_t * symbols)
 *
 * Takes an arbitrary message of up to 13 allowable characters and returns
 * a channel symbol table.
 *
 * message - Plaintext Type 6 message.
 * symbols - Array of channel symbols to transmit retunred by the method.
 *  Ensure that you pass a uint8_t array of size JT4_SYMBOL_COUNT to the method.
 *
 */
void KJ6FOWSPR::jt4_encode(const char * msg, uint8_t * symbols)
{
  char message[14];
  memset(message, 0, 14);
  strcpy(message, msg);

  // Ensure that the message text conforms to standards
  // --------------------------------------------------
  jt_message_prep(message);

  // Bit packing
  // -----------
  uint8_t c[13];
  jt9_bit_packing(message, c);

  // Convolutional Encoding
  // ---------------------
  uint8_t s[JT4_SYMBOL_COUNT];
  convolve(c, s, 13, JT4_BIT_COUNT);

  // Interleaving
  // ------------
  jt9_interleave(s);
  memmove(s + 1, s, JT4_BIT_COUNT);
  s[0] = 0; // Append a 0 bit to start of sequence

  // Merge with sync vector
  // ----------------------
  jt4_merge_sync_vector(s, symbols);
}

/*
 * wspr_encode(const char * call, const char * loc, const uint8_t dbm, uint8_t * symbols)
 *
 * Takes an arbitrary message of up to 13 allowable characters and returns
 *
 * call - Callsign (6 characters maximum).
 * loc - Maidenhead grid locator (4 charcters maximum).
 * dbm - Output power in dBm.
 * symbols - Array of channel symbols to transmit retunred by the method.
 *  Ensure that you pass a uint8_t array of size WSPR_SYMBOL_COUNT to the method.
 *
 */
void KJ6FOWSPR::wspr_encode(const char * call, const char * loc, const uint8_t dbm, uint8_t * symbols)
{
  char call_[7];
  char loc_[5];
  uint8_t dbm_ = dbm;
  strcpy(call_, call);
  strcpy(loc_, loc);

  // Ensure that the message text conforms to standards
  // --------------------------------------------------
  wspr_message_prep(call_, loc_, dbm_);

  // Bit packing
  // -----------
  uint8_t c[11];
  wspr_bit_packing(c);

  // Convolutional Encoding
  // ---------------------
  uint8_t s[WSPR_SYMBOL_COUNT];
  convolve(c, s, 11, WSPR_BIT_COUNT);

  // Interleaving
  // ------------
  wspr_interleave(s);

  // Merge with sync vector
  // ----------------------
  wspr_merge_sync_vector(s, symbols);
}



/*
 * fsq_encode(const char * from_call, const char * message, uint8_t * symbols)
 *
 * Takes an arbitrary message and returns a FSQ channel symbol table.
 *
 * from_call - Callsign of issuing station (maximum size: 20)
 * message - Null-terminated message string, no greater than 130 chars in length
 * symbols - Array of channel symbols to transmit retunred by the method.
 *  Ensure that you pass a uint8_t array of at least the size of the message
 *  plus 5 characters to the method. Terminated in 0xFF.
 *
 */
uint16_t KJ6FOWSPR::fsq_encode(const char * from_call, const char * message, uint8_t * symbols)
{
  char tx_buffer[155];
  char * tx_message;
  uint16_t symbol_pos = 0;
  uint8_t i, fch, vcode1, vcode2, tone;
  uint8_t cur_tone = 0;

  // Clear out the transmit buffer
  // -----------------------------
  memset(tx_buffer, 0, 155);

  // Create the message to be transmitted
  // ------------------------------------
  //sprintf(tx_buffer, "  \n%s: %s", from_call, message);
  strcpy(tx_buffer, "  \n");
  strcat(tx_buffer, from_call);
  strcat(tx_buffer, ": ");
  strcat(tx_buffer, message);

  tx_message = tx_buffer;

  // Iterate through the message and encode
  // --------------------------------------
  while(*tx_message != '\0')
  {
    for(i = 0; i < NGLYPHS; i++)
    {
      uint8_t ch = (uint8_t)*tx_message;

      // Check each element of the varicode table to see if we've found the
      // character we're trying to send.
      fch = pgm_read_byte(&fsq_code_table[i].ch);

      if(fch == ch)
      {
          // Found the character, now fetch the varicode chars
          vcode1 = pgm_read_byte(&(fsq_code_table[i].var[0]));
          vcode2 = pgm_read_byte(&(fsq_code_table[i].var[1]));

          // Transmit the appropriate tone per a varicode char
          if(vcode2 == 0)
          {
            // If the 2nd varicode char is a 0 in the table,
            // we are transmitting a lowercase character, and thus
            // only transmit one tone for this character.

            // Generate tone
            cur_tone = ((cur_tone + vcode1 + 1) % 33);
            symbols[symbol_pos++] = cur_tone;
          }
          else
          {
            // If the 2nd varicode char is anything other than 0 in
            // the table, then we need to transmit both

            // Generate 1st tone
            cur_tone = ((cur_tone + vcode1 + 1) % 33);
            symbols[symbol_pos++] = cur_tone;

            // Generate 2nd tone
            cur_tone = ((cur_tone + vcode2 + 1) % 33);
            symbols[symbol_pos++] = cur_tone;
          }
          break; // We've found and transmitted the char,
             // so exit the for loop
        }
    }

    tx_message++;
  }

  // Message termination
  // ----------------
  symbols[symbol_pos] = 0xff;
  return(symbol_pos);
}


/*
 * fsq_dir_encode(const char * from_call, const char * to_call, const char cmd, const char * message, uint8_t * symbols)
 *
 * Takes an arbitrary message and returns a FSQ channel symbol table.
 *
 * from_call - Callsign from which message is directed (maximum size: 20)
 * to_call - Callsign to which message is directed (maximum size: 20)
 * cmd - Directed command
 * message - Null-terminated message string, no greater than 100 chars in length
 * symbols - Array of channel symbols to transmit retunred by the method.
 *  Ensure that you pass a uint8_t array of at least the size of the message
 *  plus 5 characters to the method. Terminated in 0xFF.
 *
 */
uint16_t KJ6FOWSPR::fsq_dir_encode(const char * from_call, const char * to_call, const char cmd, const char * message, uint8_t * symbols)
{
  char tx_buffer[155];
  char * tx_message;
  uint16_t symbol_pos = 0;
  uint8_t i, fch, vcode1, vcode2, tone, from_call_crc;
  uint8_t cur_tone = 0;

  // Generate a CRC on from_call
  // ---------------------------
  from_call_crc = crc8(from_call);

  // Clear out the transmit buffer
  // -----------------------------
  memset(tx_buffer, 0, 155);

  // Create the message to be transmitted
  // We are building a directed message here.
  // FSQ very specifically needs "  \b  " in
  // directed mode to indicate EOT. A single backspace won't do it.
  //sprintf(tx_buffer, "  \n%s:%02x%s%c%s%s", from_call, from_call_crc, to_call, cmd, message, "  \b  ");
  char HexCRC[3];
  char cmdbuf[2];
  strcpy(tx_buffer, "  \n");
  strcat(tx_buffer, from_call);
  strcat(tx_buffer, ":");
  strcat(tx_buffer, CRC2Hex(from_call_crc, HexCRC));
  strcat(tx_buffer, to_call);
  cmdbuf[0] = cmd;
  cmdbuf[1] = 0;
  strcat(tx_buffer, cmdbuf);
  strcat(tx_buffer, message);
  strcat(tx_buffer, "  \b  ");

  tx_message = tx_buffer;

  // Iterate through the message and encode
  // --------------------------------------
  while(*tx_message != '\0')
  {
    for(i = 0; i < NGLYPHS; i++)
    {
      uint8_t ch = (uint8_t)*tx_message;

      // Check each element of the varicode table to see if we've found the
      // character we're trying to send.
      fch = pgm_read_byte(&fsq_code_table[i].ch);

      if(fch == ch)
      {
          // Found the character, now fetch the varicode chars
          vcode1 = pgm_read_byte(&(fsq_code_table[i].var[0]));
          vcode2 = pgm_read_byte(&(fsq_code_table[i].var[1]));

          // Transmit the appropriate tone per a varicode char
          if(vcode2 == 0)
          {
            // If the 2nd varicode char is a 0 in the table,
            // we are transmitting a lowercase character, and thus
            // only transmit one tone for this character.

            // Generate tone
            cur_tone = ((cur_tone + vcode1 + 1) % 33);
            symbols[symbol_pos++] = cur_tone;
          }
          else
          {
            // If the 2nd varicode char is anything other than 0 in
            // the table, then we need to transmit both

            // Generate 1st tone
            cur_tone = ((cur_tone + vcode1 + 1) % 33);
            symbols[symbol_pos++] = cur_tone;

            // Generate 2nd tone
            cur_tone = ((cur_tone + vcode2 + 1) % 33);
            symbols[symbol_pos++] = cur_tone;
          }
          break; // We've found and transmitted the char,
             // so exit the for loop
        }
    }

    tx_message++;
  }

  // Message termination
  // ----------------
  symbols[symbol_pos] = 0xff;
  return(symbol_pos);
}

// Converts one byte to hex string cvalue (2 chars + null)
#define TO_HEX(i) (i <= 9 ? '0' + i : 'a' - 10 + i)
char * KJ6FOWSPR::CRC2Hex(uint8_t crc, char *HexStr)
{
	if (crc <= 0xFF)
	{
		HexStr[0] = TO_HEX(((crc & 0x00F0) >> 4));
		HexStr[1] = TO_HEX((crc & 0x000F));
		HexStr[2] = '\0';
	}
	else
	{
		HexStr[0] = 0;
	}
	return HexStr;
}
/* Private Class Members */

uint8_t KJ6FOWSPR::jt_code(char c)
{
  // Validate the input then return the proper integer code.
  // Return 255 as an error code if the char is not allowed.

  if(isdigit(c))
  {
    return (uint8_t)(c - 48);
  }
  else if(c >= 'A' && c <= 'Z')
  {
    return (uint8_t)(c - 55);
  }
  else if(c == ' ')
  {
    return 36;
  }
  else if(c == '+')
  {
    return 37;
  }
  else if(c == '-')
  {
    return 38;
  }
  else if(c == '.')
  {
    return 39;
  }
  else if(c == '/')
  {
    return 40;
  }
  else if(c == '?')
  {
    return 41;
  }
  else
  {
    return 255;
  }
}

uint8_t KJ6FOWSPR::wspr_code(char c)
{
  // Validate the input then return the proper integer code.
  // Return 255 as an error code if the char is not allowed.

  if(isdigit(c))
	{
		return (uint8_t)(c - 48);
	}
	else if(c == ' ')
	{
		return 36;
	}
	else if(c >= 'A' && c <= 'Z')
	{
		return (uint8_t)(c - 55);
	}
	else
	{
		return 255;
	}
}

uint8_t KJ6FOWSPR::gray_code(uint8_t c)
{
  return (c >> 1) ^ c;
}

void KJ6FOWSPR::jt_message_prep(char * message)
{
  uint8_t i;

  // Pad the message with trailing spaces
  uint8_t len = strlen(message);
  if(len < 13)
  {
    for(i = len; i <= 13; i++)
    {
      message[i] = ' ';
    }
  }

  // Convert all chars to uppercase
  for(i = 0; i < 13; i++)
  {
    if(islower(message[i]))
    {
      message[i] = toupper(message[i]);
    }
  }
}

void KJ6FOWSPR::wspr_message_prep(char * call, char * loc, uint8_t dbm)
{
  // Callsign validation and padding
  // -------------------------------

	// If only the 2nd character is a digit, then pad with a space.
	// If this happens, then the callsign will be truncated if it is
	// longer than 5 characters.
	if((call[1] >= '0' && call[1] <= '9') && (call[2] < '0' || call[2] > '9'))
	{
		memmove(call + 1, call, 5);
		call[0] = ' ';
	}

	// Now the 3rd charcter in the callsign must be a digit
	if(call[2] < '0' || call[2] > '9')
	{
    // TODO: need a better way to handle this
		call[2] = '0';
	}

	// Ensure that the only allowed characters are digits and
	// uppercase letters
	uint8_t i;
	for(i = 0; i < 6; i++)
	{
		call[i] = toupper(call[i]);
		if(!(isdigit(call[i]) || isupper(call[i])))
		{
			call[i] = ' ';
		}
	}

  memcpy(callsign, call, 6);

	// Grid locator validation
	for(i = 0; i < 4; i++)
	{
		loc[i] = toupper(loc[i]);
		if(!(isdigit(loc[i]) || (loc[i] >= 'A' && loc[i] <= 'R')))
		{
      memcpy(loc, "AA00", 5);
      //loc = "AA00";
		}
	}

  memcpy(locator, loc, 4);

	// Power level validation
	// Only certain increments are allowed
	if(dbm > 60)
	{
		dbm = 60;
	}
   static const uint8_t valid_dbm[19] PROGMEM =
    {0, 3, 7, 10, 13, 17, 20, 23, 27, 30, 33, 37, 40,
     43, 47, 50, 53, 57, 60};
  for(i = 0; i < 19; i++)
  {
	uint8_t db = pgm_read_byte(&valid_dbm[i]);
    if(dbm == db)
    {
      power = dbm;
    }
  }
  // If we got this far, we have an invalid power level, so we'll round down
  for(i = 1; i < 19; i++)
  {
    if(dbm < valid_dbm[i] && dbm >= valid_dbm[i - 1])
    {
      power = valid_dbm[i - 1];
    }
  }
}

void KJ6FOWSPR::jt65_bit_packing(char * message, uint8_t * c)
{
  uint32_t n1, n2, n3;

  // Find the N values
  n1 = jt_code(message[0]);
  n1 = n1 * 42 + jt_code(message[1]);
  n1 = n1 * 42 + jt_code(message[2]);
  n1 = n1 * 42 + jt_code(message[3]);
  n1 = n1 * 42 + jt_code(message[4]);

  n2 = jt_code(message[5]);
  n2 = n2 * 42 + jt_code(message[6]);
  n2 = n2 * 42 + jt_code(message[7]);
  n2 = n2 * 42 + jt_code(message[8]);
  n2 = n2 * 42 + jt_code(message[9]);

  n3 = jt_code(message[10]);
  n3 = n3 * 42 + jt_code(message[11]);
  n3 = n3 * 42 + jt_code(message[12]);

  // Pack bits 15 and 16 of N3 into N1 and N2,
  // then mask reset of N3 bits
  n1 = (n1 << 1) + ((n3 >> 15) & 1);
  n2 = (n2 << 1) + ((n3 >> 16) & 1);
  n3 = n3 & 0x7fff;

  // Set the freeform message flag
  n3 += 32768;

  c[0] = (n1 >> 22) & 0x003f;
  c[1] = (n1 >> 16) & 0x003f;
  c[2] = (n1 >> 10) & 0x003f;
  c[3] = (n1 >> 4) & 0x003f;
  c[4] = ((n1 & 0x000f) << 2) + ((n2 >> 26) & 0x0003);
  c[5] = (n2 >> 20) & 0x003f;
  c[6] = (n2 >> 14) & 0x003f;
  c[7] = (n2 >> 8) & 0x003f;
  c[8] = (n2 >> 2) & 0x003f;
  c[9] = ((n2 & 0x0003) << 4) + ((n3 >> 12) & 0x000f);
  c[10] = (n3 >> 6) & 0x003f;
  c[11] = n3 & 0x003f;
}

void KJ6FOWSPR::jt9_bit_packing(char * message, uint8_t * c)
{
  uint32_t n1, n2, n3;

  // Find the N values
  n1 = jt_code(message[0]);
  n1 = n1 * 42 + jt_code(message[1]);
  n1 = n1 * 42 + jt_code(message[2]);
  n1 = n1 * 42 + jt_code(message[3]);
  n1 = n1 * 42 + jt_code(message[4]);

  n2 = jt_code(message[5]);
  n2 = n2 * 42 + jt_code(message[6]);
  n2 = n2 * 42 + jt_code(message[7]);
  n2 = n2 * 42 + jt_code(message[8]);
  n2 = n2 * 42 + jt_code(message[9]);

  n3 = jt_code(message[10]);
  n3 = n3 * 42 + jt_code(message[11]);
  n3 = n3 * 42 + jt_code(message[12]);

  // Pack bits 15 and 16 of N3 into N1 and N2,
  // then mask reset of N3 bits
  n1 = (n1 << 1) + ((n3 >> 15) & 1);
  n2 = (n2 << 1) + ((n3 >> 16) & 1);
  n3 = n3 & 0x7fff;

  // Set the freeform message flag
  n3 += 32768;

  // 71 message bits to pack, plus 1 bit flag for freeform message.
  // 31 zero bits appended to end.
  // N1 and N2 are 28 bits each, N3 is 16 bits
  // A little less work to start with the least-significant bits
  c[3] = (uint8_t)((n1 & 0x0f) << 4);
  n1 = n1 >> 4;
  c[2] = (uint8_t)(n1 & 0xff);
  n1 = n1 >> 8;
  c[1] = (uint8_t)(n1 & 0xff);
  n1 = n1 >> 8;
  c[0] = (uint8_t)(n1 & 0xff);

  c[6] = (uint8_t)(n2 & 0xff);
  n2 = n2 >> 8;
  c[5] = (uint8_t)(n2 & 0xff);
  n2 = n2 >> 8;
  c[4] = (uint8_t)(n2 & 0xff);
  n2 = n2 >> 8;
  c[3] |= (uint8_t)(n2 & 0x0f);

  c[8] = (uint8_t)(n3 & 0xff);
  n3 = n3 >> 8;
  c[7] = (uint8_t)(n3 & 0xff);

  c[9] = 0;
  c[10] = 0;
  c[11] = 0;
  c[12] = 0;
}

void KJ6FOWSPR::wspr_bit_packing(uint8_t * c)
{
  uint32_t n, m;

	n = wspr_code(callsign[0]);
	n = n * 36 + wspr_code(callsign[1]);
	n = n * 10 + wspr_code(callsign[2]);
	n = n * 27 + (wspr_code(callsign[3]) - 10);
	n = n * 27 + (wspr_code(callsign[4]) - 10);
	n = n * 27 + (wspr_code(callsign[5]) - 10);

	m = ((179 - 10 * (locator[0] - 'A') - (locator[2] - '0')) * 180) +
		(10 * (locator[1] - 'A')) + (locator[3] - '0');
	m = (m * 128) + power + 64;

	// Callsign is 28 bits, locator/power is 22 bits.
	// A little less work to start with the least-significant bits
	c[3] = (uint8_t)((n & 0x0f) << 4);
	n = n >> 4;
	c[2] = (uint8_t)(n & 0xff);
	n = n >> 8;
	c[1] = (uint8_t)(n & 0xff);
	n = n >> 8;
	c[0] = (uint8_t)(n & 0xff);

	c[6] = (uint8_t)((m & 0x03) << 6);
	m = m >> 2;
	c[5] = (uint8_t)(m & 0xff);
	m = m >> 8;
	c[4] = (uint8_t)(m & 0xff);
	m = m >> 8;
	c[3] |= (uint8_t)(m & 0x0f);
	c[7] = 0;
	c[8] = 0;
	c[9] = 0;
	c[10] = 0;
}

void KJ6FOWSPR::jt65_interleave(uint8_t * s)
{
  uint8_t i, j;
  uint8_t d[JT65_ENCODE_COUNT];

  // Interleave
  for(i = 0; i < 9; i++)
  {
    for(j = 0; j < 7; j++)
    {
      d[(j * 9) + i] = s[(i * 7) + j];
    }
  }

  memcpy(s, d, JT65_ENCODE_COUNT);
}

void KJ6FOWSPR::jt9_interleave(uint8_t * s)
{
  uint8_t i, j;
  uint8_t d[JT9_BIT_COUNT];

  // Do the interleave
  for(i = 0; i < JT9_BIT_COUNT; i++)
  {
    //#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
    #if defined(__arm__)
    d[jt9i[i]] = s[i];
    #else
    j = pgm_read_byte(&jt9i[i]);
    d[j] = s[i];
    #endif
  }

  memcpy(s, d, JT9_BIT_COUNT);
}

void KJ6FOWSPR::wspr_interleave(uint8_t * s)
{
  uint8_t d[WSPR_BIT_COUNT];
	uint8_t rev, index_temp, i, j, k;

	
	i = 0;

	for(j = 0; j < 255; j++)
	{
		// Bit reverse the index
		index_temp = j;
		rev = 0;

		for(k = 0; k < 8; k++)
		{
			if(index_temp & 0x01)
			{
				rev = rev | (1 << (7 - k));
			}
			index_temp = index_temp >> 1;
		}

		if(rev < WSPR_BIT_COUNT)
		{
			d[rev] = s[i];
			i++;
		}

		if(i >= WSPR_BIT_COUNT)
		{
			break;
		}
	}

	
  memcpy(s, d, WSPR_BIT_COUNT);
}

void KJ6FOWSPR::jt9_packbits(uint8_t * d, uint8_t * a)
{
  uint8_t i, k;
  k = 0;
  memset(a, 0, JT9_ENCODE_COUNT);

  for(i = 0; i < JT9_ENCODE_COUNT; i++)
  {
    a[i] = (d[k] & 1) << 2;
    k++;

    a[i] |= ((d[k] & 1) << 1);
    k++;

    a[i] |= (d[k] & 1);
    k++;
  }
}

void KJ6FOWSPR::jt_gray_code(uint8_t * g, uint8_t symbol_count)
{
  uint8_t i;

  for(i = 0; i < symbol_count; i++)
  {
    g[i] = gray_code(g[i]);
  }
}

void KJ6FOWSPR::jt65_merge_sync_vector(uint8_t * g, uint8_t * symbols)
{
  uint8_t i, j = 0;
  const uint8_t sync_vector[JT65_SYMBOL_COUNT] =
  {1, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0,
   0, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1,
   0, 1, 1, 0, 1, 1, 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1,
   0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
   1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1,
   0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1, 1,
   1, 1, 1, 1, 1, 1};

  for(i = 0; i < JT65_SYMBOL_COUNT; i++)
  {
    if(sync_vector[i])
    {
      symbols[i] = 0;
    }
    else
    {
      symbols[i] = g[j] + 2;
      j++;
    }
  }
}

void KJ6FOWSPR::jt9_merge_sync_vector(uint8_t * g, uint8_t * symbols)
{
  uint8_t i, j = 0;
  const uint8_t sync_vector[JT9_SYMBOL_COUNT] =
  {1, 1, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
   0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 1,
   0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 1, 0, 1};

  for(i = 0; i < JT9_SYMBOL_COUNT; i++)
  {
    if(sync_vector[i])
    {
      symbols[i] = 0;
    }
    else
    {
      symbols[i] = g[j] + 1;
      j++;
    }
  }
}

void KJ6FOWSPR::jt4_merge_sync_vector(uint8_t * g, uint8_t * symbols)
{
  uint8_t i;
  const uint8_t sync_vector[JT4_SYMBOL_COUNT] =
	{0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0,
   0, 0, 0, 0, 1, 1, 0, 0, 0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,1 ,0 ,1 ,1,
   0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0,
   1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 0,
   0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0,
   1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1,
   1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1,
   0, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1,
   1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1,
   0, 1, 1, 1, 1, 0, 1, 0, 1};

	for(i = 0; i < JT4_SYMBOL_COUNT; i++)
	{
		symbols[i] = sync_vector[i] + (2 * g[i]);
	}
}

void KJ6FOWSPR::wspr_merge_sync_vector(uint8_t * g, uint8_t * symbols)
{
  uint8_t i;
  static const uint8_t sync_vector[WSPR_SYMBOL_COUNT] PROGMEM =
	{1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0,
	 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0,
	 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1,
	 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0,
	 1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1,
	 0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1,
	 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0,
	 1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0};

	for(i = 0; i < WSPR_SYMBOL_COUNT; i++)
	{
		uint8_t b = pgm_read_byte(&sync_vector[i]);
		symbols[i] = b + (2 * g[i]);
	}
}

void KJ6FOWSPR::convolve(uint8_t * c, uint8_t * s, uint8_t message_size, uint8_t bit_size)
{
  uint32_t reg_0 = 0;
  uint32_t reg_1 = 0;
  uint32_t reg_temp = 0;
  uint8_t input_bit, parity_bit;
  uint8_t bit_count = 0;
  uint8_t i, j, k;

  for(i = 0; i < message_size; i++)
  {
    for(j = 0; j < 8; j++)
    {
      // Set input bit according the MSB of current element
      input_bit = (((c[i] << j) & 0x80) == 0x80) ? 1 : 0;

      // Shift both registers and put in the new input bit
      reg_0 = reg_0 << 1;
      reg_1 = reg_1 << 1;
      reg_0 |= (uint32_t)input_bit;
      reg_1 |= (uint32_t)input_bit;

      // AND Register 0 with feedback taps, calculate parity
      reg_temp = reg_0 & 0xf2d05351;
      parity_bit = 0;

	 

      for(k = 0; k < 32; k++)
      {
        parity_bit = parity_bit ^ (reg_temp & 0x01);

        reg_temp = reg_temp >> 1;
      }
	  
      s[bit_count] = parity_bit;
      bit_count++;

      // AND Register 1 with feedback taps, calculate parity
      reg_temp = reg_1 & 0xe4613c47;
      parity_bit = 0;
      for(k = 0; k < 32; k++)
      {
        parity_bit = parity_bit ^ (reg_temp & 0x01);
        reg_temp = reg_temp >> 1;
      }
	 
      s[bit_count] = parity_bit;
      bit_count++;
      if(bit_count >= bit_size)
      {
        break;
      }
    }
  }
}

//void KJ6FOWSPR::rs_encode(uint8_t * data, uint8_t * symbols)
//{
//  // Adapted from wrapkarn.c in the WSJT-X source code
//  uint8_t dat1[12];
//  uint8_t b[51];
//  uint8_t sym[JT65_ENCODE_COUNT];
//  uint8_t i;
//
//  // Reverse data order for the Karn codec.
//  for(i = 0; i < 12; i++)
//  {
//    dat1[i] = data[11 - i];
//  }
//
//  // Compute the parity symbols
//  encode_rs_int(rs_inst, dat1, b);
//
//  // Move parity symbols and data into symbols array, in reverse order.
//  for (i = 0; i < 51; i++)
//  {
//    sym[50 - i] = b[i];
//  }
//
//  for (i = 0; i < 12; i++)
//  {
//    sym[i + 51] = dat1[11 - i];
//  }
//
//  memcpy(symbols, sym, JT65_ENCODE_COUNT);
//}

uint8_t KJ6FOWSPR::crc8(const char * text)
{
  uint8_t crc = '\0';
  uint8_t ch;

  int i;
  for(i = 0; i < strlen(text); i++)
  {
    ch = text[i];
    //#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__)
    #if defined(__arm__)
    crc = crc8_table[(crc) ^ ch];
    #else
    crc = pgm_read_byte(&(crc8_table[(crc) ^ ch]));
    #endif
    crc &= 0xFF;
  }

  return crc;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
// WSPR Extention functions



// Compound Callsign up to 8 characters AA6ABC/B
void KJ6FOWSPR::wspr_encodeType2(char * Callsign, const uint8_t ndbm, uint8_t * symbols)
{

	// Ensure that the message text conforms to standards
	// --------------------------------------------------
	//wspr_message_prep(call_, loc_, dbm_);

	// Bit packing
	// -----------
	uint8_t c[11];

	int nadd = 0;
	unsigned long ng = 0;
	char call1[9];
	unsigned long n1 = packpfx(Callsign, call1, &nadd, &ng);

	int ntype = ndbm + 1 + nadd;
	unsigned long n2 = 128 * ng + ntype + 64;
	
	pack50(c, n1, n2);


	// Convolutional Encoding
	// ---------------------
	uint8_t s[WSPR_SYMBOL_COUNT];
	convolve(c, s, 11, WSPR_BIT_COUNT);

	// Interleaving
	// ------------
	wspr_interleave(s);

	// Merge with sync vector
	// ----------------------
	wspr_merge_sync_vector(s, symbols);
}

void KJ6FOWSPR::wspr_encodeType3(char * CallSign, const char * My6DigitGridSquare, const uint8_t ndbm, uint8_t * symbols)
{
	uint8_t c[11];

	int nadd = 0;
	unsigned long ng = 0;
	char call1[9];


	long ihash = hash(CallSign);
	for (int ii = 1; ii < 6; ii++)  // 6 Digit GridSquare
	{
		call1[ii - 1] = My6DigitGridSquare[ii];
	};
	call1[5] = My6DigitGridSquare[0];
	unsigned long n1 = packcall(call1);
	int ntype = -(ndbm + 1);
	unsigned long n2 = 128 * ihash + ntype + 64;
	pack50(c, n1, n2);


	// Convolutional Encoding
	// ---------------------
	uint8_t s[WSPR_SYMBOL_COUNT];

	convolve(c, s, 11, WSPR_BIT_COUNT);

	// Interleaving
	// ------------
	wspr_interleave(s);

	// Merge with sync vector
	// ----------------------
	wspr_merge_sync_vector(s, symbols);
}



#define HASH_LITTLE_ENDIAN 1

/*
-------------------------------------------------------------------------------
mix -- mix 3 32-bit values reversibly.

This is reversible, so any information in (a,b,c) before mix() is
still in (a,b,c) after mix().

If four pairs of (a,b,c) inputs are run through mix(), or through
mix() in reverse, there are at least 32 bits of the output that
are sometimes the same for one pair and different for another pair.
This was tested for:
* pairs that differed by one bit, by two bits, in any combination
of top bits of (a,b,c), or in any combination of bottom bits of
(a,b,c).
* "differ" is defined as +, -, ^, or ~^.  For + and -, I transformed
the output delta to a Gray code (a^(a>>1)) so a string of 1's (as
is commonly produced by subtraction) look like a single 1-bit
difference.
* the base values were pseudorandom, all zero but one bit set, or
all zero plus a counter that starts at zero.

Some k values for my "a-=c; a^=rot(c,k); c+=b;" arrangement that
satisfy this are
4  6  8 16 19  4
9 15  3 18 27 15
14  9  3  7 17  3
Well, "9 15 3 18 27 15" didn't quite get 32 bits diffing
for "differ" defined as + with a one-bit base and a two-bit delta.  I
used http://burtleburtle.net/bob/hash/avalanche.html to choose
the operations, constants, and arrangements of the variables.

This does not achieve avalanche.  There are input bits of (a,b,c)
that fail to affect some output bits of (a,b,c), especially of a.  The
most thoroughly mixed value is c, but it doesn't really even achieve
avalanche in c.

This allows some parallelism.  Read-after-writes are good at doubling
the number of bits affected, so the goal of mixing pulls in the opposite
direction as the goal of parallelism.  I did what I could.  Rotates
seem to cost as much as shifts on every machine I could lay my hands
on, and rotates are much kinder to the top and bottom bits, so I used
rotates.
-------------------------------------------------------------------------------
*/
#define mix(a,b,c) \
{ \
  a -= c;  a ^= rot(c, 4);  c += b; \
  b -= a;  b ^= rot(a, 6);  a += c; \
  c -= b;  c ^= rot(b, 8);  b += a; \
  a -= c;  a ^= rot(c,16);  c += b; \
  b -= a;  b ^= rot(a,19);  a += c; \
  c -= b;  c ^= rot(b, 4);  b += a; \
}

/*
-------------------------------------------------------------------------------
final -- final mixing of 3 32-bit values (a,b,c) into c

Pairs of (a,b,c) values differing in only a few bits will usually
produce values of c that look totally different.  This was tested for
* pairs that differed by one bit, by two bits, in any combination
of top bits of (a,b,c), or in any combination of bottom bits of
(a,b,c).
* "differ" is defined as +, -, ^, or ~^.  For + and -, I transformed
the output delta to a Gray code (a^(a>>1)) so a string of 1's (as
is commonly produced by subtraction) look like a single 1-bit
difference.
* the base values were pseudorandom, all zero but one bit set, or
all zero plus a counter that starts at zero.

These constants passed:
14 11 25 16 4 14 24
12 14 25 16 4 14 24
and these came close:
4  8 15 26 3 22 24
10  8 15 26 3 22 24
11  8 15 26 3 22 24
-------------------------------------------------------------------------------
*/
#define final(a,b,c) \
{ \
  c ^= b; c -= rot(b,14); \
  a ^= c; a -= rot(c,11); \
  b ^= a; b -= rot(a,25); \
  c ^= b; c -= rot(b,16); \
  a ^= c; a -= rot(c,4);  \
  b ^= a; b -= rot(a,14); \
  c ^= b; c -= rot(b,24); \
}

#define hashsize(n) ((uint32_t)1<<(n))
#define hashmask(n) (hashsize(n)-1)
#define rot(x,k) (((x)<<(k)) | ((x)>>(32-(k))))


// Pack callsign w prefix.
unsigned long KJ6FOWSPR::packpfx(char * Callsign, char *call1, int *nadd_out, unsigned long *ng_out)
{
	char pfx[3];
	int Len;
	int slash;
	int nc;
	int n;

	unsigned long n1;  // Used in pack50()
	int nadd;
	unsigned long ng;


	for (int i = 0; i < 7; i++)
	{
		call1[i] = 0;
	};
	Len = strlen(Callsign);
	for (int i = 0; i < 13; i++)
	{
		if (Callsign[i] == 47) slash = i;
	};
	if (Callsign[slash + 2] == 0) //single char
	{
		for (int i = 0; i < slash; i++)
		{
			call1[i] = Callsign[i];
		};
		n1 = packcall(call1);
		nadd = 1;
		nc = int(Callsign[slash + 1]);
		if (nc >= 48 && nc <= 57) n = nc - 48;
		else if (nc >= 65 && nc <= 90) n = nc - 65 + 10;
		else if (nc >= 97 && nc <= 122) n = nc - 97 + 10;
		else n = 38;
		ng = 60000 - 32768 + n;
	}
	else
		if (Callsign[slash + 3] == 0)  // Two char 
		{
			for (int i = 0; i < slash; i++)
			{
				call1[i] = Callsign[i];
			};
			n1 = packcall(call1);
			n = 10 * (int(Callsign[slash + 1]) - 48) + int(Callsign[slash + 2]) - 48;
			nadd = 1;
			ng = 60000 + 26 + n;
		}
		else  //??
		{
			for (int i = 0; i < slash; i++)
			{
				pfx[i] = Callsign[i];
			};
			if (slash == 2)
			{
				pfx[2] = pfx[1];
				pfx[1] = pfx[0];
				pfx[0] = ' ';
			};
			if (slash == 1)
			{
				pfx[2] = pfx[0];
				pfx[1] = ' ';
				pfx[0] = ' ';
			};
			int ii = 0;
			for (int i = slash + 1; i < Len; i++)
			{
				call1[ii] = Callsign[i];
				ii++;
			};
			n1 = packcall(call1);
			ng = 0;
			for (int i = 0; i < 3; i++)
			{
				nc = int(pfx[i]);
				if (nc >= 48 && nc <= 57) n = nc - 48;
				else if (nc >= 65 && nc <= 90) n = nc - 65 + 10;
				else if (nc >= 97 && nc <= 122) n = nc - 97 + 10;
				else n = 36;
				ng = 37 * ng + n;
			};
			nadd = 0;
			if (ng >= 32768)
			{
				ng = ng - 32768;
				nadd = 1;
			};
		}

	*nadd_out = nadd;
	*ng_out = ng;
	return n1;
}

unsigned long KJ6FOWSPR::packcall(char *call1)
{
	unsigned long n1;

	// coding of callsign
	if (chr_normf(call1[2]) > 9)
	{
		call1[5] = call1[4];
		call1[4] = call1[3];
		call1[3] = call1[2];
		call1[2] = call1[1];
		call1[1] = call1[0];
		call1[0] = ' ';
	}

	n1 = chr_normf(call1[0]);
	n1 = n1 * 36 + chr_normf(call1[1]);
	n1 = n1 * 10 + chr_normf(call1[2]);
	n1 = n1 * 27 + chr_normf(call1[3]) - 10;
	n1 = n1 * 27 + chr_normf(call1[4]) - 10;
	n1 = n1 * 27 + chr_normf(call1[5]) - 10;

	return n1;
}

// normalize characters 0..9 A..Z Space in order 0..36
char KJ6FOWSPR::chr_normf(char bc)
{
	char cc = 36;
	if (bc >= '0' && bc <= '9') cc = bc - '0';
	if (bc >= 'A' && bc <= 'Z') cc = bc - 'A' + 10;
	if (bc >= 'a' && bc <= 'z') cc = bc - 'a' + 10;
	if (bc == ' ') cc = 36;

	return(cc);
}

void KJ6FOWSPR::pack50(uint8_t *EncodedMessage, unsigned long n1, unsigned long n2)
{
	// merge coded callsign into message array c[]
	unsigned long t1 = n1;
	EncodedMessage[0] = (byte)(t1 >> 20);
	t1 = n1;
	EncodedMessage[1] = (byte)(t1 >> 12);
	t1 = n1;
	EncodedMessage[2] = (byte)(t1 >> 4);
	t1 = n1;
	EncodedMessage[3] = (byte)(t1 << 4);
	t1 = n2;
	EncodedMessage[3] = EncodedMessage[3] + (0x0f & t1 >> 18);
	t1 = n2;
	EncodedMessage[4] = (byte)(t1 >> 10);
	t1 = n2;
	EncodedMessage[5] = (byte)(t1 >> 2);
	t1 = n2;
	EncodedMessage[6] = (byte)(t1 << 6);

	// The array is 11 elements long, but why? Only 7 are used here, but on convolve 8 are used. This comes from original JTEncode code.
	// Probably an error here, but init values to zero to keep random calsc from occuring.
	EncodedMessage[7] = 0;
	EncodedMessage[8] = 0;
	EncodedMessage[9] = 0;
	EncodedMessage[10] = 0;
}

long KJ6FOWSPR::hash(char *Callsign)
{
	int Len;
	uint32_t jhash;
	int *pLen = &Len;
	Len = strlen(Callsign);
	byte IC[12];
	byte *pIC = IC;
	//for (i = 0;i<12;i++)  // What is this doing?   Advances pointer to end of something? Why not just use math?
	//{
	//	//pIC + 1; // Bump pointer up to next item  not assigned to anything
	//	&IC[i];  // Does nothing?
	//}
	uint32_t Val = 146;
	//uint32_t *pVal = &Val;
	for (int i = 0; i < Len; i++) // Copy callsign to IC byte Array
	{
		IC[i] = int(Callsign[i]);
	};
	jhash = nhash_(pIC, pLen, &Val);

	//#define MASK15  32767L;
	long ihash = jhash & 32767L;  // 15 Bit Mask
	return ihash;
}


uint32_t KJ6FOWSPR::nhash_(const void *key, int *length0, uint32_t *initval0)
{
	uint32_t a, b, c;                                          /* internal state */
	size_t length;
	uint32_t initval;
	union { const void *ptr; size_t i; } u;     /* needed for Mac Powerbook G4 */

	length = *length0;
	initval = *initval0;

	/* Set up the internal state */
	a = b = c = 0xdeadbeef + ((uint32_t)length) + initval;

	u.ptr = key;
	if (HASH_LITTLE_ENDIAN && ((u.i & 0x3) == 0)) {
		const uint32_t *k = (const uint32_t *)key;         /* read 32-bit chunks */
		const uint8_t  *k8;

		k8 = 0;                                     //Silence compiler warning
													/*------ all but last block: aligned reads and affect 32 bits of (a,b,c) */
		while (length > 12)
		{
			a += k[0];
			b += k[1];
			c += k[2];
			mix(a, b, c);
			length -= 12;
			k += 3;
		}

		/*----------------------------- handle the last (probably partial) block */
		/*
		* "k[2]&0xffffff" actually reads beyond the end of the string, but
		* then masks off the part it's not allowed to read.  Because the
		* string is aligned, the masked-off tail is in the same word as the
		* rest of the string.  Every machine with memory protection I've seen
		* does it on word boundaries, so is OK with this.  But VALGRIND will
		* still catch it and complain.  The masking trick does make the hash
		* noticably faster for short strings (like English words).
		*/
#ifndef VALGRIND

		switch (length)
		{
		case 12: c += k[2]; b += k[1]; a += k[0]; break;
		case 11: c += k[2] & 0xffffff; b += k[1]; a += k[0]; break;
		case 10: c += k[2] & 0xffff; b += k[1]; a += k[0]; break;
		case 9: c += k[2] & 0xff; b += k[1]; a += k[0]; break;
		case 8: b += k[1]; a += k[0]; break;
		case 7: b += k[1] & 0xffffff; a += k[0]; break;
		case 6: b += k[1] & 0xffff; a += k[0]; break;
		case 5: b += k[1] & 0xff; a += k[0]; break;
		case 4: a += k[0]; break;
		case 3: a += k[0] & 0xffffff; break;
		case 2: a += k[0] & 0xffff; break;
		case 1: a += k[0] & 0xff; break;
		case 0: return c;              /* zero length strings require no mixing */
		}

#else /* make valgrind happy */

		k8 = (const uint8_t *)k;
		switch (length)
		{
		case 12: c += k[2]; b += k[1]; a += k[0]; break;
		case 11: c += ((uint32_t)k8[10]) << 16;  /* fall through */
		case 10: c += ((uint32_t)k8[9]) << 8;    /* fall through */
		case 9: c += k8[8];                   /* fall through */
		case 8: b += k[1]; a += k[0]; break;
		case 7: b += ((uint32_t)k8[6]) << 16;   /* fall through */
		case 6: b += ((uint32_t)k8[5]) << 8;    /* fall through */
		case 5: b += k8[4];                   /* fall through */
		case 4: a += k[0]; break;
		case 3: a += ((uint32_t)k8[2]) << 16;   /* fall through */
		case 2: a += ((uint32_t)k8[1]) << 8;    /* fall through */
		case 1: a += k8[0]; break;
		case 0: return c;
		}

#endif /* !valgrind */

	}
	else if (HASH_LITTLE_ENDIAN && ((u.i & 0x1) == 0)) {
		const uint16_t *k = (const uint16_t *)key;         /* read 16-bit chunks */
		const uint8_t  *k8;

		/*--------------- all but last block: aligned reads and different mixing */
		while (length > 12)
		{
			a += k[0] + (((uint32_t)k[1]) << 16);
			b += k[2] + (((uint32_t)k[3]) << 16);
			c += k[4] + (((uint32_t)k[5]) << 16);
			mix(a, b, c);
			length -= 12;
			k += 6;
		}

		/*----------------------------- handle the last (probably partial) block */
		k8 = (const uint8_t *)k;
		switch (length)
		{
		case 12: c += k[4] + (((uint32_t)k[5]) << 16);
			b += k[2] + (((uint32_t)k[3]) << 16);
			a += k[0] + (((uint32_t)k[1]) << 16);
			break;
		case 11: c += ((uint32_t)k8[10]) << 16;     /* fall through */
		case 10: c += k[4];
			b += k[2] + (((uint32_t)k[3]) << 16);
			a += k[0] + (((uint32_t)k[1]) << 16);
			break;
		case 9: c += k8[8];                      /* fall through */
		case 8: b += k[2] + (((uint32_t)k[3]) << 16);
			a += k[0] + (((uint32_t)k[1]) << 16);
			break;
		case 7: b += ((uint32_t)k8[6]) << 16;      /* fall through */
		case 6: b += k[2];
			a += k[0] + (((uint32_t)k[1]) << 16);
			break;
		case 5: b += k8[4];                      /* fall through */
		case 4: a += k[0] + (((uint32_t)k[1]) << 16);
			break;
		case 3: a += ((uint32_t)k8[2]) << 16;      /* fall through */
		case 2: a += k[0];
			break;
		case 1: a += k8[0];
			break;
		case 0: return c;                     /* zero length requires no mixing */
		}

	}
	else {                        /* need to read the key one byte at a time */
		const uint8_t *k = (const uint8_t *)key;

		/*--------------- all but the last block: affect some 32 bits of (a,b,c) */
		while (length > 12)
		{
			a += k[0];
			a += ((uint32_t)k[1]) << 8;
			a += ((uint32_t)k[2]) << 16;
			a += ((uint32_t)k[3]) << 24;
			b += k[4];
			b += ((uint32_t)k[5]) << 8;
			b += ((uint32_t)k[6]) << 16;
			b += ((uint32_t)k[7]) << 24;
			c += k[8];
			c += ((uint32_t)k[9]) << 8;
			c += ((uint32_t)k[10]) << 16;
			c += ((uint32_t)k[11]) << 24;
			mix(a, b, c);
			length -= 12;
			k += 12;
		}

		/*-------------------------------- last block: affect all 32 bits of (c) */
		switch (length)                   /* all the case statements fall through */
		{
		case 12: c += ((uint32_t)k[11]) << 24;
		case 11: c += ((uint32_t)k[10]) << 16;
		case 10: c += ((uint32_t)k[9]) << 8;
		case 9: c += k[8];
		case 8: b += ((uint32_t)k[7]) << 24;
		case 7: b += ((uint32_t)k[6]) << 16;
		case 6: b += ((uint32_t)k[5]) << 8;
		case 5: b += k[4];
		case 4: a += ((uint32_t)k[3]) << 24;
		case 3: a += ((uint32_t)k[2]) << 16;
		case 2: a += ((uint32_t)k[1]) << 8;
		case 1: a += k[0];
			break;
		case 0: return c;
		}
	}

	final(a, b, c);
	return c;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////