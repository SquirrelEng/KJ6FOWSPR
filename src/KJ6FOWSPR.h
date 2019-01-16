/*
 * KJ6FOWSPR.h - JT65/JT9/WSPR/FSQ encoder library for Arduino
 *
 * Modified by Don Gibson KJ6FO to support WSPR Type 2 & Type 3 message formats.
 * Modified from the original code at https://github.com/etherkit/JTEncode
 *
 * The original code only Supported WSPR Type 1 (4 Digit Grid w Standard Callsigns)
 * Added WSPR TYPE 2 & 3 Message types. This allows for compund callsigns and 6 Digit Grids Squares.
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

#include "intKJ6FO.h"
//#include "rs_commonKJ6FO.h"

#include "Arduino.h"

#include <stdint.h>

#define JT65_SYMBOL_COUNT                   126
#define JT9_SYMBOL_COUNT                    85
#define JT4_SYMBOL_COUNT                    207
#define WSPR_SYMBOL_COUNT                   162

#define JT65_ENCODE_COUNT                   63
#define JT9_ENCODE_COUNT                    69

#define JT9_BIT_COUNT                       206
#define JT4_BIT_COUNT                       206
#define WSPR_BIT_COUNT                      162

// Define the structure of a varicode table
typedef struct fsq_varicode
{
    uint8_t ch;
    uint8_t var[2];
} Varicode;

// The FSQ varicode table, based on the FSQ Varicode V3.0
// document provided by Murray Greenman, ZL1BPU

static const Varicode fsq_code_table[] PROGMEM =
{
  {' ', {00, 00}}, // space
  {'!', {11, 30}},
  {'"', {12, 30}},
  {'#', {13, 30}},
  {'$', {14, 30}},
  {'%', {15, 30}},
  {'&', {16, 30}},
  {'\'', {17, 30}},
  {'(', {18, 30}},
  {')', {19, 30}},
  {'*', {20, 30}},
  {'+', {21, 30}},
  {',', {27, 29}},
  {'-', {22, 30}},
  {'.', {27, 00}},
  {'/', {23, 30}},
  {'0', {10, 30}},
  {'1', {01, 30}},
  {'2', {02, 30}},
  {'3', {03, 30}},
  {'4', {04, 30}},
  {'5', {05, 30}},
  {'6', {06, 30}},
  {'7', {07, 30}},
  {'8', {8, 30}},
  {'9', {9, 30}},
  {':', {24, 30}},
  {';', {25, 30}},
  {'<', {26, 30}},
  {'=', {00, 31}},
  {'>', {27, 30}},
  {'?', {28, 29}},
  {'@', {00, 29}},
  {'A', {01, 29}},
  {'B', {02, 29}},
  {'C', {03, 29}},
  {'D', {04, 29}},
  {'E', {05, 29}},
  {'F', {06, 29}},
  {'G', {07, 29}},
  {'H', {8, 29}},
  {'I', {9, 29}},
  {'J', {10, 29}},
  {'K', {11, 29}},
  {'L', {12, 29}},
  {'M', {13, 29}},
  {'N', {14, 29}},
  {'O', {15, 29}},
  {'P', {16, 29}},
  {'Q', {17, 29}},
  {'R', {18, 29}},
  {'S', {19, 29}},
  {'T', {20, 29}},
  {'U', {21, 29}},
  {'V', {22, 29}},
  {'W', {23, 29}},
  {'X', {24, 29}},
  {'Y', {25, 29}},
  {'Z', {26, 29}},
  {'[', {01, 31}},
  {'\\', {02, 31}},
  {']', {03, 31}},
  {'^', {04, 31}},
  {'_', {05, 31}},
  {'`', {9, 31}},
  {'a', {01, 00}},
  {'b', {02, 00}},
  {'c', {03, 00}},
  {'d', {04, 00}},
  {'e', {05, 00}},
  {'f', {06, 00}},
  {'g', {07, 00}},
  {'h', {8, 00}},
  {'i', {9, 00}},
  {'j', {10, 00}},
  {'k', {11, 00}},
  {'l', {12, 00}},
  {'m', {13, 00}},
  {'n', {14, 00}},
  {'o', {15, 00}},
  {'p', {16, 00}},
  {'q', {17, 00}},
  {'r', {18, 00}},
  {'s', {19, 00}},
  {'t', {20, 00}},
  {'u', {21, 00}},
  {'v', {22, 00}},
  {'w', {23, 00}},
  {'x', {24, 00}},
  {'y', {25, 00}},
  {'z', {26, 00}},
  {'{', {06, 31}},
  {'|', {07, 31}},
  {'}', {8, 31}},
  {'~', {00, 30}},
  {127, {28, 31}}, // DEL
  {13,  {28, 00}}, // CR
  {10,  {28, 00}}, // LF
  {0,   {28, 30}}, // IDLE
  {241, {10, 31}}, // plus/minus
  {246, {11, 31}}, // division sign
  {248, {12, 31}}, // degrees sign
  {158, {13, 31}}, // multiply sign
  {156, {14, 31}}, // pound sterling sign
  {8,   {27, 31}}  // BS
};

const static uint8_t crc8_table[] PROGMEM = {
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
    0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
    0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
    0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
    0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
    0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
    0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
    0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
    0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
    0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
    0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
    0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
    0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
    0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
    0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
    0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
    0xfa, 0xfd, 0xf4, 0xf3
};

const uint8_t jt9i[JT9_BIT_COUNT] PROGMEM = {
  0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0x10, 0x90, 0x50, 0x30, 0xb0, 0x70,
  0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0x18, 0x98, 0x58, 0x38, 0xb8, 0x78,
  0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0x14, 0x94, 0x54, 0x34, 0xb4, 0x74,
  0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0x1c, 0x9c, 0x5c, 0x3c, 0xbc, 0x7c,
  0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0x12, 0x92, 0x52, 0x32, 0xb2, 0x72,
  0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0x1a, 0x9a, 0x5a, 0x3a, 0xba, 0x7a,
  0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0x16, 0x96, 0x56, 0x36, 0xb6, 0x76,
  0x0e, 0x8e, 0x4e, 0x2e, 0xae, 0x6e, 0x1e, 0x9e, 0x5e, 0x3e, 0xbe, 0x7e, 0x01,
  0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0x11, 0x91, 0x51, 0x31, 0xb1, 0x71, 0x09,
  0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0x19, 0x99, 0x59, 0x39, 0xb9, 0x79, 0x05,
  0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0x15, 0x95, 0x55, 0x35, 0xb5, 0x75, 0x0d,
  0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0x1d, 0x9d, 0x5d, 0x3d, 0xbd, 0x7d, 0x03,
  0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0x13, 0x93, 0x53, 0x33, 0xb3, 0x73, 0x0b,
  0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0x1b, 0x9b, 0x5b, 0x3b, 0xbb, 0x7b, 0x07,
  0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0x17, 0x97, 0x57, 0x37, 0xb7, 0x77, 0x0f,
  0x8f, 0x4f, 0x2f, 0xaf, 0x6f, 0x1f, 0x9f, 0x5f, 0x3f, 0xbf, 0x7f
};

class KJ6FOWSPR
{
public:
  KJ6FOWSPR(void);
  void jt65_encode(const char *, uint8_t *);
  void jt9_encode(const char *, uint8_t *);
  void jt4_encode(const char *, uint8_t *);
  void wspr_encode(const char *, const char *, const uint8_t, uint8_t *);
  uint16_t fsq_encode(const char *, const char *, uint8_t *);
  uint16_t fsq_dir_encode(const char *, const char *, const char, const char *, uint8_t *);
  char *CRC2Hex(uint8_t crc, char *HexStr);
private:
  uint8_t jt_code(char);
  uint8_t wspr_code(char);
  uint8_t gray_code(uint8_t);
  void jt_message_prep(char *);
  void wspr_message_prep(char *, char *, uint8_t);
  void jt65_bit_packing(char *, uint8_t *);
  void jt9_bit_packing(char *, uint8_t *);
  void wspr_bit_packing(uint8_t *);
  void jt65_interleave(uint8_t *);
  void jt9_interleave(uint8_t *);
  void wspr_interleave(uint8_t *);
  void jt9_packbits(uint8_t *, uint8_t *);
  void jt_gray_code(uint8_t *, uint8_t);
  void jt65_merge_sync_vector(uint8_t *, uint8_t *);
  void jt9_merge_sync_vector(uint8_t *, uint8_t *);
  void jt4_merge_sync_vector(uint8_t *, uint8_t *);
  void wspr_merge_sync_vector(uint8_t *, uint8_t *);
  void convolve(uint8_t *, uint8_t *, uint8_t, uint8_t);
  void rs_encode(uint8_t *, uint8_t *);
  void encode_rs_int(void *,data_t *, data_t *);
  void free_rs_int(void *);
  void * init_rs_int(int, int, int, int, int, int);
  uint8_t crc8(const char *);
  //void * rs_inst;
  char callsign[7];
  char locator[5];
  uint8_t power;

  // Added Extentions for WSPR
public:void wspr_encodeType2(char * call, const uint8_t dbm, uint8_t * symbols);
private:unsigned long packpfx(char * Callsign, char *call1, int *nadd_out, unsigned long *ng_out);
private:unsigned long packcall(char *call1);
private:char chr_normf(char bc);
private:void pack50(uint8_t *EncodedMessage, unsigned long n1, unsigned long n2);
public:void wspr_encodeType3(char * call, const char * loc, const uint8_t dbm, uint8_t * symbols);
private:long hash(char *Callsign);
private:uint32_t nhash_(const void *key, int *length0, uint32_t *initval0);
};
