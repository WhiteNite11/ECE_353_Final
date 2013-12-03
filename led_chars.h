#ifndef __LED_CHARS_H__
#define __LED_CHARS_H__

#define CHAR_NUM              16
#define CHAR_ROW              8

static uint8_t ucDispChar [CHAR_NUM][CHAR_ROW] = 
 {
      { // 0
     0x7E,    // Row 0
     0x7E,    // Row 1
     0x66,    // Row 2
     0x66,    // Row 3
     0x66,    // Row 4
     0x66,    // Row 5
     0x7E,    // Row 6
     0x7E,    // Row 7
   },
   { // 1
     0x18,    // Row 0
     0x18,    // Row 1
     0x18,    // Row 2
     0x18,    // Row 3
     0x18,    // Row 4
     0x18,    // Row 5
     0x18,    // Row 6
     0x18,    // Row 7
   },
   { // 2
     0x7E,    // Row 0
     0x7E,    // Row 1
     0x60,    // Row 2
     0x7E,    // Row 3
     0x7E,    // Row 4
     0x06,    // Row 5
     0x7E,    // Row 6
     0x7E,    // Row 7
   },
   { // 3
     0x7E,    // Row 0
     0x7E,    // Row 1
     0x06,    // Row 2
     0x7E,    // Row 3
     0x7E,    // Row 4
     0x06,    // Row 5
     0x7E,    // Row 6
     0x7E,    // Row 7
   },
   { // 4
     0x06,    // Row 0
     0x06,    // Row 1
     0x06,    // Row 2
     0x7E,    // Row 3
     0x7E,    // Row 4
     0x66,    // Row 5
     0x66,    // Row 6
     0x66,    // Row 7
   },
   { // 5
     0x7E,    // Row 0
     0x7E,    // Row 1
     0x06,    // Row 2
     0x7E,    // Row 3
     0x7E,    // Row 4
     0x60,    // Row 5
     0x7E,    // Row 6
     0x7E,    // Row 7
   },
      { // 6
     0x7E,    // Row 0
     0x7E,    // Row 1
     0x66,    // Row 2
     0x66,    // Row 3
     0x7E,    // Row 4
     0x7E,    // Row 5
     0x60,    // Row 6
     0x7E,    // Row 7
   },
   { // 7
     0x06,    // Row 0
     0x06,    // Row 1
     0x06,    // Row 2
     0x06,    // Row 3
     0x06,    // Row 4
     0x06,    // Row 5
     0x7E,    // Row 6
     0x7E,    // Row 7
   },
   { // 8
     0x7E,    // Row 0
     0x7E,    // Row 1
     0x66,    // Row 2
     0x7E,    // Row 3
     0x7E,    // Row 4
     0x66,    // Row 5
     0x7E,    // Row 6
     0x7E,    // Row 7
   },
   { // 9
     0x06,    // Row 0
     0x06,    // Row 1
     0x06,    // Row 2
     0x7E,    // Row 3
     0x7E,    // Row 4
     0x66,    // Row 5
     0x7E,    // Row 6
     0x7E,    // Row 7
   },
   { //10 (E)
     0x7E,    // Row 0
     0x7E,    // Row 1
     0x60,    // Row 2
     0x7E,    // Row 3
     0x7E,    // Row 4
     0x60,    // Row 5
     0x7E,    // Row 6
     0x7E,    // Row 7
   },
   { // 11 (C)
     0x7E,    // Row 0
     0x7E,    // Row 1
     0x60,    // Row 2
     0x60,    // Row 3
     0x60,    // Row 4
     0x60,    // Row 5
     0x7E,    // Row 6
     0x7E,    // Row 7
   },
 { // 12 (I)
     0x7E,    // Row 0
     0x7E,    // Row 1
     0x18,    // Row 2
     0x18,    // Row 3
     0x18,    // Row 4
     0x18,    // Row 5
     0x7E,    // Row 6
     0x7E,    // Row 7
   },
   { // 13 (HEART)
     0x18,    // Row 0
     0x3C,    // Row 1
     0x7E,    // Row 2
     0xFF,    // Row 3
     0xFF,    // Row 4
     0xFF,    // Row 5
     0x66,    // Row 6
     0x00,    // Row 7
   },
   { // 14 (FACE)
     0x3C,    // Row 0
     0x42,    // Row 1
     0xBD,    // Row 2
     0xC3,    // Row 3
     0x81,    // Row 4
     0xA5,    // Row 5
     0x42,    // Row 6
     0x3C,    // Row 7
   },
   { // 15 (DIAMOND)
     0x18,    // Row 0
     0x3C,    // Row 1
     0x7E,    // Row 2
     0xFF,    // Row 3
     0xFF,    // Row 4
     0x7E,    // Row 5
     0x3C,    // Row 6
     0x18,    // Row 7
   },
 };
 
#endif

