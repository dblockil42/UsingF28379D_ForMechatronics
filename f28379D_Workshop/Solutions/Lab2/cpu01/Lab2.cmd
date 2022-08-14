/* FILE FOR LAB2.CMD */

MEMORY
{
PAGE 0:    /* Program Memory */
   RAMLS4              : origin = 0x00A000, length = 0x000800
   RAMLS5              : origin = 0x00A800, length = 0x000800
   RAMGS0123           : origin = 0x00C000, length = 0x004000

PAGE 1:    /* Data Memory */
   RAMM0               : origin = 0x000000, length = 0x000400
   RAMM1               : origin = 0x000400, length = 0x000400
   RAMLS0              : origin = 0x008000, length = 0x000800
   RAMLS1              : origin = 0x008800, length = 0x000800
   RAMLS2              : origin = 0x009000, length = 0x000800
   RAMLS3              : origin = 0x009800, length = 0x000800
   RAMD0               : origin = 0x00B000, length = 0x000800
   RAMD1               : origin = 0x00B800, length = 0x000800
   RAMGS4              : origin = 0x010000, length = 0x001000
   RAMGS5              : origin = 0x011000, length = 0x001000
   RAMGS6              : origin = 0x012000, length = 0x001000
   RAMGS7              : origin = 0x013000, length = 0x001000
   RAMGS89ABCDEF       : origin = 0x014000, length = 0x001000
}

SECTIONS
{
   .text               : > RAMGS0123,             PAGE = 0
   .cinit              : > RAMGS0123,             PAGE = 0
   .ebss               : > RAMM0,                 PAGE = 1
   .stack              : > RAMM1,                 PAGE = 1
   .reset              : > RAMGS0123,             PAGE = 0, TYPE = DSECT  /* Not using the .reset section */
}
