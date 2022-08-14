/**********************************************************************************
* File: Lab_11_cpu1.cmd -- File for Lab 11_cpu1 (Boot to SARAM boot mode)
* Devices: TMS320F28379D
* Author: C2000 Technical Training, Texas Instruments
**********************************************************************************/

MEMORY
{
PAGE 0:    /* Program Memory */
   BEGIN_M0            : origin = 0x000000, length = 0x000002     /* Part of M0 RAM - used for "Boot to M0" bootloader mode */
   RAMLS4              : origin = 0x00A000, length = 0x000800     /* L4 RAM, DCSM secure, CLA Program RAM */
   RAMLS5              : origin = 0x00A800, length = 0x000800     /* L5 RAM, DCSM secure, CLA Program RAM */
   RAMGS0123           : origin = 0x00C000, length = 0x004000     /* GS0-3 RAM, Parity, DMA */
   BEGIN_FLASH         : origin = 0x080000, length = 0x000002     /* Part of FLASH Sector A - used for "Jump to flash" bootloader mode */
   FLASH_A             : origin = 0x080002, length = 0x001FFE     /* Part of FLASH Sector A - DCSM secure */
   FLASH_BCDEFGHIJKLMN : origin = 0x082000, length = 0x03E000     /* FLASH Sectors B,C,D,E,F,G,H,I,J,K,L,M,N combined - DCSM secure */
   RESET           (R) : origin = 0x3FFFC0, length = 0x000002     /* Part of Boot ROM */

PAGE 1:    /* Data Memory */
   BOOT_RSVD           : origin = 0x000002, length = 0x00004E     /* Part of M0 RAM, BOOT rom will use this for stack */
   RAMM0               : origin = 0x000050, length = 0x0003B0     /* M0 RAM */
   RAMM1               : origin = 0x000400, length = 0x000400     /* M1 RAM */
   CLA1_MSGRAMLOW      : origin = 0x001480, length = 0x000080     /* CLA to CPU Message RAM, DCSM secure */
   CLA1_MSGRAMHIGH     : origin = 0x001500, length = 0x000080     /* CPU to CLA Message RAM, DCSM secure */
   RAMLS0              : origin = 0x008000, length = 0x000800     /* L0 RAM, DCSM secure, CLA Data RAM */
   RAMLS1              : origin = 0x008800, length = 0x000800     /* L1 RAM, DCSM secure, CLA Data RAM */
   RAMLS2              : origin = 0x009000, length = 0x000800     /* L2 RAM, DCSM secure, CLA Data RAM */
   RAMLS3              : origin = 0x009800, length = 0x000800     /* L3 RAM, DCSM secure, CLA Data RAM */
   RAMD0               : origin = 0x00B000, length = 0x000800     /* D0 RAM, DCSM secure, ECC */
   RAMD1               : origin = 0x00B800, length = 0x000800     /* D1 RAM, DCSM secure, ECC */   
   RAMGS4              : origin = 0x010000, length = 0x001000     /* GS4 RAM, Parity, DMA */
   RAMGS5              : origin = 0x011000, length = 0x001000     /* GS5 RAM, Parity, DMA */
   RAMGS6              : origin = 0x012000, length = 0x001000     /* GS6 RAM, Parity, DMA */
   RAMGS7              : origin = 0x013000, length = 0x001000     /* GS7 RAM, Parity, DMA */
   RAMGS89ABCDEF       : origin = 0x014000, length = 0x008000     /* GS8-15 RAM, Parity, DMA */
}

 
SECTIONS
{
/*** Compiler Required Sections ***/

  /* Program memory (PAGE 0) sections */
   .text               : > RAMGS0123,             PAGE = 0
   .cinit              : > RAMGS0123,             PAGE = 0
   .const              : > RAMGS0123,             PAGE = 0
   .econst             : > RAMGS0123,             PAGE = 0      
   .pinit              : > RAMGS0123,             PAGE = 0
   .reset              : > RESET,                 PAGE = 0, TYPE = DSECT  /* Not using the .reset section */
   .switch             : > RAMGS0123,             PAGE = 0

  /* Data Memory (PAGE 1) sections */
   .bss                : > RAMM0,                 PAGE = 1
   .ebss               : > RAMM0,                 PAGE = 1
   .cio                : > RAMM0,                 PAGE = 1
   .stack              : > RAMM1,                 PAGE = 1
   .sysmem             : > RAMM1,                 PAGE = 1
   .esysmem            : > RAMM1,                 PAGE = 1

/*** User Defined Sections ***/
   codestart          : > BEGIN_M0,              PAGE = 0                /* Used by file CodeStartBranch.asm */
}

/******************* end of file ************************/
