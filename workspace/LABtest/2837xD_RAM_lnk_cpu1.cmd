// The following definitions will help to align the input buffer.For the complex FFT
// of size N, the input buffer must be aligned to a 4N word boundary. For a real FFT
// of size N, the input buffer must be aligned to a 2N word boundary. The user may define
// the macro either in the linker command file, as shown here, or
// through the project properties under,
// C2000 Linker -> Advanced Options -> Command File Preprocessing -> --define
--define FFT_ALIGN=2048
#if !defined(FFT_ALIGN)
#error define FFT_ALIGN under C2000 Linker -> Advanced Options -> Command File Preprocessing -> --define
#endif

MEMORY
{
PAGE 0 :
   /* BEGIN is used for the "boot to SARAM" bootloader mode   */

   BEGIN            : origin = 0x000000, length = 0x000002
   RAMM0            : origin = 0x000123, length = 0x0002DD
   RAMD0            : origin = 0x00B000, length = 0x000800
   RAMLS04           : origin = 0x008000, length = 0x002800

//   RAMGS010      : origin = 0x00C000, length = 0x00b000
   RAMGS0008      : origin = 0x00C000, length = 0x009000

   RESET            : origin = 0x3FFFC0, length = 0x000002

PAGE 1 :

   BOOT_RSVD       : origin = 0x000002, length = 0x000121     /* Part of M0, BOOT rom will use this for stack */
   RAMM1           : origin = 0x000400, length = 0x0003F8     /* on-chip RAM block M1 */
//   RAMM1_RSVD      : origin = 0x0007F8, length = 0x000008     /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */
   RAMD1           : origin = 0x00B800, length = 0x000800

   RAMLS5      : origin = 0x00A800, length = 0x000800


//   RAMGS11     : origin = 0x017000, length = 0x000FF8   /* Uncomment for F28374D, F28376D devices */

//   RAMGS11_RSVD : origin = 0x017FF8, length = 0x000008    /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */

   RAMGS0915     : origin = 0x015000, length = 0x006FF8     /* Only Available on F28379D, F28377D, F28375D devices. Remove line on other devices. */
   
//   RAMGS15_RSVD : origin = 0x01BFF8, length = 0x000008    /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */
                                                            /* Only on F28379D, F28377D, F28375D devices. Remove line on other devices. */

   CPU2TOCPU1RAM   : origin = 0x03F800, length = 0x000400
   CPU1TOCPU2RAM   : origin = 0x03FC00, length = 0x000400

   CANA_MSG_RAM     : origin = 0x049000, length = 0x000800
   CANB_MSG_RAM     : origin = 0x04B000, length = 0x000800
}


SECTIONS
{
   codestart        : > BEGIN,     PAGE = 0
   .text            : >> RAMLS04 | RAMGS0008,   PAGE = 0
   .cinit           : > RAMD0,     PAGE = 0
   .switch          : > RAMD0,     PAGE = 0
   .reset           : > RESET,     PAGE = 0, TYPE = DSECT /* not used, */
   .stack           : > RAMD1,     PAGE = 1
   .my_arrs			: > RAMGS0915 PAGE = 1
   .my_vars			: > RAMGS0915 PAGE = 1

#if defined(__TI_EABI__)
   .bss             : > RAMLS5 | RAMGS0915,    PAGE = 1
   .bss:output      : > RAMLS04 | RAMGS0008,    PAGE = 0
   .init_array      : > RAMD0,     PAGE = 0
   .const           : > RAMLS5 | RAMGS0915,    PAGE = 1
   .data            : > RAMLS5 | RAMGS0915,    PAGE = 1
   .sysmem          : > RAMLS5 | RAMGS0915,    PAGE = 1
#else
   .pinit           : > RAMD0,     PAGE = 0
   .ebss            : > RAMLS5 | RAMGS0915,    PAGE = 1
   .econst          : > RAMLS5 | RAMGS0915,    PAGE = 1
   .esysmem         : > RAMLS5 | RAMGS0915,    PAGE = 1
#endif
/*
   Filter_RegsFile  : > RAMGS0,    PAGE = 1


   ramgs0           : > RAMGS0,    PAGE = 1
   ramgs1           : > RAMGS1,    PAGE = 1
*/

	FFT_buffer_1 : >> RAMGS0915, ALIGN = FFT_ALIGN

	FFT_buffer_2 : >> RAMGS0915
	FPUmathTables : >> RAMGS0915

	FPUfftTables : >> RAMGS0915
	FpuRegsFile : >> RAMGS0915


#ifdef __TI_COMPILER_VERSION__
   #if __TI_COMPILER_VERSION__ >= 15009000
    .TI.ramfunc : {} > RAMM0,      PAGE = 0
   #else
    ramfuncs    : > RAMM0      PAGE = 0   
   #endif
#endif

   /* The following section definitions are required when using the IPC API Drivers */
    GROUP : > CPU1TOCPU2RAM, PAGE = 1
    {
        PUTBUFFER
        PUTWRITEIDX
        GETREADIDX
    }

    GROUP : > CPU2TOCPU1RAM, PAGE = 1
    {
        GETBUFFER :    TYPE = DSECT
        GETWRITEIDX :  TYPE = DSECT
        PUTREADIDX :   TYPE = DSECT
    }

    /* The following section definition are for SDFM examples */
/*   Filter1_RegsFile : > RAMGS1, PAGE = 1, fill=0x1111
   Filter2_RegsFile : > RAMGS2, PAGE = 1, fill=0x2222
   Filter3_RegsFile : > RAMGS3, PAGE = 1, fill=0x3333
   Filter4_RegsFile : > RAMGS4, PAGE = 1, fill=0x4444
   Difference_RegsFile : >RAMGS5,   PAGE = 1, fill=0x3333
*/
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
