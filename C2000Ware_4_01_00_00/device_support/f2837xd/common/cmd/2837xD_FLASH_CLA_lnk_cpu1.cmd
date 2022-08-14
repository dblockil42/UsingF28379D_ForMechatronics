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

/*// The user must define CLA_C in the project linker settings if using the
// CLA C compiler
// Project Properties -> C2000 Linker -> Advanced Options -> Command File
// Preprocessing -> --define */
#ifdef CLA_C
/* // Define a size for the CLA scratchpad area that will be used
// by the CLA compiler for local symbols and temps
// Also force references to the special symbols that mark the
// scratchpad are. */
CLA_SCRATCHPAD_SIZE = 0x100;
--undef_sym=__cla_scratchpad_end
--undef_sym=__cla_scratchpad_start
#endif /*//CLA_C */

MEMORY
{
PAGE 0 :  /* Program Memory */
          /* Memory (RAM/FLASH) blocks can be moved to PAGE1 for data allocation */
          /* BEGIN is used for the "boot to Flash" bootloader mode   */

   BEGIN           	: origin = 0x080000, length = 0x000002
   RAMM0           	: origin = 0x000123, length = 0x0002DD
//   RAMD0           	: origin = 0x00B000, length = 0x000800
   RAMLS01			: origin = 0x008000, length = 0x001000
   RAMLS35          : origin = 0x009800, length = 0x001800


//   RAMGS15_RSVD     : origin = 0x01BFF8, length = 0x000008    /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */

   RESET           	: origin = 0x3FFFC0, length = 0x000002

   /* Flash sectors */
   FLASHA           : origin = 0x080002, length = 0x001FFE	/* on-chip Flash */
   FLASHB           : origin = 0x082000, length = 0x002000	/* on-chip Flash */
   FLASHC           : origin = 0x084000, length = 0x002000	/* on-chip Flash */
   FLASHD           : origin = 0x086000, length = 0x002000	/* on-chip Flash */
   FLASHE           : origin = 0x088000, length = 0x008000	/* on-chip Flash */
   FLASHF           : origin = 0x090000, length = 0x008000	/* on-chip Flash */
   FLASHG           : origin = 0x098000, length = 0x008000	/* on-chip Flash */
   FLASHH           : origin = 0x0A0000, length = 0x008000	/* on-chip Flash */
   FLASHI           : origin = 0x0A8000, length = 0x008000	/* on-chip Flash */
   FLASHJ           : origin = 0x0B0000, length = 0x008000	/* on-chip Flash */
   FLASHK           : origin = 0x0B8000, length = 0x002000	/* on-chip Flash */
   FLASHL           : origin = 0x0BA000, length = 0x002000	/* on-chip Flash */
   FLASHM           : origin = 0x0BC000, length = 0x002000	/* on-chip Flash */
   FLASHN           : origin = 0x0BE000, length = 0x001FF0	/* on-chip Flash */

//   FLASHN_RSVD     : origin = 0x0BFFF0, length = 0x000010    /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */

PAGE 1 : /* Data Memory */
         /* Memory (RAM/FLASH) blocks can be moved to PAGE0 for program allocation */

   BOOT_RSVD       : origin = 0x000002, length = 0x000121     /* Part of M0, BOOT rom will use this for stack */
   RAMM1           : origin = 0x000400, length = 0x0003F8     /* on-chip RAM block M1 */
//   RAMM1_RSVD      : origin = 0x0007F8, length = 0x000008     /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */
   RAMD01           : origin = 0x00B000, length = 0x001000
   RAMLS2			: origin = 0x009000, length = 0x000800


   RAMGS015      : origin = 0x00C000, length = 0x00FFF8
   //RAMGS115      : origin = 0x00D000, length = 0x00EFF8


//   RAMGS11     : origin = 0x017000, length = 0x000FF8   /* Uncomment for F28374D, F28376D devices */

//   RAMGS11_RSVD : origin = 0x017FF8, length = 0x000008    /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */

   CLAToCPU_MSGRAM  : origin = 0x001480, length = 0x000080
   CPUToCLA_MSGRAM  : origin = 0x001500, length = 0x000080
   
   CPU2TOCPU1RAM   : origin = 0x03F800, length = 0x000400
   CPU1TOCPU2RAM   : origin = 0x03FC00, length = 0x000400
}

SECTIONS
{
   /* Allocate program areas: */
   .cinit              : > FLASHA | FLASHB | FLASHC | FLASHD | FLASHE |
                            FLASHF | FLASHG | FLASHH | FLASHI | FLASHJ |
                            FLASHK | FLASHL | FLASHM | FLASHN PAGE = 0, ALIGN(8)
   .text               : >> FLASHA | FLASHB | FLASHC | FLASHD | FLASHE |
                            FLASHF | FLASHG | FLASHH | FLASHI | FLASHJ |
                            FLASHK | FLASHL | FLASHM | FLASHN PAGE = 0, ALIGN(8)
   codestart           : > BEGIN       PAGE = 0, ALIGN(8)
   /* Allocate uninitalized data sections: */
   .stack              : > RAMD01       PAGE = 1
   .my_arrs			: > RAMGS015		PAGE = 1
   .my_vars			: > RAMGS015		PAGE = 1
   .switch             : > FLASHA | FLASHB | FLASHC | FLASHD | FLASHE |
                            FLASHF | FLASHG | FLASHH | FLASHI | FLASHJ |
                            FLASHK | FLASHL | FLASHM | FLASHN PAGE = 0, ALIGN(8)
   .reset              : > RESET,      PAGE = 0, TYPE = DSECT /* not used, */

#if defined(__TI_EABI__)
   .init_array         : > FLASHA | FLASHB | FLASHC | FLASHD | FLASHE |
                            FLASHF | FLASHG | FLASHH | FLASHI | FLASHJ |
                            FLASHK | FLASHL | FLASHM | FLASHN,       PAGE = 0,       ALIGN(8)
   .bss                : > RAMGS015,       PAGE = 1
   .bss:output         : > RAMLS01,       PAGE = 0
   .bss:cio            : > RAMGS015,       PAGE = 1
   .data               : > RAMGS015,       PAGE = 1
   .sysmem             : > RAMGS015,       PAGE = 1
   /* Initalized sections go in Flash */
   .const              : > FLASHA | FLASHB | FLASHC | FLASHD | FLASHE |
                            FLASHF | FLASHG | FLASHH | FLASHI | FLASHJ |
                            FLASHK | FLASHL | FLASHM | FLASHN,       PAGE = 0,       ALIGN(8)
#else
   .pinit              : > FLASHA | FLASHB | FLASHC | FLASHD | FLASHE |
                            FLASHF | FLASHG | FLASHH | FLASHI | FLASHJ |
                            FLASHK | FLASHL | FLASHM | FLASHN,       PAGE = 0,       ALIGN(8)
   .ebss               : >> RAMGS015,    PAGE = 1
   .esysmem            : > RAMGS015,       PAGE = 1
   .cio                : > RAMGS015,       PAGE = 1
   /* Initalized sections go in Flash */
   .econst             : >> FLASHA | FLASHB | FLASHC | FLASHD | FLASHE |
                            FLASHF | FLASHG | FLASHH | FLASHI | FLASHJ |
                            FLASHK | FLASHL | FLASHM | FLASHN      PAGE = 0, ALIGN(8)
#endif

    /* CLA specific sections */
   Cla1Prog         : LOAD = FLASHA | FLASHB | FLASHC | FLASHD | FLASHE |
                            FLASHF | FLASHG | FLASHH | FLASHI | FLASHJ |
                            FLASHK | FLASHL | FLASHM | FLASHN,
                      RUN = RAMLS35,
                      LOAD_START(_Cla1funcsLoadStart),
                      LOAD_END(_Cla1funcsLoadEnd),
                      RUN_START(_Cla1funcsRunStart),
                      LOAD_SIZE(_Cla1funcsLoadSize),
                      PAGE = 0, ALIGN(4)

/*   CLADataLS0               : > RAMLS0, PAGE=1 */
   CLADataLS2                   : > RAMLS2, PAGE=1

   Cla1ToCpuMsgRAM  : > CLAToCPU_MSGRAM, PAGE = 1
   CpuToCla1MsgRAM  : > CPUToCLA_MSGRAM, PAGE = 1

/*   Filter_RegsFile     : > RAMGS0,	   PAGE = 1

   SHARERAMGS0		: > RAMGS0,		PAGE = 1
   SHARERAMGS1		: > RAMGS1,		PAGE = 1
   ramgs0           : > RAMGS0,     PAGE = 1
   ramgs1           : > RAMGS1,     PAGE = 1
*/

	FFT_buffer_1 : >> RAMGS015, ALIGN = FFT_ALIGN

	FFT_buffer_2 : >> RAMGS015
	FPUmathTables : >> RAMGS015

	FPUfftTables : >> RAMGS015
	FpuRegsFile : >> RAMGS015


#ifdef __TI_COMPILER_VERSION__
    #if __TI_COMPILER_VERSION__ >= 15009000
        #if defined(__TI_EABI__)
            .TI.ramfunc : {} LOAD = FLASHA | FLASHB | FLASHC | FLASHD | FLASHE |
                            FLASHF | FLASHG | FLASHH | FLASHI | FLASHJ |
                            FLASHK | FLASHL | FLASHM | FLASHN,
                                 RUN = RAMLS01,
                                 LOAD_START(RamfuncsLoadStart),
                                 LOAD_SIZE(RamfuncsLoadSize),
                                 LOAD_END(RamfuncsLoadEnd),
                                 RUN_START(RamfuncsRunStart),
                                 RUN_SIZE(RamfuncsRunSize),
                                 RUN_END(RamfuncsRunEnd),
                                 PAGE = 0, ALIGN(8)
        #else
            .TI.ramfunc : {} LOAD = FLASHA | FLASHB | FLASHC | FLASHD | FLASHE |
                            FLASHF | FLASHG | FLASHH | FLASHI | FLASHJ |
                            FLASHK | FLASHL | FLASHM | FLASHN,
                             RUN = RAMLS01,
                             LOAD_START(_RamfuncsLoadStart),
                             LOAD_SIZE(_RamfuncsLoadSize),
                             LOAD_END(_RamfuncsLoadEnd),
                             RUN_START(_RamfuncsRunStart),
                             RUN_SIZE(_RamfuncsRunSize),
                             RUN_END(_RamfuncsRunEnd),
                             PAGE = 0, ALIGN(8)
        #endif
    #else
   ramfuncs            : LOAD = FLASHA | FLASHB | FLASHC | FLASHD | FLASHE |
                            FLASHF | FLASHG | FLASHH | FLASHI | FLASHJ |
                            FLASHK | FLASHL | FLASHM | FLASHN,
                         RUN = RAMLS01,
                         LOAD_START(_RamfuncsLoadStart),
                         LOAD_SIZE(_RamfuncsLoadSize),
                         LOAD_END(_RamfuncsLoadEnd),
                         RUN_START(_RamfuncsRunStart),
                         RUN_SIZE(_RamfuncsRunSize),
                         RUN_END(_RamfuncsRunEnd),
                         PAGE = 0, ALIGN(8)
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
/*   Filter1_RegsFile : > RAMGS1,	PAGE = 1, fill=0x1111
   Filter2_RegsFile : > RAMGS2,	PAGE = 1, fill=0x2222
   Filter3_RegsFile : > RAMGS3,	PAGE = 1, fill=0x3333
   Filter4_RegsFile : > RAMGS4,	PAGE = 1, fill=0x4444
   Difference_RegsFile : >RAMGS5, 	PAGE = 1, fill=0x3333
*/
#ifdef CLA_C
   /* CLA C compiler sections */
/*   //
   // Must be allocated to memory the CLA has write access to
   // */

   CLAscratch       :
                     { *.obj(CLAscratch)
                     . += CLA_SCRATCHPAD_SIZE;
                     *.obj(CLAscratch_end) } >  RAMLS2,  PAGE = 1

   .scratchpad      : > RAMLS2,       PAGE = 1
   .bss_cla         : > RAMLS2,       PAGE = 1
   .const_cla            :  LOAD = FLASHA | FLASHB | FLASHC | FLASHD | FLASHE |
                            FLASHF | FLASHG | FLASHH | FLASHI | FLASHJ |
                            FLASHK | FLASHL | FLASHM | FLASHN,
                       RUN = RAMLS2,
                       RUN_START(_Cla1ConstRunStart),
                       LOAD_START(_Cla1ConstLoadStart),
                       LOAD_SIZE(_Cla1ConstLoadSize),
                       PAGE = 1
#endif /*//CLA_C */
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
