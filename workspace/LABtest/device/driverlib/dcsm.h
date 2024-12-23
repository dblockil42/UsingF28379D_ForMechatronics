//#############################################################################
//
// FILE:   dcsm.h
//
// TITLE:  C28x Driver for the DCSM security module.
//
//#############################################################################
// $Copyright:
// Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

#ifndef DCSM_H
#define DCSM_H

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \addtogroup dcsm_api DCSM
//! @{
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_dcsm.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "cpu.h"
#include "debug.h"

//*****************************************************************************
//
// Defines for the unlockZone1CSM() and unlockZone2CSM().
// These are not parameters for any function.
// These are not intended for application code.
//
//*****************************************************************************

#define DCSM_O_Z1_CSMPSWD0              0x08U //!< Z1 CSMPSWD0 offset
#define DCSM_O_Z1_CSMPSWD1              0x0AU //!< Z1 CSMPSWD1 offset
#define DCSM_O_Z1_CSMPSWD2              0x0CU //!< Z1 CSMPSWD2 offset
#define DCSM_O_Z1_CSMPSWD3              0x0EU //!< Z1 CSMPSWD3 offset
#define DCSM_O_Z2_CSMPSWD0              0x08U //!< Z2 CSMPSWD0 offset
#define DCSM_O_Z2_CSMPSWD1              0x0AU //!< Z2 CSMPSWD1 offset
#define DCSM_O_Z2_CSMPSWD2              0x0CU //!< Z2 CSMPSWD2 offset
#define DCSM_O_Z2_CSMPSWD3              0x0EU //!< Z2 CSMPSWD3 offset

//*****************************************************************************
//
// Register key defines.
//
//*****************************************************************************
#define FLSEM_KEY                       0xA5U //!< Zone semaphore key

//*****************************************************************************
//
//! Data structures to hold password keys.
//
//*****************************************************************************
typedef struct
{
    uint32_t csmKey0;
    uint32_t csmKey1;
    uint32_t csmKey2;
    uint32_t csmKey3;
} DCSM_CSMPasswordKey;

//*****************************************************************************
//
//! Values to distinguish the status of RAM or FLASH sectors. These values
//! describe which zone the memory location belongs too.
//! These values can be returned from DCSM_getRAMZone(),
//! DCSM_getFlashSectorZone().
//
//*****************************************************************************
typedef enum
{
    DCSM_MEMORY_INACCESSIBLE, //!< Inaccessible
    DCSM_MEMORY_ZONE1,        //!< Zone 1
    DCSM_MEMORY_ZONE2,        //!< Zone 2
    DCSM_MEMORY_FULL_ACCESS   //!< Full access
} DCSM_MemoryStatus;

//*****************************************************************************
//
//! Values to pass to DCSM_claimZoneSemaphore(). These values are used
//! to describe the zone that can write to Flash Wrapper registers.
//
//*****************************************************************************
typedef enum
{
    DCSM_FLSEM_ZONE1 = 0x01U, //!< Flash semaphore Zone 1
    DCSM_FLSEM_ZONE2 = 0x02U  //!< Flash semaphore Zone 2
} DCSM_SemaphoreZone;

//*****************************************************************************
//
//! Values to distinguish the security status of the zones.
//! These values can be returned from DCSM_getZone1CSMSecurityStatus(),
//! DCSM_getZone2CSMSecurityStatus().
//
//*****************************************************************************
typedef enum
{
    DCSM_STATUS_SECURE,   //!< Secure
    DCSM_STATUS_UNSECURE, //!< Unsecure
    DCSM_STATUS_LOCKED,   //!< Locked
} DCSM_SecurityStatus;

//*****************************************************************************
//
// Values to distinguish the status of the Control Registers. These values
// describe can be used with the return values of
// DCSM_getZone1ControlStatus(), and DCSM_getZone2ControlStatus().
//
//*****************************************************************************
#define DCSM_ALLZERO     0x08U  //!< CSM Passwords all zeros
#define DCSM_ALLONE      0x10U  //!< CSM Passwords all ones
#define DCSM_UNSECURE    0x20U  //!< Zone is secure/unsecure
#define DCSM_ARMED       0x40U  //!< CSM is armed

//*****************************************************************************
//
//! Values to decribe the EXEONLY Status.
//! These values are returned from  to DCSM_getZone1RAMEXEStatus(),
//! DCSM_getZone2RAMEXEStatus(), DCSM_getZone1FlashEXEStatus(),
//! DCSM_getZone2FlashEXEStatus().
//
//*****************************************************************************
typedef enum
{
    DCSM_PROTECTED,      //!< Protected
    DCSM_UNPROTECTED,    //!< Unprotected
    DCSM_INCORRECT_ZONE  //!< Incorrect Zone
}DCSM_EXEOnlyStatus;

//*****************************************************************************
//
//! Values to distinguish RAM Module.
//! These values can be passed to DCSM_getZone1RAMEXEStatus()
//! DCSM_getZone2RAMEXEStatus(), DCSM_getRAMZone().
//
//*****************************************************************************
typedef enum
{
    //
    //C28x RAMs
    //
    DCSM_RAMLS0, //!< RAMLS0
    DCSM_RAMLS1, //!< RAMLS1
    DCSM_RAMLS2, //!< RAMLS2
    DCSM_RAMLS3, //!< RAMLS3
    DCSM_RAMLS4, //!< RAMLS4
    DCSM_RAMLS5, //!< RAMLS5
    DCSM_RAMD0,  //!< RAMD0
    DCSM_RAMD1,  //!< RAMD1
    DCSM_CLA    = 14U //!<Offset of CLA field in in RAMSTAT divided by two
} DCSM_RAMModule;

//*****************************************************************************
//
//! Values to distinguish Flash Sector.
//! These values can be passed to DCSM_getZone1FlashEXEStatus()
//! DCSM_getZone2FlashEXEStatus(), DCSM_getFlashSectorZone().
//
//*****************************************************************************
typedef enum
{
    DCSM_SECTOR_A, //!< Sector A
    DCSM_SECTOR_B, //!< Sector B
    DCSM_SECTOR_C, //!< Sector C
    DCSM_SECTOR_D, //!< Sector D
    DCSM_SECTOR_E, //!< Sector E
    DCSM_SECTOR_F, //!< Sector F
    DCSM_SECTOR_G, //!< Sector G
    DCSM_SECTOR_H, //!< Sector H
    DCSM_SECTOR_I, //!< Sector I
    DCSM_SECTOR_J, //!< Sector J
    DCSM_SECTOR_K, //!< Sector K
    DCSM_SECTOR_L, //!< Sector L
    DCSM_SECTOR_M, //!< Sector M
    DCSM_SECTOR_N,  //!< Sector N
} DCSM_Sector;

//*****************************************************************************
//
// DCSM functions
//
//*****************************************************************************

//*****************************************************************************
//
//! Secures zone 1 by setting the FORCESEC bit of Z1_CR register
//!
//! This function resets the state of the zone. If the zone is unlocked,
//! it will lock(secure) the zone and also reset all the bits in the
//! Control Register.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCSM_secureZone1(void)
{
    //
    // Write to the FORCESEC bit.
    //
    HWREGH(DCSM_Z1_BASE + DCSM_O_Z1_CR)|= DCSM_Z1_CR_FORCESEC;
}

//*****************************************************************************
//
//! Secures zone 2 by setting the FORCESEC bit of Z2_CR register
//!
//! This function resets the state of the zone. If the zone is unlocked,
//! it will lock(secure) the zone and also reset all the bits in the
//! Control Register.
//!
//! \return None.
//
//*****************************************************************************
static inline void
DCSM_secureZone2(void)
{
    //
    // Write to the FORCESEC bit.
    //
    HWREGH(DCSM_Z2_BASE + DCSM_O_Z2_CR)|= DCSM_Z2_CR_FORCESEC;
}

//*****************************************************************************
//
//! Returns the CSM security status of zone 1
//!
//! This function returns the security status of zone 1 CSM
//!
//! \return Returns security status as an enumerated type DCSM_SecurityStatus.
//
//*****************************************************************************
static inline DCSM_SecurityStatus
DCSM_getZone1CSMSecurityStatus(void)
{
    uint16_t status;
    DCSM_SecurityStatus returnStatus;
    status = HWREGH(DCSM_Z1_BASE + DCSM_O_Z1_CR);

    //
    // if ARMED bit is set and UNSECURED bit or ALLONE bit or both UNSECURED
    // and ALLONE bits are set then CSM is unsecured. Else it is secure.
    //
    if(((status & DCSM_Z1_CR_ARMED) != 0U) &&
      (((status & DCSM_Z1_CR_UNSECURE) != 0U) ||
      ((status & DCSM_Z1_CR_ALLONE) != 0U )))
    {
        returnStatus = DCSM_STATUS_UNSECURE;
    }
    else if((status & DCSM_Z1_CR_ALLZERO) == DCSM_Z1_CR_ALLZERO)
    {
        returnStatus = DCSM_STATUS_LOCKED;
    }
    else
    {
        returnStatus = DCSM_STATUS_SECURE;
    }

    return(returnStatus);
}

//*****************************************************************************
//
//! Returns the CSM security status of zone 2
//!
//! This function returns the security status of zone 2 CSM
//!
//! \return Returns security status as an enumerated type DCSM_SecurityStatus.
//
//*****************************************************************************
static inline DCSM_SecurityStatus
DCSM_getZone2CSMSecurityStatus(void)
{
    uint16_t status;
    DCSM_SecurityStatus returnStatus;
    status = HWREGH(DCSM_Z2_BASE + DCSM_O_Z2_CR);

    //
    // if ARMED bit is set and UNSECURED bit or ALLONE bit or both UNSECURED
    // and ALLONE bits are set then CSM is unsecured. Else it is secure.
    //
    if(((status & DCSM_Z2_CR_ARMED) != 0U) &&
      (((status & DCSM_Z2_CR_UNSECURE) != 0U) ||
      ((status & DCSM_Z2_CR_ALLONE) != 0U )))
    {
        returnStatus = DCSM_STATUS_UNSECURE;
    }
    else if((status & DCSM_Z2_CR_ALLZERO) == DCSM_Z2_CR_ALLZERO)
    {
        returnStatus = DCSM_STATUS_LOCKED;
    }
    else
    {
        returnStatus = DCSM_STATUS_SECURE;
    }

    return(returnStatus);
}

//*****************************************************************************
//
//! Returns the Control Status of zone 1
//!
//! This function returns the Control Status of zone 1 CSM
//!
//! \return Returns the contents of the Control Register which can be
//! used with provided defines.
//
//*****************************************************************************
static inline uint16_t
DCSM_getZone1ControlStatus(void)
{
    //
    // Return the contents of the CR register.
    //
    return(HWREGH(DCSM_Z1_BASE + DCSM_O_Z1_CR));

}

//*****************************************************************************
//
//! Returns the Control Status of zone 2
//!
//! This function returns the Control Status of zone 2 CSM
//!
//! \return Returns the contents of the Control Register which can be
//! used with the provided defines.
//
//*****************************************************************************
static inline uint16_t
DCSM_getZone2ControlStatus(void)
{
    //
    // Return the contents of the CR register.
    //
    return(HWREGH(DCSM_Z2_BASE + DCSM_O_Z2_CR));
}

//*****************************************************************************
//
//! Returns the security zone a RAM section belongs to
//!
//! \param module is the RAM module value. Valid values are type DCSM_RAMModule
//! C28x RAMs :
//! - \b DCSM_RAMLS0
//! - \b DCSM_RAMLS1
//! - \b DCSM_RAMLS2
//! - \b DCSM_RAMLS3
//! - \b DCSM_RAMLS4
//! - \b DCSM_RAMLS5
//! - \b DCSM_RAMD0
//! - \b DCSM_RAMD1
//!
//! This function returns the security zone a RAM section belongs to.
//!
//! \return Returns DCSM_MEMORY_INACCESSIBLE if the section is inaccessible,
//! DCSM_MEMORY_ZONE1 if the section belongs to zone 1, DCSM_MEMORY_ZONE2 if
//! the section belongs to zone 2 and DCSM_MEMORY_FULL_ACCESS if the section
//! doesn't  belong to any zone (or if the section is unsecure).
//
//*****************************************************************************
static inline DCSM_MemoryStatus
DCSM_getRAMZone(DCSM_RAMModule module)
{
    uint16_t shift = (uint16_t)module * 2U;
    //
    //Read the RAMSTAT register for the specific RAM Module.
    //
    return((DCSM_MemoryStatus)((HWREG(DCSMCOMMON_BASE + DCSM_O_RAMSTAT) >>
                                shift) & 0x03U));
}

//*****************************************************************************
//
//! Returns the security zone a flash sector belongs to
//!
//! \param sector is the flash sector value.  Use DCSM_Sector type.
//!
//! This function returns the security zone a flash sector belongs to.
//!
//! \return Returns DCSM_MEMORY_INACCESSIBLE if the section is inaccessible ,
//! DCSM_MEMORY_ZONE1 if the section belongs to zone 1, DCSM_MEMORY_ZONE2 if
//! the section belongs to zone 2 and DCSM_MEMORY_FULL_ACCESS if the section
//! doesn't  belong to any zone (or if the section is unsecure)..
//
//*****************************************************************************
static inline DCSM_MemoryStatus
DCSM_getFlashSectorZone(DCSM_Sector sector)
{
    uint32_t sectStat;
    uint16_t shift;

    //
    // Get the Sector status register for the specific bank
    //
    sectStat = HWREG(DCSMCOMMON_BASE + DCSM_O_SECTSTAT);
    shift = (uint16_t)sector * 2U;

    //
    //Read the SECTSTAT register for the specific Flash Sector.
    //
    return((DCSM_MemoryStatus)((sectStat >> shift) & 0x3U));
}

//*****************************************************************************
//
//! Read Zone 1 Link Pointer Error
//!
//! A non-zero value indicates an error on the bit position that is set to 1.
//!
//! \return Returns the value of the Zone 1 Link Pointer error.
//
//*****************************************************************************
static inline uint32_t
DCSM_getZone1LinkPointerError(void)
{
    //
    // Return the LinkPointer Error for specific bank
    //
    return(HWREG(DCSM_Z1_BASE + DCSM_O_Z1_LINKPOINTERERR));
}

//*****************************************************************************
//
//! Read Zone 2 Link Pointer Error
//!
//! A non-zero value indicates an error on the bit position that is set to 1.
//!
//! \return Returns the value of the Zone 2 Link Pointer error.
//
//*****************************************************************************
static inline uint32_t
DCSM_getZone2LinkPointerError(void)
{
    //
    // Return the LinkPointer Error for specific bank
    //
    return(HWREG(DCSM_Z2_BASE + DCSM_O_Z2_LINKPOINTERERR));
}

//*****************************************************************************
//
//! Unlocks Zone 1 CSM.
//!
//! \param psCMDKey is a pointer to the DCSM_CSMPasswordKey struct that has the
//! CSM  password for zone 1.
//!
//! This function unlocks the CSM password. It first reads the
//! four password locations in the User OTP. If any of the password values is
//! different from 0xFFFFFFFF, it unlocks the device by writing the provided
//! passwords into CSM Key registers
//!
//! \return None.
//
//*****************************************************************************
extern void
DCSM_unlockZone1CSM(const DCSM_CSMPasswordKey * const psCMDKey);

//*****************************************************************************
//
//! Unlocks Zone 2 CSM.
//!
//! \param psCMDKey is a pointer to the CSMPSWDKEY that has the CSM
//!  password for zone 2.
//!
//! This function unlocks the CSM password. It first reads
//! the four password locations in the User OTP. If any of the password values
//! is different from 0xFFFFFFFF, it unlocks the device by writing the
//! provided passwords into CSM Key registers
//!
//! \return None.
//
//*****************************************************************************
extern void
DCSM_unlockZone2CSM(const DCSM_CSMPasswordKey * const psCMDKey);
//*****************************************************************************
//
//! Returns the EXE-ONLY status of zone 1 for a flash sector
//!
//! \param sector is the flash sector value.  Use DCSM_Sector type.
//!
//! This function takes in a valid sector value and returns the status of EXE
//! ONLY security protection for the sector.
//!
//! \return Returns DCSM_PROTECTED if the sector is EXE-ONLY protected,
//! DCSM_UNPROTECTED if the sector is not EXE-ONLY protected,
//! DCSM_INCORRECT_ZONE if sector does not belong to this zone.
//
//*****************************************************************************
extern DCSM_EXEOnlyStatus
DCSM_getZone1FlashEXEStatus(DCSM_Sector sector);

//*****************************************************************************
//
//! Returns the EXE-ONLY status of zone 1 for a RAM module
//!
//! \param module is the RAM module value. Valid values are type DCSM_RAMModule
//! C28x RAMs :
//! - \b DCSM_RAMLS0
//! - \b DCSM_RAMLS1
//! - \b DCSM_RAMLS2
//! - \b DCSM_RAMLS3
//! - \b DCSM_RAMLS4
//! - \b DCSM_RAMLS5
//! - \b DCSM_RAMD0
//! - \b DCSM_RAMD1
//!
//! This function takes in a valid module value and returns the status of EXE
//! ONLY security protection for that module.  DCSM_CLA is an invalid module
//! value.  There is no EXE-ONLY available for DCSM_CLA.
//!
//! \return Returns DCSM_PROTECTED if the module is EXE-ONLY protected,
//! DCSM_UNPROTECTED if the module is not EXE-ONLY protected,
//! DCSM_INCORRECT_ZONE if module does not belong to this zone.
//
//*****************************************************************************
extern DCSM_EXEOnlyStatus
DCSM_getZone1RAMEXEStatus(DCSM_RAMModule module);

//*****************************************************************************
//
//! Returns the EXE-ONLY status of zone 2 for a flash sector
//!
//! \param sector is the flash sector value. Use DCSM_Sector type.
//!
//! This function takes in a valid sector value and returns the status of EXE
//! ONLY security protection for the sector.
//!
//! \return Returns DCSM_PROTECTED if the sector is EXE-ONLY protected,
//! DCSM_UNPROTECTED if the sector is not EXE-ONLY protected,
//! DCSM_INCORRECT_ZONE if sector does not belong to this zone.
//
//*****************************************************************************
extern DCSM_EXEOnlyStatus
DCSM_getZone2FlashEXEStatus(DCSM_Sector sector);

//*****************************************************************************
//
//! Returns the EXE-ONLY status of zone 2 for a RAM module
//!
//! \param module is the RAM module value. Valid values are type DCSM_RAMModule
//! C28x RAMs :
//! - \b DCSM_RAMLS0
//! - \b DCSM_RAMLS1
//! - \b DCSM_RAMLS2
//! - \b DCSM_RAMLS3
//! - \b DCSM_RAMLS4
//! - \b DCSM_RAMLS5
//! - \b DCSM_RAMD0
//! - \b DCSM_RAMD1
//!
//! This function takes in a valid module value and returns the status of EXE
//! ONLY security protection for that module.  DCSM_CLA is an invalid module
//! value.  There is no EXE-ONLY available for DCSM_CLA.
//!
//! \return Returns DCSM_PROTECTED if the module is EXE-ONLY protected,
//! DCSM_UNPROTECTED if the module is not EXE-ONLY protected,
//! DCSM_INCORRECT_ZONE if module does not belong to this zone.
//
//*****************************************************************************
extern DCSM_EXEOnlyStatus
DCSM_getZone2RAMEXEStatus(DCSM_RAMModule module);

//*****************************************************************************
//
//! Claims the zone semaphore which allows access to the Flash Wrapper register
//! for that zone.
//!
//! \param zone is the zone which is trying to claim the semaphore which allows
//! access to the Flash Wrapper registers.
//!
//! \return Returns true for a successful semaphore capture, false if it was
//! unable to capture the semaphore.
//
//*****************************************************************************
extern bool
DCSM_claimZoneSemaphore(DCSM_SemaphoreZone zone);

//*****************************************************************************
//
//! Releases the zone semaphore.
//!
//! \return Returns true if it was successful in releasing the zone semaphore
//! and false if it was unsuccessful in releasing the zone semaphore.
//!
//! \note  If the calling function is not in the right zone to be able
//!        to access this register, it will return a false.
//
//*****************************************************************************
extern bool
DCSM_releaseZoneSemaphore(void);


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif //  DCSM_H
