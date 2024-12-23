//###########################################################################
//
// FILE: ecap.h
//
// TITLE: C28x ECAP driver
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

#ifndef ECAP_H
#define ECAP_H

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
//! \addtogroup ecap_api eCAP
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Includes
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ecap.h"
#include "cpu.h"
#include "debug.h"

//*****************************************************************************
//
// eCAP minimum and maximum values
//
//*****************************************************************************
#define ECAP_MAX_PRESCALER_VALUE       32U  // Maximum Pre-scaler value

//*****************************************************************************
//
// Values that can be passed to ECAP_enableInterrupt(),
// ECAP_disableInterrupt(), ECAP_clearInterrupt() and ECAP_forceInterrupt() as
// the intFlags parameter and returned by ECAP_getInterruptSource().
//
//*****************************************************************************
//! Event 1 ISR source
//!
#define ECAP_ISR_SOURCE_CAPTURE_EVENT_1 0x2U
//! Event 2 ISR source
//!
#define ECAP_ISR_SOURCE_CAPTURE_EVENT_2 0x4U
//! Event 3 ISR source
//!
#define ECAP_ISR_SOURCE_CAPTURE_EVENT_3 0x8U
//! Event 4 ISR source
//!
#define ECAP_ISR_SOURCE_CAPTURE_EVENT_4 0x10U
//! Counter overflow ISR source
//!
#define ECAP_ISR_SOURCE_COUNTER_OVERFLOW 0x20U
//! Counter equals period ISR source
//!
#define ECAP_ISR_SOURCE_COUNTER_PERIOD 0x40U
//! Counter equals compare ISR source
//!
#define ECAP_ISR_SOURCE_COUNTER_COMPARE 0x80U

//*****************************************************************************
//
//! Values that can be passed to ECAP_setEmulationMode() as the
//! \e mode parameter.
//
//*****************************************************************************
typedef enum
{
    //! TSCTR is stopped on emulation suspension
    ECAP_EMULATION_STOP             = 0x0U,
    //! TSCTR runs until 0 before stopping on emulation suspension
    ECAP_EMULATION_RUN_TO_ZERO      = 0x1U,
    //! TSCTR is not affected by emulation suspension
    ECAP_EMULATION_FREE_RUN         = 0x2U
}ECAP_EmulationMode;

//*****************************************************************************
//
//! Values that can be passed to ECAP_setCaptureMode() as the
//! \e mode parameter.
//
//*****************************************************************************
typedef enum
{
    //! eCAP operates in continuous capture mode
    ECAP_CONTINUOUS_CAPTURE_MODE    = 0U,
    //! eCAP operates in one shot capture mode
    ECAP_ONE_SHOT_CAPTURE_MODE      = 1U
}ECAP_CaptureMode;

//*****************************************************************************
//
//! Values that can be passed to ECAP_setEventPolarity(),ECAP_setCaptureMode(),
//! ECAP_enableCounterResetOnEvent(),ECAP_disableCounterResetOnEvent(),
//! ECAP_getEventTimeStamp(),ECAP_setDMASource() as the \e event parameter.
//
//*****************************************************************************
typedef enum
{
    ECAP_EVENT_1 = 0U,   //!< eCAP event 1
    ECAP_EVENT_2 = 1U,   //!< eCAP event 2
    ECAP_EVENT_3 = 2U,   //!< eCAP event 3
    ECAP_EVENT_4 = 3U    //!< eCAP event 4
}ECAP_Events;

//*****************************************************************************
//
//! Values that can be passed to ECAP_setSyncOutMode() as the \e mode
//! parameter.
//
//*****************************************************************************
typedef enum
{
    //! sync out on the sync in signal and software force
    ECAP_SYNC_OUT_SYNCI         = 0x00U,
    //! sync out on counter equals period
    ECAP_SYNC_OUT_COUNTER_PRD   = 0x40U,
    //! Disable sync out signal
    ECAP_SYNC_OUT_DISABLED      = 0x80U
}ECAP_SyncOutMode;

//*****************************************************************************
//
//! Values that can be passed to ECAP_setAPWMPolarity() as the \e polarity
//! parameter.
//
//*****************************************************************************
typedef enum
{
    ECAP_APWM_ACTIVE_HIGH   = 0x000, //!< APWM is active high
    ECAP_APWM_ACTIVE_LOW    = 0x400  //!< APWM is active low
}ECAP_APWMPolarity;

//*****************************************************************************
//
//! Values that can be passed to ECAP_setEventPolarity() as the \e polarity
//! parameter.
//
//*****************************************************************************
typedef enum
{
    ECAP_EVNT_RISING_EDGE   = 0U, //!< Rising edge polarity
    ECAP_EVNT_FALLING_EDGE  = 1U  //!< Falling edge polarity
}ECAP_EventPolarity;

//*****************************************************************************
//
//! \internal
//! Checks eCAP base address.
//!
//! \param base specifies the eCAP module base address.
//!
//! This function determines if an eCAP module base address is valid.
//!
//! \return Returns \b true if the base address is valid and \b false
//! otherwise.
//
//*****************************************************************************
#ifdef DEBUG
static inline bool ECAP_isBaseValid(uint32_t base)
{
    return(
           (base == ECAP1_BASE) ||
           (base == ECAP2_BASE) ||
           (base == ECAP3_BASE) ||
           (base == ECAP4_BASE) ||
           (base == ECAP5_BASE) ||
           (base == ECAP6_BASE)
          );
}
#endif

//*****************************************************************************
//
//! Sets the input prescaler.
//!
//! \param base is the base address of the ECAP module.
//! \param preScalerValue is the pre scaler value for ECAP input
//!
//! This function divides the ECAP input scaler. The pre scale value is
//! doubled inside the module. For example a preScalerValue of 5 will divide
//! the scaler by 10. Use a value of 1 to divide the pre scaler by 1.
//! The \e preScalerValue should be less than \b ECAP_MAX_PRESCALER_VALUE.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_setEventPrescaler(uint32_t base,
                                          uint16_t preScalerValue)
{
    ASSERT(ECAP_isBaseValid(base));

    ASSERT(preScalerValue < ECAP_MAX_PRESCALER_VALUE);


    //
    // Write to PRESCALE bit
    //
    HWREGH(base + ECAP_O_ECCTL1) =
                 ((HWREGH(base + ECAP_O_ECCTL1) & (~ECAP_ECCTL1_PRESCALE_M)) |
                  (preScalerValue << ECAP_ECCTL1_PRESCALE_S));
}

//*****************************************************************************
//
//! Sets the Capture event polarity.
//!
//! \param base is the base address of the ECAP module.
//! \param event is the event number.
//! \param polarity is the polarity of the event.
//!
//! This function sets the polarity of a given event. The value of event
//! is between \b ECAP_EVENT_1 and \b ECAP_EVENT_4 inclusive corresponding to
//! the four available events.For each event the polarity value determines the
//! edge on which the capture is activated. For a rising edge use a polarity
//! value of \b ECAP_EVNT_RISING_EDGE and for a falling edge use a polarity of
//! \b ECAP_EVNT_FALLING_EDGE.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_setEventPolarity(uint32_t base,
                                         ECAP_Events event,
                                         ECAP_EventPolarity polarity)
{

    uint16_t shift;

    ASSERT(ECAP_isBaseValid(base));

    shift = ((uint16_t)event) << 1U;


    //
    // Write to CAP1POL, CAP2POL, CAP3POL or CAP4POL
    //
    HWREGH(base + ECAP_O_ECCTL1) =
                         (HWREGH(base + ECAP_O_ECCTL1) & ~(1U << shift)) |
                         ((uint16_t)polarity << shift);
}

//*****************************************************************************
//
//! Sets the capture mode.
//!
//! \param base is the base address of the ECAP module.
//! \param mode is the capture mode.
//! \param event is the event number at which the counter stops or wraps.
//!
//! This function sets the eCAP module to a continuous or one-shot mode.
//! The value of mode should be either \b ECAP_CONTINUOUS_CAPTURE_MODE or
//! \b ECAP_ONE_SHOT_CAPTURE_MODE corresponding to continuous or one-shot mode
//! respectively.
//!
//! The value of event determines the event number at which the counter stops
//! (in one-shot mode) or the counter wraps (in continuous mode). The value of
//! event should be between \b ECAP_EVENT_1 and \b ECAP_EVENT_4 corresponding
//! to the valid event numbers.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_setCaptureMode(uint32_t base,
                                       ECAP_CaptureMode mode,
                                       ECAP_Events event)
{
    ASSERT(ECAP_isBaseValid(base));


    //
    // Write to CONT/ONESHT
    //
    HWREGH(base + ECAP_O_ECCTL2) =
               ((HWREGH(base + ECAP_O_ECCTL2) & (~ECAP_ECCTL2_CONT_ONESHT)) |
                (uint16_t)mode);

    //
    // Write to STOP_WRAP
    //
    HWREGH(base + ECAP_O_ECCTL2) =
               ((HWREGH(base + ECAP_O_ECCTL2) & (~ECAP_ECCTL2_STOP_WRAP_M)) |
                (((uint16_t)event) << ECAP_ECCTL2_STOP_WRAP_S ));
}

//*****************************************************************************
//
//! Re-arms the eCAP module.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function re-arms the eCAP module.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_reArm(uint32_t base)
{
    ASSERT(ECAP_isBaseValid(base));


    //
    // Write to RE-ARM bit
    //
    HWREGH(base + ECAP_O_ECCTL2) |= ECAP_ECCTL2_REARM;
}

//*****************************************************************************
//
//! Enables interrupt source.
//!
//! \param base is the base address of the ECAP module.
//! \param intFlags is the interrupt source to be enabled.
//!
//! This function sets and enables eCAP interrupt source. The following are
//! valid interrupt sources.
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_1 - Event 1 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_2 - Event 2 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_3 - Event 3 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_4 - Event 4 generates interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_OVERFLOW - Counter overflow generates interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_PERIOD   - Counter equal period generates
//!                                       interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_COMPARE  - Counter equal compare generates
//!                                       interrupt
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_enableInterrupt(uint32_t base,
                                        uint16_t intFlags)
{
    ASSERT(ECAP_isBaseValid(base));
    ASSERT((intFlags & ~(ECAP_ISR_SOURCE_CAPTURE_EVENT_1 |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_2 |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_3 |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_4 |
                         ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                         ECAP_ISR_SOURCE_COUNTER_PERIOD |
                         ECAP_ISR_SOURCE_COUNTER_COMPARE)) == 0U);



    //
    // Set bits in ECEINT register
    //
    HWREGH(base + ECAP_O_ECEINT) |= intFlags;
}

//*****************************************************************************
//
//! Disables interrupt source.
//!
//! \param base is the base address of the ECAP module.
//! \param intFlags is the interrupt source to be disabled.
//!
//! This function clears and disables eCAP interrupt source. The following are
//! valid interrupt sources.
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_1   - Event 1 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_2   - Event 2 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_3   - Event 3 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_4   - Event 4 generates interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_OVERFLOW  - Counter overflow generates interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_PERIOD    - Counter equal period generates
//!                                        interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_COMPARE   - Counter equal compare generates
//!                                        interrupt
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_disableInterrupt(uint32_t base,
                                         uint16_t intFlags)
{

    ASSERT(ECAP_isBaseValid(base));
    ASSERT((intFlags & ~(ECAP_ISR_SOURCE_CAPTURE_EVENT_1 |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_2 |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_3 |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_4 |
                         ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                         ECAP_ISR_SOURCE_COUNTER_PERIOD |
                         ECAP_ISR_SOURCE_COUNTER_COMPARE)) == 0U);


    //
    // Clear bits in ECEINT register
    //
    HWREGH(base + ECAP_O_ECEINT) &= ~intFlags;
}

//*****************************************************************************
//
//! Returns the interrupt flag.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function returns the eCAP interrupt flag. The following are valid
//! interrupt sources corresponding to the eCAP interrupt flag.
//!
//! \return Returns the eCAP interrupt that has occurred. The following are
//!  valid return values.
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_1   - Event 1 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_2   - Event 2 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_3   - Event 3 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_4   - Event 4 generates interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_OVERFLOW  - Counter overflow generates interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_PERIOD    - Counter equal period generates
//!                                        interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_COMPARE   - Counter equal compare generates
//!                                        interrupt
//!
//! \note - User can check if a combination of various interrupts have occurred
//!         by ORing the above return values.
//
//*****************************************************************************
static inline uint16_t ECAP_getInterruptSource(uint32_t base)
{
    ASSERT(ECAP_isBaseValid(base));

    //
    // Return contents of ECFLG register
    //
    return(HWREGH(base + ECAP_O_ECFLG) & 0xFEU);
}

//*****************************************************************************
//
//! Returns the Global interrupt flag.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function returns the eCAP Global interrupt flag.
//!
//! \return Returns true if there is a global eCAP interrupt, false otherwise.
//
//*****************************************************************************
static inline bool ECAP_getGlobalInterruptStatus(uint32_t base)
{
    ASSERT(ECAP_isBaseValid(base));

    //
    // Return contents of Global interrupt bit
    //
    return((HWREGH(base + ECAP_O_ECFLG) & 0x1U) == 0x1U);
}

//*****************************************************************************
//
//! Clears interrupt flag.
//!
//! \param base is the base address of the ECAP module.
//! \param intFlags is the interrupt source.
//!
//! This function clears eCAP interrupt flags. The following are valid
//! interrupt sources.
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_1 - Event 1 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_2 - Event 2 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_3 - Event 3 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_4 - Event 4 generates interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_OVERFLOW - Counter overflow generates interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_PERIOD   - Counter equal period generates
//!                                       interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_COMPARE  - Counter equal compare generates
//!                                       interrupt
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_clearInterrupt(uint32_t base,
                                       uint16_t intFlags)
{
    ASSERT(ECAP_isBaseValid(base));
    ASSERT((intFlags & ~(ECAP_ISR_SOURCE_CAPTURE_EVENT_1 |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_2 |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_3 |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_4 |
                         ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                         ECAP_ISR_SOURCE_COUNTER_PERIOD |
                         ECAP_ISR_SOURCE_COUNTER_COMPARE)) == 0U);

    //
    // Write to ECCLR register
    //
    HWREGH(base + ECAP_O_ECCLR) = intFlags;
}

//*****************************************************************************
//
//! Clears global interrupt flag
//!
//! \param base is the base address of the ECAP module.
//!
//! This function clears the global interrupt bit.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_clearGlobalInterrupt(uint32_t base)
{
    ASSERT(ECAP_isBaseValid(base));

    //
    // Write to INT bit
    //
    HWREGH(base + ECAP_O_ECCLR) = ECAP_ECCLR_INT;
}

//*****************************************************************************
//
//! Forces interrupt source.
//!
//! \param base is the base address of the ECAP module.
//! \param intFlags is the interrupt source.
//!
//! This function forces and enables eCAP interrupt source. The following are
//! valid interrupt sources.
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_1 - Event 1 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_2 - Event 2 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_3 - Event 3 generates interrupt
//!  - ECAP_ISR_SOURCE_CAPTURE_EVENT_4 - Event 4 generates interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_OVERFLOW - Counter overflow generates interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_PERIOD   - Counter equal period generates
//!                                       interrupt
//!  - ECAP_ISR_SOURCE_COUNTER_COMPARE  - Counter equal compare generates
//!                                       interrupt
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_forceInterrupt(uint32_t base,
                                       uint16_t intFlags)
{
    ASSERT(ECAP_isBaseValid(base));
    ASSERT((intFlags & ~(ECAP_ISR_SOURCE_CAPTURE_EVENT_1 |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_2 |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_3 |
                         ECAP_ISR_SOURCE_CAPTURE_EVENT_4 |
                         ECAP_ISR_SOURCE_COUNTER_OVERFLOW |
                         ECAP_ISR_SOURCE_COUNTER_PERIOD |
                         ECAP_ISR_SOURCE_COUNTER_COMPARE)) == 0U);


    //
    // Write to ECFRC register
    //
    HWREGH(base + ECAP_O_ECFRC) = intFlags;
}

//*****************************************************************************
//
//! Sets eCAP in Capture mode.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function sets the eCAP module to operate in Capture mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_enableCaptureMode(uint32_t base)
{
    ASSERT(ECAP_isBaseValid(base));


    //
    // Clear CAP/APWM bit
    //
    HWREGH(base + ECAP_O_ECCTL2) &= ~ECAP_ECCTL2_CAP_APWM;
}

//*****************************************************************************
//
//! Sets eCAP in APWM mode.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function sets the eCAP module to operate in APWM mode.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_enableAPWMMode(uint32_t base)
{
    ASSERT(ECAP_isBaseValid(base));


    //
    // Set CAP/APWM bit
    //
    HWREGH(base + ECAP_O_ECCTL2) |= ECAP_ECCTL2_CAP_APWM;
}

//*****************************************************************************
//
//! Enables counter reset on an event.
//!
//! \param base is the base address of the ECAP module.
//! \param event is the event number the time base gets reset.
//!
//! This function enables the base timer, TSCTR, to be reset on capture
//! event provided by the variable event. Valid inputs for event are
//! \b ECAP_EVENT_1 to \b ECAP_EVENT_4.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_enableCounterResetOnEvent(uint32_t base,
                                                  ECAP_Events event)
{
    ASSERT(ECAP_isBaseValid(base));


    //
    // Set CTRRST1,CTRRST2,CTRRST3 or CTRRST4 bits
    //
    HWREGH(base + ECAP_O_ECCTL1) |= 1U << ((2U * (uint16_t)event) + 1U);
}

//*****************************************************************************
//
//! Disables counter reset on events.
//!
//! \param base is the base address of the ECAP module.
//! \param event is the event number the time base gets reset.
//!
//! This function disables the base timer, TSCTR, from being reset on capture
//! event provided by the variable event. Valid inputs for event are
//! \b ECAP_EVENT_1 to \b ECAP_EVENT_4.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_disableCounterResetOnEvent(uint32_t base,
                                                   ECAP_Events event)
{
    ASSERT(ECAP_isBaseValid(base));


    //
    // Clear CTRRST1,CTRRST2,CTRRST3 or CTRRST4 bits
    //
    HWREGH(base + ECAP_O_ECCTL1) &= ~(1U << ((2U * (uint16_t)event) + 1U));
}

//*****************************************************************************
//
//! Enables time stamp capture.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function enables time stamp count to be captured
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_enableTimeStampCapture(uint32_t base)
{
    ASSERT(ECAP_isBaseValid(base));


    //
    // Set CAPLDEN bit
    //
    HWREGH(base + ECAP_O_ECCTL1) |= ECAP_ECCTL1_CAPLDEN;
}

//*****************************************************************************
//
//! Disables time stamp capture.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function disables time stamp count to be captured
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_disableTimeStampCapture(uint32_t base)
{
    ASSERT(ECAP_isBaseValid(base));


    //
    // Clear CAPLDEN bit
    //
    HWREGH(base + ECAP_O_ECCTL1) &= ~ECAP_ECCTL1_CAPLDEN;
}

//*****************************************************************************
//
//! Sets a phase shift value count.
//!
//! \param base is the base address of the ECAP module.
//! \param shiftCount is the phase shift value.
//!
//! This function writes a phase shift value to be loaded into the main time
//! stamp counter.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_setPhaseShiftCount(uint32_t base, uint32_t shiftCount)
{
    ASSERT(ECAP_isBaseValid(base));

    //
    // Write to CTRPHS
    //
    HWREG(base + ECAP_O_CTRPHS) = shiftCount;
}

//*****************************************************************************
//
//! Enable counter loading with phase shift value.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function enables loading of the counter with the value present in the
//! phase shift counter as defined by the ECAP_setPhaseShiftCount() function.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_enableLoadCounter(uint32_t base)
{

    ASSERT(ECAP_isBaseValid(base));


    //
    // Write to SYNCI_EN
    //
    HWREGH(base + ECAP_O_ECCTL2) |= ECAP_ECCTL2_SYNCI_EN;
}

//*****************************************************************************
//
//! Disable counter loading with phase shift value.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function disables loading of the counter with the value present in the
//! phase shift counter as defined by the ECAP_setPhaseShiftCount() function.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_disableLoadCounter(uint32_t base)
{

    ASSERT(ECAP_isBaseValid(base));


    //
    // Write to SYNCI_EN
    //
    HWREGH(base + ECAP_O_ECCTL2) &= ~ECAP_ECCTL2_SYNCI_EN;
}

//*****************************************************************************
//
//! Load time stamp counter
//!
//! \param base is the base address of the ECAP module.
//!
//! This function forces the value in the phase shift counter register to be
//! loaded into Time stamp counter register.
//! Make sure to enable loading of Time stamp counter by calling
//! ECAP_enableLoadCounter() function before calling this function.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_loadCounter(uint32_t base)
{
    ASSERT(ECAP_isBaseValid(base));


    //
    // Write to SWSYNC
    //
    HWREGH(base + ECAP_O_ECCTL2) |= ECAP_ECCTL2_SWSYNC;
}

//*****************************************************************************
//
//! Configures Sync out signal mode.
//!
//! \param base is the base address of the ECAP module.
//! \param mode is the sync out mode.
//!
//! This function sets the sync out mode. Valid parameters for mode are:
//! - ECAP_SYNC_OUT_SYNCI - Trigger sync out on sync-in event.
//! - ECAP_SYNC_OUT_COUNTER_PRD - Trigger sync out when counter equals period.
//! - ECAP_SYNC_OUT_DISABLED - Disable sync out.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_setSyncOutMode(uint32_t base,
                                       ECAP_SyncOutMode mode)
{
    ASSERT(ECAP_isBaseValid(base));


    //
    // Write to SYNCO_SEL
    //
     HWREGH(base + ECAP_O_ECCTL2) =
                ((HWREGH(base + ECAP_O_ECCTL2) & (~ECAP_ECCTL2_SYNCO_SEL_M)) |
                 (uint16_t)mode);
}

//*****************************************************************************
//
//! Stops Time stamp counter.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function stops the time stamp counter.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_stopCounter(uint32_t base)
{
    ASSERT(ECAP_isBaseValid(base));


    //
    // Clear TSCTR
    //
    HWREGH(base + ECAP_O_ECCTL2) &= ~ECAP_ECCTL2_TSCTRSTOP;
}

//*****************************************************************************
//
//! Starts Time stamp counter.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function starts the time stamp counter.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_startCounter(uint32_t base)
{
    ASSERT(ECAP_isBaseValid(base));


    //
    // Set TSCTR
    //
    HWREGH(base + ECAP_O_ECCTL2) |= ECAP_ECCTL2_TSCTRSTOP;
}

//*****************************************************************************
//
//! Set eCAP APWM polarity.
//!
//! \param base is the base address of the ECAP module.
//! \param polarity is the polarity of APWM
//!
//! This function sets the polarity of the eCAP in APWM mode. Valid inputs for
//! polarity are:
//!  - ECAP_APWM_ACTIVE_HIGH - For active high.
//!  - ECAP_APWM_ACTIVE_LOW - For active low.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_setAPWMPolarity(uint32_t base,
                                        ECAP_APWMPolarity polarity)
{
    ASSERT(ECAP_isBaseValid(base));

    HWREGH(base + ECAP_O_ECCTL2) =
               ((HWREGH(base + ECAP_O_ECCTL2) & ~ECAP_ECCTL2_APWMPOL) |
                (uint16_t)polarity);
}

//*****************************************************************************
//
//! Set eCAP APWM period.
//!
//! \param base is the base address of the ECAP module.
//! \param periodCount is the period count for APWM.
//!
//! This function sets the period count of the APWM waveform.
//! periodCount takes the actual count which is written to the register. The
//! user is responsible for converting the desired frequency or time into
//! the period count.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_setAPWMPeriod(uint32_t base, uint32_t periodCount)
{
    ASSERT(ECAP_isBaseValid(base));

    //
    // Write to CAP1
    //
    HWREG(base + ECAP_O_CAP1) = periodCount;
}

//*****************************************************************************
//
//! Set eCAP APWM on or off time count.
//!
//! \param base is the base address of the ECAP module.
//! \param compareCount is the on or off count for APWM.
//!
//! This function sets the on or off time count of the APWM waveform depending
//! on the polarity of the output. If the output , as set by
//! ECAP_setAPWMPolarity(), is active high then compareCount determines the on
//! time. If the output is active low then compareCount determines the off
//! time. compareCount takes the actual count which is written to the register.
//! The user is responsible for converting the desired frequency or time into
//! the appropriate count value.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_setAPWMCompare(uint32_t base, uint32_t compareCount)
{
    ASSERT(ECAP_isBaseValid(base));

    //
    // Write to CAP2
    //
    HWREG(base + ECAP_O_CAP2) = compareCount;
}

//*****************************************************************************
//
//! Load eCAP APWM shadow period.
//!
//! \param base is the base address of the ECAP module.
//! \param periodCount is the shadow period count for APWM.
//!
//! This function sets the shadow period count of the APWM waveform.
//! periodCount takes the actual count which is written to the register. The
//! user is responsible for converting the desired frequency or time into
//! the period count.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_setAPWMShadowPeriod(uint32_t base,
                                            uint32_t periodCount)
{
    ASSERT(ECAP_isBaseValid(base));

    //
    // Write to CAP3
    //
    HWREG(base + ECAP_O_CAP3) = periodCount;
}

//*****************************************************************************
//
//! Set eCAP APWM shadow on or off time count.
//!
//! \param base is the base address of the ECAP module.
//! \param compareCount is the on or off count for APWM.
//!
//! This function sets the shadow on or off time count of the APWM waveform
//! depending on the polarity of the output. If the output , as set by
//! ECAP_setAPWMPolarity() , is active high then compareCount determines the
//! on time. If the output is active low then compareCount determines the off
//! time. compareCount takes the actual count which is written to the register.
//! The user is responsible for converting the desired frequency or time into
//! the appropriate count value.
//!
//! \return None.
//
//*****************************************************************************
static inline void ECAP_setAPWMShadowCompare(uint32_t base,
                                             uint32_t compareCount)
{
    ASSERT(ECAP_isBaseValid(base));

    //
    // Write to CAP4
    //
    HWREG(base + ECAP_O_CAP4) = compareCount;
}

//*****************************************************************************
//
//! Returns the time base counter value.
//!
//! \param base is the base address of the ECAP module.
//!
//! This function returns the time base counter value.
//!
//! \return Returns the time base counter value.
//
//*****************************************************************************
static inline uint32_t ECAP_getTimeBaseCounter(uint32_t base)
{
    ASSERT(ECAP_isBaseValid(base));

    //
    // Read the Time base counter value
    //
    return(HWREG(base + ECAP_O_TSCTR));
}

//*****************************************************************************
//
//! Returns event time stamp.
//!
//! \param base is the base address of the ECAP module.
//! \param event is the event number.
//!
//! This function returns the current time stamp count of the given event.
//! Valid values for event are \b ECAP_EVENT_1 to \b ECAP_EVENT_4.
//!
//! \return Event time stamp value or 0 if \e event is invalid.
//
//*****************************************************************************
static inline uint32_t ECAP_getEventTimeStamp(uint32_t base, ECAP_Events event)
{
    uint32_t count;

    ASSERT(ECAP_isBaseValid(base));


    switch(event)
    {
        case ECAP_EVENT_1:

            //
            // Read CAP1 register
            //
            count = HWREG(base + ECAP_O_CAP1);
        break;

        case ECAP_EVENT_2:
            //
            // Read CAP2 register
            //
            count = HWREG(base + ECAP_O_CAP2);
        break;

        case ECAP_EVENT_3:

            //
            // Read CAP3 register
            //
            count = HWREG(base + ECAP_O_CAP3);
        break;

        case ECAP_EVENT_4:

            //
            // Read CAP4 register
            //
            count = HWREG(base + ECAP_O_CAP4);
        break;

        default:

            //
            // Invalid event parameter
            //
            count = 0U;
        break;
    }

    return(count);
}

//*****************************************************************************
//
//! Configures emulation mode.
//!
//! \param base is the base address of the ECAP module.
//! \param mode is the emulation mode.
//!
//! This function configures the eCAP counter, TSCTR,  to the desired emulation
//! mode when emulation suspension occurs. Valid inputs for mode are:
//! - ECAP_EMULATION_STOP  - Counter is stopped immediately.
//! - ECAP_EMULATION_RUN_TO_ZERO - Counter runs till it reaches 0.
//! - ECAP_EMULATION_FREE_RUN - Counter is not affected.
//!
//! \return None.
//
//*****************************************************************************
extern void ECAP_setEmulationMode(uint32_t base, ECAP_EmulationMode mode);

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

#endif // ECAP_H
