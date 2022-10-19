//###########################################################################
//
// FILE:  Flash2833x_API_Config.h
//
// TITLE: F2833x Flash Algo's - User Settings
//
// NOTE:  This file contains user defined settings that
//        are used by the F2833x Flash APIs.
//
//###########################################################################
// $TI Release: $
// $Release Date: $
// $Copyright:
// Copyright (C) 2009-2022 Texas Instruments Incorporated - http://www.ti.com/
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
//###########################################################################

#ifndef FLASH2833X_API_CONFIG_H
#define FLASH2833X_API_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

//
// Variables that can be configured by the user
//

//
//   1. Specify the device.
//      Define the device to be programmed as "1" (no quotes).
//      Define all other devices as "0" (no quotes).  
//
#define FLASH_F28335   1
#define FLASH_F28334   0
#define FLASH_F28332   0

//
//   2. Specify the clock rate of the CPU (SYSCLKOUT) in nS.
//
//      Take into account the input clock frequency and the PLL multiplier
//      that your application will use.
//
//      Use one of the values provided, or define your own.
//      The trailing L is required tells the compiler to treat
//      the number as a 64-bit value.
//
//      Only one statement should be uncommented.
//
//      Example:  CLKIN is a 30MHz crystal.
//
//                If the application will set PLLCR = 0xA then the CPU clock
//                will be 150Mhz (SYSCLKOUT = 150MHz).
//
//                In this case, the CPU_RATE will be 6.667L
//                Uncomment the line:  #define CPU_RATE  6.667L
//
#define CPU_RATE    6.667L   // for a 150MHz CPU clock speed (SYSCLKOUT)
//#define CPU_RATE   10.000L   // for a 100MHz CPU clock speed (SYSCLKOUT)
//#define CPU_RATE   13.330L   // for a 75MHz CPU clock speed (SYSCLKOUT)
//#define CPU_RATE   20.000L   // for a 50MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE   33.333L   // for a 30MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE   41.667L   // for a 24MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE   50.000L   // for a 20MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE   66.667L   // for a 15MHz CPU clock speed  (SYSCLKOUT)
//#define CPU_RATE  100.000L   // for a 10MHz CPU clock speed  (SYSCLKOUT)

//
// DO NOT modify the code below this line
//
#define SCALE_FACTOR  1048576.0L*( (200L/CPU_RATE) )  // IQ20

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif // -- end FLASH2833X_API_CONFIG_H 

//
// End of File
//

