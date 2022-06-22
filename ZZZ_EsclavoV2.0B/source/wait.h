/**
 * Copyright 2019 rpc0027
 *
 * Licensed under the Apache License, Version 2.0 (the "License")
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file wait.h
 * @author RPC0027
 * @date December 2018
 * @version 1.0
 *
 * @brief Create waiting cycles on the CPU.
 */
#ifndef WAIT_H
#define WAIT_H

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdint.h>

#include "wait_config.h"

#if WAIT_CONFIG_USE_RTOS_WAIT
#include "FreeRTOS.h"
#include "task.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/** Clock frequency variable defined by the SDK */
extern uint32_t SystemCoreClock;
/** Core clock frequency in Hz. */
#define WAIT_INSTR_CLOCK_HZ       SystemCoreClock
/** Calculate the needed cycles based on bus clock frequency for ms. */
#define WAIT_NofCyclesMs(ms, hz)  ((ms)*((hz)/1000))
/** Calculate the needed cycles based on bus clock frequency for us. */
#define WAIT_NofCyclesUs(us, hz)  ((us)*(((hz)/1000)/1000))
/** Calculate the needed cycles based on bus clock frequency for ns.*/
#define WAIT_NofCyclesNs(ns, hz)  (((ns)*(((hz)/1000)/1000))/1000)
/** Wait for some cycles. */
#define WAIT_WAIT_C(cycles) \
        ( (cycles)<=10 ? \
                wait_wait0Cycles() \
                : wait_WaitCycles((uint16_t)cycles) \
        )
/** Wait for some microseconds. */
#define wait_Waitus(us)  \
        /*lint -save -e(505,506,522) Constant value Boolean, Redundant left argument to comma. */\
        (  ((WAIT_NofCyclesUs((us),WAIT_INSTR_CLOCK_HZ)==0)||(us)==0) ? \
                (void)0 : \
                ( ((us)/1000)==0 ? (void)0 : wait_Waitms((uint16_t)((us)/1000))) \
                , (WAIT_NofCyclesUs(((us)%1000), WAIT_INSTR_CLOCK_HZ)==0) ? (void)0 : \
                        WAIT_WAIT_C(WAIT_NofCyclesUs(((us)%1000), WAIT_INSTR_CLOCK_HZ)) \
                        /*lint -restore */\
        )

/** Wait for some nanoseconds. */
#define wait_Waitns(ns)  \
        /*lint -save -e(505,506,522) Constant value Boolean, Redundant left argument to comma. */\
        (  ((wait_NofCyclesNs((ns), wait_INSTR_CLOCK_HZ)==0)||(ns)==0) ? \
                (void)0 : \
                wait_Waitus((ns)/1000) \
                , (wait_NofCyclesNs((ns)%1000, wait_INSTR_CLOCK_HZ)==0) ? \
                        (void)0 : \
                        wait_WAIT_C(wait_NofCyclesNs(((ns)%1000), wait_INSTR_CLOCK_HZ)) \
                        /*lint -restore */\
        )

#if wait_CONFIG_USE_RTOS_WAIT
#define wait_WaitOSms(ms) vTaskDelay(pdMS_TO_TICKS(ms)) /* Use FreeRTOS API */
#else
#define wait_WaitOSms(ms)  wait_Waitms(ms) /* Use normal wait */
#endif

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void wait_wait0Cycles(void);
void wait_wait00Cycles(void);
void wait_WaitCycles(uint16_t cycles);
void wait_WaitLongCycles(uint32_t cycles);
void wait_Waitms(uint16_t ms);
void wait_Init(void);
void wait_DeInit(void);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* WAIT_H */

/*** end of file ***/
