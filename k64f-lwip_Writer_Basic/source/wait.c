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
 * @file wait.c
 * @author RPC0027
 * @date December 2018
 * @version 1.0
 *
 * @brief Create waiting cycles on the CPU.
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "wait.h"

/*******************************************************************************
 * Code
 ******************************************************************************/
/**
 * @brief Wait 10 CPU cycles.
 *
 * This function will wait 10 CPU cycles (including call overhead).
 * lint -save -e522 function lacks side effect.
 * NOTE: Cortex-M0 and M4 have 1 cycle for a NOP.
 * Compiler is GNUC.
 */
#ifdef __GNUC__
#ifdef __cplusplus  /* gcc 4.7.3 in C++ mode does not like no_instrument_function: error: can't set 'no_instrument_function' attribute after definition */
  __attribute__((naked))
#else
  __attribute__((naked, no_instrument_function))
#endif
#endif
void wait_wait0Cycles(void)
{
	/*lint -save -e522 function lacks side effect. */
	__asm (
			/* bl to here   [4] */
			"nop   \n\t" /* [1] */
			"nop   \n\t" /* [1] */
			"nop   \n\t" /* [1] */
			"bx lr \n\t" /* [3] */
	);
	/*lint -restore */
}

/**
 * @brief Wait 100 CPU cycles.
 *
 * This function will wait 10 CPU cycles (including call overhead).
 * lint -save -e522 function lacks side effect.
 * NOTE: Cortex-M0 and M4 have 1 cycle for a NOP.
 * Compiler is GNUC.
 */
#ifdef __GNUC__
#ifdef __cplusplus  /* gcc 4.7.3 in C++ mode does not like no_instrument_function: error: can't set 'no_instrument_function' attribute after definition */
  __attribute__((naked))
#else
  __attribute__((naked, no_instrument_function))
#endif
#endif
void wait_wait00Cycles(void)
{
	/*lint -save -e522 function lacks side effect. */
	__asm (
			/* bl to here:               [4] */
			"push {r0}   \n\t"        /* [2] */
			"movs r0, #0 \n\t"        /* [1] */
			"loop:       \n\t"
			"nop         \n\t"        /* [1] */
			"nop         \n\t"        /* [1] */
			"nop         \n\t"        /* [1] */
			"nop         \n\t"        /* [1] */
			"add r0,#1   \n\t"        /* [1] */
			"cmp r0,#9   \n\t"        /* [1] */
			"bls loop    \n\t"        /* [3] taken, [1] not taken */
			"nop         \n\t"        /* [1] */
			"nop         \n\t"        /* [1] */
			"nop         \n\t"        /* [1] */
			"nop         \n\t"        /* [1] */
			"nop         \n\t"        /* [1] */
			"nop         \n\t"        /* [1] */
			"nop         \n\t"        /* [1] */
			"pop {r0}    \n\t"        /* [2] */
			"bx lr       \n\t"        /* [3] */
	);
	/*lint -restore */
}

/**
 * @brief Wait the specified CPU cycles.
 *
 * @param[in] cycles the number of CPU cycles to wait.
 */
void wait_WaitCycles(uint16_t cycles)
{
	/*lint -save -e522 function lacks side effect. */
#if wait_CONFIG_USE_CYCLE_COUNTER
	uint32_t counter = cycles;
	counter += KIN1_GetCycleCounter();

	while (KIN1_GetCycleCounter() < counter)
	{
		/* wait */
	}
#else
	while (cycles > 100)
	{
		wait_wait00Cycles();
		cycles -= 100;
	}

	while (cycles > 10)
	{
		wait_wait0Cycles();
		cycles -= 10;
	}
#endif
	/*lint -restore */
}


/**
 * @brief Wait the specified CPU cycles.
 *
 * @param[in] cycles the long number of CPU cycles to wait.
 */
void wait_WaitLongCycles(uint32_t cycles)
{
#if wait_CONFIG_USE_CYCLE_COUNTER
	uint32_t counter = cycles;
	counter += KIN1_GetCycleCounter();

	while (KIN1_GetCycleCounter() < counter)
	{
		/* wait */
	}
#else
	/*lint -save -e522 function lacks side effect. */
	while (cycles > 60000)
	{
		wait_WaitCycles(60000);
		cycles -= 60000;
	}

	wait_WaitCycles((uint16_t) cycles);
	/*lint -restore */
#endif
}

/**
 * @brief Wait the specified milliseconds.
 *
 * @param[in] ms the milliseconds to wait.
 */
void wait_Waitms(uint16_t ms)
{
	/*lint -save -e522 function lacks side effect. */
	uint32_t msCycles;
	msCycles = WAIT_NofCyclesMs(1, WAIT_INSTR_CLOCK_HZ);

	while (ms > 0)
	{
		wait_WaitLongCycles(msCycles);
		ms--;
	}
	/*lint -restore */
}

/**
 * @brief Init the cycle counter.
 */
void wait_Init(void)
{
#if wait_CONFIG_USE_CYCLE_COUNTER
	/* init cycle counter */
	KIN1_InitCycleCounter();
	KIN1_ResetCycleCounter();
	KIN1_EnableCycleCounter();
#endif
}

/**
 * @brief Deinit the cycle counter.
 */
void wait_DeInit(void)
{
#if wait_CONFIG_USE_CYCLE_COUNTER
	/* disable hardware cycle counter */
	KIN1_DisableCycleCounter();
#endif
}

/*** end of file ***/
