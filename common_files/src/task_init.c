/*
 * m365
 *
 * Copyright (c) 2021 Jens Kerrinnes
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "task_init.h"
#include "task_LED.h"
#include "task_pwr.h"
#include "product.h"
#include "app.h"
#include "main.h"
#include "task.h"
#include "VescToSTM.h"


/* ---------------------------------------------------------------------------
 * RTOS fault hooks
 *
 * Enabled by configCHECK_FOR_STACK_OVERFLOW 2 and configUSE_MALLOC_FAILED_HOOK 1
 * in FreeRTOSConfig.h. These are STRONG definitions that deliberately override
 * the __WEAK no-ops in FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c (:1819, :1833);
 * without them the config flags would link and still do nothing.
 *
 * Both faults mean memory has already been corrupted, so there is no safe way to
 * carry on driving a motor. The hooks put the power stage into a hardware-safe
 * state first, then blink a distinguishable pattern forever:
 *
 *     2 blinks, pause  ->  stack overflow  (rtos_fault_task names the task)
 *     3 blinks, pause  ->  pvPortMalloc() returned NULL
 *
 * Deliberately not a silent while(1). rtos_fault_code / rtos_fault_task /
 * rtos_fault_count are plain globals so a debugger can read them out, and the
 * LED tells you which fault it was without one.
 * ------------------------------------------------------------------------- */

volatile uint32_t rtos_fault_code  = RTOS_FAULT_NONE;
volatile uint32_t rtos_fault_count = 0u;
volatile char     rtos_fault_task[configMAX_TASK_NAME_LEN + 1] = {0};

/* Busy-wait. Interrupts are masked by the time this runs, so SysTick and
 * therefore HAL_Delay() are unavailable. Roughly 5 cycles per iteration; the
 * exact period does not matter, only that the blink is visible. */
static void prv_rtos_fault_delay(uint32_t ms)
{
	volatile uint32_t i = (CPU_CLOCK / 5000u) * ms;
	while (i-- != 0u)
	{
	}
}

static void prv_rtos_fault_halt(uint32_t code, const char * name)
{
	uint32_t i = 0u;

	rtos_fault_code = code;
	rtos_fault_count++;
	if (name != NULL)
	{
		for (; i < configMAX_TASK_NAME_LEN && name[i] != '\0'; i++)
		{
			rtos_fault_task[i] = name[i];
		}
	}
	rtos_fault_task[i] = '\0';

	/* Safe the power stage BEFORE masking interrupts. R3_2_SwitchOffPWM()
	 * clears TIM1 MOE via LL_TIM_DisableAllOutputs() and disables the update
	 * ISR, so the bridge stays off once we stop servicing interrupts. It spins
	 * on the TIM1 update flag, which still toggles because the timer itself
	 * keeps counting - so this cannot deadlock here. */
	VescToSTM_pwm_stop();

	__disable_irq();

	for (;;)
	{
		uint32_t n;
		for (n = 0u; n < code; n++)
		{
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			prv_rtos_fault_delay(120u);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			prv_rtos_fault_delay(200u);
		}
		prv_rtos_fault_delay(900u);
	}
}

/* Signature matches the call site in FreeRTOS/Source/tasks.c:406 and
 * stack_macros.h, which pass pxCurrentTCB->pcTaskName as plain char *. */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char * pcTaskName)
{
	(void)xTask;
	prv_rtos_fault_halt(RTOS_FAULT_STACK_OVERFLOW, pcTaskName);
}

void vApplicationMallocFailedHook(void)
{
	prv_rtos_fault_halt(RTOS_FAULT_MALLOC_FAILED, NULL);
}

unsigned long getRunTimeCounterValue(void){
	return HAL_GetTick();
}

port_str main_uart = {	.uart = &VESC_USART_DMA,
					    .rx_buffer_size = 512,
						.phandle = NULL,
						.half_duplex = false,
						.task_handle = NULL
};
port_str aux_uart = {	.uart = &APP_USART_DMA,
					    .rx_buffer_size = 128,
						.phandle = NULL,
						.half_duplex = true,
						.task_handle = NULL
};

void task_init(){
	app_adc_init_timer();
	task_cli_init(&main_uart);
	task_LED_init(&main_uart);  //Bring up the blinky
	task_PWR_init(&main_uart);  //Manage power button
}
