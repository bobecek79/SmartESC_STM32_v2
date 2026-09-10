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

#include "main.h"
#include "app.h"
#include "utils.h"
#include "VescToSTM.h"
#include "FreeRTOS.h"
#include "task.h"
#include "product.h"
#include "ninebot.h"
#include "VescCommand.h"
#include "task_init.h"
#include "timers.h"
#include <math.h>

NinebotPack frame;

m365Answer m365_to_display = {.start1=NinebotHeader0, .start2=NinebotHeader1, .len=8, .addr=0x21, .cmd=0x64, .arg=0, .mode=M365_MODE_SPORT};

uint8_t app_connection_timout = 8;

void vTimerCallback( TimerHandle_t xTimer );
uint32_t app_updaterate();

TaskHandle_t task_app_handle;
TimerHandle_t xTimer;

void my_uart_send_data(unsigned char *buf, unsigned int len, port_str * port){
	if(port->half_duplex){
		port->uart->Instance->CR1 &= ~USART_CR1_RE;
		vTaskDelay(1);
	}
	HAL_UART_Transmit_DMA(port->uart, buf, len);
	while(port->uart->hdmatx->State != HAL_DMA_STATE_READY){
		port->uart->gState = HAL_UART_STATE_READY;
		vTaskDelay(1);
	}
	if(port->half_duplex) port->uart->Instance->CR1 |= USART_CR1_RE;
}

static uint32_t uart_get_write_pos(port_str * port){
	return ( ((uint32_t)port->rx_buffer_size - port->uart->hdmarx->Instance->CNDTR) & ((uint32_t)port->rx_buffer_size -1));
}

static uint8_t adc1;
static uint8_t adc2;
void app_adc_set_adc(uint8_t AD1, uint8_t AD2){
	if(xTimer!=NULL && xTimerIsTimerActive(xTimer)==pdFALSE){
		xTimerStart(xTimer, 100);
	}
	adc1 = AD1;
	adc2 = AD2;
}



static float decoded_level = 0.0;
static float decoded_level2 = 0.0;

#define config		appconf.app_adc_conf

#define TimeMsElapsedSinceX(start)                                        \
  ((uint32_t)((xTaskGetTickCount()- (start))/2))


void vTimerCallback( TimerHandle_t xTimer ){
	//uint32_t cycles = *DWT_CYCCNT;
	if(SpeednTorqCtrlM1.SPD->open_loop==true) return;
	uint32_t temp1 = adc1;
	uint32_t temp2 = adc2;
	temp1 <<= 4;
	temp2 <<= 4;
	static uint16_t aver1=0;
	static uint16_t aver2=0;

	if(config.use_filter){
		aver1 += temp1;
		aver1 /=2;
		aver2 += temp2;
		aver2 /=2;
	}else{
		aver1 = temp1;
		aver2 = temp2;
	}

	float pwr = utils_map(aver1, 0, 255<<4, 0, 3.3); //Throttle;
	float brake = utils_map(aver2, 0, 255<<4, 0, 3.3); //Brake;
	VescToSTM_set_ADC1(pwr);
	VescToSTM_set_ADC2(brake);


	// Map the read voltage
	switch (config.ctrl_type) {
	case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
	case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
	case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
	case ADC_CTRL_TYPE_DUTY_REV_CENTER:
	case ADC_CTRL_TYPE_PID_REV_CENTER:
		// Mapping with respect to center voltage
		if (pwr < config.voltage_center) {
			pwr = utils_map(pwr, config.voltage_start,
					config.voltage_center, 0.0, 0.5);
		} else {
			pwr = utils_map(pwr, config.voltage_center,
					config.voltage_end, 0.5, 1.0);
		}
		break;

	default:
		// Linear mapping between the start and end voltage
		pwr = utils_map(pwr, config.voltage_start, config.voltage_end, 0.0, 1.0);
		break;
	}

	// Optionally invert the read voltage
	if (config.voltage_inverted) {
		pwr = 1.0 - pwr;
	}
	pwr *= mc_conf.lo_current_max_scale;
	utils_truncate_number(&pwr, 0.0, 1.0);

	brake = utils_map(brake, config.voltage2_start, config.voltage2_end, 0.0, 1.0);
	// Optionally invert the read voltage
	if (config.voltage2_inverted) {
		brake = 1.0 - brake;
	}
	utils_truncate_number(&brake, 0.0, 1.0);
	decoded_level = pwr;
	decoded_level2 = brake;

	switch (config.ctrl_type) {
		case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
		case ADC_CTRL_TYPE_DUTY_REV_CENTER:
		case ADC_CTRL_TYPE_PID_REV_CENTER:
			// Scale the voltage and set 0 at the center
			pwr *= 2.0;
			pwr -= 1.0;
			break;

		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC:
		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC:
			pwr -= brake;
			break;

		default:
			break;
	}

	// Apply deadband
	utils_deadband(&pwr, config.hyst, 1.0);
	utils_deadband(&brake, config.hyst, 1.0);

	// Apply throttle curve
	pwr = utils_throttle_curve(pwr, config.throttle_exp, config.throttle_exp_brake, config.throttle_exp_mode);

	// Apply ramping
	static uint32_t last_time = 0;
	static float pwr_ramp = 0.0;
	float ramp_time = fabsf(pwr) > fabsf(pwr_ramp) ? config.ramp_time_pos : config.ramp_time_neg;

	if (ramp_time > 0.01) {
		const float ramp_step = (float)TimeMsElapsedSinceX(last_time) / (ramp_time * 1000.0);
		utils_step_towards(&pwr_ramp, pwr, ramp_step);
		last_time = xTaskGetTickCount();
		pwr = pwr_ramp;
	}

	if(app_is_output_disabled()){
		return;
	}

	// Use the filtered and mapped voltage for control according to the configuration.
	switch (config.ctrl_type) {
	case ADC_CTRL_TYPE_CURRENT:
	case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
		VescToSTM_set_current_rel(pwr);
		break;

	case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
		if(pwr>=0){
			VescToSTM_set_current_rel(pwr);
		}else{
			VescToSTM_set_brake_current_rel(pwr);
		}
		break;
	case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC:
		if(brake>0){
			VescToSTM_set_brake_current_rel(brake);
		}else if(pwr>=0){
			VescToSTM_set_current_rel(pwr);
		}

		break;

	case ADC_CTRL_TYPE_PID:
	case ADC_CTRL_TYPE_PID_REV_CENTER:
	case ADC_CTRL_TYPE_PID_REV_BUTTON:{
		float speed = 0.0;
		if (pwr >= 0.0) {
			speed = pwr * mc_conf.lo_max_erpm;
		} else {
			speed = pwr * fabsf(mc_conf.lo_min_erpm);
		}
		VescToSTM_set_speed(speed);
	}
		break;
	default:
		break;
	}
//	FOCVars[M1].cycles_last = *DWT_CYCCNT - cycles;
//		if(FOCVars[M1].cycles_last > FOCVars[M1].cycles_max){
//			FOCVars[M1].cycles_max = FOCVars[M1].cycles_last;
//		}
}

float app_adc_get_decoded_level(void) {
	return decoded_level;
}
float app_adc_get_decoded_level2(void) {
	return decoded_level2;
}

void app_adc_stop_output(void) {
	if(xTimer!=NULL){
		xTimerStop(xTimer, 2000);
	}
}

void app_adc_set_mode(uint8_t mode_bit){
	m365_to_display.mode |= mode_bit;
}

void app_adc_clear_mode(uint8_t mode_bit){
	m365_to_display.mode &= ~mode_bit;
}

void app_adc_speed_mode(uint8_t speed){
	m365_to_display.mode &= 0xF8;
	m365_to_display.mode |= speed;
}

/* Re-apply the selected drive mode's current scale to the live configuration.
 *
 * conf_general_setup_mc() unconditionally resets lo_current_max_scale to 1.0
 * every time a configuration is loaded or written, and nothing used to put the
 * mode's scale back. That meant any config write from VESC Tool silently
 * restored full throttle authority while the display still showed SLOW or
 * DRIVE. conf_general_setup_mc() now calls this as its last action.
 *
 * lo_current_max_scale is consumed in exactly one place - the throttle scaling
 * at app_uartcomm.c app_adc_get_decoded_level() - so writing the global mc_conf
 * here is sufficient; the caller's struct keeps the 1.0 that gets stored to
 * flash, which is correct for a runtime override field.
 *
 * M365_MODE_* are bit flags (DRIVE=1, SLOW=2, SPORT=4) living in the low three
 * bits; M365_MODE_TEMP (128) and friends share the byte, hence the 0x07 mask.
 * The index order matches the mode-change switch in task_pwr.c. */
void app_adc_apply_mode_scale(void){
	switch(m365_to_display.mode & 0x07){
		case M365_MODE_SLOW:
			mc_conf.lo_current_max_scale = mc_conf.modes_curr_scale[0];
			break;
		case M365_MODE_DRIVE:
			mc_conf.lo_current_max_scale = mc_conf.modes_curr_scale[1];
			break;
		case M365_MODE_SPORT:
			mc_conf.lo_current_max_scale = mc_conf.modes_curr_scale[2];
			break;
		default:
			/* No recognised speed mode set: leave the 1.0 conf_general_setup_mc()
			 * just wrote rather than guessing a cap. */
			break;
	}
}

uint32_t app_updaterate(){
	return 2000 / config.update_rate_hz;
}

void app_check_timer(){
	if(xTimerIsTimerActive(xTimer)==pdFALSE){
		xTimerChangePeriod(xTimer, app_updaterate(),100);
		xTimerStart(xTimer, 100);
	}
}

void app_adc_init_timer(){
	if(xTimer==NULL){
		xTimer = xTimerCreate("ADC_UP",app_updaterate() , pdTRUE, ( void * ) 0,vTimerCallback );
	}
}

void app_timer_update_period(){
	xTimerChangePeriod(xTimer, app_updaterate(), 1000);
}

void task_app(void * argument)
{
	uint32_t rd_ptr=0;
	port_str * port = (port_str*) argument;
	uint8_t * usart_rx_dma_buffer = pvPortMalloc(port->rx_buffer_size);
	HAL_UART_MspInit(port->uart);
	if(port->half_duplex){
		HAL_HalfDuplex_Init(port->uart);
	}
	HAL_UART_Receive_DMA(port->uart, usart_rx_dma_buffer, port->rx_buffer_size);
	CLEAR_BIT(port->uart->Instance->CR3, USART_CR3_EIE);

	app_adc_init_timer();

	xTimerStart(xTimer, 100);


	/* Telemetry refresh divider. This loop runs every vTaskDelay(10 ticks) +
	 * ulTaskNotifyTake(...,10 ticks) = 20 ticks = 10 ms at configTICK_RATE_HZ
	 * 2000, so the divider is in units of 10 ms.
	 * 10 -> a 100 ms refresh, half the ESP32's 200 ms 0x64 poll interval, so
	 * every poll still gets data at most one period old. NOT 50: that would be
	 * 500 ms, 2.5x slower than the poll, and the display would repeat stale
	 * speed and battery readings for two or three polls in a row. */
#define TELEMETRY_UPDATE_DIV 10
	uint16_t slow_update_cnt=0;
  /* Infinite loop */
	for(;;)
	{
		while(rd_ptr != uart_get_write_pos(port)) {
			if(ninebot_parse(usart_rx_dma_buffer[rd_ptr] ,&frame)	==0){
				//commands_printf(main_uart.phandle, "LEN: %d CMD: %x ARG: %x PAY: %02x %02x %02x %02x", frame.len, frame.cmd, frame.arg, frame.payload[0], frame.payload[1], frame.payload[2], frame.payload[3]);
				switch(frame.cmd){
					case 0x64:
						addCRC((uint8_t*)&m365_to_display, m365_to_display.len+6);
						my_uart_send_data((uint8_t*)&m365_to_display, sizeof(m365_to_display), port);
					break;
					case 0x65:
						adc1 = frame.payload[1];
						adc2 = frame.payload[2];
						VescToSTM_timeout_reset();
						app_check_timer();
						//commands_printf(main_uart.phandle, "LEN: %d CMD: %x ARG: %x PAY: %02x %02x %02x %02x", frame.len, frame.cmd, frame.arg, frame.payload[0], frame.payload[1], frame.payload[2], frame.payload[3]);
					break;
				}
			}
			rd_ptr++;
			rd_ptr &= ((uint32_t)port->rx_buffer_size - 1);
		}

		if(slow_update_cnt==0){
			if(m365_to_display.mode & 0x40){
				m365_to_display.speed = VescToSTM_get_speed()*2.2369;
			}else{
				m365_to_display.speed = VescToSTM_get_speed()*3.6;

			}
			m365_to_display.speed *= DIR_MUL;
			int temp = utils_map(VescToSTM_get_battery_level(0), 0, 1, 0, 100);
			m365_to_display.battery = temp>100?100:temp;

			/* [9] ESC temperature, degrees C. NTC_GetAvTemp_C() already returns
			 * whole degrees, so the cast loses nothing; clamp only guards a
			 * sensor fault producing something outside int8_t. */
			float t_c = VescToSTM_get_temperature();
			m365_to_display.esc_temp = (t_c > 127.0f) ? 127 : ((t_c < -128.0f) ? -128 : (int8_t)t_c);

			/* [11] Electrical power in 20 W steps, negative on regen.
			 * VescToSTM_get_input_current() is motor power divided by
			 * VBS_GetAvBusVoltage_V(), a uint16_t volt count, so it divides by
			 * zero on a dead bus - the short-circuit guard keeps that call out
			 * of the expression entirely below 1 V rather than casting an inf. */
			float vbus  = VescToSTM_get_bus_voltage();
			float watts = (vbus > 1.0f) ? (vbus * VescToSTM_get_input_current()) : 0.0f;
			float p20   = watts / 20.0f;
			m365_to_display.power = (p20 > 127.0f) ? 127 : ((p20 < -127.0f) ? -127 : (int8_t)p20);

		}

		/* Was inside an else on the branch above, so it never ran: the counter
		 * stayed 0, the if-branch was taken every iteration and the "slow"
		 * update was not slow at all. Incrementing unconditionally is the fix. */
		slow_update_cnt++;
		if(slow_update_cnt >= TELEMETRY_UPDATE_DIV) slow_update_cnt=0;


		vTaskDelay(10);
		if(ulTaskNotifyTake(pdTRUE, 10)){
			xTimerStop(xTimer, 2000);
			HAL_UART_MspDeInit(port->uart);
			port->task_handle = NULL;
			vPortFree(usart_rx_dma_buffer);
			vTaskDelete(NULL);
			vTaskDelay(portMAX_DELAY);
		}

	}
}

void task_app_init(port_str * port){
	if(port->task_handle == NULL){
		xTaskCreate(task_app, "APP-ADC", 128, (void*)port, PRIO_BELOW_NORMAL, &port->task_handle);
	}else{
		app_timer_update_period();
	}
}

void task_app_kill(port_str * port){
	if(port->task_handle){
		xTaskNotify(port->task_handle, 0, eIncrement);
		vTaskDelay(200);
	}
}
