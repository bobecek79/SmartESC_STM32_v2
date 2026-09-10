#include "main.h"
#include "defines.h"


#ifndef APP_PRODUCT_H_
#define APP_PRODUCT_H_
 /*  Phase current (int16_t 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *
                                   *Amplifying network gain)/(MCU supply voltage/2)
*/

#ifdef G30P

#define VBUS_ADC_CHANNEL                                                     MC_ADC_CHANNEL_1
#define VOLTAGE_DIVIDER_GAIN     											 (float)3363.5

#define VESC_USART                                                 			 USART1

#define VESC_USART_DMA													     huart1
#define VESC_USART_TX_DMA													 hdma_usart1_tx
#define VESC_USART_RX_DMA													 hdma_usart1_rx

#define APP_USART_DMA														 huart2
#define APP_USART_TX_DMA												     hdma_usart2_tx
#define APP_USART_RX_DMA													 hdma_usart2_rx

#define APP2_USART_DMA														 huart3
#define APP2_USART_TX_DMA												     hdma_usart3_tx
#define APP2_USART_RX_DMA													 hdma_usart3_rx

#define RSHUNT                        										 0.00200
#define AMPLIFICATION_GAIN            										 9.4336
#define NOMINAL_CURRENT         											 2000
#define ID_DEMAG															 -2000

#define SCOPE_UVW															 0

#define POLE_PAIR_NUM                                                 	 	 (uint8_t)14
#define HALL_PHASE_SHIFT        											 90
#define HALL_SENSORS_PLACEMENT  											 DEGREES_120
#define HALL_FAULT_RESET_CNT												 200

#define TEMP_SENSOR_TYPE													 VIRTUAL_SENSOR
#define CURR_SENSOR_TYPE													 REAL_SENSOR

#define ADC_SAMPLE_MAX_LEN 													 500

#define BRAKE_LIGHT_GPIO_Port												 REAR_LED_GPIO_Port
#define BRAKE_LIGHT_Pin														 REAR_LED_Pin

// Setting limits
#define HW_LIM_CURRENT			-70.0, 70.0
#define HW_LIM_CURRENT_IN		-70.0, 70.0
#define HW_LIM_CURRENT_ABS		0.0, 100.0
#define HW_LIM_VIN				6.0, 57.0
#define HW_LIM_ERPM				-100e3, 100e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 75.0
#define HW_LIM_F_SW			    4000.0, 20000.0

#define MOT_TMR_MHZ 64
#define HEAP_SIZE_KB 14
#define CPU_MHZ  (64*1000000)
#define APP_PAGE				126
#define CONF_PAGE				127
#define PAGE_SIZE				0x400

#endif

#ifdef M365
#define VBUS_ADC_CHANNEL                                                     MC_ADC_CHANNEL_2
#define PHASE_A_V_ADC_CHANNEL                                                MC_ADC_CHANNEL_6
#define PHASE_B_V_ADC_CHANNEL                                                MC_ADC_CHANNEL_7
#define PHASE_C_V_ADC_CHANNEL                                                MC_ADC_CHANNEL_9
/* Calibrated 2026-09-06 against a meter: firmware reported 62.6 V where the
 * meter read 61.5 V. The reading is digits / BATTERY_VOLTAGE_GAIN, so reading
 * HIGH means the gain was LOW: new = 1510.0 * 62.6/61.5 = 1537.0.
 * That lands on the theoretical 1M/33k value, 2808.6359 * (33/1033)/(6.2/106.2)
 * = 1536.9, so measurement and theory agree to 0.007%. Full scale is now
 * 103.34 V (was 105.19); 63 V reads 39953 of 65535 counts. Was 1510.0. */
#define VOLTAGE_DIVIDER_GAIN     											 (float)1537.0

#define VESC_USART_DMA													     huart3
#define VESC_USART_TX_DMA													 hdma_usart3_tx
#define VESC_USART_RX_DMA													 hdma_usart3_rx

#define APP_USART_DMA														 huart1
#define APP_USART_TX_DMA												     hdma_usart1_tx
#define APP_USART_RX_DMA													 hdma_usart1_rx

//Current Measurement
#define RSHUNT                        										 0.00100  // shunts doubled in parallel (was 0.00200)
#define AMPLIFICATION_GAIN            										 8.00
#define NOMINAL_CURRENT         											 2000
#define ID_DEMAG														     -2000

#define SCOPE_UVW															 1

#define POLE_PAIR_NUM                                                 	 	 (uint8_t)15
#define HALL_PHASE_SHIFT        											 90
#define HALL_FAULT_RESET_CNT												 200

#define TEMP_SENSOR_TYPE													 REAL_SENSOR
#define CURR_SENSOR_TYPE													 VIRTUAL_SENSOR

#define ADC_SAMPLE_MAX_LEN 													 500   // halves the VESC Tool scope heap burst: 500*(2+2+1+1+1) = 3500 B, was 1000 -> 7000 B

// Setting limits
#define HW_LIM_CURRENT			-70.0, 70.0
#define HW_LIM_CURRENT_IN		-70.0, 70.0
#define HW_LIM_CURRENT_ABS		0.0, 100.0
/* Clamps BOTH l_max_vin and l_min_vin (conf_general.c:266-267) - it is one
 * min/max pair shared by two fields. The 6.0 floor is unchanged.
 * Max was 56.0, which is 99% of the STOCK divider's 56.55 V ADC full scale
 * (100k/6.2k, BATTERY_VOLTAGE_GAIN 1158.845) - a measurement ceiling, not a
 * 10S battery figure. With the 1M/33k divider full scale is now 105.19 V
 * (BVG 623.027), so 56.0 was a leftover that sat BELOW this 15S pack's 63 V
 * full charge and made a correct over-voltage threshold unsettable.
 * 75.0 V: +19% over the 63 V pack for regen headroom, 25% below the 100 V
 * IRFB4110 / bulk cap rating so switching overshoot has 25 V of room before
 * Vds(max), and 29% below the 105.19 V ADC ceiling so the reading stays
 * unwrapped across the whole permitted range (75 V = 46727 of 65535 counts). */
#define HW_LIM_VIN				6.0, 75.0
#define HW_LIM_ERPM				-100e3, 100e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 75.0
#define HW_LIM_F_SW			    4000.0, 20000.0

#define MOT_TMR_MHZ 64
/* Sized from measurement, not prediction. Live `top` with the ESP32 polling:
 * MinEverFree 6168 B => peak used 8168 B of the old 14336 B pool.
 *   ceil(8168 * 1.30 / 1024) = ceil(10.37) = 11 KB
 * The old 14 KB existed to cover a 7000 B SCOPE_UVW capture burst
 * (VescCommand.c:557-564); that allowance is dropped because this board
 * does not use the scope. If SCOPE_UVW capture is ever wanted again, this
 * must go back up - 11 KB leaves 3096 B over the measured peak, and a
 * capture at ADC_SAMPLE_MAX_LEN 500 alone needs 3500 B.
 * Largest burst not present in the measurement is motor detection /
 * COMM_GET_MCCONF at 1056 B (540 B mc_configuration + 516 B send buffer
 * held concurrently), which fits with 2040 B to spare. Was 14. */
#define HEAP_SIZE_KB 11
#define CPU_MHZ  (64*1000000)
#define APP_PAGE				126
#define CONF_PAGE				127
#define PAGE_SIZE				0x400


#endif

#ifdef M365_gd32
#define VBUS_ADC_CHANNEL                                                     MC_ADC_CHANNEL_2
#define PHASE_A_V_ADC_CHANNEL                                                MC_ADC_CHANNEL_6
#define PHASE_B_V_ADC_CHANNEL                                                MC_ADC_CHANNEL_7
#define PHASE_C_V_ADC_CHANNEL                                                MC_ADC_CHANNEL_9
#define VOLTAGE_DIVIDER_GAIN     											 (float)2808.6359

#define VESC_USART_DMA													     huart3
#define VESC_USART_TX_DMA													 hdma_usart3_tx
#define VESC_USART_RX_DMA													 hdma_usart3_rx

#define APP_USART_DMA														 huart1
#define APP_USART_TX_DMA												     hdma_usart1_tx
#define APP_USART_RX_DMA													 hdma_usart1_rx

//Current Measurement
#define RSHUNT                        										 0.00200
#define AMPLIFICATION_GAIN            										 8.00
#define NOMINAL_CURRENT         											 2000
#define ID_DEMAG														     -2000

#define SCOPE_UVW															 1

#define POLE_PAIR_NUM                                                 	 	 (uint8_t)15
#define HALL_PHASE_SHIFT        											 90
#define HALL_FAULT_RESET_CNT												 200

#define TEMP_SENSOR_TYPE													 REAL_SENSOR
#define CURR_SENSOR_TYPE													 VIRTUAL_SENSOR

#define ADC_SAMPLE_MAX_LEN 													 2000

// Setting limits
#define HW_LIM_CURRENT			-70.0, 70.0
#define HW_LIM_CURRENT_IN		-70.0, 70.0
#define HW_LIM_CURRENT_ABS		0.0, 100.0
#define HW_LIM_VIN				6.0, 56.0
#define HW_LIM_ERPM				-100e3, 100e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.99
#define HW_LIM_TEMP_FET			-40.0, 75.0
#define HW_LIM_F_SW			    4000.0, 50000.0

#define MOT_TMR_MHZ 60
#define HEAP_SIZE_KB 80
#define CPU_MHZ  (120*1000000)
#define APP_PAGE				255
#define CONF_PAGE				254
#define PAGE_SIZE				0x800


#endif
/****************************************************************************/

#define KMH_NO_LIMIT														 1337
#define PRODUCT_FIRMWARE_VERSION                                      		 0x0001
#define VESC_TOOL_ENABLE													 1
#define AUTO_RESET_FAULT													 1
#define ERROR_PRINTING														 1
#define MUSIC_ENABLE														 0
#define BATTERY_SUPPORT_LIION												 1
#define BATTERY_SUPPORT_LIFEPO												 1
#define BATTERY_SUPPORT_LEAD												 1
#define ABS_OVR_CURRENT_TRIP_MS												 2.0
#define MIN_DUTY_FOR_PWM_FREEWHEEL											 20
#define CURRENT_DISPLAY_OFFSET											     80   //in cnts

#define MODE_SLOW_CURR														 0.5
#define MODE_DRIVE_CURR														 0.8
#define MODE_SPORT_CURR														 1.0
/* All three at KMH_NO_LIMIT so the drive modes differentiate by current scale
 * only. Any value other than KMH_NO_LIMIT (1337) makes task_pwr.c compute a
 * reduced lo_max_erpm and hand it to MCI_ExecSpeedRamp(), putting the mode into
 * speed-limited operation - which is what made SLOW and DRIVE feel gutless
 * rather than simply weaker. Current scales (above) are untouched.
 * Was 10 / 25 / KMH_NO_LIMIT. */
#define MODE_SLOW_SPEED														 KMH_NO_LIMIT
#define MODE_DRIVE_SPEED													 KMH_NO_LIMIT
#define MODE_SPORT_SPEED													 KMH_NO_LIMIT

#define BATTERY_VOLTAGE_GAIN     											 ((VOLTAGE_DIVIDER_GAIN * ADC_GAIN) * 512.0)
#define CURRENT_FACTOR_A 													 ((32767.0*RSHUNT*AMPLIFICATION_GAIN)/(3.3/2))
#define CURRENT_FACTOR_mA 													 (CURRENT_FACTOR_A/1000.0)
#define MIN_DUTY_PWM												         (32768 * MIN_DUTY_FOR_PWM_FREEWHEEL / 100)

#define DEMCR_TRCENA    0x01000000
#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))
#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA       (1<<0)
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT
#define CPU_CLOCK		64000000


#define PRIO_BELOW_NORMAL 4
#define PRIO_NORMAL  5
#define PRIO_HIGHER  6

#endif /* APP_PRODUCT_H_ */
