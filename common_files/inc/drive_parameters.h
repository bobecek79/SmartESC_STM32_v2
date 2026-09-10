
/**
  ******************************************************************************
  * @file    drive_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a motor drive.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRIVE_PARAMETERS_H
#define __DRIVE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/*** Speed measurement settings ***/
#define MAX_APPLICATION_SPEED_RPM       2500 /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED_RPM       0 /*!< rpm, mechanical,
                                                           absolute value */
#define MEAS_ERRORS_BEFORE_FAULTS       20 /*!< Number of speed
                                                             measurement errors before
                                                             main sensor goes in fault */
/****** Hall sensors ************/
#define HALL_MEAS_ERRORS_BEFORE_FAULTS  3 /*!< Number of failed
                                                           derived class specific speed
                                                           measurements before main sensor
                                                           goes in fault */

/* Was 12. The FIFO averages one entry per hall edge (6 per electrical
 * revolution), so its lag in time depends on speed, not on a fixed period: at
 * 26.5 Hz electrical (5 km/h) 12 entries span ~75 ms, which is a long delay to
 * feed a 1 kHz speed loop and reads as sluggish throttle response. 6 entries
 * halves that to ~38 ms.
 * Tradeoff: fewer samples means less smoothing of hall-sensor mechanical
 * misalignment, so AvrElSpeedDpp gets noisier - the averaging exists precisely
 * to hide uneven magnet/sensor spacing. If low-speed running becomes rough or
 * the speed reading jitters, this is the first thing to put back up.
 * Safe against the buffer: SensorPeriod[] is sized by HALL_SPEED_FIFO_SIZE (16,
 * hall_speed_pos_fdbk.h:45), a different constant, and every loop bounds itself
 * with the runtime SpeedBufferSize field. The index wraps by comparison
 * (hall_speed_pos_fdbk.c:583), not by a mask, so the depth need not be a power
 * of two and no RAM changes. */
#define HALL_AVERAGING_FIFO_DEPTH        6 /*!< depth of the FIFO used to
                                                           average mechanical speed in
                                                           0.1Hz resolution */
#define HALL_MTPA  false

/* USER CODE BEGIN angle reconstruction M1 */
/* Lead compensation for the delay between sampling the phase currents and the
 * computed voltage actually reaching the bridge. Used in FOC_CurrControllerM1()
 * (mc_tasks.c) as:
 *     hElAngle += SPD_GetInstElSpeedDpp(speedHandle) * REV_PARK_ANGLE_COMPENSATION_FACTOR;
 * SPD_GetInstElSpeedDpp() returns dpp - per UM1052 s10.4.2, the change in
 * electrical angle (s16, 65536 = 360 deg el) within one FOC period. With
 * REGULATION_EXECUTION_RATE 1 the FOC period IS the PWM period, so this factor
 * is simply the delay expressed in PWM periods.
 *
 * Delay for this configuration (center-aligned TIM1, three-shunt sampling at the
 * zero vector, CCRs preloaded):
 *     currents sampled at the counter overflow          t = 0
 *     new CCRs take effect at the next update event     +1.0 T_pwm
 *     centroid of the voltage applied over that period  +0.5 T_pwm
 *                                                       ---------
 *                                                        1.5 T_pwm
 *
 * The theoretical value is therefore 1.5. It is set to 1 because hElAngle and
 * hElSpeedDpp are both int16_t: a value of 1.5 makes the expression a
 * double-precision multiply inside the 16 kHz current loop on an FPU-less
 * Cortex-M3, which is not worth the residual error it removes. At 15 pole pairs,
 * a 0.250 m wheel and 16 kHz, one PWM period of lag is 2.98 deg el at 25 km/h and
 * 5.37 deg at 45 km/h; using 1 instead of 1.5 leaves half of that uncompensated
 * (1.49 deg / 2.69 deg), worth under 0.11 % of torque. Under-compensating is also
 * the safer side to err on - over-compensation leads the angle and is mildly
 * destabilising at speed.
 *
 * Neither UM1052 nor UM2392 documents this constant or gives a formula for it;
 * the derivation above is from the PWM/ADC timing in UM2392 section 4 plus the
 * dpp definition in UM1052 s10.4.2. Was 0 (no compensation at all). */
#define REV_PARK_ANGLE_COMPENSATION_FACTOR 1
/* USER CODE END angle reconstruction M1 */

/**************************    DRIVE SETTINGS SECTION   **********************/
/* PWM generation and current reading */

#define PWM_FREQUENCY   16000
#define PWM_FREQ_SCALING 1

#define LOW_SIDE_SIGNALS_ENABLING        LS_PWM_TIMER
#define SW_DEADTIME_NS                   800 /*!< Dead-time to be inserted
                                                           by FW, only if low side
                                                           signals are enabled */

/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE     1    /*!< FOC execution rate in
                                                           number of PWM cycles */
/* Gains values for torque and flux control loops */
#define PID_TORQUE_KP_DEFAULT         500
#define PID_TORQUE_KI_DEFAULT         300
#define PID_TORQUE_KD_DEFAULT         100
#define PID_FLUX_KP_DEFAULT           500
#define PID_FLUX_KI_DEFAULT           300
#define PID_FLUX_KD_DEFAULT           100

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV                      1024
#define TF_KIDIV                      16384
#define TF_KDDIV                      8192
#define TF_KPDIV_LOG                  LOG2(1024)
#define TF_KIDIV_LOG                  LOG2(16384)
#define TF_KDDIV_LOG                  LOG2(8192)
#define TFDIFFERENTIAL_TERM_ENABLING  DISABLE

/* Speed control loop */
#define SPEED_LOOP_FREQUENCY_HZ       1000 /*!<Execution rate of speed
                                                      regulation loop (Hz) */

#define PID_SPEED_KP_DEFAULT          400/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KI_DEFAULT          50/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KD_DEFAULT          0/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
/* Speed PID parameter dividers */
#define SP_KPDIV                      16
#define SP_KIDIV                      256
#define SP_KDDIV                      16
#define SP_KPDIV_LOG                  LOG2(16)
#define SP_KIDIV_LOG                  LOG2(256)
#define SP_KDDIV_LOG                  LOG2(16)

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV */
#define PID_SPEED_INTEGRAL_INIT_DIV 1 /*  */
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV */

#define SPD_DIFFERENTIAL_TERM_ENABLING DISABLE
#define IQMAX                          4766

/* Default settings */
#define DEFAULT_CONTROL_MODE           STC_TORQUE_MODE /*!< STC_TORQUE_MODE or
                                                        STC_SPEED_MODE */
#define DEFAULT_TARGET_SPEED_RPM      0
#define DEFAULT_TARGET_SPEED_UNIT      (DEFAULT_TARGET_SPEED_RPM*SPEED_UNIT/_RPM)
#define DEFAULT_TORQUE_COMPONENT       0
#define DEFAULT_FLUX_COMPONENT         0

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
#define OV_VOLTAGE_PROT_ENABLING        ENABLE
#define UV_VOLTAGE_PROT_ENABLING        ENABLE
#define OV_VOLTAGE_THRESHOLD_V          42 /*!< Over-voltage
                                                         threshold */
#define UD_VOLTAGE_THRESHOLD_V          10 /*!< Under-voltage
                                                          threshold */
#if 0
#define ON_OVER_VOLTAGE                 TURN_OFF_PWM /*!< TURN_OFF_PWM,
                                                         TURN_ON_R_BRAKE or
                                                         TURN_ON_LOW_SIDES */
#endif /* 0 */
#define R_BRAKE_SWITCH_OFF_THRES_V      34

#define OV_TEMPERATURE_THRESHOLD_C      70 /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C     5 /*!< Celsius degrees */

#define HW_OV_CURRENT_PROT_BYPASS       DISABLE /*!< In case ON_OVER_VOLTAGE
                                                          is set to TURN_ON_LOW_SIDES
                                                          this feature may be used to
                                                          bypass HW over-current
                                                          protection (if supported by
                                                          power stage) */
/******************************   START-UP PARAMETERS   **********************/

#define TRANSITION_DURATION            25  /* Switch over duration, ms */
/******************************   BUS VOLTAGE Motor 1  **********************/
/* 55.5 ADC cycles. The battery divider is 1M/33k on this board => 31.95 kOhm
 * source impedance. At f_ADC = 10.667 MHz (PCLK2 64 MHz / 6) 55.5 cycles gives
 * t_s = 5.20 us, good for ~47 kOhm on a conservative settling model and ~66 kOhm
 * per the F103 datasheet R_AIN table scaled from 14 MHz. 41.5 cycles would leave
 * under 10 % margin. Was LL_ADC_SAMPLING_CYCLE(1) = 1.5 cycles = ~0.8 kOhm,
 * roughly 40x too short for this divider.
 * This is the value that actually reaches the hardware: RCM_RegisterRegConv()
 * reprograms the channel's SMPR field from the RegConv_t handle, overriding
 * whatever MX_ADC1_Init() wrote. */
#define  M1_VBUS_SAMPLING_TIME  LL_ADC_SAMPLING_CYCLE(55)
/******************************   Temperature sensing Motor 1  **********************/
/* 55.5 ADC cycles, matching M1_VBUS_SAMPLING_TIME. Channel 0 (PA0,
 * M1_TEMPERATURE) did not change impedance, so this is for consistency rather
 * than necessity. Keep it equal to CurrentSensorParams.CurrRegConv.samplingTime
 * in mc_config.c: both handles sit on channel 0 and share one SMPR field, and
 * RCM_ExecRegularConv() does not reprogram sampling time, so if both ever
 * register the last one to register wins. Today CURR_SENSOR_TYPE is
 * VIRTUAL_SENSOR for M365, so CurrRegConv is never registered and only this
 * value is ever written. */
#define  M1_TEMP_SAMPLING_TIME  LL_ADC_SAMPLING_CYCLE(55)
/******************************   Current sensing Motor 1   **********************/
#define ADC_SAMPLING_CYCLES (1 + SAMPLING_CYCLE_CORRECTION)

/******************************   FEED FORWARD Motor 1   *********************/
/* Feed forward compensates back-EMF and cancels d-q cross-coupling, both of
 * which grow with speed. Set to 0 to build a control binary with the feature
 * out entirely - no handle, no calls, and libmc-gcc_M3.lib's feed_forward_ctrl.o
 * is then not extracted at link time. */
/* Disabled 2026-09-06 after a runaway on throttle release; re-enabled once the
 * three structural faults behind it were fixed - see "Feed forward" in
 * docs/CHANGES.md:
 *   (a) FF_DataProcess() is no longer called, so Vqdff no longer carries a
 *       low-passed copy of the PI output and the current-loop gain is not
 *       doubled;
 *   (b) the PWM-off gate in VescToSTM.c now tests AvVolt_qd_pi, the PI output
 *       with the feed-forward contribution removed, so the bridge freewheels on
 *       release instead of holding at back-EMF;
 *   (c) the min-duty deadband moved after the feed-forward add, so it no longer
 *       deletes the PI's correction when feed forward carries the voltage.
 * All three were confirmed by disassembling feed_forward_ctrl.o.
 *
 * UNVALIDATED ON HARDWARE. Bench first, wheel off the ground, current limits
 * low. */
#define FEED_FORWARD_ENABLED             1

/* Scale applied to both feed-forward constants. 1.0 reproduces ST MC Workbench
 * behaviour exactly and is the default.
 *
 * It exists because the s16 voltage domain is genuinely disputed. Four sources
 * disagree about what 32767 output digits are worth, spanning a factor of two:
 *
 *   Vbus/2      ST's own Workbench constants. Fitted from five reference
 *               projects to 0.1%, and independently reproduced exactly
 *               (CONSTANT1_D 733361) from a sixth with feed forward enabled.
 *   Vbus        VescToSTM_get_Vq() in this fork: Vin/32768 * AvVolt_qd.q, and
 *               FW_DataProcess() is a plain low-pass so the domain is the same.
 *   Vbus/sqrt3  SVPWM textbook linear limit, argued from SQRT3FACTOR and
 *               MAX_MODULE = 0.95*32767 (that ratio is not our MAX_MODULE,
 *               which is 32767).
 *   Vbus*sqrt3/2  a pure-alpha reading of PWMC_SetPhaseVoltage().
 *
 * If the modulator is really Vbus/sqrt(3), ST's constants over-inject by
 * 2/sqrt(3) and the correction is 0.8660254f. That is a claim about a shipping
 * ST product and is not established, so it is NOT applied by default - but it
 * is one line to try on the bench. Feed forward is added to the current-PI
 * output and the loop stays closed around the total, so a systematic
 * over-estimate is absorbed in steady state; expect the difference in transient
 * overshoot and in how early the circle limiter saturates, not in steady
 * current. See docs/CHANGES.md. */
#define FF_VOLTAGE_SCALE                 1.0f

/* The Vqd low-pass filter feeding FF_DataProcess() is M1_VQD_SW_FILTER_BW_FACTOR
 * / _LOG, already defined in parameters_conversion.h (128, log2 = 7) and shared
 * with the flux-weakening handle - not redefined here. */

/******************************   ADDITIONAL FEATURES   **********************/
#define FW_VOLTAGE_REF                985 /*!<Vs reference, tenth
                                                        of a percent */
#define FW_KP_GAIN                    2000 /*!< Default Kp gain */
#define FW_KI_GAIN                    5000 /*!< Default Ki gain */
#define FW_KPDIV                      32768
                                                /*!< Kp gain divisor.If FULL_MISRA_C_COMPLIANCY
                                                is not defined the divisor is implemented through
                                                algebrical right shifts to speed up PIs execution.
                                                Only in this case this parameter specifies the
                                                number of right shifts to be executed */
#define FW_KIDIV                      32768
                                                /*!< Ki gain divisor.If FULL_MISRA_C_COMPLIANCY
                                                is not defined the divisor is implemented through
                                                algebrical right shifts to speed up PIs execution.
                                                Only in this case this parameter specifies the
                                                number of right shifts to be executed */
#define FW_KPDIV_LOG                  LOG2(32768)
#define FW_KIDIV_LOG                  LOG2(32768)


/*  Maximum Torque Per Ampere strategy parameters */

#define MTPA_ENABLING
#define SEGDIV                         595
#define ANGC                           {0,0,0,0,-55,0,-54,-54}
#define OFST                           {0,0,0,0,4,-1,4,4}

/*** On the fly start-up ***/

/**************************
 *** Control Parameters ***
 **************************/

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

#endif /*__DRIVE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
