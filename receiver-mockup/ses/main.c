/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "app_error.h"
#include "bsp.h"
#include "deca_device_api.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_types.h"
#include "port_platform.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_pwm.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "app_pwm.h"
#include "nrf_uart.h"
#include "uart.h"
#include <stdbool.h>
#include <stdint.h>

#define OUTPUT_PIN 12

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
 
// Declare variables holding PWM sequence values. In this example only one channel is used  
nrf_pwm_values_individual_t seq_values[] = {0, 0, 0, 0}; 
nrf_pwm_sequence_t const seq = 
{ 
    .values.p_individual = seq_values, 
    .length          = NRF_PWM_VALUES_LENGTH(seq_values), 
    .repeats         = 0, 
    .end_delay       = 0 
}; 
 
int main(void) 
{ 
    nrf_gpio_cfg_output(OUTPUT_PIN); 
    // Set up the PWM peripheral and sequence 
    nrf_drv_pwm_config_t config = NRF_DRV_PWM_DEFAULT_CONFIG; 
    config.output_pins[0] = OUTPUT_PIN; 
    config.base_clock = NRF_PWM_CLK_1MHz; 
    config.count_mode = NRF_PWM_MODE_UP,
    config.load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
    config.step_mode    = NRF_PWM_STEP_AUTO,
    config.top_value = 2500; 
    nrf_drv_pwm_init(&m_pwm0, &config, NULL); 
     
    seq_values->channel_0 = 1250; 
    nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP); 
 
    // Wait indefinitely 
    // Start clock for accurate frequencies
    NRF_CLOCK->TASKS_HFCLKSTART = 1; 
    // Wait for clock to start
    while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
    while (true) 
    { 
        __WFE(); 
    } 
}