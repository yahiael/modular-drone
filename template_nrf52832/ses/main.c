/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 * @defgroup nrf_dev_timer_example_main main.c
 * @{
 * @ingroup nrf_dev_timer_example
 * @brief Timer Example Application main file.
 *
 * This file contains the source code for a sample application using Timer0.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_ppi.h"
#include "nrf_uart.h"
#include "uart.h"
#include "bsp.h"
#include "app_error.h"

#define CH1 2
#define LED 14

typedef struct{
  uint32_t t1,t2;
  } channel_times_t;

channel_times_t ch1_times;

const nrf_drv_timer_t TIMER_CH1 = NRF_DRV_TIMER_INSTANCE(0);
const nrf_drv_timer_t TIMER_RELOAD = NRF_DRV_TIMER_INSTANCE(1);

/**
 * @brief Handler for timer events.
 */
void timer_ch1_event_handler(nrf_timer_event_t event_type, void* p_context)
{
  printf("Timer reload Interrupt\r\n");
}

static void timer_reload_handler(nrf_timer_event_t event_type, void * p_context)
{
    printf("Timer reload Interrupt\r\n");
}

void ch1_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action){
  if (action == NRF_GPIOTE_POLARITY_TOGGLE && pin == CH1){
    if (nrf_gpio_pin_read(CH1)){
      NRF_TIMER0->TASKS_CAPTURE[0] = 0;
      ch1_times.t2 = NRF_TIMER0->CC[0];
      printf("Interrupt Low to High -- %d\r\n", ch1_times.t2);
       printf("Ton duration is: %d[ms] \r\n", (ch1_times.t2 - ch1_times.t1)/1000);

    }
    else{
      NRF_TIMER0->TASKS_CAPTURE[0] = 0;
      ch1_times.t1 = NRF_TIMER0->CC[0];
      printf("Interrupt High to Low -- %d\r\n", ch1_times.t1);
    }
    
  }
}


/**
 * @brief Function for main application entry.
 */

int main(void)
{
    uint32_t err_code = NRF_SUCCESS;

    //Configure all leds on board.
    bsp_board_leds_init();

// TIMER Configuration
// You have to check that the timer reload does not occur during a measurement
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.frequency = NRF_TIMER_FREQ_1MHz;
    err_code = nrf_drv_timer_init(&TIMER_CH1, &timer_cfg, timer_ch1_event_handler);
    APP_ERROR_CHECK(err_code);

    uint32_t time_reload_ms = 1000;
    uint32_t time_reload_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_CH1, time_reload_ms);

    nrf_drv_timer_extended_compare(
         &TIMER_CH1, NRF_TIMER_CC_CHANNEL0, 4294967295 , NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    nrf_drv_timer_enable(&TIMER_CH1);


// GPIOTE Configuration
    nrf_drv_gpiote_init();
    nrf_drv_gpiote_in_config_t ch1_gpiote_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    ch1_gpiote_config.pull = NRF_GPIO_PIN_PULLUP;
    nrf_drv_gpiote_in_init(CH1, &ch1_gpiote_config, ch1_handler);


// PPI Connection
    nrf_ppi_channel_t ppi_ch1;
    nrf_drv_ppi_channel_alloc(&ppi_ch1);
    nrf_drv_ppi_channel_assign(ppi_ch1, 
                               nrf_drv_gpiote_in_event_addr_get(CH1), 
                               nrf_drv_timer_task_address_get(&TIMER_CH1, NRF_TIMER_TASK_CAPTURE0)); // configure interrupt on ch1_1 to save cnt value in cc0

    nrf_drv_ppi_channel_enable(ppi_ch1);

    nrf_gpio_cfg_output(LED_3); // Configure LED as output
    boUART_Init();
    nrf_drv_gpiote_in_event_enable(CH1, true);

    printf("Example PPI \r\n");
//    printf("Simple Transmitter \r\n");
 uint8_t cnt_1_value = 0;
 uint8_t cnt_2_value = 0;
    while (1)
    {
        nrf_delay_ms(100);
        
    }
}

/** @} */
