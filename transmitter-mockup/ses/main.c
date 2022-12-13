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
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "nrf_uart.h"
#include "uart.h"
#include <stdbool.h>
#include <stdint.h>

#define CH1 12
#define PIN_DEBUG 8
#define LED 14
#define PWM_IN_FREQ 400 // Frequency of input signal [Hz]

// Static function definition
static void send_message(uint8 *msg);
static void timer_ch1_event_handler(nrf_timer_event_t event_type, void *p_context);
static void timer_reload_handler(nrf_timer_event_t event_type, void *p_context);
static void ch1_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

typedef struct {
  uint32_t t1, t2;
  uint8_t dc;
} channel_times_t;

uint8_t data_ready = 0b00000000;
float to_dc = PWM_IN_FREQ / 1e6 * 100; // Scaled to be used when the ton is expressed in us
uint8_t tx_msg[30] = {0xC5, 0, 0xC6, '0', 'C', 'A', 'W', 'A','C', 'A', 'W', 'A', 'V', 'E', 0, 0};

static dwt_config_t dwt_config = {
    5,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    10,              /* TX preamble code. Used in TX only. */
    10,              /* RX preamble code. Used in RX only. */
    0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (129 + 8 - 8)    /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

channel_times_t ch1_times;

const nrf_drv_timer_t TIMER_CH1 = NRF_DRV_TIMER_INSTANCE(0);
const nrf_drv_timer_t TIMER_RELOAD = NRF_DRV_TIMER_INSTANCE(1);

/**
 * @brief Function for main application entry.
 */

int main(void) {
 // ----------------- DWM initialization ----------------------------------- //
  /* Setup DW1000 IRQ pin */
  nrf_gpio_cfg_input(DW1000_IRQ, NRF_GPIO_PIN_NOPULL); //irq

  /* Reset DW1000 */
  reset_DW1000();

  /* Set SPI clock to 2MHz */
  port_set_dw1000_slowrate();

  /* Init the DW1000 */
  if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
    //Init of DW1000 Failed
    while (1) {
    };
  }
  port_set_dw1000_fastrate();
  dwt_configure(&dwt_config);

  // -------------------- End DWM configuration ----------------------------- //

  boUART_Init();
  printf("DWM config OK \r\n");

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
      &TIMER_CH1, NRF_TIMER_CC_CHANNEL0, 4294967295, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
  nrf_drv_timer_enable(&TIMER_CH1);

  // GPIOTE Configuration
  nrf_drv_gpiote_init();
  nrf_drv_gpiote_in_config_t ch1_gpiote_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
//  ch1_gpiote_config.pull = NRF_GPIO_PIN_PULLUP;
  nrf_drv_gpiote_in_init(CH1, &ch1_gpiote_config, ch1_handler);

  // PPI Connection
  nrf_ppi_channel_t ppi_ch1;
  nrf_drv_ppi_channel_alloc(&ppi_ch1);
  nrf_drv_ppi_channel_assign(ppi_ch1,
      nrf_drv_gpiote_in_event_addr_get(CH1),
      nrf_drv_timer_task_address_get(&TIMER_CH1, NRF_TIMER_TASK_CAPTURE0)); // configure interrupt on ch1_1 to save cnt value in cc0

  nrf_drv_ppi_channel_enable(ppi_ch1);
  boUART_Init();
  nrf_drv_gpiote_in_event_enable(CH1, true);

  nrf_gpio_cfg_output(PIN_DEBUG);

  printf("Running main loop \r\n");
  while (1) {
    if (data_ready == 0b00000001){
      ch1_times.dc = 100 - (ch1_times.t1 - ch1_times.t2) * to_dc;
      tx_msg[1] = ch1_times.dc;
      data_ready = data_ready & 0b11111110; // Reset the last bit
      send_message(tx_msg); // Send data and wait
      nrf_gpio_pin_toggle(PIN_DEBUG);
//      printf("Ton duration: %d[ms], \tDuty cycle is: %d[%%] \r\n",  (ch1_times.t2-ch1_times.t1)/1000, ch1_times.dc);

    }
  }
}


// Function definition
static void send_message(uint8 *msg) {
  /* Write frame data to DW1000 and prepare transmission. See NOTE 4 below.*/
  dwt_writetxdata(sizeof(msg), msg, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(msg), 0, 0);  /* Zero offset in TX buffer, no ranging. */

  dwt_starttx(DWT_START_TX_IMMEDIATE);

  /* Poll DW1000 until TX frame sent event set. See NOTE 5 below.
         * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
         * function to access it.*/
  while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {
  };
  /* Clear TX frame sent event. */
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
}

static void timer_ch1_event_handler(nrf_timer_event_t event_type, void *p_context) {
  printf("Timer reload Interrupt\r\n");
}

static void timer_reload_handler(nrf_timer_event_t event_type, void *p_context) {
  printf("Timer reload Interrupt\r\n");
}

static void ch1_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  if (action == NRF_GPIOTE_POLARITY_TOGGLE && pin == CH1) {
    if (nrf_gpio_pin_read(CH1)) {
      NRF_TIMER0->TASKS_CAPTURE[0] = 0;
      ch1_times.t1 = NRF_TIMER0->CC[0];
      data_ready = data_ready | 0b00000001;

//      printf("Interrupt Low to High -- %d\r\n", ch1_times.t2);

    } else {
      NRF_TIMER0->TASKS_CAPTURE[0] = 0;
      ch1_times.t2 = NRF_TIMER0->CC[0];
//      printf("Interrupt High to Low -- %d\r\n", ch1_times.t1);
    }
  }
}