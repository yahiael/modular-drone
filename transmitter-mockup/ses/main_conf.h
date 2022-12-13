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

/* --------------------- Definitions -------------------------- */
#define CH1 12
#define PIN_DEBUG 8

/* --------------------- User parameters -------------------------- */

#define PWM_IN_FREQ 400 // Signal frequency [Hz]


/* --------------------- Global variables -------------------------- */

//// General ////
float to_dc = PWM_IN_FREQ / 1e6 * 100; // Scaled to be used when the ton is expressed in us

//// PWM ////
const nrf_drv_timer_t TIMER_CH1 = NRF_DRV_TIMER_INSTANCE(0);

typedef struct {
  uint32_t t1, t2;
  uint8_t dc;
} channel_times_t; // Struct to store the signal times and duty cycle

channel_times_t ch1_times;


//// Transmission ////
uint8_t tx_msg[30] = {0xC5, 0, 0xC6, '0', 'C', 'A', 'W', 'A','C', 'A', 'W', 'A', 'V', 'E', 0, 0}; // Buffer to send
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
uint8_t data_ready = 0b00000000; // Mask to indicate when data has to be send



/* --------------------- Functions -------------------------- */
static void send_message(uint8 *msg);
static void timer_ch1_event_handler(nrf_timer_event_t event_type, void *p_context);
static void timer_reload_handler(nrf_timer_event_t event_type, void *p_context);
static void ch1_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);