#include "app_error.h"
#include "app_pwm.h"
#include "bsp.h"
#include "deca_device_api.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_types.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_pwm.h"
#include "nrf_drv_timer.h"
#include "nrf_uart.h"
#include "port_platform.h"
#include "uart.h"
#include <stdbool.h>
#include <stdint.h>

//-----------------dw1000----------------------------

static dwt_config_t config = {
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

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);

// Declare variables holding PWM sequence values. Only one channel is used
nrf_pwm_values_individual_t seq_values[] = {0, 0, 0, 0};
nrf_pwm_sequence_t const seq =
    {
        .values.p_individual = seq_values,
        .length = NRF_PWM_VALUES_LENGTH(seq_values),
        .repeats = 0,
        .end_delay = 0};

#define RECEIVER_ADDR 0xC5
// Define the IO
#define FRAME_LEN_MAX 127
#define CH 12
#define PIN_DEBUG 8

// Static declaration
static void print_msg(uint8 *msg, int n);

// Global variables
static volatile bool ready_flag;
int new_data = 0;
static uint8 rx_buffer[FRAME_LEN_MAX];
/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;
float PWM_IN_FREQ = 400;

/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
static uint16 frame_len = 0;

//--------------dw1000---end---------------

int main(void) {
  /* Setup some LEDs for debug Green and Blue on DWM1001-DEV */
  //  LEDS_CONFIGURE(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK);
  //  LEDS_ON(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK );

  //-------------dw1000  ini------------------------------------

  /* Setup DW1000 IRQ pin */
  nrf_gpio_cfg_input(DW1000_IRQ, NRF_GPIO_PIN_NOPULL); //irq

  /*Initialization UART*/
  boUART_Init();
  printf("Simple receiver \r\n");

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

  // Set SPI to 8MHz clock
  port_set_dw1000_fastrate();

  /* Configure DW1000. */
  dwt_configure(&config);

  //-------------dw1000  ini------end---------------------------
  // IF WE GET HERE THEN THE LEDS WILL BLINK

  nrf_gpio_cfg_output(CH);
  nrf_gpio_cfg_output(PIN_DEBUG);
  uint32_t T = 1 / PWM_IN_FREQ * 1e6; // Scaled to be used when the ton is expressed in us

  // Set up the PWM peripheral and sequence
  nrf_drv_pwm_config_t config = NRF_DRV_PWM_DEFAULT_CONFIG;
  config.output_pins[0] = CH;
  config.base_clock = NRF_PWM_CLK_1MHz;
  config.count_mode = NRF_PWM_MODE_UP,
  config.load_mode = NRF_PWM_LOAD_INDIVIDUAL,
  config.step_mode = NRF_PWM_STEP_AUTO,
  config.top_value = T;
  nrf_drv_pwm_init(&m_pwm0, &config, NULL);

  seq_values->channel_0 = T / 2;
  nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);

  // Wait indefinitely
  // Start clock for accurate frequencies
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  // Wait for clock to start
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)

    while (1) {

      for (int i = 0; i < FRAME_LEN_MAX; i++) {
        rx_buffer[i] = 0;
      }

      /* Activate reception immediately. */
      dwt_rxenable(DWT_START_RX_IMMEDIATE);

      /* Poll until a frame is properly received or an error/timeout occurs. */
      while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {
      };

      if (status_reg & SYS_STATUS_RXFCG) {
        /* A frame has been received, copy it to our local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
        if (frame_len <= FRAME_LEN_MAX) {
          dwt_readrxdata(rx_buffer, frame_len, 0);
          new_data = 1;
        }

        /* Clear good RX frame event in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
      } else {
        /* Clear RX error events in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
      }

      if (new_data == 1) {
        print_msg(&rx_buffer, frame_len);
        for (int i = 0; i < FRAME_LEN_MAX; i++)
        {
          if (rx_buffer[i] == RECEIVER_ADDR)
          {
            seq_values->channel_0 = (100 - rx_buffer[i+1]) * T / 100;
            nrf_gpio_pin_toggle(PIN_DEBUG);
            break;
          }
          
        }
      }

      new_data = 0;
    }
}

void pwm_ready_callback(uint32_t pwm_id) // PWM callback function
{
  ready_flag = true;
}

static void print_msg(uint8 *msg, int n) {
  printf("Message:\t[");
  for (int i = 0; i < n; i++) {
    printf(" %d ", msg[i]);
  }
  printf("]\r\n");
}