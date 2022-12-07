/*
  Set a 1-channel PWM with constant period in a GPIO using the PWM library
*/

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_error.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "app_pwm.h"

#define GPIO_8 8
APP_PWM_INSTANCE(PWM1,1);                   // Create the instance "PWM1" using TIMER1.

static volatile bool ready_flag;            // A flag indicating PWM status.
static volatile bool calibration = false;
static volatile bool loop = false;

void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    ready_flag = true;
}

int main(void)
{
    ret_code_t err_code;

    // 1-channel PWM, period of 2000 us (L is a literal and stands for long int)
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(2500L, GPIO_8);
    // Switch the polarity of the channel
    pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;

    /* Initialize and enable PWM. */
    err_code = app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback);
    APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM1);

    // value contains the percentage of duty cycle (0 - 100)
    uint8_t value;
    
    if (calibration){
      value = 99;
      nrf_delay_ms(1500);
      while (app_pwm_channel_duty_set(&PWM1, 0, value) == NRF_ERROR_BUSY);
      nrf_delay_ms(1000);
      value = 1;
      while (app_pwm_channel_duty_set(&PWM1, 0, value) == NRF_ERROR_BUSY);
      nrf_delay_ms(1000);
    }
 
    while (true)
    {
    if (loop){
      for (uint8_t i = 0; i < 40; ++i)
          {
              value = (i < 20) ? (i * 5) : (100 - (i - 20) * 5);

              ready_flag = false;

              // Set the duty cycle - keep trying until PWM is ready...
              while (app_pwm_channel_duty_set(&PWM1, 0, value) == NRF_ERROR_BUSY);
              nrf_delay_ms(25);
          }
      }
      else{

        // Set the duty cycle - keep trying until PWM is ready...
        value = 50;
        while (app_pwm_channel_duty_set(&PWM1, 0, value) == NRF_ERROR_BUSY);
      }     
    }
}

