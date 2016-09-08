/*------------------------------------------------------------------
 *  gpio.c -- gpio configuration (leds, interrupt pin sense)
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */

#include "in4073.h"

static bool sensor_int_flag = false;

void gpio_init(void)
{
	// dmp interrupt
	NRF_GPIO->PIN_CNF[INT_PIN] = (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos) & !0x2UL;
	NRF_GPIOTE->CONFIG[0] = GPIOTE_CONFIG_MODE_Event |(GPIOTE_CONFIG_POLARITY_HiToLo<<GPIOTE_CONFIG_POLARITY_Pos) | (INT_PIN<<GPIOTE_CONFIG_PSEL_Pos);

	NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Msk;
	NVIC_ClearPendingIRQ(GPIOTE_IRQn);
	NVIC_SetPriority(GPIOTE_IRQn, 3); // either 1 or 3, 3 being low. (sd present)

	//motors
	nrf_gpio_cfg_output(MOTOR_0_PIN);
	nrf_gpio_cfg_output(MOTOR_1_PIN);
	nrf_gpio_cfg_output(MOTOR_2_PIN);
	nrf_gpio_cfg_output(MOTOR_3_PIN);

	// leds
	nrf_gpio_cfg_output(RED);
	nrf_gpio_cfg_output(YELLOW);
	nrf_gpio_cfg_output(GREEN);
	nrf_gpio_cfg_output(BLUE);
	nrf_gpio_pin_set(RED);
	nrf_gpio_pin_set(YELLOW);
	nrf_gpio_pin_set(GREEN);
	nrf_gpio_pin_set(BLUE);
    
}

void GPIOTE_IRQHandler(void)
{
	if(NRF_GPIOTE->EVENTS_IN[0] != 0)
	{
		NRF_GPIOTE->EVENTS_IN[0] = 0;
		sensor_int_flag = true;
        }
}


bool check_sensor_int_flag(void)
{
	return sensor_int_flag;
}

void clear_sensor_int_flag(void)
{
	sensor_int_flag = false;
}
