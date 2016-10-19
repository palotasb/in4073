#include "qc_hal.h"
#include "in4073.h"
#include "nrf51.h"
#include "mode_constants.h"

static void qc_hal_tx_byte(uint8_t byte);
static void qc_hal_get_inputs(qc_state_t* state);
static void qc_hal_set_outputs(qc_state_t* state);
static void qc_hal_enable_motors(bool);

static bool motors_enabled = false;
static bool enable_uart_output = true;

/** =======================================================
 *  qc_hal_init -- Initialise the quadcopter HAL module.
 *  =======================================================
 *  Initialises a HAL module with quadcopter hardware-
 *  specific functions.
 *
 *  Parameters:
 *  - hal: Pointer to the HAL struct.
 *  Author: Boldizsar Palotas
**/
void qc_hal_init(qc_hal_t* hal) {
    hal->tx_byte_fn     = &qc_hal_tx_byte;
    hal->get_inputs_fn  = &qc_hal_get_inputs;
    hal->set_outputs_fn = &qc_hal_set_outputs;
    hal->enable_motors_fn = &qc_hal_enable_motors;
    hal->flash_init_fn  = &spi_flash_init;
    hal->flash_read_fn  = &flash_read_bytes;
    hal->flash_write_fn = &flash_write_bytes;
    hal->flash_erase_fn = &flash_chip_erase;
    hal->imu_init_fn    = &imu_init;
    hal->reset_fn       = &NVIC_SystemReset;
    hal->get_time_us_fn = &get_time_us;
}

/** =======================================================
 *  qc_hal_tx_byte -- Transmit a single byte of data to PC.
 *  =======================================================
 *  Transmits a single byte of data originating from the
 *  serial communication module to the PC.
 *
 *  Parameters:
 *  - byte: The byte to transmit.
 *  Author: Boldizsar Palotas
**/
void qc_hal_tx_byte(uint8_t byte) {
    if (!enable_uart_output)
        return;
    volatile uint32_t to = 1000;
    while (tx_queue.count == QUEUE_SIZE && --to) { }
    uart_put(byte);
}

/** =======================================================
 *  qc_hal_get_inputs -- Update sensor readings in state.
 *  =======================================================
 *  Updates the sensor readings in the state variable.
 *  Uses the original demo code which might need some
 *  refactoring.
 *
 *  Parameters:
 *  - state: The qc_state_t variable to update with the
 *      sensor data.
 *  Author: Boldizsar Palotas
**/
void qc_hal_get_inputs(qc_state_t* state) {
  
	adc_request_sample();
    read_baro();

    state->sensor.temperature   = temperature;
    state->sensor.pressure      = pressure;
    state->sensor.voltage       = bat_volt;
	 if(state->sensor.voltage_avg == -1) {
        state->sensor.voltage_avg = bat_volt;
	 }else{
        state->sensor.voltage_avg -= state->sensor.voltage_avg >> 4;
        state->sensor.voltage_avg += bat_volt >> 4;
    }

    state->sensor.voltage = state->sensor.voltage_avg;


	//convert from int16_t to F16P16

    state->sensor.sax           = sax * ACC_G_SCALE_INV - state->offset.sax;
    state->sensor.say           = -say * ACC_G_SCALE_INV - state->offset.say;
    state->sensor.saz           = -saz * ACC_G_SCALE_INV - state->offset.saz;
    state->sensor.sp            = GYRO_CONV_FROM_NATIVE( sp) - state->offset.sp;
    state->sensor.sq            = GYRO_CONV_FROM_NATIVE(-sq) - state->offset.sq;
    state->sensor.sr            = GYRO_CONV_FROM_NATIVE(-sr) - state->offset.sr; 

}

/** =======================================================
 *  qc_hal_set_outputs -- Sets controlles outputs.
 *  =======================================================
 *  Sets the output motor speed in the controlling timers.

 *  Parameters:
 *  - state: The state variable containing the outputs.
 *  Author: Boldizsar Palotas
**/
void qc_hal_set_outputs(qc_state_t* state) {
    if (motors_enabled) {
        NRF_TIMER1->CC[0] = 1000 + state->motor.ae1;
        NRF_TIMER1->CC[1] = 1000 + state->motor.ae2;
        NRF_TIMER1->CC[2] = 1000 + state->motor.ae3;
        NRF_TIMER1->CC[3] = 1000 + state->motor.ae4;
    } else {
        NRF_TIMER1->CC[0] = 1000;
        NRF_TIMER1->CC[1] = 1000;
        NRF_TIMER1->CC[2] = 1000;
        NRF_TIMER1->CC[3] = 1000;
    }
}

/** =======================================================
 *  qc_hal_enable_motors -- Enable or disable motors.
 *  =======================================================
 *  Enables or disables the motors of the quadcopter as a
 *  safety measure.
 *
 *  Parameters:
 *  - enable: Set to true if motors are allowed to run and
 *      false when they should be off.
 *  Author: Boldizsar Palotas
**/
void qc_hal_enable_motors(bool enable) {
    if (enable && motors_enabled == false) {
      //  NVIC_EnableIRQ(TIMER1_IRQn);
        motors_enabled = true;
    }
    if (!enable && motors_enabled == true) {
       // NVIC_DisableIRQ(TIMER1_IRQn);
        motors_enabled = false;
    }
}
