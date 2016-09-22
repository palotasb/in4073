/*------------------------------------------------------------------
 *  in4073.c -- test QR engines and sensors
 *
 *  reads ae[0-3] uart rx queue
 *  (q,w,e,r increment, a,s,d,f decrement)
 *
 *  prints timestamp, ae[0-3], sensors to uart tx queue
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  June 2016
 *------------------------------------------------------------------
 */

#include "in4073.h"

// Quadcopter flying logic
// =======================

// Variables
// ---------

qc_system_t         qc_system;
qc_mode_table_t     qc_mode_tables[MODE_COUNT];
qc_state_t          qc_state;
qc_command_t        qc_command;
serialcomm_t        serialcomm;
qc_hal_t            qc_hal;

// Custom functions
// ----------------
extern void qc_hal_tx_byte(uint8_t);

static void qc_rx_complete(message_t*);
static void init_modes(void);
static void led_display(void);
static void init_all(void);

int main(void) {
    init_all();

    while (1) {
        if (check_timer_flag()) {
            qc_system_step(&qc_system);
            led_display();
            clear_timer_flag();
        }
 
        if (check_sensor_int_flag()) {
            get_dmp_data();
            clear_sensor_int_flag();
        }

        while (rx_queue.count)
            serialcomm_receive_char(&serialcomm, dequeue(&rx_queue));
    }
}

void init_all(void) {
    // Hardware init
    uart_init();
    gpio_init();
    timers_init();
    adc_init();
    twi_init();
    imu_init(true, 100);    
    baro_init();
    spi_flash_init();
    //ble_init();
qc_mode_tables[0].test = 123; //DEBUG    
    // HAL & software init
    qc_hal_init(&qc_hal);
    init_modes();
    qc_system_init(
        &qc_system,
        MODE_0_SAFE,
        qc_mode_tables,
        &qc_state,
        &qc_command,
        &serialcomm,
        &qc_rx_complete,
        &qc_hal
    );
}

void init_modes(void) {
    mode_0_safe_init(&qc_mode_tables[MODE_0_SAFE]);
    mode_1_panic_init(&qc_mode_tables[MODE_1_PANIC]);
    mode_2_manual_init(&qc_mode_tables[MODE_2_MANUAL]);
    mode_0_safe_init(&qc_mode_tables[MODE_3_CALIBRATE]);
    mode_0_safe_init(&qc_mode_tables[MODE_4_YAW]);
    mode_0_safe_init(&qc_mode_tables[MODE_5_FULL_CONTROL]);
}

void qc_rx_complete(message_t* message) {
    qc_command_rx_message(&qc_command, message);
}

void led_display(void) {
    static uint32_t counter;
    if (counter++ %20 == 0)
        nrf_gpio_pin_toggle(BLUE);
/*
    switch (serialcomm.status) {
        case SERIALCOMM_STATUS_OK:
            nrf_gpio_pin_set(YELLOW);
            nrf_gpio_pin_set(RED);
            nrf_gpio_pin_clear(GREEN);
            break;
        case SERIALCOMM_STATUS_Prestart:
            nrf_gpio_pin_clear(RED);
            nrf_gpio_pin_set(GREEN);
            nrf_gpio_pin_set(YELLOW);
            break;
        case SERIALCOMM_STATUS_Start:
            nrf_gpio_pin_clear(YELLOW);
            nrf_gpio_pin_set(GREEN);
            nrf_gpio_pin_set(RED);
            break;
        default: break;
    }
*/
}
