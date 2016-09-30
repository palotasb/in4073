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

uint32_t led_patterns[] = {0, 0, 0, 0};

// Custom functions
// ----------------
extern void qc_hal_tx_byte(uint8_t);

static void qc_rx_complete(message_t*);
static void init_modes(void);
static void led_display(void);
static void init_all(void);
static void transmit_text(void);

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

        if (text_queue.count) {
            transmit_text();
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
    //spi_flash_init(); <-- initialized in log_init
    //ble_init(); 
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
    mode_3_calibrate_init(&qc_mode_tables[MODE_3_CALIBRATE]);
    mode_4_yaw_init(&qc_mode_tables[MODE_4_YAW]);
    mode_0_safe_init(&qc_mode_tables[MODE_5_FULL_CONTROL]);
}

void transmit_text(void) {
    message_value_t msgv;
    msgv.v32[0] = 0;
    msgv.v32[1] = 0;
    for (int i = 0; i < MESSAGE_VALUE_SIZE && text_queue.count; i++) {
        msgv.v8[i] = dequeue(&text_queue);
    }
    serialcomm_quick_send(&serialcomm, MESSAGE_TEXT_ID, msgv.v32[0], msgv.v32[1]);
}

void qc_rx_complete(message_t* message) {
    qc_command_rx_message(&qc_command, message);
}

void led_display(void) {
    static uint32_t counter = 0;
    static const uint32_t colors[] = {BLUE, GREEN, YELLOW, RED};
    //static uint8_t intensities[] = {0, 0, 0, 0};

    led_patterns[3] = 0;
    switch (qc_system.mode) {
        case MODE_0_SAFE:
            led_patterns[1] = 0xffffffff;
            break;
        case MODE_1_PANIC:
            led_patterns[1] = 0;
            led_patterns[3] = 0xffffffff;
            break;
        case MODE_2_MANUAL:
            led_patterns[1] = 0xfafafafa;
            break;
        case MODE_3_CALIBRATE:
            led_patterns[1] = 0xffeaffea;
            break;
        case MODE_4_YAW:
            led_patterns[1] = 0xffaaffaa;
            break;
        case MODE_5_FULL_CONTROL:
            led_patterns[1] = 0xfeaafeaa;
            break;
        default:
            break;
    }
    led_patterns[0] = 0xFF00FF00;

    counter++;
    for (int i = 0; i < 4; i++) {
        if ((led_patterns[i] & (1ul << ((counter >> 3) & 0x1F))) == 0) {
            nrf_gpio_pin_set(colors[i]);
        } else {
            nrf_gpio_pin_clear(colors[i]);
        }
    }
    //if ((counter & 0xFF) == 0)
    //    printf("> I'm alive!\n");
}

void Default_Handler(void) {
    nrf_gpio_pin_set(BLUE);
    nrf_gpio_pin_set(GREEN);
    nrf_gpio_pin_set(YELLOW);
    while (1) {
        volatile uint32_t to = 1000;
        while (to--) {}
        nrf_gpio_pin_toggle(RED);
    }
}

void NMI_Handler (void) {
    nrf_gpio_pin_clear(BLUE);
    nrf_gpio_pin_set(GREEN);
    nrf_gpio_pin_set(YELLOW);
    while (1) {
        volatile uint32_t to = 1000;
        while (to--) {}
        nrf_gpio_pin_toggle(RED);
    }
}

void HardFault_Handler (void) {
    nrf_gpio_pin_set(BLUE);
    nrf_gpio_pin_clear(GREEN);
    nrf_gpio_pin_set(YELLOW);
    while (1) {
        volatile uint32_t to = 1000;
        while (to--) {}
        nrf_gpio_pin_toggle(RED);
    }
}

void SVC_Handler (void) {
    nrf_gpio_pin_clear(BLUE);
    nrf_gpio_pin_clear(GREEN);
    nrf_gpio_pin_set(YELLOW);
    while (1) {
        volatile uint32_t to = 1000;
        while (to--) {}
        nrf_gpio_pin_toggle(RED);
    }
}

void PendSV_Handler (void) {
    nrf_gpio_pin_set(BLUE);
    nrf_gpio_pin_set(GREEN);
    nrf_gpio_pin_clear(YELLOW);
    while (1) {
        volatile uint32_t to = 1000;
        while (to--) {}
        nrf_gpio_pin_toggle(RED);
    }
}

void SysTick_Handler (void) {
    nrf_gpio_pin_clear(BLUE);
    nrf_gpio_pin_set(GREEN);
    nrf_gpio_pin_clear(YELLOW);
    while (1) {
        volatile uint32_t to = 1000;
        while (to--) {}
        nrf_gpio_pin_toggle(RED);
    }
}

// Start critical section code
// Original code by Boldizsar Palotas for previous university project.
unsigned int CRITICALSECTION_NestingLevel = 0;

// End critical section code
