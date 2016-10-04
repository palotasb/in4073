/*------------------------------------------------------------------
 *  in4073.h -- defines, globals, function prototypes
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */

#ifndef IN4073_H__
#define IN4073_H__

#define QUADCOPTER  1

#include <inttypes.h>
#include <stdio.h>
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ml.h"
#include "qc_system.h"
#include "mode_0_safe.h"
#include "mode_1_panic.h"
#include "mode_2_manual.h"
#include "mode_3_calibrate.h"
#include "mode_4_yaw.h"
#include "mode_5_full.h"
#include "profile.h"

// Start critical section code
// Original code by Boldizsar Palotas for previous university project.
static inline void CRITICALSECTION_FastEnter(void);
static inline void CRITICALSECTION_FastExit(void);
extern unsigned int CRITICALSECTION_NestingLevel;

inline static void CRITICALSECTION_FastEnter(void)
{
    __disable_irq();
    CRITICALSECTION_NestingLevel++;
}

inline static void CRITICALSECTION_FastExit(void)
{
    if(CRITICALSECTION_NestingLevel)
    {
        CRITICALSECTION_NestingLevel--;
        if(!CRITICALSECTION_NestingLevel)
        {
            __enable_irq();
        }
    }
}
// End critical section code

#define RED				22
#define YELLOW				24
#define GREEN				28
#define BLUE				30
#define INT_PIN				5
extern uint32_t led_patterns[4];

#define MOTOR_0_PIN			21
#define MOTOR_1_PIN			23
#define MOTOR_2_PIN			25
#define MOTOR_3_PIN			29

bool demo_done;
extern uint32_t iteration;
extern uint32_t control_iteration;
extern bool is_test_device;

#define TESTDEVICE_ID0 0x9d249f83
#define TESTDEVICE_ID1 0xa4af3109

// Control
int16_t ae[4];
void run_filters_and_control();

// Timers
// Originally -- 50000us=50ms=20Hz (MAX 16bit, 65ms)
// Currently -- 10000us=10ms=100Hz
#define TIMER_PERIOD	10000 
void timers_init(void);
uint32_t get_time_us(void);
bool check_timer_flag(void);
void clear_timer_flag(void);
void set_timer_handler(void (*handler)(void));

// GPIO
void gpio_init(void);
bool check_sensor_int_flag(void);
void clear_sensor_int_flag(void);

// Queue
// Must be power of 2
#define QUEUE_SIZE 256
typedef struct {
	uint8_t Data[QUEUE_SIZE];
	uint32_t first,last;
  	uint32_t count; 
} queue;
void init_queue(queue *q);
void enqueue(queue *q, char x);
char dequeue(queue *q);

// UART
#define RX_PIN_NUMBER  16
#define TX_PIN_NUMBER  14
queue rx_queue;
queue tx_queue;
queue text_queue;
void uart_init(void);
void uart_put(uint8_t);

// TWI
#define TWI_SCL	4
#define TWI_SDA	2
void twi_init(void);
bool i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t const *data);
bool i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);


// MPU wrapper
int16_t phi, theta, psi;
int16_t sp, sq, sr;
int16_t sax, say, saz;
uint8_t sensor_fifo_count;
void imu_init(bool dmp, uint16_t interrupt_frequency); // if dmp is true, the interrupt frequency is 100Hz - otherwise 32Hz-8kHz
void get_dmp_data(void);
void get_raw_sensor_data(void);

// Barometer
int32_t pressure;
int32_t temperature;
void read_baro(void);
void baro_init(void);

// ADC
uint16_t bat_volt;
void adc_init(void);
void adc_request_sample(void);

// Flash
bool spi_flash_init(void);
bool flash_chip_erase(void);
bool flash_write_byte(uint32_t address, uint8_t data);
bool flash_write_bytes(uint32_t address, uint8_t *data, uint32_t count);
bool flash_read_byte(uint32_t address, uint8_t *buffer);
bool flash_read_bytes(uint32_t address, uint8_t *buffer, uint32_t count);

// BLE
queue ble_rx_queue;
queue ble_tx_queue;
void ble_init(void);
void ble_send(void);

#endif // IN4073_H__
