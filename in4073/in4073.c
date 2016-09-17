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
#include "log.h"


void qc_rx_complete(message_t*);
void qc_tx_byte(uint8_t);
void send_status();


serialcomm_t sc;
frame_t rx_frame;
int quadcopter_mode = SAFEMODE;


/*------------------------------------------------------------------
 * process_key -- process command keys
 *------------------------------------------------------------------
 */
void process_key(uint8_t c) 
{
    switch (c) 
    {
        case 'q':
            ae[0] += 10;
            break;
        case 'a':
            ae[0] -= 10;
            if (ae[0] < 0) ae[0] = 0;
            break;
        case 'w':
            ae[1] += 10;
            break;
        case 's':
            ae[1] -= 10;
            if (ae[1] < 0) ae[1] = 0;
            break;
        case 'e':
            ae[2] += 10;
            break;
        case 'd':
            ae[2] -= 10;
            if (ae[2] < 0) ae[2] = 0;
            break;
        case 'r':
            ae[3] += 10;
            break;
        case 'f':
            ae[3] -= 10;
            if (ae[3] < 0) ae[3] = 0;
            break;
        case 27:
            demo_done = true;
            break;
        default:
            nrf_gpio_pin_toggle(RED);
    }
}

/*------------------------------------------------------------------
 * main -- do the test
 *------------------------------------------------------------------
 */
int main(void)
{

	 struct  LOG_FORMAT logitem;

    uart_init();
    gpio_init();
    timers_init();
    adc_init();
    twi_init();
    imu_init(true, 100);    
    baro_init();
    spi_flash_init();
//  ble_init();
	 log_init();

    // Serial buffer init
    serialcomm_init(&sc);
    sc.rx_frame = &rx_frame;
    sc.rx_complete_callback = &qc_rx_complete;
    sc.tx_byte = &qc_tx_byte;
    sc.status = SERIALCOMM_STATUS_OK;

    uint32_t counter = 0;
    demo_done = false;

 // Init message structure for sending data
   frame_t status_frame;
   message_t* message = &status_frame.message;
   sc.tx_frame = &status_frame;

    while (!demo_done)
    {   
        // TODO set mode
        while (rx_queue.count)
            serialcomm_receive_char(&sc, dequeue(&rx_queue));

        if (check_timer_flag()) 
        {
            if (counter++%20 == 0) nrf_gpio_pin_toggle(BLUE);

            adc_request_sample();
            read_baro();
				//send_status();

				if(quadcopter_mode == 2) {
						logitem.time = 12;
						logitem.bat_volt = 34;
						logitem.mode = 56;
						log_write(&logitem);
						logitem.time = 11;
						logitem.bat_volt = 22;
						logitem.mode = 33;
						log_write(&logitem);
						quadcopter_mode = 0;
				}
				if(quadcopter_mode == 0 && counter%20 == 0){
				   message->ID                     = MESSAGE_TEMP_PRESSURE_ID;
					MESSAGE_TEMP_VALUE(message)     = 12345;
					MESSAGE_PRESSURE_VALUE(message) = log_getsize();
					serialcomm_send(&sc);
				}
				if(quadcopter_mode == 3){
					for(int i = 0; i < log_getsize(); i++){
						message->ID                     = MESSAGE_TIME_MODE_VOLTAGE_ID;
						log_read(i, &logitem);
						MESSAGE_TIME_VALUE(message)     = logitem.time;
						MESSAGE_MODE_VALUE(message)     = logitem.mode;
						MESSAGE_VOLTAGE_VALUE(message)  = logitem.bat_volt;
						serialcomm_send(&sc);
					}
					quadcopter_mode = 0;
				}


            // printf("%10ld | ", get_time_us());
            // printf("%3d %3d %3d %3d | ",ae[0],ae[1],ae[2],ae[3]);
            // printf("%6d %6d %6d | ", phi, theta, psi);
            // printf("%6d %6d %6d | ", sp, sq, sr);
            // printf("%4d | %4ld | %6ld \n", bat_volt, temperature, pressure);

            clear_timer_flag();
        }

        if (check_sensor_int_flag()) 
        {
            get_dmp_data();
            run_filters_and_control();

            clear_sensor_int_flag();
        }
    }   
    
    printf("\n\t Goodbye \n\n");
    nrf_delay_ms(100);

    NVIC_SystemReset();
}


void send_status() {
   // Init message structure for sending data
   frame_t status_frame;
   message_t* message = &status_frame.message;
   sc.tx_frame = &status_frame;

   serialcomm_send_start(&sc);

   // Send message 0
   message->ID                     = MESSAGE_TIME_MODE_VOLTAGE_ID;
   MESSAGE_TIME_VALUE(message)     = get_time_us();
   MESSAGE_MODE_VALUE(message)     = quadcopter_mode;
   MESSAGE_VOLTAGE_VALUE(message)  = bat_volt;
   serialcomm_send(&sc);

   // Send message 3
   message->ID                     = MESSAGE_AE1234_ID;
   MESSAGE_AE1_VALUE(message)      = ae[0];
   MESSAGE_AE2_VALUE(message)      = ae[1];
   MESSAGE_AE3_VALUE(message)      = ae[2];
   MESSAGE_AE4_VALUE(message)      = ae[3];
   serialcomm_send(&sc);

   // Send message 6
   message->ID                     = MESSAGE_PHI_THETA_PSI_ID;
   MESSAGE_PHI_VALUE(message)      = phi;
   MESSAGE_THETA_VALUE(message)    = theta;
   MESSAGE_PSI_VALUE(message)      = psi;
   serialcomm_send(&sc);

   // Send message 1
   message->ID                     = MESSAGE_SPQR_ID;
   MESSAGE_SP_VALUE(message)       = sp;
   MESSAGE_SQ_VALUE(message)       = sq;
   MESSAGE_SR_VALUE(message)       = sr;
   serialcomm_send(&sc);

   // Send message 4
   message->ID                     = MESSAGE_TEMP_PRESSURE_ID;
   MESSAGE_TEMP_VALUE(message)     = temperature;
   MESSAGE_PRESSURE_VALUE(message) = pressure;
   serialcomm_send(&sc);

   message->ID                     = MESSAGE_TEXT_ID;
   message->value.v8[0]            = 't';
   message->value.v8[1]            = 'x';
   message->value.v8[2]            = 't';
   message->value.v8[3]            = ' ';
   message->value.v8[4]            = 'M';
   message->value.v8[5]            = 'S';
   message->value.v8[6]            = 'G';
   message->value.v8[7]            = 'S';
   serialcomm_send(&sc);
}

;

void qc_rx_complete(message_t* message) {

    switch (message->ID) {
        case MESSAGE_SET_KEYCODE_ID:
            process_key(MESSAGE_SET_KEYCODE_VALUE(message));
            break;
        case MESSAGE_SET_MODE_ID:

				nrf_gpio_pin_set(YELLOW);
				nrf_gpio_pin_set(GREEN);
				nrf_gpio_pin_set(RED);

            quadcopter_mode = MESSAGE_SET_MODE_VALUE(message);
				if(quadcopter_mode == 0)
					nrf_gpio_pin_clear(YELLOW);
				else if(quadcopter_mode == 1)
					nrf_gpio_pin_clear(RED);
				else if(quadcopter_mode == 2){
					nrf_gpio_pin_clear(RED);
					nrf_gpio_pin_clear(GREEN);
				}else if(quadcopter_mode == 3)
					nrf_gpio_pin_clear(GREEN);
            break;
        default:
            break;
    }
}

void qc_tx_byte(uint8_t byte) {
    uart_put(byte);
}
