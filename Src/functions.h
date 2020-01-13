/*
 * functions.h
 *
 *  Created on: 19 Feb 2019
 *      Author: 20795629
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include "stm32f3xx_hal.h"
#include "stm32f3xx_it.h"
#include "stdlib.h"
#include "stdio.h"
#include "main.h"
#include "bme280.h"
#include "lis2dh12_reg.h"

/*Global Flags*/
extern volatile uint8_t count_flag, message_flag, burn_flag, adc_flag;
extern volatile float voltage_readings[20], current_readings[20];
extern volatile char in_string[92];
extern volatile uint8_t runtime, in_len, syscounter, cnt_readings;
extern volatile int curr_counter, prev_counter;

/*LCD Global Variables*/
extern char lcd_state;
extern uint8_t lcd_count_down, lcd_print_flag;
extern float prev_lcd_time, curr_lcd_time;
extern char *lcd_curr_ptr;

/*Global Variables*/
extern uint8_t cur,
			hrs, mins, secs,
			burn_down_cnt, burn_up_cnt;
extern int acc1, acc2, acc3;
extern int8_t calc_checksum, in_checksum;
extern float lat, lon, alt, vol, prev_alt, max_runtime;
extern char out_string[91], lcd_string[17], alt_string[7], tmp_string[7];
extern uint8_t data_bits[6];
extern int32_t tmp, prev_tmp;
extern uint32_t hum, prs;


/*Temporary Variables*/
extern float lon_in, lat_in, in_time, adc_voltage, adc_current, sum_voltage, sum_current;
extern uint8_t lcd_pos, num_spaces1, num_spaces2;
extern int temp_alt;
extern char gps_data[50], str_data[89], str_checksum[2];
extern char *curr_ptr, *delim_ptr, *checksum_delim_ptr, *checksum_curr_ptr;

/*BME280 Variables*/
extern struct bme280_data comp_data;
extern struct bme280_dev dev;
extern int8_t rslt;

/*Accel Variables*/
extern lis2dh12_ctx_t dev_ctx;
extern axis3bit16_t data_raw_acceleration;

/*************************************UART Functions******************************************/
extern void uart_transmit(UART_HandleTypeDef huart1);
extern void uart_receive();
extern void check_burn();
extern void burn_down();
extern void checksum_generator(const char *input);

/*************************************ADC Functions******************************************/
extern void read_adc(ADC_HandleTypeDef hadc1, ADC_HandleTypeDef hadc2);
extern void calc_avg();

/*************************************LCD Functions******************************************/
extern void init_lcd();
extern void lcd_write();
extern void lcd_run_process();
extern void lcd_print_char(char c);
extern void increment_address();
extern void lcd_clear();
extern void lcd_process_state();

/*************************************BME280 Functions******************************************/
extern void init_temp_sensor();
extern void receive_temp_data();
extern int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
extern int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);
extern void user_delay_ms(uint32_t period);

/*************************************Accel Functions******************************************/
extern void init_accel_sensor();
extern void receive_accel_data();
extern int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
extern int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

#endif /* FUNCTIONS_H_ */
