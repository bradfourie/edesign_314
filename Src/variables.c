/*
 * variables.c
 *
 *  Created on: 20 Feb 2019
 *      Author: 20795629
 */

#include "functions.h"

//global variables used in interrupts
volatile uint8_t runtime, in_len, count_flag, message_flag, burn_flag, adc_flag;
volatile char in_string[92] = "";
volatile uint8_t syscounter, cnt_readings;
volatile int curr_counter, prev_counter;
volatile float voltage_readings[20], current_readings[20];

/*LCD Global Variables*/
char lcd_state = 'A';
uint8_t lcd_count_down = 0, lcd_print_flag = 0;
float prev_lcd_time = 0.0, curr_lcd_time = 0.0;
char *lcd_curr_ptr = 0;


//global variables
uint8_t cur = 0,
		hrs = 0, mins = 0, secs = 0,
		burn_down_cnt = 11, burn_up_cnt = 0;
int acc1 = 0, acc2 = 0, acc3 = 0;
int8_t calc_checksum = 0, in_checksum = 0;
float lat = 0.0, lon = 0.0, alt = 0.0, vol = 0.0,  prev_alt = 0.0, max_runtime = 0;
char out_string[91] = "", lcd_string[17] = "", alt_string[7] = "", tmp_string[7] = "";
uint8_t data_bits[6] = {0,0,0,0,0,0};
int32_t tmp =0, prev_tmp =0;
uint32_t hum = 0, prs = 0;

struct bme280_data comp_data;
struct bme280_dev dev;
int8_t rslt;

lis2dh12_ctx_t dev_ctx;
axis3bit16_t data_raw_acceleration;



//temporary variables
float lon_in, lat_in, in_time, adc_voltage, adc_current, sum_voltage, sum_current;
uint8_t lcd_pos, num_spaces1, num_spaces2;
int temp_alt, temp_tmp;
char gps_data[50], str_data[89], str_checksum[2];
char *curr_ptr, *delim_ptr, *checksum_delim_ptr, *checksum_curr_ptr;
