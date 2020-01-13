/*
 * functions.c
 *
 *  Created on: 19 Feb 2019
 *      Author: 20795629
 */

#include "functions.h"
#include "bme280.h"
#include "stdlib.h"
#include "string.h"
#include "lis2dh12_reg.h"

extern I2C_HandleTypeDef hi2c1;

/************************************************* Log Message ************************************************/
void uart_transmit(UART_HandleTypeDef huart1)
{
	count_flag = 0;
	runtime++;
	snprintf(out_string, 92, "$20795629,%5d,%2d:%2d:%2d,%3d,%3d,%3d,%4d,%4d,%4d,%10.6f,%11.6f,%7.1f,%3d,%3.1f\n",
			runtime, hrs, mins, secs,
			tmp, hum, prs,
			acc1, acc2, acc3,
			lat, lon, alt,
			cur, vol);
	HAL_UART_Transmit(&huart1, (uint8_t*)out_string, 91, 0xFFFF);
}

/************************************************* NMEA Processing ************************************************/
void uart_receive()
{
	message_flag = 0;
	lon_in = 0, lat_in = 0, in_time = 0;
	calc_checksum = 0, in_checksum = 0;
	delim_ptr = 0, checksum_delim_ptr = 0;

	if(strncmp((char*) in_string, "$GPGGA", 6) == 0 && (in_len < 92))
	{
		curr_ptr = in_string;
		delim_ptr = strchr(curr_ptr, ',');

		checksum_curr_ptr = in_string + 1;
		checksum_delim_ptr = strchr(checksum_curr_ptr, '*');

		if( (checksum_delim_ptr != 0) && (delim_ptr != 0) && ( *(checksum_delim_ptr+3)=='\r' ) )
		{
			memcpy(str_data, checksum_curr_ptr, checksum_delim_ptr - checksum_curr_ptr);
			str_data[checksum_delim_ptr - checksum_curr_ptr] = '\0';

			checksum_curr_ptr = checksum_delim_ptr + 1;
			checksum_delim_ptr = strchr(checksum_curr_ptr, '\0');

			memcpy(str_checksum, checksum_curr_ptr, checksum_delim_ptr - checksum_curr_ptr);
			str_checksum[checksum_delim_ptr - checksum_curr_ptr] = '\0';

			in_checksum = (int)strtol(str_checksum, NULL, 16);
			checksum_generator(str_data);

			if(in_checksum == calc_checksum)
			{
				for(int gps_cnt = 0; gps_cnt < 11; gps_cnt++)
				{
					if((curr_ptr != delim_ptr) && (gps_cnt == 1))
					{
						memcpy(gps_data, curr_ptr, delim_ptr - curr_ptr);
						gps_data[delim_ptr - curr_ptr] = '\0';
						in_time = atoi(gps_data);

						hrs = ((int) in_time) / 10000;
						mins = (((int) in_time) - (hrs * 10000))/ 100;
						secs = (((int) in_time) - (hrs * 10000)  - (mins * 100));
					}
					if((curr_ptr != delim_ptr) && (gps_cnt == 2))
					{
						memcpy(gps_data, curr_ptr, delim_ptr - curr_ptr);
						gps_data[delim_ptr - curr_ptr] = '\0';
						lat_in = atof(gps_data);

						lat = ((int)lat_in/100) + (lat_in - ((int)lat_in/100)*100)/60;
					}
					if((curr_ptr != delim_ptr) && (gps_cnt == 3))
					{
						memcpy(gps_data, curr_ptr, delim_ptr - curr_ptr);
						gps_data[delim_ptr - curr_ptr] = '\0';

						if(gps_data[0] == 'S')
						{
							lat = lat*-1;
						}
					}
					if((curr_ptr != delim_ptr) && (gps_cnt == 4))
					{
						memcpy(gps_data, curr_ptr, delim_ptr - curr_ptr);
						gps_data[delim_ptr - curr_ptr] = '\0';
						lon_in = atof(gps_data);

						lon = ((int)lon_in/100) + (lon_in - ((int)lon_in/100)*100)/60;
					}
					if((curr_ptr != delim_ptr) && (gps_cnt == 5))
					{
						memcpy(gps_data, curr_ptr, delim_ptr - curr_ptr);
						gps_data[delim_ptr - curr_ptr] = '\0';

						if(gps_data[0] == 'W')
						{
							lon = lon*-1;
						}
					}
					if((curr_ptr != delim_ptr) && (gps_cnt == 9))
					{
						memcpy(gps_data, curr_ptr, delim_ptr - curr_ptr);
						gps_data[delim_ptr - curr_ptr] = '\0';

						alt = atof(gps_data);
					}
					if((curr_ptr != delim_ptr) && (gps_cnt == 10))
					{
						memcpy(gps_data, curr_ptr, delim_ptr - curr_ptr);
						gps_data[delim_ptr - curr_ptr] = '\0';

						if(gps_data[0] == 'F')
						{
							alt = alt*3.28084;
						}
					}

					curr_ptr = delim_ptr + 1;
					delim_ptr = strchr(curr_ptr, ',');
				}

				if(burn_down_cnt == 11)
				{
					check_burn();
				}
				if(prev_alt != alt)
				{
					//lcd_parse_output();
					lcd_print_flag = 1;
					prev_alt = alt;
				}
			}
		}
	}

	memset(in_string, 0, sizeof in_string);
	in_len = 0;
}

/************************************************* Burn Signal ************************************************/
void check_burn()
{
	if((alt > 10000) && ((lon < 17.976343 ) || (lon > 18.9354 )) )
	{
		burn_up_cnt++;
	}
	else
	{
		burn_up_cnt = 0;
	}

	if(burn_up_cnt == 5)
	{
		burn_flag = 1;
		//lcd_parse_output();
		lcd_print_flag = 1;
	}
}

void burn_down()
{
	if(burn_down_cnt == 11)
	{
		HAL_GPIO_WritePin (GPIOA, GPIO_PIN_8, 1);
		HAL_GPIO_WritePin (GPIOA, GPIO_PIN_5, 1);
	}
	burn_down_cnt--;
	if(burn_down_cnt == 0)
	{
		HAL_GPIO_WritePin (GPIOA, GPIO_PIN_8, 0);
		HAL_GPIO_WritePin (GPIOA, GPIO_PIN_5, 0);
		burn_flag = 0;
		//lcd_parse_output();
		lcd_print_flag = 1;
	}
}

void checksum_generator(const char *input)
{
    while(*input)
    {
    	calc_checksum ^= *input++;
    }
}

/************************************************* ADC voltage and current measurement ************************************************/
void read_adc(ADC_HandleTypeDef hadc1, ADC_HandleTypeDef hadc2)
{
	adc_flag = 0;
	adc_voltage = 0;
	adc_current = 0;

	adc_voltage = HAL_ADC_GetValue(&hadc1);
	adc_current = HAL_ADC_GetValue(&hadc2);

	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);

	HAL_ADC_PollForConversion(&hadc1, 1);
	HAL_ADC_PollForConversion(&hadc2, 1);

	voltage_readings[cnt_readings] = (adc_voltage / 4095) * 16.4;

	current_readings[cnt_readings] = (adc_current/ 4095) * 346;
	cnt_readings++;
}

void calc_avg(){
	sum_voltage = 0;
	sum_current = 0;

	for(int i=0; i<cnt_readings; i++)
	{
		sum_voltage += voltage_readings[i];
		sum_current += current_readings[i];
	}

	cur = sum_current/cnt_readings;
	vol = sum_voltage/cnt_readings;
	if(vol > 9.9){
		vol = 9.9;
	}
	cnt_readings = 0;
}

/************************************************* LCD initialisation ************************************************/
void init_lcd()
{
	//send 00 0011 3 times to set to 4 bit mode
	//Step 1: Power on, then delay > 100 ms
	HAL_Delay(100);

	data_bits[0] = 0;
	data_bits[1] = 0;
	data_bits[2] = 0;
	data_bits[3] = 0;
	data_bits[4] = 1;
	data_bits[5] = 1;

	//Step 2: Instruction 00 0011b (3h), then delay > 4.1 ms
	lcd_write();
	HAL_Delay(4.2);

	//Step 3: Instruction 00 0011b (3h), then delay > 100 us
	lcd_write();
	HAL_Delay(0.2);

	//Step 4: Instruction 00 0011b (3h), then delay > 100 us
	lcd_write();
	HAL_Delay(0.2);

	//Step 5: Instruction 00 0010b (2h), then delay > 100 us
	data_bits[0] = 0;
	data_bits[1] = 0;
	data_bits[2] = 0;
	data_bits[3] = 0;
	data_bits[4] = 1;
	data_bits[5] = 0;
	lcd_write();
	HAL_Delay(0.2);

	//Step 6: Instruction 00 0010b (2h), then 00 1000b (8h), then delay > 53 us or check BF
	data_bits[0] = 0;
	data_bits[1] = 0;
	data_bits[2] = 0;
	data_bits[3] = 0;
	data_bits[4] = 1;
	data_bits[5] = 0;
	lcd_write();

	data_bits[0] = 0;
	data_bits[1] = 0;
	data_bits[2] = 1;
	data_bits[3] = 0;
	data_bits[4] = 0;
	data_bits[5] = 0;
	lcd_write();

	HAL_Delay(0.1);

	//Step 7: Instruction 0000b (0h), then 1000b (8h) then delay > 53 us or check BF
	data_bits[0] = 0;
	data_bits[1] = 0;
	data_bits[2] = 0;
	data_bits[3] = 0;
	data_bits[4] = 0;
	data_bits[5] = 0;
	lcd_write();

	data_bits[0] = 0;
	data_bits[1] = 0;
	data_bits[2] = 1;
	data_bits[3] = 0;
	data_bits[4] = 0;
	data_bits[5] = 0;
	lcd_write();

	HAL_Delay(1);

	//Step 8: Instruction 0000b (0h), then 0001b (1h) then delay > 3 ms or check BF
	data_bits[0] = 0;
	data_bits[1] = 0;
	data_bits[2] = 0;
	data_bits[3] = 0;
	data_bits[4] = 0;
	data_bits[5] = 0;
	lcd_write();

	data_bits[0] = 0;
	data_bits[1] = 0;
	data_bits[2] = 0;
	data_bits[3] = 0;
	data_bits[4] = 0;
	data_bits[5] = 1;
	lcd_write();

	HAL_Delay(4);

	//Step 9: Instruction 0000b (0h), then 0110b (6h), then delay > 53 us or check BF
	data_bits[0] = 0;
	data_bits[1] = 0;
	data_bits[2] = 0;
	data_bits[3] = 0;
	data_bits[4] = 0;
	data_bits[5] = 0;
	lcd_write();

	data_bits[0] = 0;
	data_bits[1] = 0;
	data_bits[2] = 0;
	data_bits[3] = 1;
	data_bits[4] = 1;
	data_bits[5] = 0;
	lcd_write();

	HAL_Delay(1);

	//Step 10: Initialization ends
	//yeet

	//Step 11: Instruction 0000b (0h), then 1100b (0Ch), then delay > 53 us or check BF
	data_bits[0] = 0;
	data_bits[1] = 0;
	data_bits[2] = 0;
	data_bits[3] = 0;
	data_bits[4] = 0;
	data_bits[5] = 0;
	lcd_write();

	data_bits[0] = 0;
	data_bits[1] = 0;
	data_bits[2] = 1;
	data_bits[3] = 1;
	data_bits[4] = 0;
	data_bits[5] = 0;
	lcd_write();

	HAL_Delay(10);

	//lcd_parse_output();
	lcd_print_flag = 1;

}

void lcd_run_process()
{
	if(lcd_print_flag == 1)
	{
	  lcd_state = 'A';
	  lcd_print_flag = 0;
	  lcd_count_down = 0;
	  curr_lcd_time = 0;
	  prev_lcd_time = 0;
	}

	curr_lcd_time = HAL_GetTick();


	if( (curr_lcd_time - prev_lcd_time) <= 2 )
	{
		return;
	}

	lcd_process_state();

}

void lcd_process_state()
{
	switch(lcd_state)
	{
	default:
		//do nothing, wait until another function sets state to A
		break;
	case 'A':
		memset(alt_string, 0, sizeof alt_string);
		lcd_pos = 0;
		num_spaces1 = 0;
		num_spaces2 = 0;
		lcd_curr_ptr = 0;
		temp_alt = (int) alt;
		snprintf(alt_string, 10, "%dm", temp_alt);
		snprintf(tmp_string, 6, "%dC", tmp);

		num_spaces1 = 10 - strlen(alt_string) - 1;

		if(burn_flag)
		{
			num_spaces2 = 6 - strlen(tmp_string);
			snprintf(lcd_string, 17, "%s%*sB%*s%s", alt_string, num_spaces1, "",  num_spaces2, "", tmp_string);
		}
		else
		{
			num_spaces2 = 7 - strlen(tmp_string);
			//char the_rest[7] = "    26C";
			//snprintf(lcd_string, 17, "%s%*s%s", alt_string, num_spaces1, "", the_rest);
			snprintf(lcd_string, 17, "%s%*s%*s%s", alt_string, num_spaces1, "",  num_spaces2, "", tmp_string);
		}

		lcd_curr_ptr = lcd_string;

		lcd_clear();
		lcd_state = 'B';

		prev_lcd_time = HAL_GetTick();
		break;
	case 'B':
		//print a digit and wait 2ms
		lcd_print_char(*lcd_curr_ptr);

		lcd_state = 'C';
		if(lcd_pos == 7)
		{
			lcd_state = 'D';
		}

		//lcd_count_down = 2;
		prev_lcd_time = HAL_GetTick();

		break;
	case 'C':
		//increment variables
		lcd_curr_ptr++;
		lcd_pos++;
		lcd_state = 'B';

		if(*lcd_curr_ptr == '\0')
		{
			lcd_state = 0; //set state to invalid condition to trigger default
		}

		break;
	case 'D':
		lcd_state = 'C';

		increment_address();

		//lcd_count_down = 5;
		prev_lcd_time = HAL_GetTick();
		break;
	}
}

void lcd_clear()
{
	data_bits[0] = 0;
	data_bits[1] = 0;
	data_bits[2] = 0;
	data_bits[3] = 0;
	data_bits[4] = 0;
	data_bits[5] = 0;
	lcd_write();

	data_bits[0] = 0;
	data_bits[1] = 0;
	data_bits[2] = 0;
	data_bits[3] = 0;
	data_bits[4] = 0;
	data_bits[5] = 1;
	lcd_write();
}

void increment_address()
{
	data_bits[0] = 0;
	data_bits[1] = 0;
	data_bits[2] = 1;
	data_bits[3] = 1;
	data_bits[4] = 0;
	data_bits[5] = 0;
	lcd_write();

	data_bits[0] = 0;
	data_bits[1] = 0;
	data_bits[2] = 0;
	data_bits[3] = 0;
	data_bits[4] = 0;
	data_bits[5] = 0;
	lcd_write();
}

void lcd_print_char(char c)
{
	//use sprintf and print altitude into string
	//for each character of string print to lcd
	switch(c)
	{
	default:
		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 0;
		data_bits[4] = 1;
		data_bits[5] = 0;
		lcd_write();

		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 0;
		data_bits[4] = 0;
		data_bits[5] = 0;
		lcd_write();
		break;
	case '0':
		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 0;
		data_bits[4] = 1;
		data_bits[5] = 1;
		lcd_write();

		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 0;
		data_bits[4] = 0;
		data_bits[5] = 0;
		lcd_write();
		break;
	case '1':
		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 0;
		data_bits[4] = 1;
		data_bits[5] = 1;
		lcd_write();

		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 0;
		data_bits[4] = 0;
		data_bits[5] = 1;
		lcd_write();
		break;
	case '2':
		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 0;
		data_bits[4] = 1;
		data_bits[5] = 1;
		lcd_write();

		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 0;
		data_bits[4] = 1;
		data_bits[5] = 0;
		lcd_write();
		break;
	case '3':
		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 0;
		data_bits[4] = 1;
		data_bits[5] = 1;
		lcd_write();

		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 0;
		data_bits[4] = 1;
		data_bits[5] = 1;
		lcd_write();
		break;
	case '4':
		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 0;
		data_bits[4] = 1;
		data_bits[5] = 1;
		lcd_write();

		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 1;
		data_bits[4] = 0;
		data_bits[5] = 0;
		lcd_write();
		break;
	case '5':
		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 0;
		data_bits[4] = 1;
		data_bits[5] = 1;
		lcd_write();

		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 1;
		data_bits[4] = 0;
		data_bits[5] = 1;
		lcd_write();
		break;
	case '6':
		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 0;
		data_bits[4] = 1;
		data_bits[5] = 1;
		lcd_write();


		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 1;
		data_bits[4] = 1;
		data_bits[5] = 0;
		lcd_write();
		break;
	case '7':
		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 0;
		data_bits[4] = 1;
		data_bits[5] = 1;
		lcd_write();


		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 1;
		data_bits[4] = 1;
		data_bits[5] = 1;
		lcd_write();
		break;
	case '8':
		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 0;
		data_bits[4] = 1;
		data_bits[5] = 1;
		lcd_write();


		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 1;
		data_bits[3] = 0;
		data_bits[4] = 0;
		data_bits[5] = 0;
		lcd_write();
		break;
	case '9':
		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 0;
		data_bits[4] = 1;
		data_bits[5] = 1;
		lcd_write();


		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 1;
		data_bits[3] = 0;
		data_bits[4] = 0;
		data_bits[5] = 1;
		lcd_write();
		break;
	case 'm':
		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 1;
		data_bits[4] = 1;
		data_bits[5] = 0;
		lcd_write();

		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 1;
		data_bits[3] = 1;
		data_bits[4] = 0;
		data_bits[5] = 1;
		lcd_write();
		break;
	case 'B':
		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 1;
		data_bits[4] = 0;
		data_bits[5] = 0;
		lcd_write();

		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 0;
		data_bits[4] = 1;
		data_bits[5] = 0;
		lcd_write();
		break;
	case 'C':
		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 1;
		data_bits[4] = 0;
		data_bits[5] = 0;
		lcd_write();

		data_bits[0] = 1;
		data_bits[1] = 0;
		data_bits[2] = 0;
		data_bits[3] = 0;
		data_bits[4] = 1;
		data_bits[5] = 1;
		lcd_write();
		break;
	}
}


void lcd_write()
{
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_11, 1); 	// E set high

	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_1, data_bits[0]); 	// RS
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_2, data_bits[1]); 	//RNW
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_15, data_bits[2]); 	//DB7
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, data_bits[3]); 	//DB6
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_13, data_bits[4]); 	//DB5
	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_12, data_bits[5]); 	//DB4

	HAL_GPIO_WritePin (GPIOB, GPIO_PIN_11, 0); 	// E set low
}

void init_temp_sensor()
{
	rslt = BME280_OK;

	dev.dev_id = BME280_I2C_ADDR_PRIM;
	dev.intf = BME280_I2C_INTF;
	dev.read = user_i2c_read;
	dev.write = user_i2c_write;
	dev.delay_ms = user_delay_ms;

	rslt = bme280_init(&dev);
	uint8_t settings_sel;

	(&dev) -> settings.osr_h = BME280_OVERSAMPLING_1X;
	(&dev) -> settings.osr_p = BME280_OVERSAMPLING_16X;
	(&dev) -> settings.osr_t = BME280_OVERSAMPLING_2X;
	(&dev) -> settings.filter = BME280_FILTER_COEFF_16;
	(&dev) -> settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, &dev);
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);
}

void receive_temp_data()
{
	rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);

	tmp = (int32_t)  ((&comp_data) -> temperature) / 100;
	prs = (uint32_t) ((&comp_data) -> pressure) / 1000;
	hum = (uint32_t) ((&comp_data) -> humidity) / 1024;

	if(tmp > 50)
	{
		tmp = 50;
	}
	if(tmp < -40)
	{
		tmp = -40;
	}

	if(prs > 150)
	{
		prs = 150;
	}
	if(prs < 0)
	{
		prs = 0;
	}


	if(prev_tmp != tmp)
	{
		lcd_print_flag = 1;
		prev_tmp = tmp;
	}
}

void user_delay_ms(uint32_t period)
{
    HAL_Delay(period);
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	HAL_I2C_Mem_Read(&hi2c1, (dev_id<<1), reg_addr, 1, reg_data, len, 10000);

	return 0;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	HAL_I2C_Mem_Write(&hi2c1, (dev_id<<1), reg_addr, 1, reg_data, len, 10000);

	return 0;
}

void init_accel_sensor()
{
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = &hi2c1;

	lis2dh12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	lis2dh12_data_rate_set(&dev_ctx, LIS2DH12_ODR_10Hz);
	lis2dh12_full_scale_set(&dev_ctx, LIS2DH12_2g);
	lis2dh12_operating_mode_set(&dev_ctx, LIS2DH12_HR_12bit);
}

void receive_accel_data()
{
	lis2dh12_reg_t reg;
	lis2dh12_xl_data_ready_get(&dev_ctx, &reg.byte);

	if (reg.byte)
	{
	  memset(data_raw_acceleration.u8bit, 0x00, 3*sizeof(int16_t));
	  lis2dh12_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);

	  acc1 = (int) lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration.i16bit[0]) * -1;
	  acc2 = (int) lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration.i16bit[1]);
	  acc3 = (int) lis2dh12_from_fs2_hr_to_mg(data_raw_acceleration.i16bit[2]);

	  if(acc1 > 999)
	  {
		  acc1 = 999;
	  }
	  if(acc2 > 999)
	  {
	  		  acc2 = 999;
	  }
	  if(acc3 > 999)
	  {
	  		  acc3 = 999;
	  }
	  if(acc1 < -999)
	  {
		  acc1 = -999;
	  }
	  if(acc2 < -999)
	  {
		  acc2 = -999;
	  }
	  if(acc3 < -999)
	  {
		  acc3 = -999;
	  }

	}

}

int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	reg |= 0x80;
	HAL_I2C_Mem_Read(handle, LIS2DH12_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return 0;
}

int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	reg |= 0x80;
	HAL_I2C_Mem_Write(handle, LIS2DH12_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return 0;
}



