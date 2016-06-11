/***************************************************************************
  This is a library for the BMP280 pressure sensor

 ***************************************************************************/


#include "BMP280.h"
#include "BMP280_defs.h"
#include "simple_i2c.h"
#include <stdint.h>
#include <math.h>
#define BMP_BUS 2


typedef struct bmp280_cal_t{
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

}bmp280_cal_t;


typedef struct bmp280_data_t{
	float temp;
	uint32_t t_fine;
	float alt_0;
	float alt;
	float pressure;
	
}bmp280_data_t;


bmp280_cal_t cal;
bmp280_data_t data;


/*******************************************************************************
    @brief  Reads the factory-set coefficients
*******************************************************************************/
int readCoefficients(){
	uint8_t buf[2];
	
	i2c_read_bytes(BMP_BUS,BMP280_DIG_T1,2,buf);
	cal.dig_T1 = (uint16_t) ((buf[1] << 8) | buf [0]);
	i2c_read_bytes(BMP_BUS,BMP280_DIG_T2,2,buf);
	cal.dig_T2 = (uint16_t) ((buf[1] << 8) | buf [0]);
	i2c_read_bytes(BMP_BUS,BMP280_DIG_T3,2,buf);
	cal.dig_T3 = (uint16_t) ((buf[1] << 8) | buf [0]);
	
	i2c_read_bytes(BMP_BUS,BMP280_DIG_P1,2,buf);
	cal.dig_P1 = (uint16_t) ((buf[1] << 8) | buf [0]);
	i2c_read_bytes(BMP_BUS,BMP280_DIG_P2,2,buf);
	cal.dig_P2 = (uint16_t) ((buf[1] << 8) | buf [0]);
	i2c_read_bytes(BMP_BUS,BMP280_DIG_P3,2,buf);
	cal.dig_P3 = (uint16_t) ((buf[1] << 8) | buf [0]);
	i2c_read_bytes(BMP_BUS,BMP280_DIG_P4,2,buf);
	cal.dig_P4 = (uint16_t) ((buf[1] << 8) | buf [0]);
	i2c_read_bytes(BMP_BUS,BMP280_DIG_P5,2,buf);
	cal.dig_P5 = (uint16_t) ((buf[1] << 8) | buf [0]);
	i2c_read_bytes(BMP_BUS,BMP280_DIG_P6,2,buf);
	cal.dig_P6 = (uint16_t) ((buf[1] << 8) | buf [0]);
	i2c_read_bytes(BMP_BUS,BMP280_DIG_P7,2,buf);
	cal.dig_P7 = (uint16_t) ((buf[1] << 8) | buf [0]);
	i2c_read_bytes(BMP_BUS,BMP280_DIG_P8,2,buf);
	cal.dig_P8 = (uint16_t) ((buf[1] << 8) | buf [0]);
	i2c_read_bytes(BMP_BUS,BMP280_DIG_P9,2,buf);
	cal.dig_P9 = (uint16_t) ((buf[1] << 8) | buf [0]);

	return 0;
}

/**************************************************************************/

/**************************************************************************/
int readTemperature(){
	int32_t var1, var2, t_fine, T;
	
	uint8_t raw_temp[3] = {0};
	
	i2c_read_bytes(BMP_BUS,BMP280_TEMPERATURE_MSB,3,raw_temp);

	int32_t adc_T;
	adc_T = (raw_temp[0] << 12)|(raw_temp[1] << 4)|(raw_temp[2] >> 4);
	
	var1  = ((((adc_T>>3) - ((int32_t)cal.dig_T1 <<1))) *
			((int32_t)cal.dig_T2)) >> 11;
	var2  = (((((adc_T>>4) - ((int32_t)cal.dig_T1)) *
			   ((adc_T>>4) - ((int32_t)cal.dig_T1))) >> 12) *
			   ((int32_t)cal.dig_T3)) >> 14;

	data.t_fine = var1 + var2;
	
	T  = (data.t_fine * 5 + 128) >> 8;
	data.temp =  T/100.0;

	return 0;
}

/**************************************************************************/

/**************************************************************************/
int readPressure() {
	int64_t var1, var2, p;

	uint8_t raw_pressure[3] = {0};
	
	i2c_read_bytes(BMP_BUS,BMP280_PRESSURE_MSB,3,raw_pressure);
	
	int32_t adc_P;
	adc_P = (raw_pressure[0] << 12)|(raw_pressure[1] << 4)|(raw_pressure[2] >> 4);
	
	var1 = ((int64_t)data.t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)cal.dig_P6;
	var2 = var2 + ((var1*(int64_t)cal.dig_P5)<<17);
	var2 = var2 + (((int64_t)cal.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)cal.dig_P3)>>8) +
    ((var1 * (int64_t)cal.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)cal.dig_P1)>>33;

	if (var1 == 0){
		return 0;  // avoid exception caused by division by zero
	}
  
	p = 1048576 - adc_P;
	p = (((p<<31) - var2)*3125) / var1;
	var1 = (((int64_t)cal.dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)cal.dig_P8) * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (((int64_t)cal.dig_P7)<<4);
	data.pressure = (float)p/256;
	//return data.pressure;
	return 0;
}


float get_temperature(){
	readTemperature();
	return data.temp;
}

float get_pressure(){
	readPressure();
	return data.pressure;
}

float readAltitude() {
	
	// #define SEALEVELHPA	1013.25
	float altitude;
	float pressure = get_pressure();
	pressure = pressure/100.0;

	altitude = 44330.0*(1.0 - pow((pressure/1013.25), 0.1903));

	return altitude;
}
