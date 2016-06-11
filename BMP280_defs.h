/***************************************************************************
  This is a library for the BMP280 pressure sensor

  
 ***************************************************************************/
#ifndef _BMP280_H
#define _BMP280_H


//============================================================================//
   
#define BMP280_ADDRESS          0x77

//=======================Registers Addresses==================================// 
#define BMP280_DIG_T1			0x88
#define BMP280_DIG_T2			0x8A
#define BMP280_DIG_T3			0x8C
#define BMP280_DIG_P1			0x8E
#define BMP280_DIG_P2			0x90
#define BMP280_DIG_P3			0x92
#define BMP280_DIG_P4			0x94
#define BMP280_DIG_P5			0x96
#define BMP280_DIG_P6			0x98
#define BMP280_DIG_P7			0x9A
#define BMP280_DIG_P8			0x9C
#define BMP280_DIG_P9			0x9E

#define BMP280_CHIP_ID			0xD0
#define BMP280_RST_REG 			0xE0
#define BMP280_CAL26			0xE1	// R calibration stored in 0xE1-0xF0
#define BMP280_STAT_REG			0xF3
#define BMP280_CTRL_MEAS		0xF4
#define BMP280_CONFIG			0xF5

#define BMP280_PRESSURE_MSB		0xF7
#define BMP280_PRESSURE_LSB		0xF8
#define BMP280_PRESSURE_XLSB	0xF9

#define BMP280_TEMPERATURE_MSB	0xFA
#define BMP280_TEMPERATURE_LSB	0xFB
#define BMP280_TEMPERATURE_XLSB	0xFC






#endif
