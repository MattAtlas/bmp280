#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include "simple_i2c.h"
#include "BMP280.h"

#include <robotics_cape.h>

void initialize_BMP(){
	
	i2c_init(2,0x77);
	i2c_write_byte(2,0xF4,0x57);		// ctrl_meas oversample and mode
	i2c_write_byte(2,0xF5,0x10);		// config t_standby, filter, spi_en_
}

/*Mode 	Oversampling 	osrs_p osrs_t IIRfiltercoeff. IDD[µA] ODR[Hz] RMSNoise
 Normal Ultrahighres 	×16 	×2 		16 				650 	26.3 	1.6 */
 
void main(){
	

	initialize_BMP();
	readCoefficients();
	float temp, pressure, altitude;	
	
	while(1){

		
		temp = get_temperature();
		pressure = get_pressure();
		altitude = readAltitude();
		printf("temp = %0.2f\npressure = %0.2f\naltitude = %0.2f\n\n",temp,pressure,altitude);
		usleep(100000);
	}
	
}
