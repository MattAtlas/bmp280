#ifndef BMP280_H
#define BMP280_H

int read_bmp280();

float get_temperature();
float get_pressure();
float get_altitude();

int readCoefficients();

int read_bmp280_when_ready(int timeout);

#endif
