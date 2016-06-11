#ifndef BMP280_H
#define BMP280_H

float get_temperature();
float get_pressure();

int readCoefficients();
float readAltitude();

#endif
