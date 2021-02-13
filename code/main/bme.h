#pragma once
class BME
{
public:
	BME();
	void get_BME_Sensor_Values(float *extTemp, float *Ambientpressure, float *Relativehumidity);
	~BME();

protected:
	float ET, AP, RH;
	int setupI2C(),fileNo;
	void calcBME_Sensor_Values(int file);
};

