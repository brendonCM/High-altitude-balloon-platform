#pragma once
class Temperature
{
public:
	Temperature(int pin);
	float Get_Temp();
	~Temperature();

protected:
	int pin_number, analog_value;
	float internal_temperature;
	void Read_Analog_Input();
	void Calc_Temp();
};

