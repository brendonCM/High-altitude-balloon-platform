#pragma once
class Voltage_Check
{
public:
	Voltage_Check(int pin);
	float Get_Voltage();
	~Voltage_Check();

protected:
	int pin_number, analog_value;
	float batteryVoltage;
	void Read_Analog_Input();
	void Calc_Voltage();
};

