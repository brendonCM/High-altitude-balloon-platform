#include "Voltage_Check.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdlib.h>
using namespace std;

#define Analog_Path "/sys/bus/iio/devices/iio:device0/in_voltage"



Voltage_Check::Voltage_Check(int pin)
{
	pin_number = pin;
	Read_Analog_Input();
	Calc_Voltage();
}


float Voltage_Check::Get_Voltage()
{
	return batteryVoltage;
}

Voltage_Check::~Voltage_Check()
{
}

void Voltage_Check::Read_Analog_Input()
{
	stringstream pin_path;
	pin_path << Analog_Path << pin_number << "_raw";
	ifstream path_file(pin_path.str().c_str(), ios_base::in);
	if (path_file.good())
	{
		path_file >> analog_value;
	}
	else
	{
		cerr << "Could not open path :" << pin_path.str() << endl;
	}
}

void Voltage_Check::Calc_Voltage()
{
	float gradient = 1.65f / 1481.0f;
	float batteryVoltage = (gradient * analog_value)*2.724 + 0.013;
	// Voltage for a battery cannot be less than zero
	if (batteryVoltage < 0)
		batteryVoltage = 0;
}

