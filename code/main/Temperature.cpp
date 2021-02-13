#include "Temperature.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdlib.h>
using namespace std;

#define Analog_Path "/sys/bus/iio/devices/iio:device0/in_voltage"

Temperature::Temperature(int pin):pin_number{pin}
{
	Read_Analog_Input();
	Calc_Temp();
}

float Temperature::Get_Temp()
{
	return internal_temperature;
}

void Temperature::Read_Analog_Input()
{
	stringstream pin_path;
	pin_path << Analog_Path << pin_number << "_raw";
	ifstream path_file(pin_path.str().c_str(),ios_base::in); 
	if (path_file.good())
	{
		path_file >> analog_value;
	}
	else
	{
		cerr << "Could not open path :" << pin_path.str() << endl;
	}
}

void Temperature::Calc_Temp()
{
	float cur_voltage = analog_value * (3.30f / 3000.0f) - 0.1421;
	// to check if voltage is close to zero
	if (cur_voltage < 0)
		cur_voltage = 0;
	internal_temperature = (cur_voltage - 0.5f) / 0.01f;
	// to set the temperature to zero, becuase if not zero, internal temp does not work 
	if (internal_temperature < 0){internal_temperature = 0;}
}

Temperature::~Temperature()
{
}
