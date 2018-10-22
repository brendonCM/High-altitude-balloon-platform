#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdlib.h>
using namespace std;

#define Analog_in "/sys/bus/iio/devices/iio:device0/in_voltage"  // directory for analog inputs on the pocketbeagle

// opens the file in located in the directory according to the pin number and returns the ADC value
int readAnalog(int number) {
	 // returns the input as an int
	stringstream ss;
	ss << Analog_in << number << "_raw";
	fstream fs;
	fs.open(ss.str().c_str(), fstream::in);
	fs >> number;
	fs.close();
	return number;
}

// calculates the temperature using a linear equation. if output value is 0.75V then temperature is 25 degree celsius.
float getTemperature(int adc_value) {
	float cur_voltage = adc_value * (3.30f / 3000.0f) - 0.1421; 
	float Temp = (cur_voltage - 0.5f) / 0.01f;
	return Temp;
}

// Calculates voltage according to the voltage divider vlues to convert input voltage of 3.3 to 9V 
float getVoltage(int adc_value)
{
	float m = 1.65f / 1481.0f;
	float voltage = (m * adc_value)*2.724 + 0.013;
	return voltage;
}

// selects which analog pin needs to be exported
float getAnalogReadings(int analog_pin)
{
	if(analog_pin == 6)
	{
		float value = readAnalog(analog_pin);
		float temperature = getTemperature(value);
		return temperature;
	}
	
	else if(analog_pin == 5)
	{
		float value = readAnalog(analog_pin);
		float voltage = getVoltage(value);
		return voltage;
	}
}

int main(int argc, char* argv[]) {
	float temperature = getAnalogReadings(6); // set to read pin 6 analog input
	cout << "The internal temperatures is: " << temperature << " degrees Celsius." << endl; // displays the temperature value
	float voltage = getAnalogReadings(5); // set to read pin 5 analog input
	cout << "Voltage at the battery is: " << voltage << " V" << endl; // displays the voltage value
	return 0;
}
