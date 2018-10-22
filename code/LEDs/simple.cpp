
#include <iostream>
#include <unistd.h> //for usleep
#include <fstream>
#include <string>
#include <stdio.h>
#include <cstring>
#include "GPIO.h"

using namespace exploringBB;
using namespace std;
int main(){
	int storage_capcity = 1;   // indicates if storage capacity is full, if on, it is full else it is not
	if (storage_capcity == 1)
	{
		GPIO outputGPIO(57);
		outputGPIO.setDirection(exploringBB::GPIO::OUTPUT); // change pin to output mode
		outputGPIO.setValue(exploringBB::GPIO::HIGH);	 // set pin high
		usleep(30000000); // Keep the LED on for 30 seconds
		outputGPIO.setValue(exploringBB::GPIO::LOW); // set pin low
		outputGPIO.setDirection(exploringBB::GPIO::INPUT); // change oin to input mode
		
	}
	float voltage_level = 3.3;
	if (voltage_level < 4.5) // if the voltage of the batteries  is too low, LED goes on
	{
		GPIO outputGPIO(60);
		outputGPIO.setDirection(exploringBB::GPIO::OUTPUT);
		outputGPIO.setValue(exploringBB::GPIO::HIGH);
		usleep(30000000);  // Keep the LED on for 30 seconds
		outputGPIO.setValue(exploringBB::GPIO::LOW);
		outputGPIO.setDirection(exploringBB::GPIO::INPUT);
		
	}
	
	float recovery_system = 1; // indicates if there is a recovery system on board
	if (recovery_system ==1)
	{
		GPIO outputGPIO(52);
		outputGPIO.setDirection(exploringBB::GPIO::OUTPUT);
		outputGPIO.setValue(exploringBB::GPIO::HIGH);	
		usleep(30000000);  // Keep the LED on for 30 seconds
		outputGPIO.setValue(exploringBB::GPIO::LOW);
		outputGPIO.setDirection(exploringBB::GPIO::INPUT);
	}
   return 0;
}



