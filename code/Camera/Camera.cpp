#include <iostream>
#include <unistd.h> //for usleep
#include <fstream>
#include <string>
#include <stdio.h>
#include <cstring>
#include "GPIO.h"

using namespace exploringBB;
using namespace std;

int main(int argc, char *argv[])
{
	GPIO outputGPIO(45); // set which GPIO pin is selected. GPIO45 is selected to be the control for the camera
	outputGPIO.setDirection(exploringBB::GPIO::OUTPUT);
	outputGPIO.setValue(exploringBB::GPIO::HIGH);
	
	// start video recording
	outputGPIO.setValue(exploringBB::GPIO::LOW);
	usleep(700000);
	outputGPIO.setValue(exploringBB::GPIO::HIGH);
	
	usleep(120000000); // record video for 2 minutes
	// stop video recording
	outputGPIO.setValue(exploringBB::GPIO::LOW);
	usleep(700000);
	outputGPIO.setValue(exploringBB::GPIO::HIGH);
	
	usleep(5000000); //wait 5 seconds for camera to switch off
	
	// To take a picture
	outputGPIO.setValue(exploringBB::GPIO::LOW);
	usleep(200000);
	outputGPIO.setValue(exploringBB::GPIO::HIGH);
	
	
}