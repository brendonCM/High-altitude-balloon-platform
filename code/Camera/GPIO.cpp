/*
 * GPIO.cpp
 */

#include "GPIO.h"
#include "util.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/epoll.h>
#include <pthread.h>
using namespace std;

namespace exploringBB {
/**
 * The constructor will set up the states and export the pin.
 * @param number The GPIO number to be exported
 */
GPIO::GPIO(int number) {
	this->number = number;
	ostringstream s;
	s << "gpio" << number;
	this->name = string(s.str());
	this->path = GPIO_PATH + this->name + '/';
	// this->exportGPIO(); removed because path is already set
	// need to give Linux time to set up the sysfs structure
	usleep(250000); // 250ms delay
}
/**
 * Private write method that writes a single string value to a file in the path provided
 * @param Path The sysfs path of the file to be modified
 * @param Filename The file to be written to in that path
 * @param String value The value to be written to the file
 * @return
 */

int GPIO::write(string path, string filename, string value){
	std::ofstream fs;
   fs.open(path + filename);
   if (!fs.is_open()){
	   perror("GPIO: write failed to open file ");
	   return -1;
   }
   fs << value;
   fs.close();
   return 0;
}
	
int GPIO::write(string path, string filename, int value) {
		std::ofstream fs;
		fs.open((path + filename).c_str());
		if (!fs.is_open()) {
			perror("GPIO: write failed to open file ");
			return -1;
		}
		fs << value;
		fs.close();
		return 0;
	}	

string GPIO::read(string path, string filename){
   ifstream fs;
   fs.open(path + filename);
   if (!fs.is_open()){
	   perror("GPIO: read failed to open file ");
    }
   string input;
   getline(fs,input);
   fs.close();
   return input;
}

/**
 * Private write method that writes a single int value to a file in the path provided
 * @param Path The sysfs path of the file to be modified
 * @param Filename The file to be written to in that path
 * @param int value The int value to be written to the file
 * @return
 */

/**
 * Private method to export the GPIO
 * @return int that describes if the operation fails
 */
int GPIO::exportGPIO(){
	std::ofstream fs;
	fs.open(GPIO_PATH_EXPORT);
	if (!fs.is_open()) {
		perror("GPIO: write failed to open file ");
		return -1;
	}
	fs << this->number;
	fs.close();
	return 0;
}

int GPIO::unexportGPIO(){
	std::ofstream fs;
	fs.open(GPIO_PATH_UNEXPORT);
	if (!fs.is_open()) {
		perror("GPIO: write failed to open file ");
		return -1;
	}
	fs << this->number;
	fs.close();
	return 0;
}

int GPIO::setDirection(GPIO::DIRECTION dir){
   switch(dir){
   case INPUT: return write(this->path, "direction", "in");
      break;
   case OUTPUT:return write(this->path, "direction", "out");
      break;
   }
   return -1;
}

int GPIO::setValue(GPIO::VALUE value){
   switch(value){
   case HIGH: return write(this->path, "value", 1);
      break;
   case LOW: return write(this->path, "value", 0);
      break;
   }
   return -1;
}


int GPIO::setActiveLow(bool isLow){
   if(isLow) return write(this->path, "active_low",1);
   else return write(this->path, "active_low",0);
}

int GPIO::setActiveHigh(){
   return this->setActiveLow(false);
}

GPIO::VALUE GPIO::getValue(){
	string input = read(this->path, "value");
	if (input == "0") return LOW;
	else return HIGH;
}

GPIO::DIRECTION GPIO::getDirection(){
	string input = read(this->path, "direction");
	if (input == "in") return INPUT;
	else return OUTPUT;
}


int GPIO::toggleOutput(){
	this->setDirection(OUTPUT);
	if ((bool) this->getValue()) this->setValue(LOW);
	else this->setValue(HIGH);
    return 0;
}
	

} /* namespace exploringBB */
