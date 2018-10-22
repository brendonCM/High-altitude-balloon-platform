/*
 GPIO.h
 */

#ifndef GPIO_H_
#define GPIO_H_
#include<string>
#include<fstream>
using std::string;
using std::ofstream;

#define GPIO_PATH_EXPORT "/sys/class/gpio/export" // path for the exporting a GPIO pin
#define GPIO_PATH_UNEXPORT "/sys/class/gpio/unexport" // path for the unexporting a GPIO pin
#define GPIO_PATH "/sys/class/gpio/" // path for location of GPIO pins

namespace exploringBB {



/**
 * @class GPIO
 * @brief GPIO class for input and output functionality on a single GPIO pin
 */
class GPIO {
public:
	enum DIRECTION{ INPUT, OUTPUT };
	enum VALUE{ LOW=0, HIGH=1 };

private:
	int number;			/**< The GPIO number of the object */

	string name;		/**< The name of the GPIO e.g. gpio50 */
	string path;  		/**< The full path to the GPIO e.g. /sys/class/gpio/gpio50/ */

public:
	GPIO(int number);
	virtual int getNumber() { return number; } /**< Returns the GPIO number as an int. */

	// General Input and Output Settings
	virtual int setDirection(GPIO::DIRECTION);
	virtual GPIO::DIRECTION getDirection();
	virtual int setValue(GPIO::VALUE);
	virtual int toggleOutput();
	virtual GPIO::VALUE getValue();
	virtual int setActiveLow(bool isLow=true);  //low=1, high=0
	virtual int setActiveHigh(); //default
	

private:
	int write(string path, string filename, string value);
	int write(string path, string filename, int value);
	string read(string path, string filename);
	int exportGPIO();
	int unexportGPIO();
};


} /* namespace exploringBB */
#endif /* GPIO_H_ */
