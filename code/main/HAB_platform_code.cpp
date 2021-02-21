#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <termios.h> 
#include <cstring>
#include "Temperature.h"
#include "Voltage_Check.h"
#include "bme.h"
#include "GPIO.h"
#include "PWM.h"
#include <pthread.h>

using namespace std;
using namespace exploringBB;

#define Data_path "/home/debian/projects/data.txt"   // directory where all the data will be saved


float voltage, intTemp;
float extTemp, pressure, humidity;
float voltage_level = 3.3;
float recovery_system = 1;

int  GPS_UART_file;
int intAltitude, minAltitude, maxAltitude; 
int storage_capcity = 1;    // indicates if storage capacity is full, if on, it is full else it is not
int countCalcSpeed = 0;
int prevAltitude , VerticalSpeed;
int check;

FILE *fp;

int GPS_UART_setup()
{
	int file, count;


	if ((file = open("/dev/ttyO0", O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
		perror("UART: Failed to open the file.\n");
		return -1;
	}
	struct termios options;               //The termios structure is vital
	tcgetattr(file, &options);               //Sets the parameters associated with file

	// Set up the communications options:	
	//   9600 baud, 8-bit, enable receiver, no modem control lines
	options.c_cflag =  CS8 | CREAD | CLOCAL;
	options.c_iflag = IGNPAR | ICRNL;       //ignore partity errors, CR -> newline
	tcflush(file, TCIFLUSH);                //discard file information not transmitted
	cfsetospeed(&options, B9600);           // set output baud rate to 9600
	cfsetispeed(&options, B9600);  		  // set input baud rate to 9600
	tcsetattr(file, TCSANOW, &options);     // sets the parameters for the terminal ,changes occur immmediately
	
	return file;
	
	/*unsigned char transmit[255] = "PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";        //set GPRMC only
	if((count = write(file, &transmit, 255)) < 0) {
		perror("Failed to write to the output\n");
		return -1;
	}
	usleep(1000000);      // wait for 1 second*/
}

int releaseCapsule()
{
	// sets the speed of the motors
		int PWM_choose = 0;   // to set to either A or to B
		PWM pwm("pwm-2:0", PWM_choose, "2");    // this is the file created 
		pwm.setPeriod(5);    //set the period in ns
		pwm.setDutyCycle(40.0f);    //can use percentage or time in ns
		pwm.setPolarity(PWM::ACTIVE_LOW);    //ACTIVE_LOW or ACTIVE_HIGH
		pwm.run();    //start the output
		char dir = 'o';
		
	if (dir == 'o')
	{
		// sets direction to open
		PWM_choose = 0;      // set to PWM0A OPEN
		PWM pwm("pwm-0:0", PWM_choose, "0");       // this is the file created 
		pwm.setPeriod(1000000);       //set the period in ns
		pwm.setDutyCycle(95.0f);       //can use percentage or time in ns
		pwm.setPolarity(PWM::ACTIVE_LOW);       //ACTIVE_LOW or ACTIVE_HIGH
		pwm.run();       //start the output
		usleep(10000000);
		pwm.stop();
		dir = 'c';
	}
		
	usleep(1000000);  // wait 1 second 
		
		
	if(dir == 'c')
	{
		// sets direction to close
		int PWM_choose = 1;       // set to PWM2B CLOSE
		PWM pwm("pwm-4:1", PWM_choose, "4");        // this is the file created 
		pwm.setPeriod(1000000);        //set the period in ns
		pwm.setDutyCycle(95.0f);        //can use percentage or time in ns
		pwm.setPolarity(PWM::ACTIVE_LOW);        //ACTIVE_LOW or ACTIVE_HIGH
		pwm.run();        //start the output
		usleep(10000000);
		pwm.stop();
	}
}

int getVerticalSpeed()
{
	if (countCalcSpeed == 0)
	{
		prevAltitude = atoi(altitude);
	}
	
	if (countCalcSpeed == 1)
	{
		VerticalSpeed = (atoi(altitude) - prevAltitude) / 0.5;
		countCalcSpeed = 0;
	}
	countCalcSpeed++;
	return 0;
}

int start_stop_Video()
{
	
	GPIO outputGPIO(45);
	outputGPIO.setValue(exploringBB::GPIO::LOW);
	usleep(700000);
	outputGPIO.setValue(exploringBB::GPIO::HIGH);
	return 0;
}

int main(int argc, char* argv[]) {

	// Set pin values for Pocketbeagle

	// Get the internal temperature from pin 6
	Temperature internalTemp(6);
	// Get the battery voltage from pin 5
	Voltage_Check voltageSensor(5);

	// set the release point altitude
	//minAltitude = 22000;
	//maxAltitude = 23000;
	// setting pin high so camera is able to start video when pin goes low
	//GPIO outputGPIO(45);
	//outputGPIO.setDirection(exploringBB::GPIO::OUTPUT);
	//outputGPIO.setValue(exploringBB::GPIO::HIGH);
	
	
	// Setup for peripherals below
	BME bme;
	GPS_UART_file = GPS_UART_setup();

	// check for the subsytems if they are connected

	/*if(storage_capcity == 1)
	{
		GPIO outputGPIO(57);
		outputGPIO.setDirection(exploringBB::GPIO::OUTPUT);
		outputGPIO.setValue(exploringBB::GPIO::HIGH);
		usleep(30000000);
		outputGPIO.setValue(exploringBB::GPIO::LOW);
		outputGPIO.setDirection(exploringBB::GPIO::INPUT);
	}
	if (voltageSensor.Get_Voltage() < 4.5) // if the voltage sensor is less than 4.5V
		{
			GPIO outputGPIO(60);
			outputGPIO.setDirection(exploringBB::GPIO::OUTPUT);
			outputGPIO.setValue(exploringBB::GPIO::HIGH);
			usleep(30000000);
			outputGPIO.setValue(exploringBB::GPIO::LOW);
			outputGPIO.setDirection(exploringBB::GPIO::INPUT);			
		}
	if (recovery_system == 1)
	{
		GPIO outputGPIO(52);
		outputGPIO.setDirection(exploringBB::GPIO::OUTPUT);
		outputGPIO.setValue(exploringBB::GPIO::HIGH);
		usleep(30000000);
		outputGPIO.setValue(exploringBB::GPIO::LOW);
		outputGPIO.setDirection(exploringBB::GPIO::INPUT);
	} */

	// System Starts
	while(1)
	{
		// Read the internal temperature and voltage from the analog input
		intTemp = internalTemp.Get_Temp();
		voltage = voltageSensor.Get_Voltage();
		
		// Read the BME sensor values
		bme.get_BME_Sensor_Values(&extTemp, &pressure, &humidity);
		
		// Calculate vertical speed
		//getVerticalSpeed();
		
		//intAltitude = atoi(altitude);
		// Check if time to release capsule at the desired altitude
		/*if (intAltitude > minAltitude && intAltitude < maxAltitude)
		{
			start_stop_Video(); // starts the video
			usleep(2000000); //allow 5 second for the camera to switch on and get ready
			releaseCapsule();
			usleep(180000000); // allow camera to record drop for 3 mins
			// stop the video recording
			start_stop_Video(); // stops the video
		}*/
		
		// save all the data to a textfile
		fp = fopen(Data_path, "a");
		fprintf(fp, "$%s,%s,%s,%s,%s,%s,%d,%.2f,%.2f,%.2f,%.2f,%.2f;", platformtime , date , latitude , longitude, altitude , speed , VerticalSpeed , voltage, intTemp, extTemp, pressure, humidity);
		fclose(fp);
		//sample every 0.5 seconds
		usleep(500000);
	}
	return 0;
}