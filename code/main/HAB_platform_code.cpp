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
#include "GPIO.h"
#include "PWM.h"
#include <pthread.h>

using namespace std;
using namespace exploringBB;

#define Analog_in "/sys/bus/iio/devices/iio:device0/in_voltage"  // directory for analog inputs
#define Data_path "/home/debian/projects/data.txt"   // directory where all the data will be saved


float voltage, intTemp;
float extTemp, pressure, humidity;
float voltage_level = 3.3;
float recovery_system = 1;

int I2C1_file, GPS_UART_file;
int intAltitude, minAltitude, maxAltitude; 
int storage_capcity = 1;    // indicates if storage capacity is full, if on, it is full else it is not
int countCalcSpeed = 0;
int prevAltitude , VerticalSpeed;
int check;

char NMEA[128];
char platformtime[12];
char latitude[13];
char longitude[14];
char speed[5];
char date[7];
char altitude[9];

FILE *fp;





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

float getTemperature(int adc_value) {
	float cur_voltage = adc_value * (3.30f / 3000.0f) - 0.1421;
	float Temp = (cur_voltage - 0.5f) / 0.01f;
	return Temp;
}

float getVoltage(int adc_value)
{
	float m = 1.65f / 1481.0f;
	float voltage = ((m) * adc_value) * 2.724 + 0.013;
	return (voltage);
}

float getAnalogReadings(int analog_pin)
{
	if (analog_pin == 6)
	{
		float value = readAnalog(analog_pin);
		float temperature = getTemperature(value);
		return temperature;
	}
	
	else if (analog_pin == 5)
	{
		float value = readAnalog(analog_pin);
		float voltage = getVoltage(value);
		return voltage;
	}
}

int setupI2C1()
{
	// Create I2C bus
	int file;
	char *bus = "/dev/i2c-1";
	if ((file = open(bus, O_RDWR)) < 0) 
	{
		printf("Failed to open the bus. \n");
		exit(1);
	}
	// Get I2C device, BME280 I2C address is 0x76(136)
	ioctl(file, I2C_SLAVE, 0x77);
	return file;
}

int getBME_sensor_values(int file, float *extTemp, float *Ambientpressure, float *Relativehumidity)
{
	// Read 24 bytes of data from register(0x88)
	char reg[1] = { 0x88 };
	write(file, reg, 1);
	char b1[24] = { 0 };
	if (read(file, b1, 24) != 24)
	{
		printf("Error : Input/Output error \n");
		exit(1);
	}

	// Convert the data
	// temp coefficents
	int dig_T1 = (b1[0] + b1[1] * 256);
	int dig_T2 = (b1[2] + b1[3] * 256);
	if (dig_T2 > 32767)
	{
		dig_T2 -= 65536;
	}
	int dig_T3 = (b1[4] + b1[5] * 256);
	if (dig_T3 > 32767)
	{
		dig_T3 -= 65536;
	}

	// pressure coefficents
	int dig_P1 = (b1[6] + b1[7] * 256);
	int dig_P2 = (b1[8] + b1[9] * 256);
	if (dig_P2 > 32767)
	{
		dig_P2 -= 65536;
	}
	int dig_P3 = (b1[10] + b1[11] * 256);
	if (dig_P3 > 32767)
	{
		dig_P3 -= 65536;
	}
	int dig_P4 = (b1[12] + b1[13] * 256);
	if (dig_P4 > 32767)
	{
		dig_P4 -= 65536;
	}
	int dig_P5 = (b1[14] + b1[15] * 256);
	if (dig_P5 > 32767)
	{
		dig_P5 -= 65536;
	}
	int dig_P6 = (b1[16] + b1[17] * 256);
	if (dig_P6 > 32767)
	{
		dig_P6 -= 65536;
	}
	int dig_P7 = (b1[18] + b1[19] * 256);
	if (dig_P7 > 32767)
	{
		dig_P7 -= 65536;
	}
	int dig_P8 = (b1[20] + b1[21] * 256);
	if (dig_P8 > 32767)
	{
		dig_P8 -= 65536;
	}
	int dig_P9 = (b1[22] + b1[23] * 256);
	if (dig_P9 > 32767)
	{
		dig_P9 -= 65536;
	}

	// Read 1 byte of data from register(0xA1)
	reg[0] = 0xA1;
	write(file, reg, 1);
	char data[8] = { 0 };
	read(file, data, 1);
	int dig_H1 = data[0];

	// Read 7 bytes of data from register(0xE1)
	reg[0] = 0xE1;
	write(file, reg, 1);
	read(file, b1, 7);

	// Convert the data
	// humidity coefficents
	int dig_H2 = (b1[0] + b1[1] * 256);
	if (dig_H2 > 32767)
	{
		dig_H2 -= 65536;
	}
	int dig_H3 = b1[2] & 0xFF;
	int dig_H4 = (b1[3] * 16 + (b1[4] & 0xF));
	if (dig_H4 > 32767)
	{
		dig_H4 -= 65536;
	}
	int dig_H5 = (b1[4] / 16) + (b1[5] * 16);
	if (dig_H5 > 32767)
	{
		dig_H5 -= 65536;
	}
	int dig_H6 = b1[6];
	if (dig_H6 > 127)
	{
		dig_H6 -= 256;
	}

	// Select control humidity register(0xF2)
	// Humidity over sampling rate = 1(0x01)
	char config[2] = { 0 };
	config[0] = 0xF2;
	config[1] = 0x01;
	write(file, config, 2);
	// Select control measurement register(0xF4)
	// Normal mode, temp and pressure over sampling rate = 1(0x27)
	config[0] = 0xF4;
	config[1] = 0x27;
	write(file, config, 2);
	// Select config register(0xF5)
	// Stand_by time = 1000 ms(0xA0)
	config[0] = 0xF5;
	config[1] = 0xA0;
	write(file, config, 2);

	// Read 8 bytes of data from register(0xF7)
	// pressure msb1, pressure msb, pressure lsb, temp msb1, temp msb, temp lsb, humidity lsb, humidity msb
	reg[0] = 0xF7;
	write(file, reg, 1);
	read(file, data, 8);

	// Convert pressure and temperature data to 19-bits
	long adc_p = ((long)(data[0] * 65536 + ((long)(data[1] * 256) + (long)(data[2] & 0xF0)))) / 16;
	long adc_t = ((long)(data[3] * 65536 + ((long)(data[4] * 256) + (long)(data[5] & 0xF0)))) / 16;
	// Convert the humidity data
	long adc_h = (data[6] * 256 + data[7]);

	// Temperature offset calculations
	float var1 = (((float)adc_t) / 16384.0 - ((float)dig_T1) / 1024.0) * ((float)dig_T2);
	float var2 = ((((float)adc_t) / 131072.0 - ((float)dig_T1) / 8192.0) *
					(((float)adc_t) / 131072.0 - ((float)dig_T1) / 8192.0)) * ((float)dig_T3);
	float t_fine = (long)(var1 + var2);
	float cTemp = (var1 + var2) / 5120.0;
	float fTemp = cTemp * 1.8 + 32;

	// Pressure offset calculations
	var1 = ((float)t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((float)dig_P6) / 32768.0;
	var2 = var2 + var1 * ((float)dig_P5) * 2.0;
	var2 = (var2 / 4.0) + (((float)dig_P4) * 65536.0);
	var1 = (((float) dig_P3) * var1 * var1 / 524288.0 + ((float) dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((float)dig_P1);
	float p = 1048576.0 - (float)adc_p;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((float) dig_P9) * p * p / 2147483648.0;
	var2 = p * ((float) dig_P8) / 32768.0;
	float pressure = (p + (var1 + var2 + ((float)dig_P7)) / 16.0) / 100;

	// Humidity offset calculations
	float var_H = (((float)t_fine) - 76800.0);
	var_H = (adc_h - (dig_H4 * 64.0 + dig_H5 / 16384.0 * var_H)) * (dig_H2 / 65536.0 * (1.0 + dig_H6 / 67108864.0 * var_H * (1.0 + dig_H3 / 67108864.0 * var_H)));
	float humidity = var_H * (1.0 -  dig_H1 * var_H / 524288.0);
	if (humidity > 100.0)
	{
		humidity = 100.0;
	}
	else
	if (humidity < 0.0) 
	{
		humidity = 0.0;
	}
	*extTemp = cTemp;
	*Ambientpressure = pressure;
	*Relativehumidity = humidity;
	;
	return 0;
}

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

	// set the release point altitude
	minAltitude = 22000;
	maxAltitude = 23000;
	// setting pin high so camera is able to start video when pin goes low
	GPIO outputGPIO(45);
	outputGPIO.setDirection(exploringBB::GPIO::OUTPUT);
	outputGPIO.setValue(exploringBB::GPIO::HIGH);
	
	
	// setup for peripherals below
	I2C1_file = setupI2C1();

	
	// check for the subsytems if they are connected

	if(storage_capcity == 1)
	{
		GPIO outputGPIO(57);
		outputGPIO.setDirection(exploringBB::GPIO::OUTPUT);
		outputGPIO.setValue(exploringBB::GPIO::HIGH);
		usleep(30000000);
		outputGPIO.setValue(exploringBB::GPIO::LOW);
		outputGPIO.setDirection(exploringBB::GPIO::INPUT);
	}
	if (getAnalogReadings(5) < 4.5) // if the voltage sensor is less than 4.5V
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
	}

	
	// reading of sensor values below
	while(1)
	{

		GPS_UART_file = GPS_UART_setup();
		check = 0;
		while (check ==0)  
		{
			read(GPS_UART_file, NMEA, 128);
			if (NMEA[0] == '$' || NMEA[0] == '\n')
			{
				if (NMEA[5] == 'C')   // check to see if the GPRMC packet then stores the relevant information of the packets
					{	
						for (int i = 0; i < 10; i++)
						{
							platformtime[i] = NMEA[i + 7];
						}
						if (NMEA[18] == 'A')
						{
							// get the latitude coordinates from the NMEA packet
							for(int i = 0 ; i < 11 ; i++)
							{
								latitude[i] = NMEA[i + 20]; 
							}
							// get the longitude coordinates from the NMEA packet
							for(int i = 0 ; i < 12 ; i++)
							{
								longitude[i] = NMEA[i + 32]; 
							}
							// get the speed from the NMEA packet
							for(int i = 0 ; i < 4 ; i++)
							{
								speed[i] = NMEA[i + 45]; 
							}
						
							// get the date from the NMEA packet
							int len = strlen(NMEA);
							int count = 0;
							int posD1, posD2;
							for (int i = 0; i < len; i++)
							{
								if (NMEA[i] == ',')
								{
									count++;
									if (count == 9)
									{
										posD1 = i;
									}
									if (count == 10)
									{
										posD2 = i;
									}
								}
							}
							int x = 0;
							for (int i = posD1 + 1; i < posD2; i++)
							{
								date[x] = NMEA[i];
								x++;
							}
							check = 1;
						}
					}
			}
		}
		check = 0;
		while (check ==0)
		{
			read(GPS_UART_file, NMEA, 255);
			if (NMEA[0] == '$' || NMEA[0] == '\n')
			{
				if (NMEA[4] == 'G' && NMEA[5] == 'A')  // Used to check for the GPGGA packet to get the altitude of the platform
					{
						int len = strlen(NMEA);
						int count = 0;
						int pos1, pos2;
						for (int i = 0; i < len; i++)
						{
							if (NMEA[i] == ',')
							{
								count++;
								if (count == 9)
								{
									pos1 = i;
								}
								if (count == 10)
								{
									pos2 = i;
								}
							}
						}
						int x = 0;
						for (int i = pos1 + 1; i < pos2; i++)
						{
							altitude[x] = NMEA[i];
							x++;
						}
						check = 1;
					}
			}
		}
		close(GPS_UART_file);
		  
		// Read the internal temperature and voltage from the analog input
		 intTemp = getAnalogReadings(6);
		 voltage = getAnalogReadings(5);
		
		// Read the BME sensor values
		 getBME_sensor_values(I2C1_file, &extTemp, &pressure, &humidity);
		
		// Calculate vertical speed
		getVerticalSpeed();
		
		intAltitude = atoi(altitude);
		// Check if time to release capsule at the desired altitude
		if (intAltitude > minAltitude && intAltitude < maxAltitude)
		{
			start_stop_Video(); // starts the video
			usleep(2000000); //allow 5 second for the camera to switch on and get ready
			releaseCapsule();
			usleep(180000000); // allow camera to record drop for 3 mins
			// stop the video recording
			start_stop_Video(); // stops the video
		}
		
		// save all the data to a textfile
		fp = fopen(Data_path, "a");
		fprintf(fp, "$%s,%s,%s,%s,%s,%s,%d,%.2f,%.2f,%.2f,%.2f,%.2f;", platformtime , date , latitude , longitude, altitude , speed , VerticalSpeed , voltage, intTemp, extTemp, pressure, humidity);
		fclose(fp);
		//sample every 0.5 seconds
		usleep(500000);
	}
	return 0;
}