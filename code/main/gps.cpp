#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <fstream>
#include <sstream>

#include "gps.h"
#include "nmea.h"
#include "gprmc.h"
#include "mtk.h"


#define pathUART "/dev/ttyO0"
#define max_CHARACTERS 82 



using namespace std;

gps::gps()
{
	setupUART();
	configureL80Module();
}

void gps::getGPSData()
{
	readNMEAPacket();
	parseNMEA();
}

void gps::setupUART()
{
	// Opens the file to show we are connected to the device
	// If greater than 0 then connected else not connected
	if ((gpsFileNo = open(pathUART, O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
		perror("UART: Failed to open the file.\n");
		exit(1);
	}

	struct termios options;               //The termios structure is vital
	tcgetattr(gpsFileNo, &options);              //Sets the parameters associated with file

	// Set up the communications options:	
	//   9600 baud, 8-bit, enable receiver, no modem control lines
	options.c_cflag = CS8 | CREAD | CLOCAL;
	options.c_iflag = IGNPAR | ICRNL;      //ignore partity errors, CR -> newline
	tcflush(gpsFileNo, TCIFLUSH);               //discard file information not transmitted
	cfsetospeed(&options, B9600);          // set output baud rate to 9600
	cfsetispeed(&options, B9600); 		  // set input baud rate to 9600
	tcsetattr(gpsFileNo, TCSANOW, &options);   // sets the parameters for the terminal ,changes occur immmediately
}

void gps::configureL80Module()
{
	mtkNMEA test;
	test.generateMTKMessage(PMTK_SYS_MSG, "Brendon");
	if (write(fileNo,test.getMTKMessage, max_CHARACTERS) < 0)
	{
		perror("Can't write to the gps device.");
		exit(1);
	}

	// code to read if initialization was done
}

void gps::readNMEAPacket()
{
	while (NMEA == "")
	{
		// might produce 0 or -1 if nothing is read NB!!!!!
		if (read(fileNo, NMEA, max_CHARACTERS) < 0)
		{
			perror("Can't read from the gps device.");
			exit(1);
		}
	}
}

void gps::parseNMEA()
{
	ostringstream part_Data_1;
			// GPS Read
	/*	while (check ==0)  
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
		close(GPS_UART_file);*/
}


gps::~gps()
{
	delete NMEA;
	close(gpsFileNo);
}