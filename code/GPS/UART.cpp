/*Code used to send data between the STM32F051C6 and the pocketbeagle
 *as well as for the GPS reciever*/


#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>   // using the termios.h library

using namespace std;
char STM_file[128];
char NMEA[160];
char platformtime[12];
char latitude[13];
char longitude[14];
char speed[5];
char date[7];
char altitude[9];
int check;

int GPS_UART_setup()
{
	int file, count;


	if ((file = open("/dev/ttyO0", O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
		perror("UART: Failed to open the file.\n");
		return -1;
	}
	struct termios options;               //The termios structure is vital
	tcgetattr(file, &options);              //Sets the parameters associated with file

	// Set up the communications options:	
	//   9600 baud, 8-bit, enable receiver, no modem control lines
	options.c_cflag =  CS8 | CREAD | CLOCAL;
	options.c_iflag = IGNPAR | ICRNL;      //ignore partity errors, CR -> newline
	tcflush(file, TCIFLUSH);               //discard file information not transmitted
	cfsetospeed(&options, B9600);          // set output baud rate to 9600
	cfsetispeed(&options, B9600); 		  // set input baud rate to 9600
	tcsetattr(file, TCSANOW, &options);    // sets the parameters for the terminal ,changes occur immmediately
	
	return file;
}
int STM_UART_setup()
{
	int file, count;


	if ((file = open("/dev/ttyO4", O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
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
}

int main() {
	// setup to write to the STM32F051C6
	//int STM_UART_file = STM_UART_setup();
	//STM_file[0] = 'h';
	//write(STM_UART_file, STM_file, 6);
	
	//continuously read from the gps
	int GPS_UART_file = GPS_UART_setup();  // setup to read from the GPS receiver
	while (1)
	{
		check = 0;
		while (check == 0)  // While loop separates the GPRMC sentence into its sections that can be used
			{
				read(GPS_UART_file, NMEA, 255);
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
									date[x] = NMEA[i];
									x++;
								}
								check = 1;
							}
						}
				}
			}
		
		check = 0;
		while (check == 0) //While loop separates the GPGGA sentence into its sections that can be used
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
		cout << platformtime << ',' << date << ',' << altitude << ',' << latitude << ',' << longitude << ',' << speed << endl; // outputs the data read
	}
	close(GPS_UART_file);
	return 0;
}

