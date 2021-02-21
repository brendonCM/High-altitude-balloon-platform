/* Used to control and configure of the L80 GPS module.
   Create commands than can be sent to the GPS module to
   get various different configurations.*/

#include "mtk.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <string.h>
using namespace std;
#define TALKER_ID_NMEA "PMTK"



mtkNMEA::mtkNMEA()
{
}

void mtkNMEA::generateMTKMessage(char packetType[3], char packetData[50])
{
	temp << TALKER_ID_NMEA << packetType << "," << packetData << " ";
	int length = strlen(temp.str().c_str());

	int Total = 0;

	for (int x = 0; x < length - 1; x++)
	{
		Total ^= temp.str().c_str()[x]; // The ^ performs the bitwise XOR
	}

	mtk_nmea_message << "$" << TALKER_ID_NMEA << packetType << "," << packetData << "*" << checksumCalc(Total)
		<< "\r" << "\n";
}

char mtkNMEA::getMTKMessage()
{
	return mtk_nmea_message.str().c_str();
}

mtkNMEA::~mtkNMEA()
{
}

char *mtkNMEA::checksumCalc(int num)
{
	static char temp[16], hexadecimalValue[16];
	int x = 0, z = 0;
	int r;

	while (num != 0)
	{
		r = num % 16;
		temp[x++] = (num % 16 < 10) ? (48 + r) : (55 + r);
		num /= 16;
	}

	for (int y = x; y >= 0; y--)
	{
		hexadecimalValue[z++] = temp[y - 1];
	}
	return(hexadecimalValue);
}


