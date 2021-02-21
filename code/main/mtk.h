#pragma once

class mtkNMEA
{
public:
	mtkNMEA();
	void generateMTKMessage(char packetType[3], char packetData[50]);
	char getMTKMessage();
	~mtkNMEA();

protected:
	char *checksumCalc(int num);
	ostringstream mtk_nmea_message, temp;
	
};

