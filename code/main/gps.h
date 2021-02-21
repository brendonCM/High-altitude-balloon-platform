
#ifndef GPS_H_
#define GPS_H_

// Different packet types for various commands
#define PMTK_SYS_MSG "010"
#define PMTK_TXT_MSG "011"
#define PMTK_ACK     "001"
#define PMTK_CMD_HOT_START "101"
#define PMTK_CMD_WARM_START "102"
#define PMTK_CMD_COLD_START "103"
#define PMTK_CMD_FULL_COLD_START "104"
#define PMTK_CMD_STANDBY_MODE "161"
#define PMTK_LOCUS_QUERY_STATUS "183"
#define PMTK_LOCUS_ERASE_FLASH "184"
#define PMTK_LOCUS_STOP_LOGGER "185"
#define PMTK_Q_LOCUS_DATA "622"
#define PMTK_SET_POS_FIX "220"
#define PMTK_SET_AL_DEE_CFG "223"
#define PMTK_SET_PERIODIC_MODE "225"
#define PMTK_SET_NMEA_BAUDRATE "251"
#define PMTK_SET_AIC_ENABLED "286"
#define PMTK_API_SET_FIX_CTL "300"
#define PMTK_API_SET_DGPS_MODE "301"
#define PMTK_API_SET_SBAS_ENABLED "313"
#define PMTK_API_SET_NMEA_OUTPUT "314"
#define PMTK_API_SET_HDOP_THRESHOLD "356"
#define PMTK_API_SET_STATIC_NAV_THD "386"
#define PMTK_API_Q_FIX_CTL "400"
#define PMTK_API_Q_DGPS_MODE "401"
#define PMTK_API_Q_SBAS_ENABLED "413"
#define PMTK_API_Q_NMEA_OUTPUT "414"
#define PMTK_Q_RELEASE "605"
#define PMTK_DT_FIX_CTL "500"
#define PMTK_DT_DGPS_MODE "501"
#define PMTK_DT_SBAS_ENABLED "513"
#define PMTK_DT_NMEA_OUTPUT "514"
#define PMTK_DT_RELEASE "705"
#define PMTK_EASY_ENABLE "869"
#define PMTK_PMTKLSC_STN_OUTPUT "875"

class gps
{
public:
	gps();
	void getGPSData();
	void getLatAndLon();
	void getPlatformTime();
	void getSpeed();
	void getDate();
	void getAltitude();
	~gps();

protected:
	char *NMEA;
	int gpsFileNo;
	void setupUART();
	void configureL80Module();
	void readNMEAPacket();
	void parseNMEA();
	
};


#endif /*GPS_H_*/
