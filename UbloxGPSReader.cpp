/*
UBlox 7 & 8 GPS library for use with Navio/Navio+

Code by: Lars Soltmann
Code contribution by: EMLID
*/

#include "UbloxGPSReader.h"

unsigned int SPI_SPEED_HZ = 5000000;

UbloxGPSReader::UbloxGPSReader(std::string name) : spiDeviceName(name) {
    spiTransferDataLength = 1;
    readSequence = 0;
	messageIsAvailable = false;
}


int UbloxGPSReader::submitCommand(unsigned char *command) {
	int length = (sizeof(command) / sizeof(*command));
    unsigned char receiveArray[length];
    return SPIdev::transfer(
		spiDeviceName.c_str(), 
		command, 
		receiveArray, 
		length, 
		SPI_SPEED_HZ	
	);
}
int UbloxGPSReader::enableNAV_POSLLH() {
    unsigned char gps_nav_posllh[] = {0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47};
	return submitCommand(gps_nav_posllh);
}

int UbloxGPSReader::enableNAV_VELNED() {
    unsigned char gps_nav_velned[] = {0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67};
	return submitCommand(gps_nav_velned);
}

int UbloxGPSReader::enableNAV_STATUS() {
    unsigned char gps_nav_status[] = {0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49};
	return submitCommand(gps_nav_status);
}

int UbloxGPSReader::enableNAV_PVT() {
    unsigned char gps_nav_status[] = {0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51};
	return submitCommand(gps_nav_status);
}

int UbloxGPSReader::setUpdateFrequency() {
    //unsigned char gps_rate[] = {0xb5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xFA, 0x00, 0x01, 0x00, 0x01, 0x00, 0x10, 0x96}; //4Hz
    unsigned char gps_rate[] = {0xb5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A}; //5Hz
	return submitCommand(gps_rate);
}

int UbloxGPSReader::setNavEngine() {
    	//Platform - 0 - portable
    	//UBlox8 ONLY - StaticHoldMaxDist-200
    //unsigned char gps_naveng[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x14, 0x5C};

    	//Platform - 6 - airborned <1g
    	//UBlox8 ONLY - StaticHoldMaxDist-200
    //unsigned char gps_naveng[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1A, 0x28};

    	//Platform - 7 - airborned <2g
    	//UBlox8 ONLY - StaticHoldMaxDist-200
    //unsigned char gps_naveng[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1B, 0x4A};

    	//Platform - 8 - airborned <4g
    	//UBlox8 ONLY - StaticHoldMaxDist-200
    //unsigned char gps_naveng[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x6C};


    	//Platform - 0 - portable
    	//UBlox7 or 8
    //unsigned char gps_naveng[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x1C};

    	//Platform - 6 - airborned <1g
    	//UBlox7 or 8
    //unsigned char gps_naveng[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x52, 0xE8};

    	//Platform - 7 - airborned <2g
    	//UBlox7 or 8
    unsigned char gps_naveng[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x53, 0x0A};

    	//Platform - 8 - airborned <4g
    	//UBlox7 or 8
    //unsigned char gps_naveng[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x54, 0x2C};

	return submitCommand(gps_naveng);
}

int UbloxGPSReader::disableNMEA_GLL() {
    unsigned char gps_nmea_gll[] = {0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
	return submitCommand(gps_nmea_gll);
}

int UbloxGPSReader::disableNMEA_GGA() {
    unsigned char gps_nmea_gga[] = {0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24};
	return submitCommand(gps_nmea_gga);
}

int UbloxGPSReader::disableNMEA_GSA() {
    unsigned char gps_nmea_gsa[] = {0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
	return submitCommand(gps_nmea_gsa);
}

int UbloxGPSReader::disableNMEA_GSV() {
    unsigned char gps_nmea_gsv[] = {0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
	return submitCommand(gps_nmea_gsv);
}

int UbloxGPSReader::disableNMEA_RMC() {
    unsigned char gps_nmea_rmc[] = {0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40};
	return submitCommand(gps_nmea_rmc);
}

int UbloxGPSReader::disableNMEA_VTG() {
    unsigned char gps_nmea_vtg[] = {0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47};
	return submitCommand(gps_nmea_vtg);
}

int UbloxGPSReader::GNSSReset() {
    unsigned char gps_gnss_rst[] = {0xb5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x02, 0x00, 0x10, 0x68};
	return submitCommand(gps_gnss_rst);
}

int UbloxGPSReader::initialize()
{
	// Disable all NMEA messages in order to cut down on I/O
    if (
		disableNMEA_GLL() < 0 ||
		disableNMEA_GGA() < 0 ||
		disableNMEA_GSA() < 0 ||
		disableNMEA_GSV() < 0 ||
		disableNMEA_RMC() < 0 ||
		disableNMEA_VTG() < 0 ||
		enableNAV_PVT() < 0 ||
		setNavEngine() < 0 ||
		setUpdateFrequency() < 0
	) {
		// initialization error
		return 1;
	}
	return 0;
}


void UbloxGPSReader::indefiniteGPSIO() {
    unsigned char toGPSData = 0x00;
	unsigned char fromGPSData = 0x00;

	// 100 bytes is the size of the PVT message (92+8)
    unsigned char dataArray[100] = {0x00}; 

    while (true) {
		for (int i = 0; i <= spiTransferDataLength-1; i++) {
				SPIdev::transfer(spiDeviceName.c_str(), &toGPSData, &fromGPSData, 1, SPI_SPEED_HZ);
				dataArray[i+4]=fromGPSData;
		}

		if(!messageIsAvailable) {
			int messageID = messageType(fromGPSData, dataArray);
			switch(messageID){
			case 0x03:
				spiTransferDataLength = 20;
				messageIsAvailable = true;
				break;
			case 0x02:
				spiTransferDataLength = 32;
				messageIsAvailable = true;
				break;
			case 0x12:
				spiTransferDataLength = 40;
				messageIsAvailable = true;
				break;
			case 0x07:
				spiTransferDataLength = 96;
				messageIsAvailable = true;
				break;
			default:
				break;
			}
		} else if (messageIsAvailable) {
			mutateGPSData(dataArray);
			resetReadState(dataArray);
		}
		if(!messageIsAvailable) {
			usleep(100);
		}

    }
}

int UbloxGPSReader::messageType(unsigned char fromGPSData, unsigned char dataArray[100]){
	if (fromGPSData == 0xB5 && readSequence == 0){
		readSequence++;
		dataArray[0] = fromGPSData;
		return 0;
	} else if (fromGPSData == 0x62 && readSequence == 1){
		readSequence++;
		dataArray[1] = fromGPSData;
		return 0;
	} else if (readSequence == 2){
		readSequence++;
		dataArray[2] = fromGPSData;
		return 0;
	} else if (readSequence == 3){
		readSequence = 0;
		dataArray[3] = fromGPSData;
		return fromGPSData;
	} else {
		readSequence = 0;
		return 0;
	}
}


void UbloxGPSReader::resetReadState(unsigned char dataArray[100]) {
	messageIsAvailable = false;
	spiTransferDataLength = 1;
	dataArray[0] = 0x00;
	dataArray[1] = 0x00;
	dataArray[2] = 0x00;
	dataArray[3] = 0x00;
	readSequence = 0;
}

void UbloxGPSReader::mutateGPSData(unsigned char dataArray[100]){
	switch(dataArray[3]) {
		case 0x02:
			gpsLat = (double)(((dataArray[17]) << 24) | ((dataArray[16]) << 16) | ((dataArray[15]) << 8) | (dataArray[14]))/ (double)10000000; //deg
			gpsLon = (double)(((dataArray[13]) << 24) | ((dataArray[12]) << 16) | ((dataArray[11]) << 8) | (dataArray[10]))/ (double)10000000;  //deg
			gpsHeightAboveGeoid = (float)(((dataArray[21]) << 24) | ((dataArray[20]) << 16) | ((dataArray[19]) << 8) | (dataArray[18])) * 0.001;  //mm to m
			gpsHeightAboveMeanSeaLevel = (float)(((dataArray[25]) << 24) | ((dataArray[24]) << 16) | ((dataArray[23]) << 8) | (dataArray[22])) * 0.001;  //mm to m
			break;

		case 0x03:
			gpsITOW = (float)(((dataArray[9]) << 24) | ((dataArray[8]) << 16) | ((dataArray[7]) << 8) | (dataArray[6]))/ (float)1000;  //sec
			gpsStatus = (int)dataArray[10];
			break;

		case 0x12:
			gpsVelocityNorth = (float)(((dataArray[13]) << 24) | ((dataArray[12]) << 16) | ((dataArray[11]) << 8) | (dataArray[10])) * 0.01;  // cm/s to m/s
			gpsVelocityEast = (float)(((dataArray[17]) << 24) | ((dataArray[16]) << 16) | ((dataArray[15]) << 8) | (dataArray[14])) * 0.01;  // cm/s to m/s
			gpsVelocityDown = (float)(((dataArray[21]) << 24) | ((dataArray[20]) << 16) | ((dataArray[19]) << 8) | (dataArray[18])) * 0.01;  // cm/s to m/s
			gps3D = (float)(((dataArray[25]) << 24) | ((dataArray[24]) << 16) | ((dataArray[23]) << 8) | (dataArray[22])) * 0.01;  // cm/s to m/s
			gps2D = (float)(((dataArray[29]) << 24) | ((dataArray[28]) << 16) | ((dataArray[27]) << 8) | (dataArray[26])) * 0.01;  // m/s to m/s
			gpsCourse = (float)(((dataArray[33]) << 24) | ((dataArray[32]) << 16) | ((dataArray[31]) << 8) | (dataArray[30]))/ (float)100000;  //deg
			break;

		case 0x07: 
			gpsLat = (double)(((dataArray[37]) << 24) | ((dataArray[36]) << 16) | ((dataArray[35]) << 8) | (dataArray[34]))/ (double)10000000; //deg
			gpsLon = (double)(((dataArray[33]) << 24) | ((dataArray[32]) << 16) | ((dataArray[31]) << 8) | (dataArray[30]))/ (double)10000000;  //deg
			gpsHeightAboveGeoid = (float)(((dataArray[41]) << 24) | ((dataArray[40]) << 16) | ((dataArray[39]) << 8) | (dataArray[38])) * 0.001;  //mm to m
			gpsHeightAboveMeanSeaLevel = (float)(((dataArray[45]) << 24) | ((dataArray[44]) << 16) | ((dataArray[43]) << 8) | (dataArray[42])) * 0.001;  //mm to m
			gpsStatus = (int)dataArray[26];
			gpsVelocityNorth = (float)(((dataArray[57]) << 24) | ((dataArray[56]) << 16) | ((dataArray[55]) << 8) | (dataArray[54])) * 0.001;  // mm/s to m/s
			gpsVelocityEast = (float)(((dataArray[61]) << 24) | ((dataArray[60]) << 16) | ((dataArray[59]) << 8) | (dataArray[58])) * 0.001;  // mm/s to m/s
			gpsVelocityDown = (float)(((dataArray[65]) << 24) | ((dataArray[64]) << 16) | ((dataArray[63]) << 8) | (dataArray[62])) * 0.001;  // mm/s to m/s
			gpsCourse = (float)(((dataArray[73]) << 24) | ((dataArray[72]) << 16) | ((dataArray[71]) << 8) | (dataArray[70]))/ (float)100000;  //deg
			gpsNumSatellites = (int)dataArray[29];
			gpsPDOP = (float)(((dataArray[83]) << 8) | (dataArray[82])) * 0.01; // no units
			gpsVelocityAccuracy = (float)(((dataArray[77]) << 24) | ((dataArray[76]) << 16) | ((dataArray[75]) << 8) | (dataArray[74])) * 0.001;  // mm/s to m/s
			gpsAltitudeAccuracy = (float)(((dataArray[53]) << 24) | ((dataArray[52]) << 16) | ((dataArray[51]) << 8) | (dataArray[50])) * 0.001; // mm to m
			break;
		}

		/*
		printf("Lat | Lon:  %.6f %.6f\n",gpsLat,gpsLon);
      	printf("NED vel:    %.2f %.2f %.2f\n",gpsVelocityNorth,gpsVelocityEast,gpsVelocityDown);
       	printf("h | hmsl:   %.2f %.2f\n",gpsHeightAboveGeoid,gpsHeightAboveMeanSeaLevel);
        printf("Course:     %.2f\n",gpsCourse);
        printf("Status:     %d\n",gpsStatus);
        printf("Satellites: %d\n",gpsNumSatellites);
		printf("PDOP:	    %.2f\n\n",gpsPDOP);
		*/
}
