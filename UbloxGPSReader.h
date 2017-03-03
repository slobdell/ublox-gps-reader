/*
UBlox 7 & 8 GPS library

Lars Soltmann
*/

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <sys/socket.h>
#include <netdb.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>
#include "SPIdev.h"

class UbloxGPSReader {

private:
    std::string spiDeviceName;
	int submitCommand(unsigned char *command);
	void resetReadState(unsigned char dataArray[]);
    int disableNMEA_GLL();
    int disableNMEA_GGA();
    int disableNMEA_GSA();
    int disableNMEA_GSV();
    int disableNMEA_RMC();
    int disableNMEA_VTG();
    int enableNAV_POSLLH();
    int enableNAV_STATUS();
    int enableNAV_VELNED();
    int enableNAV_PVT();
    int setNavEngine();
    int setUpdateFrequency();
    int GNSSReset();
	bool messageIsAvailable;
    int readSequence;
    int spiTransferDataLength;

    void mutateGPSData(unsigned char dataArray[]);
    int messageType(unsigned char fromGPSData, unsigned char dataArray[]);
public:
    UbloxGPSReader(std::string name = "/dev/spidev0.0");
    int initialize();
    void indefiniteGPSIO();

    double gpsLat;
    double gpsLon;
    float gpsITOW;
    float gpsHeightAboveGeoid;
    float gpsHeightAboveMeanSeaLevel;
    float gpsVelocityNorth;
    float gpsVelocityEast;
    float gpsVelocityDown;
    float gps3D;
    float gps2D;
    float gpsCourse;
    int gpsStatus;
    float gpsPDOP;
    int gpsNumSatellites;
    float gpsVelocityAccuracy;
    float gpsAltitudeAccuracy;
};
