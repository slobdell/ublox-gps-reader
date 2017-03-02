#include <iostream>
#include <vector>
#include <sys/time.h>
#include "UbloxGPSReader.h"

#define NULL_SENTINEL_VALUE -1000
#define EXPECTED_READ_FREQUENCY 4
#define STANDARD_DEVIATION_SAMPLES EXPECTED_READ_FREQUENCY * 10
#define TRY_READ_FREQUENCY 16
#define ONE_SECOND_AS_MICROSECONDS 1000000.0
#define SLEEP_MICROSECONDS ONE_SECOND_AS_MICROSECONDS / TRY_READ_FREQUENCY


// 110m: 0.001
// 11m: 0.0001
// 1.1m: 0.00001

int main() {


    UbloxGPSReader gps;
	gps.initialize();
	gps.indefiniteGPSIO();
	return 0;
}
