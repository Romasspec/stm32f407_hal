#include "gps.h"
extern serialPort_t serial;
extern int32_t GPS_coord[2];
extern uint8_t GPS_numSat;
extern uint16_t GPS_altitude, GPS_speed;
extern uint16_t GPS_ground_course;

void gpsThread (void)
{
	uint8_t tempDataGPS;
	// read out available GPS bytes
	while(serialTotalBytesWaiting(&serial))	{
		tempDataGPS = serialRead(&serial);
		gpsNewData(tempDataGPS);
	}
}

static void gpsNewData(uint8_t c)
{
	if (gpsNewFrame(c)) {
        // new data received and parsed, we're in business
		
	}
}

static bool gpsNewFrame(uint8_t c)
{
	return gpsNewFrameNMEA(c);
}

#define NO_FRAME   0
#define FRAME_GGA  1
#define FRAME_RMC  2

typedef struct gpsMessage {
	int32_t latitude;
	int32_t longitude;
	uint8_t numSat;
	uint16_t altitude;
	uint16_t speed;
	uint16_t ground_course;
} gpsMessage_t;

bool GPS_FIX;

static bool gpsNewFrameNMEA(char c)
{
	uint8_t frameOK = 0;
	static uint8_t param = 0, offset = 0, parity = 0;
	static char string[15];
	static uint8_t checksum_param, gps_frame = NO_FRAME;
	static gpsMessage_t gps_msg;
	
	switch (c) {
		case	'$':
			param = 0;
			offset = 0;
			parity = 0;			
			break;
		
		case	',':			
		case	'*':
			string[offset] = 0;
            if (param == 0) {       // frame identification
                gps_frame = NO_FRAME;
                if (string[0] == 'G' && string[1] == 'N' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') {
                    gps_frame = FRAME_GGA;
					//HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
				}
                if (string[0] == 'G' && string[1] == 'P' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') {
                    gps_frame = FRAME_RMC;
					//HAL_GPIO_TogglePin(LD5_GPIO_Port, LD5_Pin);
				}
            }
			
			switch (gps_frame) {
                case FRAME_GGA:        // ************* GPGGA FRAME parsing
                    switch (param) {
                        case 2:
                            gps_msg.latitude = GPS_coord_to_degrees(string);
                            break;
                        case 3:
                            if (string[0] == 'S')
                                gps_msg.latitude *= -1;
                            break;
                        case 4:
                            gps_msg.longitude = GPS_coord_to_degrees(string);
                            break;
                        case 5:
                            if (string[0] == 'W')
                                gps_msg.longitude *= -1;
                            break;
                        case 6:
                            GPS_FIX = string[0] > '0';
                            break;
                        case 7:
                            gps_msg.numSat = grab_fields(string, 0);
                            break;
                        case 9:
                            gps_msg.altitude = grab_fields(string, 0);     // altitude in meters added by Mis
                            break;
                    }
                    break;
			
				case FRAME_RMC:        // ************* GPRMC FRAME parsing
                    switch (param) {
                        case 7:
                            gps_msg.speed = ((grab_fields(string, 1) * 5144L) / 1000L);    // speed in cm/s added by Mis
                            break;
                        case 8:
                            gps_msg.ground_course = (grab_fields(string, 1));      // ground course deg * 10
                            break;
                    }
                    break;
				}
				param++;
				offset = 0;
				if (c == '*')
					checksum_param = 1;
				else
					parity ^= c;
				break;
				
		case '\r':
        case '\n':
            if (checksum_param) {   //parity checksum
                uint8_t checksum = 16 * ((string[0] >= 'A') ? string[0] - 'A' + 10 : string[0] - '0') + ((string[1] >= 'A') ? string[1] - 'A' + 10 : string[1] - '0');
                if (checksum == parity) {
                    switch (gps_frame) {
                        case FRAME_GGA:
                            frameOK = 1;
                            if (GPS_FIX) {
                                GPS_coord[LAT] = gps_msg.latitude;
                                GPS_coord[LON] = gps_msg.longitude;
                                GPS_numSat = gps_msg.numSat;
                                GPS_altitude = gps_msg.altitude;
//                                if (!sensors(SENSOR_BARO) && f.FIXED_WING)
//                                    EstAlt = (GPS_altitude - GPS_home[ALT]) * 100;    // Use values Based on GPS
                            }
                            break;

                        case FRAME_RMC:
                            GPS_speed = gps_msg.speed;
                            GPS_ground_course = gps_msg.ground_course;
//                            if (!sensors(SENSOR_MAG) && GPS_speed > 100) {
//                                GPS_ground_course = wrap_18000(GPS_ground_course * 10) / 10;
//                                heading = GPS_ground_course / 10;    // Use values Based on GPS if we are moving.
//                            }
                            break;
                    }
                }
            }
            checksum_param = 0;
					
			break;
		
		default:
            if (offset < 15)
                string[offset++] = c;
            if (!checksum_param)
                parity ^= c;
            break;
	}
	
	return frameOK;
}

#define DIGIT_TO_VAL(_x)    (_x - '0')
uint32_t GPS_coord_to_degrees(char *s)
{
    char *p, *q;
    uint8_t deg = 0, min = 0;
    unsigned int frac_min = 0;
    int i;

    // scan for decimal point or end of field
    for (p = s; isdigit((unsigned char)*p); p++) {
        if (p >= s + 15)
            return 0; // stop potential fail
    }
    q = s;

    // convert degrees
    while ((p - q) > 2) {
        if (deg)
            deg *= 10;
        deg += DIGIT_TO_VAL(*q++);
    }
    // convert minutes
    while (p > q) {
        if (min)
            min *= 10;
        min += DIGIT_TO_VAL(*q++);
    }
    // convert fractional minutes
    // expect up to four digits, result is in
    // ten-thousandths of a minute
    if (*p == '.') {
        q = p + 1;
        for (i = 0; i < 4; i++) {
            frac_min *= 10;
            if (isdigit((unsigned char)*q))
                frac_min += *q++ - '0';
        }
    }
    return deg * 10000000UL + (min * 1000000UL + frac_min * 100UL) / 6;
}

// helper functions
static uint32_t grab_fields(char *src, uint8_t mult)
{
    // convert string to uint32
    uint32_t i;
    uint32_t tmp = 0;
    for (i = 0; src[i] != 0; i++) {
        if (src[i] == '.') {
            i++;
            if (mult == 0)
                break;
            else
                src[i + mult] = 0;
        }
        tmp *= 10;
        if (src[i] >= '0' && src[i] <= '9')
            tmp += src[i] - '0';
        if (i >= 15)
            return 0; // out of bounds
    }
    return tmp;
}
