#ifndef gps_h
#define gps_h
#include "stm32f4xx_hal.h"
#include "drv_serial.h"
#include <stdbool.h>
void gpsThread (void);
static void gpsNewData(uint8_t c);
static bool gpsNewFrame(uint8_t c);
static bool gpsNewFrameNMEA(char c);
uint32_t GPS_coord_to_degrees(char *s);

#endif
