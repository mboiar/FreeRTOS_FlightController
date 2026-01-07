#pragma once

#include <math.h>
#include <stdint.h>

#define GPS_A 6378137.0
#define GPS_F (1.0 / 298.257223563)
#define GPS_E2 (GPS_F * (2 - GPS_F))

typedef struct {
  int32_t lon;
  int32_t lat;
  uint64_t time_usec;
  float alt;
  float ve;
  float vn;
  float pdop;
  float hdop;
  float vdop;
  uint16_t year;
  uint8_t day;
  uint8_t month;
} GPS_data;

static inline void lla_to_ned(int32_t lat, int32_t lon, float alt, int32_t lat0,
                              int32_t lon0, float alt0, float ned[3]) {
  // approximation

  ned[0] =
      (float)((double)(lat - lat0) / 1e7 * M_PI / 180.0 * GPS_A * (1 - GPS_E2) *
              pow(1 - GPS_E2 * sin((double)lat0 / 1e7 * M_PI / 180.0) *
                          sin((double)lat0 / 1e7 * M_PI / 180.0),
                  -1.5));
  ned[1] = (float)((double)(lon - lon0) / 1e7 * M_PI / 180.0 * GPS_A /
                   sqrt(1 - GPS_E2 * sin((double)lat0 / 1e7 * M_PI / 180.0) *
                                sin((double)lat0 / 1e7 * M_PI / 180.0)));
  ned[2] = -alt - alt0;
}
