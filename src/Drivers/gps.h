#pragma once

#include <math.h>
#include <stdint.h>

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

static inline void lla_to_ned(uint64_t lat, uint64_t lon, float alt,
                              uint64_t lat0, uint64_t lon0, float alt0,
                              float ned[3]) {
  float a = 6378137.0f;
  float f = 1.0 / 298.257223563;
  float e2 = f * (2 - f);
  ned[0] = (float)((lat - lat0) * a * (1 - e2) /
                   pow(1 - e2 * sin(lat0) * sin(lat0), 1.5));
  ned[1] = (float)((lon - lon0) * a / sqrt(1 - e2 * sin(lat0) * sin(lat0)));
  ned[2] = alt0 - alt;
}