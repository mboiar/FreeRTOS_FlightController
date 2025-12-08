#pragma once

#include <stdint.h>

typedef struct {
  uint64_t lon;
  uint64_t lat;
  uint64_t time_usec;
  float alt;
  float speed;
  float course;
  float pdop;
  float hdop;
  float vdop;
  uint16_t year;
  uint8_t day;
  uint8_t month;
} GPS_data;