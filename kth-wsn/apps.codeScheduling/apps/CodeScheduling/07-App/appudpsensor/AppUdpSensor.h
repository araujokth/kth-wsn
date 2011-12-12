#ifndef APPUDPSENSOR_H
#define APPUDPSENSOR_H

enum {
   MEASUREMENT_PERIOD_MS  = 10000,
};

typedef nx_struct sensor_data_ht {
   nxle_uint8_t  type;
   nxle_uint8_t  length;
   nxle_uint16_t value;
} sensor_data_ht;

#endif
