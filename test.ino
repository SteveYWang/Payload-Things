#include "util.h"

// Test control logic with mock sensor data
#ifdef MOCK_SENSOR_DATA

enum {
  MOCK_UNCALLED,
  MOCK_ON_PAD,
  MOCK_ASCENDING,
  MOCK_DESCENDING,
  MOCK_GROUND,
} mock_status = MOCK_UNCALLED;
int64_t switch_time = 0; // Time of last state switch

void main_sensors() {
  // Get sensor lock
  if (!xSemaphoreTake(sensor_data_mutex, SENSOR_MAX_MUTEX_DELAY)) {
    log("ERROR: failed to obtain sensor lock for mock data");
    return;
  }

  // Time since switch
  long t = millis() - switch_time;

  // Set data based on stats
  switch (mock_status) {
  case MOCK_UNCALLED:
    imu_data = {
      .accelX = -10.,
      .accelY = 0.,
      .accelZ = 0.,
      .gyroX = 0.,
      .gyroY = 0.,
      .gyroZ = 0.,
    };
    gps_data = {
      .fix = true, .fix_quality = 1, .satellites = 4, .antenna = 0,
      .hour = 1, .minute = 1, .second = 1, .day = 1, .month = 1, .year = 11,
      .latitude = 360000000, .longitude = -790000000, .fixLat = 360000000, .fixLong = -790000000
    };
    altimeter_data = {
      .connected = true,
      .temp = 30.0,
      .pressure = 1013.25,
    };
    mock_status = MOCK_ON_PAD;
    switch_time = millis();
    break;
  case MOCK_ON_PAD:
    // Wait 15 seconds to launch
    if (millis() - switch_time >= 15000) {
      mock_status = MOCK_ASCENDING;
      switch_time = millis();
      log("MOCK: launching");
    }
    break;
  case MOCK_ASCENDING:
    // Ramp pressure down from 1000 to 700 over 30 seconds
    if (t > 30000) {
      mock_status = MOCK_DESCENDING;
      switch_time = millis();
    }
    altimeter_data.pressure = 1000.0 - ((float) t) / 100.0;
    break;
  case MOCK_DESCENDING:
    // Ramp pressure up from 700 to 1000 over 430 seconds
    if (t > 430000) {
      mock_status = MOCK_GROUND;
      switch_time = millis();
    }
    altimeter_data.pressure = 700.0 + ((float) t) / 200.0;
    break;
  case MOCK_GROUND:
    break;
  }
  xSemaphoreGive(sensor_data_mutex);
}


#endif