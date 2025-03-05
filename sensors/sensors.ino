#include <Wire.h>
#include <SPI.h>

#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
// NOTE: patched `beginSPI` to wait 200ms instead of 2ms when resetting
// (also note that beginSPI returns false due to unexpected repsonse in shtpData[0], but works fine; ignore beginSPI return value)
//#include "SparkFun_BNO080_Arduino_Library.h"
#include <Adafruit_ICM20948.h>
// NOTE: must be patched so serial is enabled for ESP32 (substitute for ESP8266 in .h and .cpp)
/include <Adafruit_GPS.h>


#include "pins.h"
#include "util.h"

// Parameters
#define SPI_FREQ 1000000
#ifdef HELIOS
#define LORA_FREQ 927.5
#endif
#ifdef HERMES
#define LORA_FREQ 924.5
#endif

// General SPI declaration
SPIClass spi(FSPI);

// Serial for GPS
// SoftwareSerial gpsSerial(PIN_GPS_TX, PIN_GPS_RX);

// Sensor variables
Adafruit_LPS22 alt;
Adafruit_GPS gps(&Serial1);
Adafruit_ICM20948 imu;

void setup_sensors() {
  log("Initializing sensors...");
  spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI);
  setup_imu();
  setup_altimeter();
  setup_gps();
  log("Sensors initialized!");
}

void setup_imu() {
  Serial.println("Setting up IMU...");
  //
  // TODO: can't lock, as beginSPI needs to delay; should be fine not too, but should look for other delays in critical sections (or just stop using spinlock and fix LoRa ISR)
  // spiLock(portMAX_DELAY);
  imu.begin_I2C();
  imu.setAccelRange(ICM20948_ACCEL_RANGE_16_G); //figure out what value should be
  imu.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS); //figure out what value should be
  // spiUnlock();
}

void setup_altimeter() {
  spiLock(portMAX_DELAY);
  altimeter_data.connected = false;
  Serial.println("Setting up altimeter...");
  if (!alt.begin_SPI(PIN_ALT_CS, &spi)) {
    Serial.println("WARN: Failed to find LPS22 chip");
    spiUnlock();
    return;
  }
  log("LPS22 found, checking settings...");

  alt.setDataRate(LPS22_RATE_25_HZ);
  log("LPS22 data rate set to: ");
  switch (alt.getDataRate()) {
    case LPS22_RATE_ONE_SHOT: log("One Shot / Power Down (ERROR)");
                              altimeter_data.connected = false;
                              return;
    case LPS22_RATE_1_HZ: log("1 Hz"); break;
    case LPS22_RATE_10_HZ: log("10 Hz"); break;
    case LPS22_RATE_25_HZ: log("25 Hz"); break;
    case LPS22_RATE_50_HZ: log("50 Hz"); break;
    case LPS22_RATE_75_HZ: log("75 Hz"); break;
  }
  altimeter_data.connected = true;
  spiUnlock();
}

void setup_gps() {
  Serial.println("Setting up GPS....");
  Serial1.begin(9600, SERIAL_8N1, PIN_GPS_TX, PIN_GPS_RX);
  gps.begin(9600);
  // gps.sendCommand(PMTK_SET_NMEA_OUTPUT_GGAONLY); // only actual GPS data
  gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // gps.sendCommand(PGCMD_ANTENNA);
  gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  delay(500);
  // Ask for firmware version
  Serial1.println(PMTK_Q_RELEASE);
  Serial.println("GPS Setup done!");
}

#ifndef MOCK_SENSOR_DATA
void main_sensors() {
  main_imu();
  main_altimeter();
  main_gps();
}
#endif

//NOTE: need to change locks to one for I2C (maybe, idk, should look into it)
void main_imu() {
  if (!spiLock(SENSOR_MAX_MUTEX_DELAY)) {
    log("ERROR: failed to obtain SPI mutex (IMU)");
    return;
  }
  if (imu.dataAvailable()) {
    spiUnlock();
    // Update sensor data (getAccelX etc. don't use SPI, all done in dataAvailable)
    if (xSemaphoreTake(sensor_data_mutex, SENSOR_MAX_MUTEX_DELAY)) {
      uint8_t accuracy; // discarded
      sensors_event_t accel;
      sensors_event_t gyro;
      imu.getEvent(&accel, &gyro, &temp, &mag);
      xSemaphoreGive(sensor_data_mutex);
    } else {
      log("ERROR: failed to obtain mutex for IMU data");
    }
  } else {
    spiUnlock();
  }
}

void main_altimeter() {
  PERIOD(10)
  if (!spiLock(SENSOR_MAX_MUTEX_DELAY)) {
    log("ERROR: failed to obtain SPI mutex (altimeter)");
    return;
  }
  sensors_event_t temp;
  sensors_event_t pressure;
  alt.getEvent(&pressure, &temp);// get pressure
  spiUnlock();
  if (!xSemaphoreTake(sensor_data_mutex, SENSOR_MAX_MUTEX_DELAY)) {
    log("ERROR: failed to obtain mutex for GPS data");
    return;
  }
  altimeter_data.temp = temp.temperature;
  altimeter_data.pressure = pressure.pressure;
  xSemaphoreGive(sensor_data_mutex);
}

void main_gps() {
  // No `PERIOD`, must read as often as possible (100 Hz like frame)
  gps.read();
  if (!gps.newNMEAreceived()) {
    return;  // if we haven't seen a new packet, return
  }
  // parse packet, which sets the newNMEAreceived() flag to false
  if (!gps.parse(gps.lastNMEA())) {
    // Log if parse failed
    log("Failed to parse GPS packet:");
    log(gps.lastNMEA());
    return;
  }
  // Serial.println(gps.lastNMEA());
  // Actually update stats if we received new data
  if (!xSemaphoreTake(sensor_data_mutex, SENSOR_MAX_MUTEX_DELAY)) {
    log("ERROR: failed to obtain mutex for GPS data");
    return;
  }
  gps_data.fix = gps.fix;
  gps_data.fix_quality = gps.fixquality;
  gps_data.satellites = gps.satellites;
  gps_data.antenna = gps.antenna;
  gps_data.hour = gps.hour;
  gps_data.minute = gps.minute;
  gps_data.second = gps.seconds;
  gps_data.day = gps.day;
  gps_data.month = gps.month;
  gps_data.year = gps.year;
  gps_data.latitude = gps.latitude_fixed;
  gps_data.longitude = gps.longitude_fixed;
  if (gps_data.fix) {
    gps_data.fixLat = gps_data.latitude;
    gps_data.fixLong = gps_data.longitude;
  }
  xSemaphoreGive(sensor_data_mutex);
}
