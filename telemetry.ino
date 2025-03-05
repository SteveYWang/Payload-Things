#include <RH_RF95.h>

#include "util.h"

// LoRa variables
RHLoRaSPI rh_spi(&spi, SPI_FREQ);
RH_RF95 lora(5, 1, rh_spi);

// Setup telemetry. Must be run after `setup_sensors()` for SPI
void setup_telemetry() {
  log("setting up telemetry...");
  bool success = setup_lora();
  if (!success) log("LoRa setup failed");
  else log("telemetry setup done!");
}

bool setup_lora() {
  if (!lora.init()) {
    return false;
  }
  if (!lora.setFrequency(LORA_FREQ)) {
    return false;
  }
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  lora.setTxPower(23, false);
  return true;
}

// Send LoRa packet with type & serialization, returning `false` on failure (timeout)
bool transmit(const String &type, const String &message) {
#define PACKET_HEADER_LEN 3
#define MAX_PACKET_LEN (RH_RF95_MAX_MESSAGE_LEN - PACKET_HEADER_LEN)

  static uint8_t transmission_number = 0;
  transmission_number += 1;

  // Construct message
  String full_message = type + " " + message;
  // Serial.printf("FULL MESSAGE?: %s\n", full_message.c_str());
  uint8_t packet_number = 0;
  int idx = 0;  // Start of next packet
  int needed_packets = (full_message.length() + MAX_PACKET_LEN - 1) / MAX_PACKET_LEN;
  static uint8_t packet[RH_RF95_MAX_MESSAGE_LEN + 1];  // +1 for getBytes null terminator
  while (idx < full_message.length()) {
    // Create packet with string starting at idx
    packet[0] = (uint8_t)transmission_number;
    packet[1] = (uint8_t)packet_number;
    packet[2] = (uint8_t)needed_packets;
    full_message.getBytes(packet + PACKET_HEADER_LEN, MAX_PACKET_LEN + 1, idx);  // copies null terminator as well, so +1

    // Find length of actual packet and log to Serial
    int packet_len = min((unsigned)MAX_PACKET_LEN, full_message.length() - idx) + PACKET_HEADER_LEN;
    Serial.printf("PACKET: %s\n", (char *)(packet + PACKET_HEADER_LEN));

    // Send packet, acquiring mutex
    if (!lora.waitPacketSent(5000)) {
      Serial.println("Packet timeout");
      return false;
    }
    lora.send(packet, packet_len);

    // Increment
    idx += MAX_PACKET_LEN;
    packet_number += 1;
  }
  return true;
}

// Track messages logged since the last transmission
const int LOG_MAX = 2047;
char current_log[LOG_MAX + 1];
int current_log_len = 0;
// short timeout on mutex b/c this is called on critical paths
#define LOG_MAX_MUTEX_DELAY (20 / portTICK_PERIOD_MS)
// Logs a message for the next LoRa transmission
void log(const char *s) {
  Serial.printf("LOG: %s\n", s);
  if (xSemaphoreTake(log_mutex, LOG_MAX_MUTEX_DELAY)) {
    // Serial.println("LOGMUTEX TAKE (log)");
    if (current_log_len == LOG_MAX) {
      // Reached max size
      xSemaphoreGive(log_mutex);
      return;
    }
    // Append newline
    current_log[current_log_len++] = '\n';
    // Write string to log, stopping at NULL or full
    while (current_log_len < LOG_MAX && (*s)) {
      current_log[current_log_len++] = *(s++);
    }
    // Serial.println("LOGMUTEX GIVE (log)");
    xSemaphoreGive(log_mutex);
  } else {
    Serial.println("ERROR: failed to get log mutex");
  }
}

void main_telemetry() {
  Serial.println("main_telemetry()...");
  static char data_string[512];

  // Send log, and error if our log overflowed
  if (xSemaphoreTake(log_mutex, portMAX_DELAY)) {
    // Serial.println("LOGMUTEX TAKE (main_tele)");
    // Take data & reset log, release mutex
    current_log[current_log_len] = 0;  // null terminator, just to be sure
    int tx_log_len = current_log_len;
    String tx_log(current_log, tx_log_len);
    current_log_len = 0;
    // Serial.println("LOGMUTEX GIVE (main_tele)");
    xSemaphoreGive(log_mutex);

    // Transmit if necessary
    Serial.printf("log len %d\n", tx_log_len);
    // Serial.printf("log: %s\n", tx_log.c_str());
    if (tx_log_len > 0) {
      transmit("LOG", tx_log);
      if (tx_log_len == LOG_MAX) {
        transmit("ERROR", "log overflow");
      }
      persist_log(tx_log);
    }
  }
  // Serial.println("TELE: log done");

  // Altimeter
  if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY)) {
    // Serial.println("TELE: alt start");
    snprintf(data_string, 512, "pressure=%.2f\ttemp=%.2f", altimeter_data.pressure, altimeter_data.temp);
    xSemaphoreGive(sensor_data_mutex);
    transmit("ALT", data_string);
  }
  // Serial.println("TELE: alt done");

  // GPS
  if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY)) {
    // Serial.println("TELE: gps start");
    snprintf(data_string, 512, "fix=%d\tqual=%d\tsats=%d\tant=%d\tdatetime=%d/%d/%dT%d:%d:%d\tlat=%d\tlong=%d\tfixLat=%d\tfixLong=%d",
             gps_data.fix, gps_data.fix_quality, gps_data.satellites, gps_data.antenna,
             gps_data.year, gps_data.month, gps_data.day, gps_data.hour, gps_data.minute, gps_data.second,
             gps_data.latitude, gps_data.longitude, gps_data.fixLat, gps_data.fixLong);
    xSemaphoreGive(sensor_data_mutex);
    transmit("GPS", data_string);
  }

// Gridfin
#ifdef HERMES
  if (xSemaphoreTake(gridfin_mutex, portMAX_DELAY)) {
    // Serial.println("TELE: gps start");
    snprintf(data_string, 512, "angle=%d",
             gridfin_angle);
    xSemaphoreGive(gridfin_mutex);
    transmit("GRIDFIN", data_string);
  }
#endif
  // Serial.println("TELE: gps done");

  // IMU
  if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY)) {
    // Serial.println("TELE: imu start");
    snprintf(data_string, 512, "accelX=%.2f\taccelY=%.2f\taccelZ=%.2f\tgyroX=%.2f\tgyroY=%.2f\tgyroZ=%.2f",
             imu_data.accelX, imu_data.accelY, imu_data.accelZ,
             imu_data.gyroX, imu_data.gyroY, imu_data.gyroZ);
    xSemaphoreGive(sensor_data_mutex);
    transmit("IMU", data_string);
  }
  // Serial.println("TELE: imu done");

  // Control
  if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY)) {
    snprintf(data_string, 512, "status=%d\taltitude=%.2f\tdownward_velocity=%.2f", control_data.flight_status, control_data.altitude, control_data.downward_velocity);
    xSemaphoreGive(sensor_data_mutex);
    transmit("CONTROL", data_string);
  }

  // Sensor loop data
  if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY)) {
    // Serial.println("TELE: loop start");
    snprintf(data_string, 512, "lastLoopStart=%lld\trecentLoops=%d\trecentMicros=%lld", loopStats.lastLoopStart, loopStats.loopsSinceLog, loopStats.microsSinceLog);
    loopStats.loopsSinceLog = 0;
    loopStats.microsSinceLog = 0;
    xSemaphoreGive(sensor_data_mutex);
    transmit("LOOP", data_string);
  }
  // Serial.println("Tele: loop done");

  // Serial.print("FREE MEMORY: ");
  // Serial.println(heap_caps_get_free_size(MALLOC_CAP_8BIT));
  persist_data();
}

void loop_telemetry(void *params) {
  while (1) {
    main_telemetry();
    delay(1000);
  }
}
