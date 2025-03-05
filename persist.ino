#include "FS.h"
#include "LittleFS.h"

bool fs_setup = false;
File data_file;
File log_file;

// Sets up LittleFS and opens a log file in append mode
void setup_persist() {
  if (!LittleFS.begin(false)) {
    log("LittleFS mount failed, no persistent logging");
    return;
  }
  data_file = LittleFS.open("/data.txt", FILE_APPEND, true);
  if (!data_file) {
    log("Failed to open data file, will not persist data");
  } else {
    log("Data file opened successfully");
    const char *to_write = "DATA BEGIN\n";
    data_file.write((uint8_t *) to_write, strlen(to_write));
  }
  log_file = LittleFS.open("/log.txt", FILE_APPEND, true);
  if (!log_file) {
    log("Failed to open log file, will not persist log");
  } else {
    log("Log file opened successfully");
    const char *to_write = "LOG BEGIN\n";
    log_file.write((uint8_t *) to_write, strlen(to_write));
  }
}

// Logs current sensor data to the filesystem
void persist_data() {
  // If no open file, try to open every 10 seconds, else return
  if (!data_file) {
    static int64_t last_attempt = 0;
    if (millis() - last_attempt > 10000) {
      // Try to open again, since it's been a while
      last_attempt = millis();
      data_file = LittleFS.open("/data.txt", FILE_APPEND, true);
      if (data_file) {
        // Success, continue with method, first writing that we began
        log("Opened data file successfully!");
        const char *to_write = "DATA BEGIN\n";
        data_file.write((uint8_t *) to_write, strlen(to_write));
      } else {
        // Failure, return
        log("Again failed to open data file, will not persist data (try again in 10 seconds)");
        return;
      }
    } else {
      // Recently tried and failed to open, return
      return;
    }
  }

  // Get all data
  char data_string[1024];
  int data_len;
  if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY)) {
    // Skip every 10th call if awaiting launch or on ground
    if (control_data.flight_status < 1 || control_data.flight_status > 3) {
      static int _iter = 0;
      if (++_iter < 10) {
        xSemaphoreGive(sensor_data_mutex);
        return;
      }
      _iter = 0;
    }
    // Get data
    data_len = snprintf(data_string, 1024, "T=%lld\tstatus=%d,alt=%.2f,down_velo=%.2f\tp=%.2f,t=%.2f,\tfix=%d,q=%d,sat=%d,ant=%d,dt=%d/%d/%dT%d:%d:%d,lat=%d,long=%d\taX=%.2f,aY=%.2f,aZ=%.2f,gX=%.2f,gY=%.2f,gZ=%.2f\n",
        loopStats.lastLoopStart,
        control_data.flight_status, control_data.altitude, control_data.downward_velocity,
        altimeter_data.pressure, altimeter_data.temp,
        gps_data.fix, gps_data.fix_quality, gps_data.satellites, gps_data.antenna,
            gps_data.year, gps_data.month, gps_data.day, gps_data.hour, gps_data.minute, gps_data.second,
            gps_data.latitude, gps_data.longitude,
        imu_data.accelX, imu_data.accelY, imu_data.accelZ,
            imu_data.gyroX, imu_data.gyroY, imu_data.gyroZ
      );
    xSemaphoreGive(sensor_data_mutex);
  }
  
  // Write to FS
  data_file.write((uint8_t *) data_string, min(data_len, 1024));
  data_file.flush();
}

// Logs some log data to the filesystem
void persist_log(const String &log_string) {
  // If no open file, try to open every 10 seconds, else return
  if (!log_file) {
    static int64_t last_attempt = 0;
    if (millis() - last_attempt > 10000) {
      // Try to open again, since it's been a while
      last_attempt = millis();
      log_file = LittleFS.open("/log.txt", FILE_APPEND, true);
      if (log_file) {
        // Success, continue with method
        log("Opened log file successfully!");
        const char *to_write = "LOG BEGIN\n";
        log_file.write((uint8_t *) to_write, strlen(to_write));
      } else {
        // Failure, return
        log("Again failed to open log file, will not persist data (try again in 10 seconds)");
        return;
      }
    } else {
      // Recently tried and failed to open, return
      return;
    }
  }

  // Write to FS
  log_file.write((uint8_t *) log_string.c_str(), log_string.length());
  log_file.flush();
}
