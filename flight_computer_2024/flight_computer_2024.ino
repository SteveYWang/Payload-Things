#include "util.h"
#include "task.h"

// Current sensor status (here so other files see)
struct {
  bool connected;
  float temp;
  float pressure;
} altimeter_data = {
  .connected = false,
  .temp = 0.0,
  .pressure = 0.0,
};

struct {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
} imu_data = {
  .accelX = 0.,
  .accelY = 0.,
  .accelZ = 0.,
  .gyroX = 0.,
  .gyroY = 0.,
  .gyroZ = 0.,
};

struct {
  bool fix;
  uint8_t fix_quality;
  uint8_t satellites;
  uint8_t antenna;
  uint16_t hour;
  uint16_t minute;
  uint16_t second;
  uint16_t day;
  uint16_t month;
  uint16_t year;
  // lat and long in 1/10000000 of degree
  int32_t latitude;
  int32_t longitude;
  int32_t fixLat;
  int32_t fixLong;
} gps_data = {
  .fix = false, .fix_quality = 0, .satellites = 0, .antenna = 0,
  .hour = 0, .minute = 0, .second = 0, .day = 0, .month = 0, .year = 0,
  .latitude = 0, .longitude = 0, .fixLat = 0, .fixLong = 0
};

struct {
  int flight_status;
  float altitude;
  float downward_velocity;
} control_data = {
  .flight_status = 0,
  .altitude = 0.0,
  .downward_velocity = 0.0,
};

int gridfin_angle = 1500;

TickType_t lastWakeTime;

// Sensor status mutex (for tasks to access sensor data)
SemaphoreHandle_t sensor_data_mutex = NULL;
// SPI mutex (for telemetry and sensors to share bus)
static portMUX_TYPE _spi_mux = portMUX_INITIALIZER_UNLOCKED;
// Log mutex
SemaphoreHandle_t log_mutex = NULL;
// Log mutex
SemaphoreHandle_t gridfin_mutex = NULL;
// Max delay when taking mutexes on the critical path in `loop`
#define SENSOR_MAX_MUTEX_DELAY (20 / portTICK_PERIOD_MS)
#define GRIDFIN_MAX_MUTEX_DELAY SENSOR_MAX_MUTEX_DELAY

void setup() {
  Serial.begin(115200);
  delay(500);
  // Create mutexes (before anything else)
  sensor_data_mutex = xSemaphoreCreateMutex();
  log_mutex = xSemaphoreCreateMutex();
  gridfin_mutex = xSemaphoreCreateMutex();

  log("setup() begin...");

  // Disable WiFi/Bluetooth, set CPU frequency
  setCpuFrequencyMhz(40);
  uint32_t freq = getCpuFrequencyMhz();
  Serial.printf("Frequency: %u\n", freq);

  // Setup peripherals
  setup_persist();
  setup_sensors();
  setup_control();
  setup_telemetry();

  // Log ready and initialize last wake time for loop's periodic timer
  log("setup() done, starting main loop and tasks!");

  // Make new task for telemetry
  xTaskCreate(loop_telemetry, "telemetry", 8192, NULL, 2, NULL);
  // Make new task for main sensor reading loop
  xTaskCreate(loop_sensor, "sensors", 8192, NULL, 5, NULL);
}

struct {
  int64_t lastLoopStart;
  int64_t microsSinceLog;
  int loopsSinceLog;
} loopStats = {
  .lastLoopStart = 0,
  .microsSinceLog = 0,
  .loopsSinceLog = 0,
};

void loop_sensor(void *params) {
  lastWakeTime = xTaskGetTickCount();
  for(;;) {
    realLoop();
    // Sleep until next loop time
    xTaskDelayUntil(&lastWakeTime, 10 / portTICK_PERIOD_MS);
  }
}

void realLoop() {
  int64_t start = micros();
  main_sensors();
  main_control(start);
  int64_t end = micros();

  // Update loop stats from prior iteration
  if (xSemaphoreTake(sensor_data_mutex, SENSOR_MAX_MUTEX_DELAY)) {
    loopStats.lastLoopStart = start;
    loopStats.microsSinceLog += end - start;
    loopStats.loopsSinceLog += 1;
    xSemaphoreGive(sensor_data_mutex);
  } else {
    log("ERROR: failed to obtain mutex for loop data");
  }
}

// Locks the SPI mutex, as well as disabling the LoRa interrupt (since the ISR uses SPI).
// Takes a timeout and returns false if the lock couldn't be acquired in that time.
bool spiLock(TickType_t delay) {
  taskENTER_CRITICAL(&_spi_mux);
  return true;
}

void spiUnlock() {
  taskEXIT_CRITICAL(&_spi_mux);
}

void loop() {
  delay(10000);
}
