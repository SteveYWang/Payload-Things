#include "util.h"
#include "task.h"
#include "pins.h"
#include "ESP32Servo.h"
#include <atomic>

// == Control parameters ==
// High altitude cutoff for launch detection (ft)
#define HIGH_ALTITUDE 2000
// Altitude below max for descent detection (ft)
#define DESCENT_AMOUNT 500
// Length of time to be at altitude for launch or descent to be detected (microseconds)
#define ALTITUDE_CUTOFF_TIME 1000000
// Altitude for lumberjack deployment (ft)
#define LUMBERJACK_ALTITUDE 2500
// Downward vertical speed (ft/s) to deploy parachute in emergency
#define TERMINAL_VELOCITY 30.0
// Duration of falling to deploy parachute in emergency
#define FALLING_TIME 5000000
// Number of datapoints to track for each metric for smoothing/consensus
#define HISTORY_WINDOW 5
// Sea level pressure for altitude algorithm
#define SEA_LEVEL_PRESSURE 1013.25

// == Servo locations ==
#define HELIOS_CHUTE_1_INIT 750
#define HELIOS_CHUTE_1_DEPLOY 1550
#define HELIOS_CHUTE_2_INIT 1400
#define HELIOS_CHUTE_2_DEPLOY 550
// #define LUMBERJACK_INIT 1611
#define LUMBERJACK_INIT 1540
// #define LUMBERJACK_OPEN 1333
#define LUMBERJACK_OPEN 1320

Servo lj;

// Possible states
enum {
  AWAITING_LAUNCH,
  ASCENDING,
  DESCENDING,
  SEPARATED,
  DONE,
} flight_status = AWAITING_LAUNCH;
std::atomic_bool in_flight{false};

// Last few measurements for velocity calculation & smoothing altiude data
int64_t last_measure_times[HISTORY_WINDOW] = {0, 0, 0, 0, 0};
double last_altitude[HISTORY_WINDOW] = {0., 0., 0., 0., 0.};

// Util to shift new stat into array
template <typename T, unsigned int N = HISTORY_WINDOW> 
void shift_array(T new_val, T *array) {
  for (int i = N-1; i > 0; i--) {
    array[i] = array[i-1];
  }
  array[0] = new_val;
}

// Start times of altitude conditions for launch/apogee detection
int64_t first_high_altitude = 0; // time above launch cutoff
int64_t first_low_altitude = 0; // time below apogee cutoff
int64_t first_terminal_velocity = 0; // time above terminal velocity downward

// Maximum altitude we've reached during ascent, for apogee detection (smoothed by taking min of history)
double max_altitude = 0.0;

void setup_control() {
  #ifdef HERMES
  // Disable video
  pinMode(VIDEO_ENABLE, OUTPUT);
  digitalWrite(VIDEO_ENABLE, LOW);
  #endif
  #ifdef HELIOS
  // Move lumberjack into place
  Servo c1, c2;
  lj.attach(SERVO4);
  lj.writeMicroseconds(LUMBERJACK_OPEN);
  delay(5000);
  lj.writeMicroseconds(LUMBERJACK_INIT);
  // delay(2000);
  // log("Detatching");
  // lj.detach();
  // Open chutes for mechanical setup
  c1.attach(SERVO1);
  c2.attach(SERVO2);
  c1.writeMicroseconds(HELIOS_CHUTE_1_DEPLOY);
  c2.writeMicroseconds(HELIOS_CHUTE_2_DEPLOY);
  delay(2000);
  // c1.detach();
  // c2.detach();
  delay(10000);
  // Close chutes to be ready
  c1.attach(SERVO1);
  c2.attach(SERVO2);
  c1.writeMicroseconds(HELIOS_CHUTE_1_INIT);
  c2.writeMicroseconds(HELIOS_CHUTE_2_INIT);
  delay(2000);
  c1.detach();
  c2.detach();
  #endif
}

void main_control(int64_t sensor_time) {
  PERIOD(10)

  // Get sensor data we need
  double pressure;
  if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY)) {
    pressure = altimeter_data.pressure;
    xSemaphoreGive(sensor_data_mutex);
  } else {
    log("ERROR: failed to acquire sensor mutex in control");
    return;
  }

  // Compute derived data (altitude in ft, total acceleration squred in (m/s^2)^2)
  double altitude = 145366.45 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 0.190284));
  if (pressure <= 1.0) {
    static int iter_since_warned = 0;
    if (iter_since_warned-- == 0) {
      log("WARN: no valid pressure found, can't find altitude (OK for first few iterations before sensor starts)");
      iter_since_warned = 50;
    }
    altitude = 0.0;
  }

  // Shift data into arrays
  shift_array(sensor_time, last_measure_times);
  shift_array(altitude, last_altitude);

  // Get min/max altitudes for use in hard cutoffs; get binary properties by consensus of array
  double current_altitude_low = 1000000.0;
  double current_altitude_high = 0.;
  int high_altitude_count = 0;
  int low_altitude_count = 0;
  for (int i = 0; i < HISTORY_WINDOW; i++) {
    if (last_altitude[i] > HIGH_ALTITUDE) high_altitude_count += 1;
    if (last_altitude[i] < max_altitude - DESCENT_AMOUNT) low_altitude_count += 1;
    if (last_altitude[i] > current_altitude_high) current_altitude_high = last_altitude[i];
    if (last_altitude[i] < current_altitude_low) current_altitude_low = last_altitude[i];
  }
  bool high_altitude = high_altitude_count > (HISTORY_WINDOW / 2);
  bool low_altitude = low_altitude_count > (HISTORY_WINDOW / 2);

  // Find vertical velocity as well for reefing and emerency deployment
  double downward_vertical_velocity = (last_measure_times[0] - last_measure_times[HISTORY_WINDOW - 1]) / 1000000.0;
  downward_vertical_velocity = downward_vertical_velocity != 0 ? (last_altitude[HISTORY_WINDOW - 1] - last_altitude[0]) / downward_vertical_velocity : 0.0;

  // Note how long we've been under certain conditions
  int64_t time_high_altitude = 0;
  if (high_altitude) {
    if (first_high_altitude == 0) first_high_altitude = sensor_time;
    time_high_altitude = sensor_time - first_high_altitude;
  } else {
    first_high_altitude = 0;
  }
  int64_t time_low_altitude = 0;
  if (low_altitude) {
    if (first_low_altitude == 0) first_low_altitude = sensor_time;
    time_low_altitude = sensor_time - first_low_altitude;
  } else {
    first_low_altitude = 0;
  }
  int64_t time_terminal_velocity = 0;
  if (downward_vertical_velocity > TERMINAL_VELOCITY) {
    if (first_terminal_velocity == 0) first_terminal_velocity = sensor_time;
    time_terminal_velocity = sensor_time - first_terminal_velocity;
  }

  // Possibly update state
  switch (flight_status) {
  case AWAITING_LAUNCH:
    // Update status if we're above altitude cutoff
    if (time_high_altitude > ALTITUDE_CUTOFF_TIME) {
      flight_status = ASCENDING;
      control_launch();
    }
    break;
  case ASCENDING:
    // Update max altitude used in descent detection
    if (current_altitude_low > max_altitude) max_altitude = current_altitude_low;

    // Begin descent if we've been below cutoff for long enough
    if (time_low_altitude > ALTITUDE_CUTOFF_TIME) {
      flight_status = DESCENDING;
      control_apogee();
    }
    break;
  case DESCENDING:
    control_descent();
    // If current altitude (overestimate) is below lumberjack altitude, separate.
    if (current_altitude_high < LUMBERJACK_ALTITUDE) {
      flight_status = SEPARATED;
      control_separation();
    }
    break;
  case SEPARATED:
    control_descent();
    control_post_separation(downward_vertical_velocity);
    if (check_ground(current_altitude_low, current_altitude_high)) {
      flight_status = DONE;
      control_done();
    }
    break;
  case DONE:
    break;
  }
  // Emergency fallback: if at terminal velocity for long enough, trigger apogee
  if (flight_status < DESCENDING && time_terminal_velocity > FALLING_TIME) {
    log("WARN: hit terminal velocity for a while, emergency apogee trigger hit");
    flight_status = DESCENDING;
    control_apogee();
  }

  // Update status
  if (xSemaphoreTake(sensor_data_mutex, SENSOR_MAX_MUTEX_DELAY)) {
    control_data.flight_status = flight_status;
    control_data.altitude = altitude;
    control_data.downward_velocity = downward_vertical_velocity;
    xSemaphoreGive(sensor_data_mutex);
  } else {
    log("WARN: Failed to acquire sensor mutex at end of control");
  }
}

bool check_ground(double current_altitude_low, double current_altitude_high) {
  // Check altitude every 20 seconds
  static int64_t previous_check_time = 0;
  if (millis() < previous_check_time + 20000) {
    return false;
  }
  previous_check_time = millis();
  // If current altitude (overestimate) is higher than last (underestimate), we've hit the ground.
  static double previous_check = 100000.0;
  if (current_altitude_high >= previous_check) {
    return true;
  }
  // Otherwise, we haven't; update previous check
  previous_check = current_altitude_low;
  return false;
}

void control_launch() {
  log("Launch detected");
  #ifdef HERMES
  digitalWrite(VIDEO_ENABLE, HIGH);
  #endif
}

void control_apogee() {
  log("Apogee detected");
  #ifdef HELIOS
  // Deploy parachute
  Servo c1;
  Servo c2;
  c1.attach(SERVO1);
  c2.attach(SERVO2);
  c1.writeMicroseconds(HELIOS_CHUTE_1_DEPLOY);
  c2.writeMicroseconds(HELIOS_CHUTE_2_DEPLOY);
  delay(2000);
  c1.detach();
  c2.detach();
  #endif
  #ifdef HERMES
  // Enable gridfins
  init_gridfins();
  #endif
}

void control_separation() {
  log("Separation height reached");
  #ifdef HELIOS
  lj.attach(SERVO4);
  lj.writeMicroseconds(LUMBERJACK_OPEN);
  delay(1000);
  // lj.detach();
  start_reefing();
  #endif
}

void control_descent() {
  #ifdef HERMES
  // Get X gyro (roll)
  double gyroX;
  if (xSemaphoreTake(sensor_data_mutex, portMAX_DELAY)) {
    gyroX = imu_data.gyroX;
    xSemaphoreGive(sensor_data_mutex);
  } else {
    log("ERROR: failed to acquire sensor mutex in gridfin control");
    return;
  }
  // Call gridfin control code
  do_gridfins(gyroX);
  #endif
}
void control_post_separation(float velocity) {
  #ifdef HELIOS
  reef(velocity, 23.0);
  #endif
}

void control_done() {
  #ifdef HERMES
  digitalWrite(VIDEO_ENABLE, LOW);
  #endif
  #ifdef HELIOS
  stop_reefing();
  #endif
}

