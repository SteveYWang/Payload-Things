#ifndef UTIL_H
#define UTIL_H

// Don't read sensors, just mock data
#define MOCK_SENSOR_DATA

// Returns from a function immediately except for every `x` calls.
#define PERIOD(x) {const int _PERIOD = x; static int _iter = 0; if (++_iter < _PERIOD) return; _iter = 0;}

// Set which cubesat we are
#define HELIOS
// #define HERMES

// Check one was defined
#if !(defined(HELIOS) ^ defined(HERMES)) 
  #error "One and only one of HELIOS or HERMES should be defined"
#endif

extern void log(const char *s);

#endif