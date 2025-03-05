#include "ESP32Servo.h"

Servo gridfin1;
Servo gridfin2;

#define GRIDFIN_PROPORTIONALITY_CONSTANT 111

// Microseconds to be parallel to ground
#define GRIDFIN_1_CENTER 1550
#define GRIDFIN_2_CENTER 1500

// Min/max angles
#define GRIDFIN_MIN 1200
#define GRIDFIN_MAX 1800
 
bool gridfins_are_deployed = false;
bool gridfins_have_initially_stabilized = false;
int current_constant_number = 0;
 
void init_gridfins(){
  // Attach
  gridfin1.attach(SERVO1);
  gridfin2.attach(SERVO2);
  // Deploy
  gridfin1.writeMicroseconds(1700);
  gridfin2.writeMicroseconds(1700);
  delay(2000);
}
 
// Write an angle in microseconds
void write_gridfins(int16_t angle){
  gridfin1.writeMicroseconds(constrain(angle + GRIDFIN_1_CENTER, GRIDFIN_MIN, GRIDFIN_MAX));
  gridfin2.writeMicroseconds(constrain(angle + GRIDFIN_2_CENTER, GRIDFIN_MIN, GRIDFIN_MAX));
  if (xSemaphoreTake(gridfin_mutex, GRIDFIN_MAX_MUTEX_DELAY)) {
    gridfin_angle = angle;
    xSemaphoreGive(gridfin_mutex);
  }else{
    log("ERROR: failed to obtain gridfin mutex");
    return;
  }
}

void do_gridfins(float gyroZ){
  int angle = GRIDFIN_PROPORTIONALITY_CONSTANT * gyroZ;
  write_gridfins(angle);
}
