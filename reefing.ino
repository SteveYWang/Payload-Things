#include <ESP32Servo.h>

Servo reefingServo;

void start_reefing() {
  reefingServo.attach(SERVO3);
}
void stop_reefing() {
  reefingServo.detach();
}

int calculate_reef(int velocity, int targetVelocity){
  //inputs: current velocity (from altimeter interpolation), target velocity (can be static or the velocity of hermes)
  //output: desired reefing servo signal (use writeMicroseconds on the returned val)
  int error = velocity - targetVelocity;

  if(abs(error) < 0.5){
    return 1500; // if the payload is within acceptable error range, dont do anything 
  }
  
  else { 
    if (error > 0){
      return 500; // disreef to slow down 
    }
    else{
      return 2500; // reef to increase speed
    }
  }

  return 1500; // just in case lol 
}

void reef(float velocity, float target) {
  PERIOD(5)

  reefingServo.writeMicroseconds(calculate_reef(velocity, target));
}
