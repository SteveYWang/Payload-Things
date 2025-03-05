#ifndef PINS_H
#define PINS_H

#define PIN_SCLK 8
#define PIN_MISO 7
#define PIN_MOSI 6

#define PIN_GPS_TX 9
#define PIN_GPS_RX 34

#ifdef HELIOS
#define PIN_IMU_CS 38
#define PIN_IMU_RST 39
#define PIN_IMU_INT 40

#define PIN_ALT_CS 15
#define PIN_ALT_INT 16
#endif

#define PIN_LORA_CS 5
#define PIN_LORA_EN 2
#define PIN_LORA_G0 1

#ifdef HELIOS
#define VIDEO_ENABLE 41

#define SERVO1 42
#define SERVO2 13
#define SERVO3 12
#define SERVO4 14

#define PIN_UNUSED 35
#endif

#ifdef HERMES
#define PIN_IMU_CS 35
#define PIN_IMU_RST 37
#define PIN_IMU_INT 36
#define PIN_ALT_INT 3
#define PIN_ALT_CS 4
#define VIDEO_ENABLE 21
#define SERVO1 43
#define SERVO2 44
// don't matter, use non-existant pins on tiny
#define SERVO3 12
#define SERVO4 14
#define PIN_UNUSED 41
#endif

#endif
