#ifndef CONFIG_H
#define CONFIG_H

#define PRODUCTION true

// The sampling rate in Hz (1Hz, 10Hz, 25Hz, 50Hz, and 100Hz possible) 
#define ACCEL_SAMPLING_RATE 25
//#define ACCEL_SAMPLES_PER_UPDATE ACCEL_SAMPLING_RATE
#define ACCEL_SAMPLES_PER_UPDATE 5


#if PRODUCTION
  #define DEBUG false
  #define LOGS false
#endif

#if !PRODUCTION
  #define DEBUG true
  #define LOGS false
#endif

#define SCREEN_W 144
#define SCREEN_H 168

#endif // CONFIG_H