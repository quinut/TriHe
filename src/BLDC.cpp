#include <Arduino.h>
#include <math.h>

void loop() {
  BLDC_1_THROTTLE = default_THROTTLE - s2_distance * cos( s2_rads -  ( 0 + spinned_rads) )
}