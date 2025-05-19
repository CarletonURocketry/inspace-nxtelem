#include <stdio.h>
#include <nuttx/config.h>
#include <testing/unity.h>

#include "test_runners.h"

// Empty but required by Unity
void setUp(void) {

}

// Empty but required by Unity
void tearDown(void) {

}

int main(void) {
  UNITY_BEGIN();
  test_rocket_state();
  test_altitude_detection();
  test_circular_buffer();
  return UNITY_END();
}
