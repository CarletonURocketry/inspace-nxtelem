#include <stdio.h>
#include <fcntl.h>
#include <nuttx/config.h>
#include <testing/unity.h>

#include "../telemetry/src/rocket-state/rocket-state.h"

/**
 * Empty the contents of the flight state file
 */
static void clear_rocket_state(void) {
  if (fclose(fopen(CONFIG_INSPACE_TELEMETRY_EEPROM, "w"))) {
    TEST_IGNORE_MESSAGE("Could not clear eeprom contents before test");
  }
}

/**
 * Test that if the flight state file is empty, we end up in the idle flightstate
 */
static void test_no_state__sent_to_idle(void) {
  clear_rocket_state();
  rocket_state_t state;
  int ret = state_init(&state);
  enum flight_state_e flight_state;
  state_get_flightstate(&state, &flight_state);
  TEST_ASSERT_EQUAL(STATE_IDLE, flight_state);
}

static void test_invalid_state__sent_to_idle(void) {

}

static void test_flying_state__sent_to_flying_state(void) {
  
}

static void test_landed_state__sent_to_landed_state(void) {
  
}

void test_rocket_state(void) {
  RUN_TEST(test_no_state__sent_to_idle);
  RUN_TEST(test_invalid_state__sent_to_idle);
  RUN_TEST(test_flying_state__sent_to_flying_state);
  RUN_TEST(test_landed_state__sent_to_landed_state);
}
