#include <stdio.h>
#include <fcntl.h>
#include <nuttx/config.h>
#include <testing/unity.h>

#include "../telemetry/src/rocket-state/rocket-state.h"

/* Writes data to the state file, clearing previous contents. Skips the test if that fails */
static void write_to_state_file(uint8_t *data, size_t len) {
  FILE *state_file = fopen(CONFIG_INSPACE_TELEMETRY_EEPROM, "w");
  if (state_file == NULL) {
    TEST_IGNORE_MESSAGE("Could not open eeprom file to set invalid state");
  }
  if (!fwrite(data, len, sizeof(*data), state_file)) {
    TEST_IGNORE_MESSAGE("Could not write test data to eeprom file");
  }
  fclose(state_file);
}

/* Eeprom may not work like regular file - overwrite to delete data to be sure */
static void clear_rocket_state(void) {
  const char *empty = "\0\0\0\0\0\0\0\0\0\0\0";
  write_to_state_file((uint8_t *)empty, sizeof(empty));
}

/* Test that if the flight state file is empty, we end up in the idle flightstate */
static void test_no_state__sent_to_airborne(void) {
  clear_rocket_state();
  rocket_state_t state;
  TEST_ASSERT_NOT_EQUAL(0, state_init(&state));
  enum flight_state_e flight_state;
  TEST_ASSERT_EQUAL(0, state_get_flightstate(&state, &flight_state));
  TEST_ASSERT_EQUAL(STATE_AIRBORNE, flight_state);
}

/* Test that an invalid flight state results in the idle state being loaded */
static void test_invalid_state__sent_to_airborne(void) {
  const char *garbage = "FFFFFFFF";
  write_to_state_file((uint8_t *)garbage, sizeof(garbage));
  rocket_state_t state;
  TEST_ASSERT_NOT_EQUAL(0, state_init(&state));
  enum flight_state_e flight_state;
  TEST_ASSERT_EQUAL(0, state_get_flightstate(&state, &flight_state));
  TEST_ASSERT_EQUAL(STATE_AIRBORNE, flight_state);
}

/* Test that a valid flight state can be loaded from nv storage */
static void test_valid_state__sent_to_state(void) {
  clear_rocket_state();
  rocket_state_t state;
  TEST_ASSERT_NOT_EQUAL(0, state_init(&state));
  TEST_ASSERT_EQUAL(0, state_set_flightstate(&state, STATE_AIRBORNE));

  // Now try loading again
  enum flight_state_e new_state;
  TEST_ASSERT_EQUAL(0, state_init(&state));
  TEST_ASSERT_EQUAL(0, state_get_flightstate(&state, &new_state));
  TEST_ASSERT_EQUAL(STATE_AIRBORNE, new_state);
}

/* Paramaterized test to check if we can set and then load a flight state, starting from an invalid state file */
static void check_set_state(enum flight_state_e flight_state) {
  clear_rocket_state();
  rocket_state_t state;
  TEST_ASSERT_NOT_EQUAL_MESSAGE(0, state_init(&state), "Could not initialize the flight state");
  TEST_ASSERT_EQUAL_MESSAGE(0, state_set_flightstate(&state, flight_state), "Could not set the flight state");

  enum flight_state_e new_state;
  TEST_ASSERT_EQUAL_MESSAGE(0, state_get_flightstate(&state, &new_state), "Could not load the flight state");
  TEST_ASSERT_EQUAL_MESSAGE(flight_state, new_state, "The wrong flight state was loaded");
}

/* Test if we can set flying state and load it */
static void test_set_flying__flying_loaded(void) {
  check_set_state(STATE_AIRBORNE);
}

/* Test if we can set landed state and load it */
static void test_set_landed__landing_loaded(void) {
  check_set_state(STATE_LANDED);
}

/* Test if we can set the idle state and load it */
static void test_set_idle__idle_loaded(void) {
  check_set_state(STATE_IDLE);
}

void test_rocket_state(void) {
  RUN_TEST(test_no_state__sent_to_airborne);
  RUN_TEST(test_invalid_state__sent_to_airborne);
  RUN_TEST(test_valid_state__sent_to_state);
  RUN_TEST(test_set_flying__flying_loaded);
  RUN_TEST(test_set_landed__landing_loaded);
  RUN_TEST(test_set_idle__idle_loaded);
}
