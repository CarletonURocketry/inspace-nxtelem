#include <testing/unity.h>
#include <stdlib.h>
#include <math.h>


#include "test_runners.h"
#include "../telemetry/src/fusion/detector.h"

static unsigned long loop_to_micro(int i, int iterations, float duration, float offset) {
  return (unsigned long)(1000000 * (i) * (duration / iterations) + (offset * 1000000.0));
}

static void test_old_samples__no_detection(void) {
    struct detector detector;
    detector_init(&detector, 0);

    // Make sure these don't break if called immediately
    TEST_ASSERT_EQUAL(0.0f, detector_get_accel(&detector));
    TEST_ASSERT_EQUAL(0.0f, detector_get_alt(&detector));

    TEST_ASSERT_EQUAL(DETECTOR_NO_EVENT, detector_detect(&detector));
}

static void test_set_state_no_samples__no_event(void) {
    struct detector detector;
    detector_init(&detector, 0);

    // Set states and make sure nothing happens with no samples
    detector_set_state(&detector, STATE_AIRBORNE, SUBSTATE_UNKNOWN);
    TEST_ASSERT_EQUAL(DETECTOR_NO_EVENT, detector_detect(&detector));

    detector_set_state(&detector, STATE_IDLE, STATE_AIRBORNE);
    TEST_ASSERT_EQUAL(DETECTOR_NO_EVENT, detector_detect(&detector));

    detector_set_state(&detector, STATE_LANDED, SUBSTATE_ASCENT);
    TEST_ASSERT_EQUAL(DETECTOR_NO_EVENT, detector_detect(&detector));
}

static void test_constant_altitude_idle_state__constant_altitude_no_event(void) {
    struct detector detector;
    detector_init(&detector, 0);

    // Substate shouldn't matter here
    detector_set_state(&detector, STATE_IDLE, SUBSTATE_UNKNOWN);

    // Provide 30 samples over what looks like 1 second
    const int num_samples = 30;
    for (int i = 0; i < num_samples; i++) {
        struct altitude_sample sample = { .altitude = 1000.0, .time = loop_to_micro(i, num_samples, 1, 0) };
        detector_add_alt(&detector, &sample);
    }

    TEST_ASSERT_EQUAL(DETECTOR_NO_EVENT, detector_detect(&detector));
    TEST_ASSERT_FLOAT_WITHIN(1.0, 1000.0, detector_get_alt(&detector));
}

static void test_constant_acceleration_idle_state__constant_acceleration_no_event(void) {
    struct detector detector;
    detector_init(&detector, 0);

    // Substate shouldn't matter here
    detector_set_state(&detector, STATE_IDLE, SUBSTATE_UNKNOWN);

    // Provide 30 samples over what looks like 1 second
    const int num_samples = 30;
    for (int i = 0; i < num_samples; i++) {
        struct accel_sample sample = { .acceleration = 9.8, .time = loop_to_micro(i, num_samples, 1, 0) };
        detector_add_accel(&detector, &sample);
    }

    TEST_ASSERT_EQUAL(DETECTOR_NO_EVENT, detector_detect(&detector));
    TEST_ASSERT_FLOAT_WITHIN(1.0, 9.8, detector_get_accel(&detector));
}

void test_detection(void) {
    RUN_TEST(test_old_samples__no_detection);
    RUN_TEST(test_set_state_no_samples__no_event);
    RUN_TEST(test_constant_altitude_idle_state__constant_altitude_no_event);
    RUN_TEST(test_constant_acceleration_idle_state__constant_acceleration_no_event);
}
