#include <testing/unity.h>
#include <stdlib.h>
#include <math.h>


#include "test_runners.h"
#include "../telemetry/src/fusion/detector.h"

static unsigned long to_micro(float seconds) {
    return seconds * 1000000;
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

    detector_set_state(&detector, STATE_IDLE, SUBSTATE_UNKNOWN);
    TEST_ASSERT_EQUAL(DETECTOR_NO_EVENT, detector_detect(&detector));

    detector_set_state(&detector, STATE_LANDED, SUBSTATE_ASCENT);
    TEST_ASSERT_EQUAL(DETECTOR_NO_EVENT, detector_detect(&detector));
}

struct altitude_generator {
    void *context;
    int (*func)(void*, int, struct altitude_sample*);
};

struct accel_generator {
    void *context;
    int (*func)(void*, int, struct accel_sample*);
};

/**
 * Parameterized test for trying different altitude/acceleration combinations
 *
 * @param alt_gen Function and its context for generating altitude samples
 * @param accel_gen Function and its context for generating accel samples
 * @param steps The number of interations to run - its up to the generator functions to decide what units of time the step represents
 * @param detector The detector to use, initialized
 * @param expected_event The only event that should be output besides DETECTOR_NO_EVENT, and which must be output at least one time
 */
static void check_input_func_generates_output_event(struct altitude_generator *alt_gen, struct accel_generator *accel_gen, 
                                                    int steps, struct detector *detector, enum detector_event expected_event) {
    struct altitude_sample alt = {0};
    struct accel_sample accel = {0};
    for (int i = 0; i < steps; i++) {
        if (accel_gen->func(accel_gen->context, i, &accel)) {
            detector_add_accel(detector, &accel);
        }
        if (alt_gen->func(alt_gen->context, i, &alt)) {
            detector_add_alt(detector, &alt);
        }
        enum detector_event event = detector_detect(detector);
        if (event != DETECTOR_NO_EVENT) {
            TEST_ASSERT_EQUAL_MESSAGE(expected_event, event, "Expected one event while providing samples, got another");
        }
    }
    if (expected_event != DETECTOR_NO_EVENT)
    TEST_FAIL_MESSAGE("Expected an event, but didn't get one");
}

/**
 * Generate contstant altitudes
 *
 * @param context Pointer to a float, which is the constant alitude to generate
 * @param step The current time step - units of 0.01 seconds
 * @param alt Pointer to where the altitude sample information should be sotred
 * @return 1 if there was an altitude generated, or 0 if this sample shouldn't be used (to simulate missed readings)
 */
static int const_alt_generator(void *context, int step, struct altitude_sample* alt) {
    alt->altitude = *(float*)(context);
    alt->time = to_micro(0.01 * step);
    return 1;
}

static int missing_alt_generator(void *context, int step, struct altitude_sample* alt) {
    return 0;
}

/**
 * Generate contstant accelerations
 *
 * @param context Pointer to a float, which is the constant acceleration to generate
 * @param step The current time step - units of 0.01 seconds
 * @param accel Pointer to where the acceleration sample information should be stored
 * @return 1 if there was an altitude generated, or 0 if this sample shouldn't be used (to simulate missed readings)
 */
static int const_accel_generator(void *context, int step, struct accel_sample* accel) {
    accel->acceleration = *(float*)(context);
    accel->time = to_micro(0.01 * step);
    return 1;
}

static int missing_accel_generator(void *context, int step, struct accel_sample* accel) {
    return 0;
}

static void check_constant_altitude_no_event(float altitude) {
    struct detector detector;
    detector_init(&detector, 0);
    // Substate shouldn't matter here
    detector_set_state(&detector, STATE_IDLE, SUBSTATE_UNKNOWN);
    /* Test with an altitude of 0 */
    struct altitude_generator alt_gen = { .context = &altitude, .func = &const_alt_generator };
    struct accel_generator accel_gen = { .context = 0, .func = &missing_accel_generator };
    check_input_func_generates_output_event(&alt_gen, &accel_gen, 1000, &detector, DETECTOR_NO_EVENT);
}

static void test_constant_altitudes_idle_state__no_event(void) {
    check_constant_altitude_no_event(0);
    // Make sure we adjust to a new altitude before detection
    check_constant_altitude_no_event(1000);
    check_constant_altitude_no_event(10000);
}

static void test_constant_accel_idle_state__no_event(void) {
    struct detector detector;
    detector_init(&detector, 0);
    detector_set_state(&detector, STATE_IDLE, SUBSTATE_UNKNOWN);
    /* Test with an altitude of 0 */
    float accel = 9.81;
    struct altitude_generator alt_gen = { .context = 0, .func = &missing_alt_generator };
    struct accel_generator accel_gen = { .context = &accel, .func = &const_accel_generator };
    check_input_func_generates_output_event(&alt_gen, &accel_gen, 1000, &detector, DETECTOR_NO_EVENT);
}

void test_detection(void) {
    RUN_TEST(test_old_samples__no_detection);
    RUN_TEST(test_set_state_no_samples__no_event);
    RUN_TEST(test_constant_altitudes_idle_state__no_event);
    RUN_TEST(test_constant_accel_idle_state__no_event);
}
