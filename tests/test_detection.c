#include <testing/unity.h>
#include <stdlib.h>
#include <math.h>


#include "test_runners.h"
#include "../telemetry/src/fusion/detector.h"

/* Convert a time in seconds to a time in microseconds*/
static unsigned long to_micro(float seconds) {
    return seconds * 1000000;
}

/* Information needed to generate altitude samples for testing*/
struct altitude_generator {
    void *context;                                                         /* Context information for the generator */
    int (*func)(void *context, int step, struct altitude_sample* to_fill); /* Pointer to a function to generate the alt sample */
};

/* Information needed to generate acceleration samples for testing */
struct accel_generator {
    void *context;                                                      /* Context information for the generator*/
    int (*func)(void *context, int step, struct accel_sample* to_fill); /* Pointer to a function to generate the accel sample */
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
static int check_input_func_generates_output_event(struct altitude_generator *alt_gen, struct accel_generator *accel_gen, 
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
            char msg[100];
            snprintf(msg, sizeof(msg), "Test failure: expected event %d, got %d", expected_event, event);
            TEST_MESSAGE(msg);
            return 0;
        }
    }
    if (expected_event != DETECTOR_NO_EVENT) {
        TEST_MESSAGE("Expected an event, but didn't get one\n");
        return 0;
    }
    return 1;
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
 * Generate altitudes using a linear function
 *
 * @param context Pointer to an array of two floats [a, b] representing the function y = ax + b
 * @param step The current time step - units of 0.01 seconds
 * @param alt Pointer to where the altitude sample information should be sotred
 * @return 1 if there was an altitude generated, or 0 if this sample shouldn't be used (to simulate missed readings)
 */
static int linear_alt_generator(void *context, int step, struct altitude_sample *alt) {
    float *params = context;
    alt->altitude = params[0] * step + params[1];
    alt->time = to_micro(0.01 * step);
    return 1;
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
    TEST_ASSERT(check_input_func_generates_output_event(&alt_gen, &accel_gen, 1000, &detector, DETECTOR_NO_EVENT));
}

static void test_no_samples__no_event(void) {
    struct detector detector;
    detector_init(&detector, 0);

    // Set states and make sure nothing happens with no samples
    detector_set_state(&detector, STATE_AIRBORNE, SUBSTATE_UNKNOWN);
    TEST_ASSERT_EQUAL_MESSAGE(DETECTOR_NO_EVENT, detector_detect(&detector), "Should not get an event without samples");

    detector_set_state(&detector, STATE_IDLE, SUBSTATE_UNKNOWN);
    TEST_ASSERT_EQUAL_MESSAGE(DETECTOR_NO_EVENT, detector_detect(&detector), "Should not get an event without samples");

    detector_set_state(&detector, STATE_LANDED, SUBSTATE_ASCENT);
    TEST_ASSERT_EQUAL_MESSAGE(DETECTOR_NO_EVENT, detector_detect(&detector), "Should not get an event without samples");
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
    TEST_ASSERT(check_input_func_generates_output_event(&alt_gen, &accel_gen, 1000, &detector, DETECTOR_NO_EVENT));
}

static void test_airborne_increasing_alt__no_event(void) {
    enum flight_substate_e substates_to_test[] = {SUBSTATE_UNKNOWN, SUBSTATE_ASCENT, SUBSTATE_DESCENT};

    for (int i = 0; i < sizeof(substates_to_test) / sizeof(substates_to_test[0]); i++) {
        struct detector detector;
        detector_init(&detector, 0);
        detector_set_state(&detector, STATE_AIRBORNE, substates_to_test[i]);

        // Following the equation y = 1x + 100, where x is the current time, in 0.01 seconds (velocity of 100m/s)
        float params[2] = { 1, 100};
        struct altitude_generator alt_gen = { .context = params, .func = &linear_alt_generator };
        struct accel_generator accel_gen = { .context = 0, .func = &missing_accel_generator };

        char msg[100];
        snprintf(msg, sizeof(msg), "Testing increasing altitude in airborne state with substate %d", substates_to_test[i]);
        TEST_MESSAGE(msg);
        TEST_ASSERT(check_input_func_generates_output_event(&alt_gen, &accel_gen, 1000, &detector, DETECTOR_NO_EVENT));
    }
}

static void test_airborne_high_accel__no_event(void) {
    /* This test is trying the type of conditions found in transonic flight, where altitude appears to be decreasing even though acceleration is high */
    enum flight_substate_e substates_to_test[] = {SUBSTATE_UNKNOWN, SUBSTATE_ASCENT};

    for (int i = 0; i < sizeof(substates_to_test) / sizeof(substates_to_test[0]); i++) {
        struct detector detector;
        detector_init(&detector, 0);
        detector_set_state(&detector, STATE_AIRBORNE, substates_to_test[i]);

        // Following the equation y = -0.01x + 100, where x is the current time, in 0.01 seconds (velocity of -1m/s)
        float params[2] = { -0.01, 1000};
        struct altitude_generator alt_gen = { .context = params, .func = &linear_alt_generator };
        // A high acceleration of 15 m/s^2 should prevent us from moving between states
        float accel = 15;
        struct accel_generator accel_gen = { .context = &accel, .func = &const_accel_generator };

        char msg[100];
        snprintf(msg, sizeof(msg), "Testing high accel in airborne state with substate %d", substates_to_test[i]);
        TEST_MESSAGE(msg);
        TEST_ASSERT(check_input_func_generates_output_event(&alt_gen, &accel_gen, 1000, &detector, DETECTOR_NO_EVENT));
    }
}

static void test_airborne_decreasing_alt__no_event(void) {
    enum flight_substate_e substates_to_test[] = {SUBSTATE_UNKNOWN, SUBSTATE_DESCENT};

    for (int i = 0; i < sizeof(substates_to_test) / sizeof(substates_to_test[0]); i++) {
        struct detector detector;
        detector_init(&detector, 0);
        detector_set_state(&detector, STATE_AIRBORNE, substates_to_test[i]);

        // Following the equation y = -0.015x + 100, where x is the current time, in 0.01 seconds (velocity of -1.5m/s)
        float params[2] = { -0.015, 1000};
        struct altitude_generator alt_gen = { .context = params, .func = &linear_alt_generator };
        // As if we were falling on the parachute
        float accel = 9.81;
        struct accel_generator accel_gen = { .context = &accel, .func = &const_accel_generator };

        char msg[100];
        snprintf(msg, sizeof(msg), "Testing slow descent in airborne state with substate %d", substates_to_test[i]);
        TEST_MESSAGE(msg);

        TEST_ASSERT(check_input_func_generates_output_event(&alt_gen, &accel_gen, 1000, &detector, DETECTOR_NO_EVENT));
    }
}

void test_detection(void) {
    RUN_TEST(test_no_samples__no_event);

    RUN_TEST(test_constant_altitudes_idle_state__no_event);
    RUN_TEST(test_constant_accel_idle_state__no_event);

    RUN_TEST(test_airborne_increasing_alt__no_event);
    RUN_TEST(test_airborne_high_accel__no_event);
    RUN_TEST(test_airborne_decreasing_alt__no_event);
}
