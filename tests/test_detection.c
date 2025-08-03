#include <testing/unity.h>

#include "../telemetry/src/fusion/detector.h"
#include "test_runners.h"

/* Convert a time in seconds to a time in microseconds*/
static unsigned long to_micro(float seconds) { return seconds * 1000000; }

/* Information needed to generate altitude or acceleration samples for testing */
struct generator {
    float *params; /* Context information for the generator */

    /* Pointer to a function, where params is a float array of parameters as expected by the pointed to function,
     * where time is the current time in seconds, and to_fill is the output of the function. The return value decides
     * if the value of to_fill should be used (1) or ignored (0)
     */
    int (*func)(float *params, float time, float *to_fill);
};

/**
 * Parameterized test for trying different altitude/acceleration combinations
 *
 * @param alt_gen Function and its context for generating altitude samples
 * @param accel_gen Function and its context for generating accel samples
 * @param start The time to start the samples at
 * @param duration The number of simulated seconds over which data will be input
 * @param detector The detector to use, initialized
 * @param expected_event The only event that should be output besides DETECTOR_NO_EVENT, and which must be output at
 * least one time
 */
static int check_gen_event_full(struct generator *alt_gen, struct generator *accel_gen, float start, float duration,
                                struct detector *detector, enum detector_event expected_event) {
    int got_expected_event = 0;
    struct altitude_sample alt = {0};
    struct accel_sample accel = {0};
    // Samples are input at 0.01 intervals - this might be good to mess with/tune to actual sensor input rates
    for (float time = start; time < start + duration; time += 0.01) {
        if (accel_gen->func(accel_gen->params, time, &accel.acceleration)) {
            accel.time = to_micro(time);
            detector_add_accel(detector, &accel);
        }
        if (alt_gen->func(alt_gen->params, time, &alt.altitude)) {
            alt.time = to_micro(time);
            detector_add_alt(detector, &alt);
        }
        enum detector_event event = detector_detect(detector);
        if (event == expected_event) {
            got_expected_event = 1;
        } else if (event != DETECTOR_NO_EVENT) {
            char msg[100];
            snprintf(msg, sizeof(msg), "Test failure: expected event %d, got %d at time %f", expected_event, event,
                     time);
            TEST_MESSAGE(msg);
            return 0;
        }
    }
    if (!got_expected_event) {
        TEST_MESSAGE("Expected an event, but didn't get one");
        return 0;
    }
    return 1;
}

static int check_gen_event(struct generator *alt_gen, struct generator *accel_gen, float duration,
                           struct detector *detector, enum detector_event expected_event) {
    return check_gen_event_full(alt_gen, accel_gen, 0, duration, detector, expected_event);
}

/* Generate contstant float values, where params is an array of length one with the constant value to output
 */
static int const_generator(float *params, float time, float *to_fill) {
    *to_fill = params[0];
    return 1;
}

/* Generate missing float values by always saying not to use to_fill, where params is unused
 */
static int missing_generator(float *params, float time, float *to_fill) { return 0; }

/* Generate one value, then another. Params describes the time threshold, first value,
 * and second value
 */
static int edge_generator(float *params, float time, float *to_fill) {
    float time_threshold = params[0];
    float level_one = params[1];
    float level_two = params[2];
    if (time > time_threshold) {
        *to_fill = level_two;
        return 1;
    }
    *to_fill = level_one;
    return 1;
}

/* Generate values using a cubic function
 */
static int cubic_generator(float *params, float time, float *to_fill) {
    *to_fill = params[0] * time * time * time + params[1] * time * time + params[2] * time + params[3];
    return 1;
}

/* Generate values using a quadratic function
 */
static int quad_generator(float *params, float time, float *to_fill) {
    *to_fill = params[0] * time * time + params[1] * time + params[2];
    return 1;
}

/* Generate values using a linear function
 */
static int linear_generator(float *params, float time, float *to_fill) {
    *to_fill = params[0] * time + params[1];
    return 1;
}

static void check_constant_altitude_no_event(float altitude) {
    struct detector detector;
    detector_init(&detector, 0);
    // Substate shouldn't matter here
    detector_set_state(&detector, STATE_IDLE, SUBSTATE_UNKNOWN);
    /* Test with an altitude of 0 */
    struct generator alt_gen = {.params = &altitude, .func = &const_generator};
    struct generator accel_gen = {.func = &missing_generator};
    TEST_ASSERT(check_gen_event(&alt_gen, &accel_gen, 10, &detector, DETECTOR_NO_EVENT));
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
    struct generator alt_gen = {.func = &missing_generator};
    struct generator accel_gen = {.params = &accel, .func = &const_generator};
    TEST_ASSERT(check_gen_event(&alt_gen, &accel_gen, 10, &detector, DETECTOR_NO_EVENT));
}

static void test_airborne_increasing_alt__no_event(void) {
    enum flight_substate_e substates_to_test[] = {SUBSTATE_UNKNOWN, SUBSTATE_ASCENT, SUBSTATE_DESCENT};

    for (int i = 0; i < sizeof(substates_to_test) / sizeof(substates_to_test[0]); i++) {
        struct detector detector;
        detector_init(&detector, 0);
        detector_set_state(&detector, STATE_AIRBORNE, substates_to_test[i]);

        // Following the equation y = 100x + 100, where x is the current time, in seconds
        float params[] = {100, 100};
        struct generator alt_gen = {.params = params, .func = &linear_generator};
        struct generator accel_gen = {.func = &missing_generator};

        char msg[100];
        snprintf(msg, sizeof(msg), "Testing increasing altitude in airborne state with substate %d",
                 substates_to_test[i]);
        TEST_MESSAGE(msg);
        TEST_ASSERT(check_gen_event(&alt_gen, &accel_gen, 10, &detector, DETECTOR_NO_EVENT));
    }
}

static void test_airborne_high_accel__no_event(void) {
    /* This test is trying the type of conditions found in transonic flight, where altitude appears to be decreasing
     * even though acceleration is high */
    enum flight_substate_e substates_to_test[] = {SUBSTATE_UNKNOWN, SUBSTATE_ASCENT};

    for (int i = 0; i < sizeof(substates_to_test) / sizeof(substates_to_test[0]); i++) {
        struct detector detector;
        detector_init(&detector, 0);
        detector_set_state(&detector, STATE_AIRBORNE, substates_to_test[i]);

        // Following the equation y = -1x + 100, where x is the current time in seconds
        float params[] = {-1, 1000};
        struct generator alt_gen = {.params = params, .func = &linear_generator};
        // A high acceleration of 15 m/s^2 should prevent us from moving between states
        float accel = 15;
        struct generator accel_gen = {.params = &accel, .func = &const_generator};

        char msg[100];
        snprintf(msg, sizeof(msg), "Testing high accel in airborne state with substate %d", substates_to_test[i]);
        TEST_MESSAGE(msg);
        TEST_ASSERT(check_gen_event(&alt_gen, &accel_gen, 10, &detector, DETECTOR_NO_EVENT));
    }
}

static void test_airborne_decreasing_alt__no_event(void) {
    enum flight_substate_e substates_to_test[] = {SUBSTATE_UNKNOWN, SUBSTATE_DESCENT};

    for (int i = 0; i < sizeof(substates_to_test) / sizeof(substates_to_test[0]); i++) {
        struct detector detector;
        detector_init(&detector, 0);
        detector_set_state(&detector, STATE_AIRBORNE, substates_to_test[i]);

        // Following the equation y = -1.5x + 100, where x is the current time in seconds
        float params[] = {-1.5, 1000};
        struct generator alt_gen = {.params = params, .func = &linear_generator};
        // As if we were falling on the parachute
        float accel = 9.81;
        struct generator accel_gen = {.params = &accel, .func = &const_generator};

        char msg[100];
        snprintf(msg, sizeof(msg), "Testing slow descent in airborne state with substate %d", substates_to_test[i]);
        TEST_MESSAGE(msg);

        TEST_ASSERT(check_gen_event(&alt_gen, &accel_gen, 10, &detector, DETECTOR_NO_EVENT));
    }
}

static void test_idle_increasing_alt__airborne_event(void) {
    struct detector detector;
    detector_init(&detector, 0);
    detector_set_state(&detector, STATE_IDLE, SUBSTATE_UNKNOWN);

    // Following the equation y = 50x + 100, where x is the current time in seconds
    float params[] = {50, 100};
    struct generator alt_gen = {.params = params, .func = &linear_generator};
    float accel = 9.81;
    struct generator accel_gen = {.params = &accel, .func = &const_generator};

    // Give five seconds to recognize liftoff, to let the idle altitude be chosen
    TEST_ASSERT(check_gen_event(&alt_gen, &accel_gen, 5, &detector, DETECTOR_AIRBORNE_EVENT));
}

static void test_idle_alt_jump__liftoff_event(void) {
    struct detector detector;
    detector_init(&detector, 0);
    detector_set_state(&detector, STATE_IDLE, SUBSTATE_UNKNOWN);

    // After 3 seconds, jump from 100 to 200 meters altitude
    float params[] = {3, 100, 200};
    struct generator alt_gen = {.params = params, .func = &edge_generator};
    float accel = 9.81;
    struct generator accel_gen = {.params = &accel, .func = &const_generator};

    // Give 2 seconds to recognize the liftoff condition
    TEST_ASSERT(check_gen_event(&alt_gen, &accel_gen, 5, &detector, DETECTOR_AIRBORNE_EVENT));
}

static void test_idle_high_accel__liftoff_event(void) {
    struct detector detector;
    detector_init(&detector, 0);
    detector_set_state(&detector, STATE_IDLE, SUBSTATE_UNKNOWN);

    struct generator alt_gen = {.func = &missing_generator};
    float accel = 20;
    struct generator accel_gen = {.params = &accel, .func = &const_generator};

    // Only give 2 seconds to recognize the liftoff condition
    TEST_ASSERT(check_gen_event(&alt_gen, &accel_gen, 2, &detector, DETECTOR_AIRBORNE_EVENT));
}

static void test_idle_liftoff_conditions__liftoff_event(void) {
    struct detector detector;
    detector_init(&detector, 0);
    detector_set_state(&detector, STATE_IDLE, SUBSTATE_UNKNOWN);

    float idle_alt = 1000;
    struct generator alt_gen = {.params = &idle_alt, .func = &const_generator};
    float idle_accel = 9.81;
    struct generator accel_gen = {.params = &idle_accel, .func = &const_generator};

    // Let the detector get used to idle conditions
    TEST_ASSERT(check_gen_event(&alt_gen, &accel_gen, 2, &detector, DETECTOR_NO_EVENT));

    float liftoff_alt_params[] = {100, idle_alt};
    alt_gen.params = liftoff_alt_params;
    alt_gen.func = &linear_generator;

    float liftoff_accel = 20;
    accel_gen.params = &liftoff_accel;
    // Only give 2 seconds to recognize the liftoff condition
    TEST_ASSERT(check_gen_event(&alt_gen, &accel_gen, 2, &detector, DETECTOR_AIRBORNE_EVENT));
}

static void test_ascent_decreasing_alt_low_accel__apogee_event(void) {
    enum flight_substate_e substates_to_test[] = {SUBSTATE_UNKNOWN, SUBSTATE_ASCENT};

    for (int i = 0; i < sizeof(substates_to_test) / sizeof(substates_to_test[0]); i++) {
        struct detector detector;
        detector_init(&detector, 0);
        detector_set_state(&detector, STATE_AIRBORNE, substates_to_test[i]);

        float params[] = {-10, 1000};
        struct generator alt_gen = {.params = params, .func = &linear_generator};
        // As if we were falling on the parachute
        float accel = 9.81;
        struct generator accel_gen = {.params = &accel, .func = &const_generator};

        char msg[100];
        snprintf(msg, sizeof(msg), "Testing with substate %d", substates_to_test[i]);
        TEST_MESSAGE(msg);

        TEST_ASSERT(check_gen_event(&alt_gen, &accel_gen, 10, &detector, DETECTOR_APOGEE_EVENT));
    }
}

static void test_ascent_increasing_decreasing_alt_low_accel__apogee_event(void) {
    enum flight_substate_e substates_to_test[] = {SUBSTATE_UNKNOWN, SUBSTATE_ASCENT};

    for (int i = 0; i < sizeof(substates_to_test) / sizeof(substates_to_test[0]); i++) {
        struct detector detector;
        detector_init(&detector, 0);
        detector_set_state(&detector, STATE_AIRBORNE, substates_to_test[i]);

        float ascent_alt_params[] = {1, 100};
        float accel = 9.81;

        struct generator alt_gen = {.params = ascent_alt_params, .func = &linear_generator};
        struct generator accel_gen = {.params = &accel, .func = &const_generator};

        TEST_ASSERT(check_gen_event(&alt_gen, &accel_gen, 10, &detector, DETECTOR_NO_EVENT));

        float descent_alt_params[] = {-10, 1000};
        // As if we were falling on the parachute
        alt_gen.params = descent_alt_params;

        char msg[100];
        snprintf(msg, sizeof(msg), "Testing with substate %d", substates_to_test[i]);
        TEST_MESSAGE(msg);

        TEST_ASSERT(check_gen_event(&alt_gen, &accel_gen, 10, &detector, DETECTOR_APOGEE_EVENT));
    }
}

static void test_descent_static_alt__landed_event(void) {
    enum flight_substate_e substates_to_test[] = {SUBSTATE_UNKNOWN, SUBSTATE_DESCENT};

    for (int i = 0; i < sizeof(substates_to_test) / sizeof(substates_to_test[0]); i++) {
        struct detector detector;
        detector_init(&detector, 0);
        detector_set_state(&detector, STATE_AIRBORNE, substates_to_test[i]);

        // Make sure can get back to landed without having to be in idle at some point

        float alt = 1000;
        struct generator alt_gen = {.params = &alt, .func = &const_generator};
        float accel = 9.81;
        struct generator accel_gen = {.params = &accel, .func = &const_generator};

        char msg[100];
        snprintf(msg, sizeof(msg), "Testing with substate %d", substates_to_test[i]);
        TEST_MESSAGE(msg);

        TEST_ASSERT(check_gen_event(&alt_gen, &accel_gen, 15, &detector, DETECTOR_LANDING_EVENT));
    }
}

static void test_descent_landed_alt_diff_than_elevation__landed_event(void) {
    enum flight_substate_e substates_to_test[] = {SUBSTATE_UNKNOWN, SUBSTATE_DESCENT};

    for (int i = 0; i < sizeof(substates_to_test) / sizeof(substates_to_test[0]); i++) {
        struct detector detector;
        detector_init(&detector, 0);
        detector_set_state(&detector, STATE_AIRBORNE, substates_to_test[i]);
        detector_set_elevation(&detector, 0);

        float alt = 1000;
        struct generator alt_gen = {.params = &alt, .func = &const_generator};
        float accel = 9.81;
        struct generator accel_gen = {.params = &accel, .func = &const_generator};

        char msg[100];
        snprintf(msg, sizeof(msg), "Testing with substate %d", substates_to_test[i]);
        TEST_MESSAGE(msg);

        TEST_ASSERT(check_gen_event(&alt_gen, &accel_gen, 15, &detector, DETECTOR_LANDING_EVENT));
        detector_set_state(&detector, STATE_IDLE, SUBSTATE_UNKNOWN);

        // Make sure that the new altitude doesn't set off the detector when it goes back to idle
        TEST_ASSERT(check_gen_event(&alt_gen, &accel_gen, 10, &detector, DETECTOR_NO_EVENT));
    }
}

static void check_flight_first_second__airborne_event(int with_startup) {
    struct detector detector;
    detector_init(&detector, 0);
    detector_set_state(&detector, STATE_IDLE, SUBSTATE_UNKNOWN);

    // Paramaters chosen by running a regression on flight data from -0.1 seconds to 1 second. Shifted to right by 1s
    // R squared = 0.9994
    float alt_params[] = {43.4089627, 42.01320401, 1249.89273701};
    // R squared = 0.9885
    float accel_params[] = {259.52483303, -1166.49775242, 1727.3929237, -755.12805506};

    float startup_duration = 1;
    if (with_startup) {
        // Pretend the altitude at the start of the test data is the landed altitude
        float alt = 0;
        quad_generator(alt_params, 0, &alt);
        struct generator alt_gen = {.params = &alt_params[2], .func = &const_generator};
        float accel = 9.81;
        struct generator accel_gen = {.params = &accel, .func = &const_generator};
        TEST_ASSERT(check_gen_event(&alt_gen, &accel_gen, startup_duration, &detector, DETECTOR_NO_EVENT));
    }

    struct generator alt_gen = {.params = alt_params, .func = &quad_generator};
    struct generator accel_gen = {.params = accel_params, .func = &cubic_generator};

    TEST_ASSERT(check_gen_event_full(&alt_gen, &accel_gen, startup_duration, 1, &detector, DETECTOR_AIRBORNE_EVENT));
}

static void test_flight_first_second_no_setup__airborne_event(void) { check_flight_first_second__airborne_event(0); }

static void test_flight_first_second_with_setup__airborne_event(void) { check_flight_first_second__airborne_event(1); }

static void check_flight_mach_lockout__no_event_or_airborne(enum flight_state_e starting_state,
                                                            enum flight_substate_e starting_substate,
                                                            enum detector_event expected_event) {
    struct detector detector;
    detector_init(&detector, 0);
    detector_set_state(&detector, starting_state, starting_substate);

    // Paramaters chosen by running a regression on flight data from 1 to 3 seconds
    // R squared = 0.9955 (altitude decreases slightly in real data, but that isn't reproduced here)
    // There is already a testcase for making sure a high acceleration prevents the apogee event
    float alt_params[] = {219.03708427810378, -1007.1096609999761, 1577.993248622043, 723.1140984079647};
    // R squared = 0.9948
    float accel_params[] = {-108.7116484966559, 462.78404694094246, -385.9271310079958, 126.97995409670543};

    struct generator alt_gen = {.params = alt_params, .func = &cubic_generator};
    struct generator accel_gen = {.params = accel_params, .func = &cubic_generator};

    TEST_ASSERT(check_gen_event_full(&alt_gen, &accel_gen, 1, 2, &detector, expected_event));
}

static void test_flight_mach_lockout_ascent__no_event(void) {
    check_flight_mach_lockout__no_event_or_airborne(STATE_AIRBORNE, SUBSTATE_UNKNOWN, DETECTOR_NO_EVENT);
    check_flight_mach_lockout__no_event_or_airborne(STATE_AIRBORNE, SUBSTATE_ASCENT, DETECTOR_NO_EVENT);
}

static void test_flight_mach_lockout_idle__airborne_event(void) {
    check_flight_mach_lockout__no_event_or_airborne(STATE_IDLE, SUBSTATE_UNKNOWN, DETECTOR_AIRBORNE_EVENT);
}

static void test_flight_apogee__apogee_event(void) {
    // In a real flight, the deploying of the parachutes causes a pressure spike (looks like a momentary loss of 500m
    // alt) but because this happens after apogee. That isn't reproduced here
    // Note - apogee occurs around 40 seconds

    struct detector detector;
    detector_init(&detector, 0);
    detector_set_state(&detector, STATE_AIRBORNE, SUBSTATE_ASCENT);

    // Paramaters chosen by running a regression on flight data from 38 to 46 seconds
    // R squared = 0.2240 (because of some pretty noisy data)
    float alt_params[] = {-1.5724512511009474, 122.24740175656993, 7664.780504229671};
    // The deployment messes with the acceleration in the flight data, because of the kalman filter. Ignore that effect
    float accel = 9.81;

    struct generator alt_gen = {.params = alt_params, .func = &quad_generator};
    struct generator accel_gen = {.params = &accel, .func = &const_generator};

    TEST_ASSERT(check_gen_event_full(&alt_gen, &accel_gen, 38, 8, &detector, DETECTOR_APOGEE_EVENT));
}

// TODO - Tests with noise

void test_detection(void) {
    RUN_TEST(test_no_samples__no_event);

    // Negatives
    RUN_TEST(test_constant_altitudes_idle_state__no_event);
    RUN_TEST(test_constant_accel_idle_state__no_event);

    RUN_TEST(test_airborne_increasing_alt__no_event);
    RUN_TEST(test_airborne_high_accel__no_event);
    RUN_TEST(test_airborne_decreasing_alt__no_event);

    // Positives
    RUN_TEST(test_idle_increasing_alt__airborne_event);
    RUN_TEST(test_idle_alt_jump__liftoff_event);
    RUN_TEST(test_idle_high_accel__liftoff_event);
    RUN_TEST(test_idle_liftoff_conditions__liftoff_event);

    RUN_TEST(test_ascent_decreasing_alt_low_accel__apogee_event);
    RUN_TEST(test_ascent_increasing_decreasing_alt_low_accel__apogee_event);

    RUN_TEST(test_descent_static_alt__landed_event);
    RUN_TEST(test_descent_landed_alt_diff_than_elevation__landed_event);

    RUN_TEST(test_flight_first_second_with_setup__airborne_event);
    RUN_TEST(test_flight_first_second_no_setup__airborne_event);

    RUN_TEST(test_flight_mach_lockout_idle__airborne_event);
    RUN_TEST(test_flight_mach_lockout_ascent__no_event);

    RUN_TEST(test_flight_apogee__apogee_event);
}
