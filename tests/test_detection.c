#include <testing/unity.h>
#include <stdlib.h>
#include <math.h>


#include "test_runners.h"
#include "flight-reproduction.h"
#include "../telemetry/src/fusion/filter.h"

static void test_filter_init(void) {
    struct filter f;
    filter_init(&f);
    struct rocket_dynamics dynamics = {0};
    struct altitude_sample sample = { .timestamp = 0, .altitude = 10 };
    filter_add_sample(&f, &sample, &dynamics);
    TEST_ASSERT_EQUAL_FLOAT(0, dynamics.altitude);
}

static void test_filter_constant_altitude(void) {
    struct filter f;
    filter_init(&f);
    struct rocket_dynamics dynamics = {0};
    for (int i = 0; i < 2 * FILTER_WINDOW_SIZE; i++) {
        struct altitude_sample sample = { .timestamp = i * 1000000, .altitude = 10 };
        filter_add_sample(&f, &sample, &dynamics);
    }
    TEST_ASSERT_EQUAL_FLOAT(10.0, dynamics.altitude);
    TEST_ASSERT_EQUAL_FLOAT(0.0, dynamics.velocity);
}

static void test_filter_constant_velocity(void) {
    struct filter f;
    filter_init(&f);
    struct rocket_dynamics dynamics = {0};
    for (int i = 0; i < 2 * FILTER_WINDOW_SIZE; i++) {
      struct altitude_sample sample = { .timestamp = i * 1000000, .altitude = 10 * i};
        filter_add_sample(&f, &sample, &dynamics);
    }
    TEST_ASSERT_EQUAL_FLOAT(10.0, dynamics.velocity);
}

static void test_filter_constant_altitude_with_noise(void) {
    struct filter f;
    filter_init(&f);
    struct rocket_dynamics dynamics = {0};
    for (int i = 0; i < 2 * FILTER_WINDOW_SIZE; i++) {
        struct altitude_sample sample = { .timestamp = i * 1000000, .altitude = 10 + (rand() % 5) };
        filter_add_sample(&f, &sample, &dynamics);
    }
    TEST_ASSERT_FLOAT_WITHIN(5.0, 10.0, dynamics.altitude);
    TEST_ASSERT_FLOAT_WITHIN(2.0, 0.0, dynamics.velocity);
}

static void test_filter_constant_velocity_with_noise(void) {
    struct filter f;
    filter_init(&f);
    struct rocket_dynamics dynamics = {0};
    for (int i = 0; i < 2 * FILTER_WINDOW_SIZE; i++) {
        struct altitude_sample sample = { .timestamp = i * 1000000, .altitude = 10 * i + (rand() % 5) };
        filter_add_sample(&f, &sample, &dynamics);
    }
    TEST_ASSERT_FLOAT_WITHIN(5.0, 10.0, dynamics.velocity);
}


static void test_filter_on_flight(double (*noise_func)(double)) {
    struct filter f;
    filter_init(&f);
    struct rocket_dynamics dynamics = {0};
    int count = 0;
    double altitude_mse = 0.0;
    double velocity_mse = 0.0;
    for (double time = 0; time < 1330.0; time += 0.1, count++) {
        struct altitude_sample sample = { .timestamp = time * 1000000, .altitude = reproduce_flight_altitude(time) + noise_func(time) };
        filter_add_sample(&f, &sample, &dynamics);
        altitude_mse += pow(dynamics.altitude - reproduce_flight_altitude(time), 2);
        velocity_mse += pow(dynamics.velocity - reproduce_flight_velocity(time), 2);
    }
    printf("Mean squared altitude error: %f\n", altitude_mse /= count);
    printf("Mean squared velocity error: %f\n", velocity_mse /= count);
}

static double no_noise_func(double t) {
    return 0;
}

static double rand_noise_func(double t) {
    return (rand() % 16) - 8;
}

static double sin_8m_pi_period_noise_func(double t) {
    return 8 * sin(t);
}

static double sin_16m_2pi_period_noise_func(double t) {
    return 16 * sin(0.5 * t);
}

static void test_filter_clean_flight(void) {
  test_filter_on_flight(no_noise_func);
}

static void test_filter_rand_16m_noise_flight(void) {
  test_filter_on_flight(rand_noise_func);
}

static void test_filter_sin_8m_2pi_period_noise_flight(void) {
  test_filter_on_flight(sin_8m_pi_period_noise_func);
}

static void test_filter_sin_16m_4pi_period_noise_flight(void) {
  test_filter_on_flight(sin_16m_2pi_period_noise_func);
}

static void test_initial_takeoff__airborne_detected(void) {

}

static void test_late_takeoff__airborne_detected(void) {

}

static void test_mid_ascent__airborne_detected(void) {

}

static void test_late_ascent__airborne_detected(void) {

}

static void test_apogee__airborne_detected(void) {

}

static void test_deployment__airborne_detected(void) {

}

static void test_early_descent__airborne_detected(void) {

}

static void test_late_descent__airborne_detected(void) {

}

static void test_landing__airborne_detected(void) {

}

void test_detection(void) {
  RUN_TEST(test_filter_init);
  RUN_TEST(test_filter_constant_altitude);
  RUN_TEST(test_filter_constant_velocity);
  RUN_TEST(test_filter_constant_altitude_with_noise);
  RUN_TEST(test_filter_constant_velocity_with_noise);
  RUN_TEST(test_filter_clean_flight);
  RUN_TEST(test_filter_rand_16m_noise_flight);
  RUN_TEST(test_filter_sin_8m_2pi_period_noise_flight);
  RUN_TEST(test_filter_sin_16m_4pi_period_noise_flight);
}
