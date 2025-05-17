#include <testing/unity.h>
#include <stdlib.h>


#include "test_runners.h"
#include "../telemetry/src/fusion/altitude-detection.h"


#define add_cluster(records, duration, offset, samples, altitude_gen) \
  for (int i = 0; i < samples; i++) { \
    struct altitude_sample sample = {to_micro_offset(offset, duration, samples, i), altitude_gen}; \
    add_sample(&records, &sample); \
  }

static unsigned long to_micro_offset(float offset, float duration, int samples, int i) {
  return (unsigned long)(1000000 * (i) * (duration / samples) + (offset * 1000000.0));
}

void test_no_samples__no_detection(void) {
  struct altitude_records records;
  init_records(&records);          
  TEST_ASSERT_EQUAL(FUSION_NO_EVENT, last_event(&records));
}

void test_one_sample__no_detection(void) {
  struct altitude_records records;
  init_records(&records);          
  struct altitude_sample sample = {0, 0};
  add_sample(&records, &sample);
  TEST_ASSERT_EQUAL(FUSION_NO_EVENT, last_event(&records));
}

void test_set_airborne__clear_after_detection(void) {
  struct altitude_records records;
  init_records(&records);          
  TEST_ASSERT_EQUAL(FUSION_NO_EVENT, last_event(&records));
  // Add a sample that triggers an airborne event
  add_cluster(records, 3, 0, 100, i);
  TEST_ASSERT_EQUAL(FUSION_AIRBORNE_EVENT, last_event(&records));
  TEST_ASSERT_EQUAL(FUSION_NO_EVENT, last_event(&records));
}

void test_fast_short_time_one_cluster__no_detection(void) {
  // Create a cluster of samples thats increasing 100 meters in 0.5 seconds
  struct altitude_records records;
  init_records(&records);          
  add_cluster(records, 0.5, 0, 100, i);
  TEST_ASSERT_EQUAL(FUSION_NO_EVENT, last_event(&records));
}

void test_fast_short_time_two_cluster__no_detection(void) {
  // Create a cluster of samples at 0 meters and one at 10000 meters 1 second later
  struct altitude_records records;
  init_records(&records);          
  // Initial 2 second cluster
  add_cluster(records, 2, 0, 50, 0);
  TEST_ASSERT_NOT_EQUAL(FUSION_AIRBORNE_EVENT, last_event(&records));
  // 1 second later, a tiny cluster at 10000 meters
  add_cluster(records, 0.1, 1, 50, 10000);
  TEST_ASSERT_EQUAL(FUSION_NO_EVENT, last_event(&records));
}

void test_fast_one_cluster_increasing__airbone_detected(void) {
  // Create a cluster of samples thats increasing 100 meters in three seconds
  struct altitude_records records;
  init_records(&records);          
  add_cluster(records, 3, 0, 100, i);
  TEST_ASSERT_EQUAL(FUSION_AIRBORNE_EVENT, last_event(&records));
}

void test_fast_one_cluster_decreasing__airbone_detected(void) {
  struct altitude_records records;
  init_records(&records);          
  add_cluster(records, 3, 0, 100, 100 - i);
  TEST_ASSERT_EQUAL(FUSION_AIRBORNE_EVENT, last_event(&records));
}

void test_fast_long_time_increasing_one_cluster__airbone_detected(void) {
  // Create a cluster of samples decreasing 10000 meters in 10 seconds
  struct altitude_records records;
  init_records(&records);          
  add_cluster(records, 10, 0, 10000, 10000 - i);
  TEST_ASSERT_EQUAL(FUSION_AIRBORNE_EVENT, last_event(&records));
}

void test_fast_long_time_decreasing_one_cluster__airbone_detected(void) {
  // Create a cluster of samples increasing 10000 meters in 10 seconds
  struct altitude_records records;
  init_records(&records);          
  add_cluster(records, 10, 0, 10000, i);
  TEST_ASSERT_EQUAL(FUSION_AIRBORNE_EVENT, last_event(&records));
}

void test_fast_two_cluster_decreasing__airborne_detected(void) {
  // Create a cluster of samples at 100 meters and one at 0 meters a few seconds later
  struct altitude_records records;
  init_records(&records);          
  add_cluster(records, 2, 0, 50, 100);
  TEST_ASSERT_NOT_EQUAL(FUSION_AIRBORNE_EVENT, last_event(&records));
  add_cluster(records, 2, 5, 50, 0);
  TEST_ASSERT_EQUAL(FUSION_AIRBORNE_EVENT, last_event(&records));
}

void test_fast_two_cluster_increasing__airborne_detected(void) {
  struct altitude_records records;
  init_records(&records);          
  add_cluster(records, 2, 0, 50, 0);
  TEST_ASSERT_NOT_EQUAL(FUSION_AIRBORNE_EVENT, last_event(&records));
  add_cluster(records, 5, 5, 50, 100);
  TEST_ASSERT_EQUAL(FUSION_AIRBORNE_EVENT, last_event(&records));
}

void test_slow_one_cluster_decreasing__no_detection(void) {
  // Create a cluster of samples that is decreasing from 100 meters in 10 seconds
  struct altitude_records records;
  init_records(&records);          
  add_cluster(records, 10, 0, 1000, 100 - (0.01 * i));
  TEST_ASSERT_EQUAL(FUSION_NO_EVENT, last_event(&records));
}

void test_slow_one_cluster_increasing__no_detection(void) {
  // Same but opposite altitudes
  struct altitude_records records;
  init_records(&records);          
  add_cluster(records, 10, 0, 1000, 100 + (0.01 * i));
  TEST_ASSERT_EQUAL(FUSION_NO_EVENT, last_event(&records));
}

void test_slow_two_cluster__no_detection(void) {
  // Create a cluster of samples at 100 meters and one at 0 meters 10 seconds later
  struct altitude_records records;
  init_records(&records);          
  add_cluster(records, 2, 0, 50, 100);
  TEST_ASSERT_NOT_EQUAL(FUSION_AIRBORNE_EVENT, last_event(&records));
  add_cluster(records, 2, 10, 50, 0);
  TEST_ASSERT_EQUAL(FUSION_NO_EVENT, last_event(&records));
}

void test_noisy_altitude__no_detection(void) {
  // Create nosiy altitude measurements at 100 meters over 10 seconds
  struct altitude_records records;
  init_records(&records);          
  add_cluster(records, 10, 0, 1000, 100 + (rand() % 10));
  TEST_ASSERT_EQUAL(FUSION_NO_EVENT, last_event(&records));
}

void test_high_static_altitue__no_detection(void) {
  // Create a cluster of samples at 1000 meters for 10 seconds
  struct altitude_records records;
  init_records(&records);          
  add_cluster(records, 10, 0, 1000, 1000);
  TEST_ASSERT_EQUAL(FUSION_NO_EVENT, last_event(&records));
}

void test_low_noisy_altitude__landing_detected(void) {
  // Create nosiy altitude measurements at 0 meters over 10 seconds
  struct altitude_records records;
  init_records(&records);          
  add_cluster(records, 10, 0, 1000, (rand() % 10));
  TEST_ASSERT_EQUAL(FUSION_LANDING_EVENT, last_event(&records));
}

void test_low_static_altitude__landing_detected(void) {
  // Create a cluster of samples at 0 meters for 10 seconds
  struct altitude_records records;
  init_records(&records);          
  add_cluster(records, 10, 0, 1000, 0);
  TEST_ASSERT_EQUAL(FUSION_LANDING_EVENT, last_event(&records));
}

void test_low_very_slow_increasing_altitude__landing_detected(void) {
  // Create a cluster of samples at 0 meters for 3 seconds
  struct altitude_records records;
  init_records(&records);          
  add_cluster(records, 3, 0, 1000, i * 0.001);
  TEST_ASSERT_EQUAL(FUSION_LANDING_EVENT, last_event(&records));
}

void test_low_very_slow_decreasing_altitude__landing_detected(void) {
  // Create a cluster of samples at 0 meters for 3 seconds
  struct altitude_records records;
  init_records(&records);          
  add_cluster(records, 3, 0, 1000, i * -0.001);
  TEST_ASSERT_EQUAL(FUSION_LANDING_EVENT, last_event(&records));
}

void test_real_takeoff__airborne_detected(void) {
  /* Equation generated by fitting a polynomial to data from flight on 2024-06-24,
   * seconds 1 - 5.4. The equation has a R value of 0.9978. Each loop is 10 ms
   */
  struct altitude_records records;
  init_records(&records);          
  for (int i = 100; i < 541; i++) {
    float x = i / 100;
    float altitude = -6E-05 * (x * x * x) + 0.0504 * (x * x) - 4.4471 * x + 1614.4;
    struct altitude_sample sample = {i * 10000.0, altitude};
    add_sample(&records, &sample);
  }
  TEST_ASSERT_EQUAL(FUSION_AIRBORNE_EVENT, last_event(&records));
}

void test_real_landing__landing_detected(void)

void test_altitude_detection() {
  RUN_TEST(test_no_samples__no_detection);
  RUN_TEST(test_one_sample__no_detection);
  RUN_TEST(test_set_airborne__clear_after_detection);
  RUN_TEST(test_fast_short_time_one_cluster__no_detection);
  RUN_TEST(test_fast_short_time_two_cluster__no_detection);
  RUN_TEST(test_fast_one_cluster_increasing__airbone_detected);
  RUN_TEST(test_fast_one_cluster_decreasing__airbone_detected);
  RUN_TEST(test_fast_long_time_increasing_one_cluster__airbone_detected);
  RUN_TEST(test_fast_long_time_decreasing_one_cluster__airbone_detected);
  RUN_TEST(test_fast_two_cluster_decreasing__airborne_detected);
  RUN_TEST(test_fast_two_cluster_increasing__airborne_detected);
  RUN_TEST(test_slow_one_cluster_decreasing__no_detection);
  RUN_TEST(test_slow_one_cluster_increasing__no_detection);
  RUN_TEST(test_slow_two_cluster__no_detection);
  RUN_TEST(test_noisy_altitude__no_detection);
  RUN_TEST(test_high_static_altitue__no_detection);
  RUN_TEST(test_low_noisy_altitude__landing_detected);
  RUN_TEST(test_low_static_altitude__landing_detected);
  RUN_TEST(test_low_very_slow_increasing_altitude__landing_detected);
  RUN_TEST(test_low_very_slow_decreasing_altitude__landing_detected);
  RUN_TEST(test_real_takeoff__airborne_detected);

}
