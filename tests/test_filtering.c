#include <float.h>
#include <nuttx/config.h>
#include <testing/unity.h>

#include "../telemetry/src/fusion/filtering.h"

/* Buffer sizes for testing */
#define TEST_FILTER_SIZE 5
#define TEST_LARGE_FILTER_SIZE 10

/* Median tests */

static void test_median_filter_single_value__returns_value(void) {
    struct median_filter filter;
    float sorted[TEST_FILTER_SIZE];
    float time_ordered[TEST_FILTER_SIZE];

    median_filter_init(&filter, sorted, time_ordered, TEST_FILTER_SIZE);

    float result = median_filter_add(&filter, 5.0f);
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(5.0f, result, "Single value should be returned as median");
}

static void test_median_filter_odd_values__returns_middle_value(void) {
    struct median_filter filter;
    float sorted[TEST_FILTER_SIZE];
    float time_ordered[TEST_FILTER_SIZE];

    median_filter_init(&filter, sorted, time_ordered, TEST_FILTER_SIZE);

    median_filter_add(&filter, 3.0f);
    median_filter_add(&filter, 1.0f);
    median_filter_add(&filter, 4.0f);
    median_filter_add(&filter, 2.0f);
    float result = median_filter_add(&filter, 5.0f);

    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(3.0f, result, "Incorrect median calculated");
}

static void test_median_filter_overflow__oldest_value_removed(void) {
    struct median_filter filter;
    float sorted[3];
    float time_ordered[3];

    median_filter_init(&filter, sorted, time_ordered, 3);

    median_filter_add(&filter, 1.0f);
    median_filter_add(&filter, 2.0f);
    median_filter_add(&filter, 3.0f);

    /* Add 4th value, should remove 1st */
    float result = median_filter_add(&filter, 4.0f);
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(3.0f, result, "Wrong median after adding to full filter");

    result = median_filter_add(&filter, 5.0f);
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(4.0f, result, "Wrong median after adding to full filter");

    result = median_filter_add(&filter, 6.0f);
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(5.0f, result, "Wrong median after adding to full filter");
}

static void test_median_filter_duplicates__handles_correctly(void) {
    struct median_filter filter;
    float sorted[TEST_FILTER_SIZE];
    float time_ordered[TEST_FILTER_SIZE];

    median_filter_init(&filter, sorted, time_ordered, TEST_FILTER_SIZE);

    median_filter_add(&filter, 2.0f);
    median_filter_add(&filter, 2.0f);
    median_filter_add(&filter, 2.0f);
    median_filter_add(&filter, 1.0f);
    float result = median_filter_add(&filter, 3.0f);

    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(2.0f, result, "Wrong median in filter with duplicates");
}

/* Average tests */

static void test_average_filter_single_value__returns_value(void) {
    struct average_filter filter;
    float buffer[TEST_FILTER_SIZE];

    average_filter_init(&filter, buffer, TEST_FILTER_SIZE);

    float result = average_filter_add(&filter, 5.0f);
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(5.0f, result, "Single value should be returned as average");
}

static void test_average_filter_multiple_values__calculates_correctly(void) {
    struct average_filter filter;
    float buffer[TEST_FILTER_SIZE];

    average_filter_init(&filter, buffer, TEST_FILTER_SIZE);

    float result = average_filter_add(&filter, 1.0f);
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(1.0f, result, "Wrong average for filter with one value");

    result = average_filter_add(&filter, 2.0f);
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(1.5f, result, "Wrong average for filter with two values");

    average_filter_add(&filter, 3.0f);
    average_filter_add(&filter, 4.0f);

    result = average_filter_add(&filter, 5.0f);
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(3.0f, result, "Wrong average of full filter");
}

/* Test average filter behavior when buffer overflows */
static void test_average_filter_overflow__moving_average(void) {
    struct average_filter filter;
    float buffer[3];

    average_filter_init(&filter, buffer, 3);

    average_filter_add(&filter, 1.0f);
    average_filter_add(&filter, 2.0f);

    float result = average_filter_add(&filter, 3.0f);
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(2.0f, result, "Wrong average for full filter");

    result = average_filter_add(&filter, 6.0f);
    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(11.0f / 3.0f, result, "Wrong average after filling filter");
}

static void test_average_filter_negative_values__handles_correctly(void) {
    struct average_filter filter;
    float buffer[TEST_FILTER_SIZE];

    average_filter_init(&filter, buffer, TEST_FILTER_SIZE);

    average_filter_add(&filter, -2.0f);
    average_filter_add(&filter, -1.0f);
    average_filter_add(&filter, 0.0f);
    average_filter_add(&filter, 1.0f);
    float result = average_filter_add(&filter, 2.0f);

    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.0f, result, "Average filter should handle negative values correctly");
}

static void test_average_filter_fractional_values__precise_calculation(void) {
    struct average_filter filter;
    float buffer[3];

    average_filter_init(&filter, buffer, 3);

    /* Add precise fractional values */
    average_filter_add(&filter, 0.1f);
    average_filter_add(&filter, 0.2f);
    float result = average_filter_add(&filter, 0.3f);

    TEST_ASSERT_EQUAL_FLOAT_MESSAGE(0.2f, result, "Average filter should handle fractional values precisely");
}

/* Window criteria tests */

static void test_window_criteria_first_value__sets_min_max(void) {
    struct window_criteria window;

    window_criteria_init(&window, 10.0f, 1000000);
    int result = window_criteria_satisfied(&window);
    TEST_ASSERT_FALSE_MESSAGE(result, "Window criteria should not be satisfied without providing values");

    // since_update satisfies the criteria here, but the window will be reset instead
    window_criteria_add(&window, 5.0f, 10000000);
    result = window_criteria_satisfied(&window);
    TEST_ASSERT_FALSE_MESSAGE(result, "Window criteria should not be satisfied on first value");
}

static void test_window_criteria_not_satisfied__returns_false(void) {
    struct window_criteria window;

    window_criteria_init(&window, 5.0f, 1000000);

    /* Add values but don't accumulate enough duration */
    window_criteria_add(&window, 5.0f, 100000);
    window_criteria_add(&window, 7.0f, 100000);

    TEST_ASSERT_FALSE_MESSAGE(window_criteria_satisfied(&window), "Duration criteria not met");
}

static void test_window_criteria_size_not_satisfied__returns_false(void) {
    struct window_criteria window;

    window_criteria_init(&window, 2.0f, 500000);

    /* Add values with large range but sufficient duration */
    window_criteria_add(&window, 5.0f, 300000);
    window_criteria_add(&window, 10.0f, 300000);

    TEST_ASSERT_FALSE_MESSAGE(window_criteria_satisfied(&window), "Size criteria not met");
}

/* Test window criteria satisfaction when both criteria met */
static void test_window_criteria_satisfied__returns_true(void) {
    struct window_criteria window;

    window_criteria_init(&window, 5.0f, 500000);

    /* Add values meeting both criteria */
    window_criteria_add(&window, 5.0f, 300000);
    window_criteria_add(&window, 7.0f, 300000);
    window_criteria_add(&window, 6.0f, 300000);

    TEST_ASSERT_TRUE_MESSAGE(window_criteria_satisfied(&window), "Criteria met but not recognized as such");
}

static void test_window_criteria_exact_target_size__satisfied(void) {
    struct window_criteria window;

    window_criteria_init(&window, 5.0f, 500000);

    window_criteria_add(&window, 5.0f, 0);
    window_criteria_add(&window, 10.0f, 600000);

    TEST_ASSERT_TRUE_MESSAGE(window_criteria_satisfied(&window), "Criteria should be met if size == target_size");
}

static void test_window_criteria_exact_target_duration__satisfied(void) {
    struct window_criteria window;

    window_criteria_init(&window, 10.0f, 500000);

    window_criteria_add(&window, 5.0f, 0);
    window_criteria_add(&window, 7.0f, 500000);

    TEST_ASSERT_TRUE_MESSAGE(window_criteria_satisfied(&window),
                             "Criteria should be met if duration == target_duration");
}

void test_filtering(void) {
    /* Median filter tests */
    RUN_TEST(test_median_filter_single_value__returns_value);
    RUN_TEST(test_median_filter_odd_values__returns_middle_value);
    RUN_TEST(test_median_filter_overflow__oldest_value_removed);
    RUN_TEST(test_median_filter_duplicates__handles_correctly);

    /* Average filter tests */
    RUN_TEST(test_average_filter_single_value__returns_value);
    RUN_TEST(test_average_filter_multiple_values__calculates_correctly);
    RUN_TEST(test_average_filter_overflow__moving_average);
    RUN_TEST(test_average_filter_negative_values__handles_correctly);
    RUN_TEST(test_average_filter_fractional_values__precise_calculation);

    /* Window criteria tests */
    RUN_TEST(test_window_criteria_first_value__sets_min_max);
    RUN_TEST(test_window_criteria_exact_target_size__satisfied);
    RUN_TEST(test_window_criteria_not_satisfied__returns_false);
    RUN_TEST(test_window_criteria_size_not_satisfied__returns_false);
    RUN_TEST(test_window_criteria_satisfied__returns_true);
    RUN_TEST(test_window_criteria_exact_target_size__satisfied);
    RUN_TEST(test_window_criteria_exact_target_duration__satisfied);
}
