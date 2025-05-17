#include <testing/unity.h>

#include "../telemetry/src/fusion/circular-buffer.h"
#include "test_runners.h"

/* Creates a circular buffer to test with, with the given size and type */
#define create_test_buffer(name, size, type) \
  struct circ_buffer name; \
  type backing[size]; \
  circ_buffer_init(&buffer, backing, size, sizeof(type)); \


static void test_sizeof_empty__zero(void) {
    create_test_buffer(buffer, 3, int);
    TEST_ASSERT_EQUAL(0, circ_buffer_size(&buffer));
}

static void test_sizeof_full__size(void) {
    create_test_buffer(buffer, 3, int);
    int values[] = {1, 2, 3};
    for (int i = 0; i < 3; i++) {
        circ_buffer_append(&buffer, &values[i]);
    }
    TEST_ASSERT_EQUAL(3, circ_buffer_size(&buffer));
}

static void test_append_empty__one_element(void) {
    create_test_buffer(buffer, 3, int);
    int value = 42;
    circ_buffer_append(&buffer, &value);
    TEST_ASSERT_EQUAL(1, circ_buffer_size(&buffer));
    TEST_ASSERT_EQUAL(42, *(int*)circ_buffer_get(&buffer));
}

static void test_append_full__overwrite_last(void) {
    create_test_buffer(buffer, 2, int);
    int values[] = {1, 2, 3};
    for (int i = 0; i < 3; i++) {
        circ_buffer_append(&buffer, &values[i]);
    }
    TEST_ASSERT_EQUAL(2, circ_buffer_size(&buffer));
    TEST_ASSERT_EQUAL(3, *(int*)circ_buffer_pop(&buffer));
    TEST_ASSERT_EQUAL(2, *(int*)circ_buffer_pop(&buffer));
    TEST_ASSERT_NULL(circ_buffer_get(&buffer));
}

static void test_get_empty__return_null(void) {
    create_test_buffer(buffer, 3, int);
    TEST_ASSERT_NULL(circ_buffer_get(&buffer));
}

static void test_get_full__return_last(void) {
    create_test_buffer(buffer, 3, int);
    int values[] = {1, 2, 3};
    for (int i = 0; i < 3; i++) {
        circ_buffer_append(&buffer, &values[i]);
    }
    TEST_ASSERT_EQUAL(3, *(int*)circ_buffer_get(&buffer));
}

static void test_pop_empty__return_null(void) {
    create_test_buffer(buffer, 3, int);
    TEST_ASSERT_NULL(circ_buffer_pop(&buffer));
    TEST_ASSERT_EQUAL(0, circ_buffer_size(&buffer));
}

static void test_pop_full__return_last(void) {
    create_test_buffer(buffer, 3, int);
    int values[] = {1, 2, 3};
    for (int i = 0; i < 3; i++) {
        circ_buffer_append(&buffer, &values[i]);
    }
    TEST_ASSERT_EQUAL(3, *(int*)circ_buffer_pop(&buffer));
    TEST_ASSERT_EQUAL(2, circ_buffer_size(&buffer));
    TEST_ASSERT_EQUAL(2, *(int*)circ_buffer_get(&buffer));
}

static void test_iterator_empty__return_null(void) {
    create_test_buffer(buffer, 3, int);
    struct circ_iterator it;
    circ_iterator_init(&it, &buffer);
    TEST_ASSERT_NULL(circ_iterator_next(&it));
}

static void test_iterator_full__return_first(void) {
    create_test_buffer(buffer, 3, int);
    int values[] = {1, 2, 3};
    for (int i = 0; i < 3; i++) {
        circ_buffer_append(&buffer, &values[i]);
    }
    struct circ_iterator it;
    circ_iterator_init(&it, &buffer);
    TEST_ASSERT_EQUAL(3, *(int*)circ_iterator_next(&it));
}

static void test_iterator_next__return_next(void) {
    create_test_buffer(buffer, 3, int);
    int values[] = {1, 2, 3};
    for (int i = 0; i < 3; i++) {
        circ_buffer_append(&buffer, &values[i]);
    }
    struct circ_iterator it;
    circ_iterator_init(&it, &buffer);
    TEST_ASSERT_EQUAL(3, *(int*)circ_iterator_next(&it));
    TEST_ASSERT_EQUAL(2, *(int*)circ_iterator_next(&it));
    TEST_ASSERT_EQUAL(1, *(int*)circ_iterator_next(&it));
    TEST_ASSERT_NULL(circ_iterator_next(&it));
}

void test_circular_buffer(void) {
  RUN_TEST(test_sizeof_empty__zero);
  RUN_TEST(test_sizeof_full__size);
  RUN_TEST(test_append_empty__one_element);
  RUN_TEST(test_append_full__overwrite_last);
  RUN_TEST(test_get_empty__return_null);
  RUN_TEST(test_get_full__return_last);
  RUN_TEST(test_pop_empty__return_null);
  RUN_TEST(test_pop_full__return_last);
  RUN_TEST(test_iterator_empty__return_null);
  RUN_TEST(test_iterator_full__return_first);
  RUN_TEST(test_iterator_next__return_next);
}
