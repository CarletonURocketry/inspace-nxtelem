#include <testing/unity.h>

#include "testfuncs.h"

void setUp(void) { return; }
void tearDown(void) { return; }

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(selftest_baro);
    RUN_TEST(selftest_gyro);
    RUN_TEST(selftest_accel);
    return UNITY_END();
}
