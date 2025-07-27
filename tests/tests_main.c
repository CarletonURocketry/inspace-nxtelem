#include <nuttx/config.h>
#include <testing/unity.h>

#include "test_runners.h"

// Empty but required by Unity
void setUp(void) {}
// Empty but required by Unity
void tearDown(void) {}

int main(void) {
    UNITY_BEGIN();
    test_rocket_state();
    test_detection();
    test_circular_buffer();
    test_filtering();
    test_logging();
    return UNITY_END();
}
