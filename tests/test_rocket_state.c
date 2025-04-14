#include <stdio.h>

#include "unity/unity.h"

void test_nothing(void) {
  printf("Do nothing");
  return;
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_nothing);
  return UNITY_END();
}
