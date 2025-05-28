#ifndef _FILTER_H_
#define _FILTER_H_

#include <stdint.h>

#include "circular-buffer.h"

#define FILTER_WINDOW_SIZE 10

struct rocket_dynamics {
  uint64_t last_update;
  float altitude;
  float velocity;
};

struct altitude_sample {
  uint64_t timestamp;
  float altitude;
};

struct filter {
  struct altitude_sample samples[FILTER_WINDOW_SIZE];
  struct circ_buffer buffer;

  // Velocity
  float x_accum;
  float y_accum;
  float xy_accum;
  float x_sq_accum;
};

void filter_init(struct filter *filter);
void filter_add_sample(struct filter *filter, struct altitude_sample *sample, struct rocket_dynamics *dynamics);

#endif /* _FILTER_H_ */
