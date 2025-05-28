#include <stdlib.h>

#include "filter.h"

void filter_init(struct filter *filter) {
  circ_buffer_init(&filter->buffer, filter->samples, FILTER_WINDOW_SIZE, sizeof(struct altitude_sample));
  filter->y_accum = 0.0;
  filter->x_accum = 0.0;
  filter->xy_accum = 0.0;
  filter->x_sq_accum = 0.0;
}

void filter_add_sample(struct filter *filter, struct altitude_sample *sample, struct rocket_dynamics *dynamics) {
  float time = sample->timestamp / 1000000.0f; // Convert to seconds
  filter->y_accum += sample->altitude;
  filter->x_accum += time;
  filter->xy_accum += (sample->altitude * time);
  filter->x_sq_accum += (time * time);

  struct altitude_sample old;
  if (circ_buffer_push_out(&filter->buffer, sample, &old)) {
    float old_time = old.timestamp / 1000000.0f; // Convert to seconds
    filter->y_accum -= old.altitude;
    filter->x_accum -= old_time;
    filter->xy_accum -= (old.altitude * old_time);
    filter->x_sq_accum -= (old_time * old_time);

    dynamics->altitude = filter->y_accum / circ_buffer_size(&filter->buffer);
    // Use the formula for a simple linear regression
    dynamics->velocity = (circ_buffer_size(&filter->buffer) * filter->xy_accum - (filter->x_accum * filter->y_accum));
    dynamics->velocity /= (circ_buffer_size(&filter->buffer) * filter->x_sq_accum - (filter->x_accum * filter->x_accum));
    dynamics->last_update = sample->timestamp;
  }
}
