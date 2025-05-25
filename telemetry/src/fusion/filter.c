#include <stdlib.h>

#include "filter.h"

void filter_init(struct filter *filter) {
  circ_buffer_init(&filter->buffer, filter->samples, FILTER_WINDOW_SIZE, sizeof(struct altitude_sample));
}

void filter_add_sample(struct filter *filter, struct altitude_sample *sample, struct rocket_dynamics *dynamics) {
  float time = sample->timestamp / 1000000.0f; // Convert to seconds
  filter->y_accumulator += sample->altitude;
  filter->x_accumulator += time;
  filter->xy_accumulator += (sample->altitude * time);
  filter->x_square_accumulator += (time * time);

  struct altitude_sample oldest_sample;
  if (circ_buffer_push_out(&filter->buffer, sample, &oldest_sample)) {
    float old_time = oldest_sample.timestamp / 1000000.0f; // Convert to seconds
    filter->y_accumulator -= oldest_sample.altitude;
    filter->x_accumulator -= old_time;
    filter->xy_accumulator -= (oldest_sample.altitude * old_time);
    filter->x_square_accumulator -= (old_time * old_time);

    dynamics->altitude = filter->y_accumulator / circ_buffer_size(&filter->buffer);
    dynamics->velocity = (circ_buffer_size(&filter->buffer) * filter->xy_accumulator - (filter->x_accumulator * filter->y_accumulator));
    dynamics->velocity /= (circ_buffer_size(&filter->buffer) * filter->x_square_accumulator - (filter->x_accumulator * filter->x_accumulator));
    dynamics->last_update = sample->timestamp;
  }
}
