#include <stdlib.h>

#include "filter.h"

void init_filter(struct filter *filter) {
  circ_buffer_init(&filter->buffer, filter->samples, FILTER_WINDOW_SIZE, sizeof(struct altitude_sample));
}

void filter_add_sample(struct filter *filter, struct altitude_sample *sample, struct rocket_dynamics *dynamics) {
  filter->y_accumulator += sample->altitude;
  filter->x_accumulator += sample->timestamp;
  filter->xy_accumulator += (sample->altitude * sample->timestamp);
  filter->x_square_accumulator += (sample->timestamp * sample->timestamp);

  struct altitude_sample oldest_sample;
  if (circ_buffer_push_out(&filter->buffer, sample, &oldest_sample)) {
    filter->y_accumulator -= oldest_sample.altitude;
    filter->x_accumulator -= oldest_sample.timestamp;
    filter->xy_accumulator -= (oldest_sample.altitude * oldest_sample.timestamp);
    filter->x_square_accumulator -= (oldest_sample.timestamp * oldest_sample.timestamp);

    dynamics->altitude = filter->y_accumulator / circ_buffer_size(&filter->buffer);
    dynamics->velocity = (circ_buffer_size(&filter->buffer) * filter->xy_accumulator - (filter->x_accumulator * filter->y_accumulator));
    dynamics->velocity /= (circ_buffer_size(&filter->buffer) * filter->x_square_accumulator - (filter->x_accumulator * filter->x_accumulator));
    dynamics->last_update = sample->timestamp;
  }
}
