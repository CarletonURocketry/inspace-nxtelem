#include <stdint.h>
#include "altitude-detection.h"

#define ALTITUDE_WINDOW_SIZE 5
#define ALTITUDE_AVERAGES_SIZE 5
#define ALTITUDE_AVERAGES 5
#define ALTITUDE_AVERAGE_PERIOD 1000

struct altitude_sample {
  uint64_t timestamp;
  float altitude;
};

struct altitude_average {
  uint64_t timestamp;
  float altitude;
};

struct altitude_records {
  struct altitude_sample window[ALTITUDE_WINDOW_SIZE];
  int window_head;
  int window_size;

  struct altitude_average averages[ALTITUDE_AVERAGES_SIZE];
  int averages_head;
  int averages_size;
};

void init_records(struct altitude_records *records) {
  records->window_head = 0;
  records->window_size = 0;
  records->averages_head = 0;
  records->averages_size = 0;
}

void add_sample(struct altitude_records *records, struct altitude_sample *sample) {
  records->window[records->window_head] = *sample;
  records-> window_head = (records->window_head + 1) % ALTITUDE_WINDOW_SIZE;
  if (records->window_size < ALTITUDE_WINDOW_SIZE) {
    records->window_size++;
  }

  if (need_new_average(records, sample)) {
    records->averages[records->averages_head] = new_average(records);
    records->averages_head = (records->averages_head + 1) % ALTITUDE_AVERAGES_SIZE;
    if (records->averages_size < ALTITUDE_AVERAGES_SIZE) {
      records->averages_size++;
    }
  }
}

static struct altitude_sample remove_sample(struct altitude_records *records) {
  struct altitude_sample output = records->window[records->window_head];
  // TODO - fix this
  records->window_head = (records->window_head - 1) % ALTITUDE_WINDOW_SIZE;
  records->window_size--;
  return output;
}

static int need_new_average(struct altitude_records *records, struct altitude_sample *sample) {
  if (records->averages_size > 0) {
    struct altitude_average *last_average = &records->averages[records->averages_head];
    if ((sample->timestamp - last_average->timestamp) < ALTITUDE_AVERAGE_PERIOD) {
      return 0;
    }
  }
  return 1;
}


static struct altitude_average new_average(struct altitude_records *records) {
  struct altitude_average output;
  while (records->window_size > 0) {
    output.timestamp += records->window[0].timestamp;
    output.altitude = 0;
    for (int i = 0; i < records->window_size; i++) {
      output.altitude += records->window[i].altitude;
    }
    output.altitude /= records->window_size;
  }
}

// Have a window of the last N samples

// Every set period of time, create an average from that window, store it and the time it was created

// Whenever we make a new average, look through a buffer of averages (stopping when we get to one that fails our time constraint) and check if the average heights are sufficient
