#include <nuttx/sensors/sensor.h>

/* Detection events */
enum fusion_events {
  FUSION_NO_EVENT, /* No event has occured */
  FUSION_AIRBORNE_EVENT, /* The rocket is airborne */
  FUSION_LANDING_EVENT /* The rocket is landed*/
};

/* The number of samples to take to use in averages */
#define ALTITUDE_WINDOW_SIZE 1000

/* The number of altitude averages to keep */
#define ALTITUDE_AVERAGES_SIZE 5

/* An average of altitude measurements */
struct altitude_average {
  uint64_t timestamp;
  float altitude;
  int num_samples;
};

/* Information related to detecting liftoff or landing */
struct altitude_records {
  enum fusion_events last_event;
  struct sensor_altitude window[ALTITUDE_WINDOW_SIZE];
  struct altitude_average averages[ALTITUDE_AVERAGES_SIZE];
  struct circ_buffer window_buffer;
  struct circ_buffer averages_buffer;
};

void init_records(struct altitude_records *records);
void add_sample(struct altitude_records *records, struct sensor_altitude *sample);
enum fusion_events last_event(struct altitude_records *records);
