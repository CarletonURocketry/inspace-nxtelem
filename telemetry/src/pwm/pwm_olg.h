#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <nuttx/timers/pwm.h>

static int set_pwm_freq(int pwm_fd, float freq);
static int pwm_turn_off(int pwm_fd);
static int play_olg_jingle(int pwm_fd);
void *startup_sound_main(void *arg);