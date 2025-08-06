#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <nuttx/timers/pwm.h>

int pwm_send_signal(int pwm_fd, float freq);
void *startup_sound_main(void *arg);