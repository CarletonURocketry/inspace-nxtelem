#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <nuttx/timers/pwm.h>

void *startup_sound_main(void *arg);