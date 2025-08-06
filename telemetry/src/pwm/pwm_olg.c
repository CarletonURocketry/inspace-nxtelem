#include "pwm_olg.h"
#include "../syslogging.h"

#define F 698.46    /* PWM frequency for F key */
#define G_SHARP 830.61   /* PWM frequency for G# key */

/*
 * Sends a PWM signal to the device based on the open or close pulse duration.
 * @param act The PWM actuator to send the signal to
 * @param open True to send an open pulse, false to send a close pulse
 * @return 0 on success, error code on failure
 */
int pwm_send_signal(int pwm_fd, float freq) {
    int err;
    struct pwm_info_s pwm_config;

    /* Get the configuration */

    ininfo("Got characteristics\n");
    err = ioctl(pwm_fd, PWMIOC_GETCHARACTERISTICS, &pwm_config);
    if (err < 0) {
        return errno;
    }

    ininfo("Freq: %lu\n", pwm_config.frequency);

    pwm_config.frequency = freq;

    ininfo("Freq: %lu\n", pwm_config.frequency);

    /* Set the configuration */

    err = ioctl(pwm_fd, PWMIOC_SETCHARACTERISTICS, &pwm_config);
    if (err < 0) {
        return errno;
    }
    ininfo("Set characteristics\n");

    /* Turn on the PWM signal */

    err = ioctl(pwm_fd, PWMIOC_START, NULL);
    if (err < 0) {
        return errno;
    }
    ininfo("Set PWM frequency to %f\n", freq);

    return err;
}

static int pwm_turn_off(int pwm_fd){
    int err;

    err = ioctl(pwm_fd, PWMIOC_STOP, NULL);
    if (err < 0) {
        return errno;
    }
    ininfo("Stopped PWM\n");

    return err;
}

void *startup_sound_main(void *arg){
    int pwm_fd;
    
    pwm_fd = open("/dev/pwm0", O_RDWR);
    if (pwm_fd < 0) {
        return errno;
    }
    
    pwm_send_signal(pwm_fd, F);
    usleep(3000);
    pwm_turn_off(pwm_fd);
}