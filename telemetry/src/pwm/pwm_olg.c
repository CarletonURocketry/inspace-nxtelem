#include <pthread.h>
#include <sys/ioctl.h>

#include "../syslogging.h"
#include "pwm_olg.h"

#define F 698.46       /* PWM frequency for F key */
#define G_SHARP 830.61 /* PWM frequency for G# key */

#define err_to_ptr(err) ((void *)((err)))

typedef struct {
    float note;                /* Frequency of note */
    uint32_t note_duration_us; /* Duration of note in */
} note_s;

static const note_s olg_jingle[] = {
    {.note = F, .note_duration_us = 130000}, {.note = G_SHARP, .note_duration_us = 130000},
    {.note = F, .note_duration_us = 130000}, {.note = G_SHARP, .note_duration_us = 700000},
    {.note = F, .note_duration_us = 130000}, {.note = G_SHARP, .note_duration_us = 130000},
    {.note = F, .note_duration_us = 130000}, {.note = G_SHARP, .note_duration_us = 700000},
    {.note = F, .note_duration_us = 130000}, {.note = G_SHARP, .note_duration_us = 130000},
    {.note = F, .note_duration_us = 130000}, {.note = G_SHARP, .note_duration_us = 200000},
    {.note = 1, .note_duration_us = 50000},  {.note = G_SHARP, .note_duration_us = 200000},
    {.note = F, .note_duration_us = 700000},
};
static const uint8_t olg_jingle_length = 15;

/*
 * Sets the frequency of a specified PWM device
 * @param pwm_fd The file descriptor of the PWM device
 * @param freq The frequency to give to the PWM device
 * @return 0 on success, error code on failure
 */
static int set_pwm_freq(int pwm_fd, float freq) {
    int err;
    struct pwm_info_s pwm_config;

    /* Get the configuration */

    indebug("Got characteristics\n");
    err = ioctl(pwm_fd, PWMIOC_GETCHARACTERISTICS, &pwm_config);
    if (err < 0) {
        return errno;
    }

    indebug("Freq: %lu\n", pwm_config.frequency);

    pwm_config.frequency = freq;
    pwm_config.duty = 32768;

    indebug("Freq: %lu\n", pwm_config.frequency);

    /* Set the configuration */

    err = ioctl(pwm_fd, PWMIOC_SETCHARACTERISTICS, &pwm_config);
    if (err < 0) {
        return errno;
    }
    indebug("Set characteristics\n");

    /* Turn on the PWM signal */

    err = ioctl(pwm_fd, PWMIOC_START, NULL);
    if (err < 0) {
        return errno;
    }
    indebug("Set PWM frequency to %f\n", freq);

    return err;
}

/*
 * Turns a PWM device off
 * @param pwm_fd The file descriptor of the PWM device
 * @return 0 on success, error code on failure
 */
static int pwm_turn_off(int pwm_fd) {
    int err;

    err = ioctl(pwm_fd, PWMIOC_STOP, NULL);
    if (err < 0) {
        return errno;
    }
    indebug("Stopped PWM\n");

    return err;
}

/*
 * Plays the OLG lottery winner jingle on a PWM device by setting the
 * device to vibrate at frequencies corresponding to different music
 * notes for certain lengths of time
 * @param pwm_fd The file descriptor of the PWM device
 * @return 0 on success, error code on failure
 */
static int play_olg_jingle(int pwm_fd) {
    int err;

    for (int i = 0; i < olg_jingle_length; ++i) {
        err = set_pwm_freq(pwm_fd, olg_jingle[i].note);
        if (err < 0) {
            return errno;
        }
        usleep(olg_jingle[i].note_duration_us);
    }

    return err;
}

void *startup_sound_main(void *arg) {
    int err;
    int pwm_fd;

    pwm_fd = open("/dev/pwm0", O_RDWR);
    if (pwm_fd < 0) {
        pthread_exit(err_to_ptr(pwm_fd));
    }

    for (int i = 0; i < 3; ++i) {
        play_olg_jingle(pwm_fd);
        set_pwm_freq(pwm_fd, 1);
        usleep(3000000);
    }

    err = pwm_turn_off(pwm_fd);
    if (err < 0) {
        pthread_exit(err_to_ptr(err));
    }

    pthread_exit(0);
}