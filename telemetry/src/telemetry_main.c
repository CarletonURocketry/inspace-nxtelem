#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "collection/downsample.h"
#include "fusion/fusion.h"
#include "logging/logging.h"
#include "pwm/pwm_olg.h"
#include "rocket-state/rocket-state.h"
#include "syslogging.h"
#include "transmission/transmit.h"

#ifdef CONFIG_INSPACE_TELEMETRY_USBSH
#include "shell/shell.h"
#endif

/* Buffers for sharing sensor data between threads */

static pthread_t transmit_thread;
static pthread_t log_thread;
static pthread_t downsample_thread;
static pthread_t fusion_thread;
static pthread_t startup_sound_thread;
#ifdef CONFIG_INSPACE_TELEMETRY_USBSH
static pthread_t shell_thread;
#endif

static rocket_state_t state; /* The shared rocket state. */
static struct config_options config;

/* Radio telemetry double-buffer storage */
static radio_raw_data radio_buf1;
static radio_raw_data radio_buf2;
static radio_telem_t radio_telem;

int main(int argc, char **argv) {
    int err;

    /* Sets up the syslogging to a file. If this fails, stdout is still open */

    if (setup_syslogging()) {
        publish_error(PROC_ID_GENERAL, ERROR_SYSLOGGING_NOT_SAVING);
    }

    /* Initialize shared state */

    err = state_init(&state);
    if (err) {
        inwarn("State not loaded properly, ensuring airborne state set: %d\n", err);
        err = state_set_flightstate(&state, STATE_AIRBORNE);
        publish_status(STATUS_TELEMETRY_CHANGED_AIRBORNE);
        if (err) {
            inwarn("Could not set flight state properly either, continuing anyways: %d\n", err);
        }
        err = state_set_flightsubstate(&state, SUBSTATE_UNKNOWN);
        if (err) {
            inerr("Could not set flight substate, continuing anyways: %d\n", err);
        }
    } else {
        enum flight_state_e flight_state;
        state_get_flightstate(&state, &flight_state);
        ininfo("Loaded state: %d from EEPROM\n", flight_state);
    }

    /* Grab configuration parameters from EEPROM */

    err = config_get(&config);
    if (err) {
        inerr("Couldn't read EEPROM contents: %d\n", err);
        // TODO maybe some sensible defaults?
    }

    // Allow apogee to be detected again (in case we happen to actually be in liftoff when loaded)
    if (state.state == STATE_AIRBORNE && state.substate == SUBSTATE_DESCENT) {
        ininfo("Loaded the descent substate, but setting to unknown to trigger apogee again");
        state_set_flightsubstate(&state, SUBSTATE_UNKNOWN);
    }

    radio_telem.full = &radio_buf1;
    radio_telem.empty = &radio_buf2;

    memset(&radio_buf1, 0, sizeof(radio_buf1));
    memset(&radio_buf2, 0, sizeof(radio_buf2));

    pthread_mutex_init(&(radio_telem.full_mux), NULL);
    pthread_mutex_init(&(radio_telem.empty_mux), NULL);

    /* Start all threads */
    struct fusion_args fusion_thread_args = {.state = &state};
    err = pthread_create(&fusion_thread, NULL, fusion_main, &fusion_thread_args);
    if (err) {
        inerr("Problem starting fusion thread: %d\n", err);
        goto exit_error;
    }

    struct downsample_args downsample_thread_args = {.state = &state, .radio_telem = &radio_telem};
    err = pthread_create(&downsample_thread, NULL, downsample_main, &downsample_thread_args);
    // TODO: handle thread creation errors better
    if (err) {
        inerr("Problem starting downsample thread: %d\n", err);
        goto exit_error;
    }

    struct transmit_args transmit_thread_args = {
        .config = config.radio,
        .radio_telem = &radio_telem,
    };
    err = pthread_create(&transmit_thread, NULL, transmit_main, &transmit_thread_args);
    if (err) {
        inerr("Problem starting transmission thread: %d\n", err);
        goto exit_error;
    }

    err = pthread_create(&log_thread, NULL, logging_main, NULL);
    if (err) {
        inerr("Problem starting logging thread: %d\n", err);
        goto exit_error;
    }

#ifdef CONFIG_INSPACE_TELEMETRY_USBSH
    struct shell_args shell_args;
    err = pthread_create(&shell_thread, NULL, shell_main, &shell_args);
    if (err) {
        inerr("Problem starting shell thread: %d\n", err);
        exit(EXIT_FAILURE);
    }
#else
#warning "Telemetry application is being compiled without configuration shell."
#endif

    /* No args needed for startup sound thread */
    err = pthread_create(&startup_sound_thread, NULL, startup_sound_main, NULL);
    if (err) {
        inerr("Problem starting startup thread: %d\n", err);
        exit(EXIT_FAILURE);
    }

    publish_status(STATUS_SYSTEMS_NOMINAL);

    /* Join on all threads: TODO handle errors */

    err = pthread_join(fusion_thread, NULL);
    err = pthread_join(downsample_thread, NULL);
    err = pthread_join(transmit_thread, NULL);
    err = pthread_join(log_thread, NULL);
#ifdef CONFIG_INSPACE_TELEMETRY_USBSH
    err = pthread_join(shell_thread, NULL);
#endif

    return err;

exit_error:
    publish_error(PROC_ID_GENERAL, ERROR_PROCESS_DEAD);
    exit(EXIT_FAILURE);
}
