#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdlib.h>

#include "rocket-state/rocket-state.h"

#include "collection/collection.h"
#include "fusion/fusion.h"
#include "logging/logging.h"
#include "packets/buffering.h"
#include "pwm/pwm_olg.h"
#include "shell/shell.h"
#include "syslogging.h"
#include "transmission/transmit.h"

/* Buffers for sharing sensor data between threads */

static packet_buffer_t transmit_buffer;
static packet_buffer_t logging_buffer;

static pthread_t transmit_thread;
static pthread_t log_thread;
static pthread_t collect_thread;
static pthread_t fusion_thread;
static pthread_t shell_thread;
static pthread_t startup_sound_thread;

static rocket_state_t state; /* The shared rocket state. */
static struct config_options config;

int main(int argc, char **argv) {
    int err;

    ininfo("You are running the Carleton University InSpace telemetry system.");

    /* Initialize shared state */

    err = state_init(&state);
    if (err) {
        inerr("State not loaded properly, ensuring airborne state set: %d\n", err);
        err = state_set_flightstate(&state, STATE_AIRBORNE);
        if (err) {
            inerr("Could not set flight state properly either, continuing anyways: %d\n", err);
        }
        err = state_set_flightsubstate(&state, SUBSTATE_UNKNOWN);
        if (err) {
            inerr("Could not set flight substate, continuing anyways: %d\n", err);
        }
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

    err = packet_buffer_init(&transmit_buffer);
    if (err) {
        inerr("Could not initialize transmit buffer: %d\n", err);
        exit(EXIT_FAILURE);
    }

    err = packet_buffer_init(&logging_buffer);
    if (err) {
        inerr("Could not initialize logging buffer: %d\n", err);
        exit(EXIT_FAILURE);
    }
    /* Start all threads */

    struct collection_args collect_thread_args = {
        .state = &state, .transmit_buffer = &transmit_buffer, .logging_buffer = &logging_buffer};
    err = pthread_create(&collect_thread, NULL, collection_main, &collect_thread_args);
    // TODO: handle thread creation errors better
    if (err) {
        inerr("Problem starting collection thread: %d\n", err);
        exit(EXIT_FAILURE);
    }

    struct transmit_args transmit_thread_args = {
        .state = &state,
        .buffer = &transmit_buffer,
        .config = config.radio,
    };
    err = pthread_create(&transmit_thread, NULL, transmit_main, &transmit_thread_args);
    if (err) {
        inerr("Problem starting transmission thread: %d\n", err);
        exit(EXIT_FAILURE);
    }

    struct logging_args logging_thread_args = {.state = &state, .buffer = &logging_buffer};
    err = pthread_create(&log_thread, NULL, logging_main, &logging_thread_args);
    if (err) {
        inerr("Problem starting logging thread: %d\n", err);
        exit(EXIT_FAILURE);
    }

    struct fusion_args fusion_thread_args = {.state = &state};
    err = pthread_create(&fusion_thread, NULL, fusion_main, &fusion_thread_args);
    if (err) {
        inerr("Problem starting fusion thread: %d\n", err);
        exit(EXIT_FAILURE);
    }

    struct shell_args shell_args;
    err = pthread_create(&shell_thread, NULL, shell_main, &shell_args);
    if (err) {
        inerr("Problem starting shell thread: %d\n", err);
        exit(EXIT_FAILURE);
    }

    /* No args needed for startup sound thread */
    err = pthread_create(&startup_sound_thread, NULL, startup_sound_main, NULL);
    if (err) {
        inerr("Problem starting startup thread: %d\n", err);
        exit(EXIT_FAILURE);
    }

    /* Join on all threads: TODO handle errors */

    err = pthread_join(collect_thread, NULL);
    err = pthread_join(transmit_thread, NULL);
    err = pthread_join(log_thread, NULL);
    err = pthread_join(fusion_thread, NULL);
    err = pthread_join(shell_thread, NULL);

    return err;
}
