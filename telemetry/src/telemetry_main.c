#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdlib.h>

#include "rocket-state/rocket-state.h"

#include "collection/collection.h"
#include "fusion/fusion.h"
#include "logging/logging.h"
#include "packets/buffering.h"
#include "syslogging.h"
#include "transmission/transmit.h"

int main(int argc, char **argv) {

    int err = 0;
    rocket_state_t state; /* The shared rocket state. */

    /* Buffers for sharing sensor data between threads */

    packet_buffer_t transmit_buffer;
    packet_buffer_t logging_buffer;

    /* Thread handles. */

    pthread_t transmit_thread;
    pthread_t log_thread;
    pthread_t collect_thread;
    pthread_t fusion_thread;

    ininfo("You are running the Carleton University InSpace telemetry system.");

    /* Initialize shared state */

    err = state_init(&state);
    if (err) {
        inerr("State not loaded properly, ensuring airborne state set: %d\n", err);
        err = state_set_flightstate(&state, STATE_AIRBORNE);
        if (err) {
            inerr("Could not set flight state properly either, continuing anyways: %d\n", err);
        }
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

    struct transmit_args transmit_thread_args = {.state = &state, .buffer = &transmit_buffer};
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

    /* Join on all threads: TODO handle errors */

    err = pthread_join(collect_thread, NULL);
    err = pthread_join(transmit_thread, NULL);
    err = pthread_join(log_thread, NULL);
    err = pthread_join(fusion_thread, NULL);

    return err;
}
