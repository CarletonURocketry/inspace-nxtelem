#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#if defined(CONFIG_LPWAN_RN2XX3)
#include <nuttx/wireless/ioctl.h>
#include <nuttx/wireless/lpwan/rn2xx3.h>
#endif

#include "../collection/status-update.h"
#include "../packets/packets.h"
#include "../syslogging.h"
#include "transmit.h"

/* If there was an error in configuration, display which line and return the
 * error */

#define config_error(err)                                                                                              \
    if (err) {                                                                                                         \
        err = errno;                                                                                                   \
        inerr("Error configuring radio, line %d: %d\n", __LINE__, err);                                                \
        return err;                                                                                                    \
    }

/* Cast an error to a void pointer */

#define err_to_ptr(err) ((void *)((err)))

#define TRANSMIT_PERIOD_MS 700

static int transmit(int radio, uint8_t *packet, size_t packet_size);
static int configure_radio(int fd, struct radio_options const *config);

/* Main thread for data transmission over radio. */
void *transmit_main(void *arg) {
    int err;
    int radio = -1; /* Radio device file descriptor */
    struct transmit_args *unpacked_args = (struct transmit_args *)arg;
    radio_telem_t *radio_telem = unpacked_args->radio_telem;
    uint32_t seq_num = 0;

    ininfo("Transmit thread started.\n");

    radio = open(CONFIG_INSPACE_TELEMETRY_RADIO, O_WRONLY | O_CREAT);
    if (radio < 0) {
        err = errno;
        inerr("Error getting radio handle: %d\n", err);
        goto err_cleanup;
    }

    err = configure_radio(radio, &unpacked_args->config);
    if (err) {
        /* Error will have been reported in configure_rn2483 where we can say which
         * config failed in particular */
        goto err_cleanup;
    }

    /* Transmit forever, regardless of rocket flight state. */

    for (;;) {
        struct timespec cycle_start;
        clock_gettime(CLOCK_MONOTONIC, &cycle_start);

        pthread_mutex_lock(&radio_telem->buff_mux);
        radio_raw_data *temp = radio_telem->buff;
        radio_telem->buff = radio_telem->empty_buff;
        radio_telem->empty_buff = temp;
        radio_telem->empty_buff->accel_n = 0;
        radio_telem->empty_buff->gyro_n = 0;
        radio_telem->empty_buff->mag_n = 0;
        radio_telem->empty_buff->gnss_n = 0;
        radio_telem->empty_buff->alt_n = 0;
        pthread_mutex_unlock(&radio_telem->buff_mux);
        sem_post(&radio_telem->swapped);

        /* create a new packet buffer */
        uint8_t packet_buffer[PACKET_MAX_SIZE];
        uint8_t *packet_ptr = packet_buffer;

        /* use mission time as current time for now, this does not account for reboots */
        struct timespec current_time;
        clock_gettime(CLOCK_REALTIME, &current_time);
        uint32_t mission_time_ms = current_time.tv_sec * 1000 + current_time.tv_nsec / 1000000;
        pkt_hdr_t *header = (pkt_hdr_t *)packet_ptr;
        packet_ptr = pkt_init(packet_ptr, seq_num++, mission_time_ms);

        if (radio_telem->buff->gnss_n > 0) {
            header->type_count++;
            blk_hdr_t blk_hdr = {
                .type = DATA_LAT_LONG,
                .count = radio_telem->buff->gnss_n,
            };
            memcpy(packet_ptr, &blk_hdr, sizeof(blk_hdr));
            packet_ptr += sizeof(blk_hdr);
            for (int i = 0; i < radio_telem->buff->gnss_n; i++) {
                struct coord_blk_t coord_blk;
                if (orb_gnss_pkt(&radio_telem->buff->gnss[i], &coord_blk, header->timestamp)) {
                    inerr("Failed to create GNSS block %d\n", i);
                    continue;
                }

                memcpy(packet_ptr, &coord_blk, sizeof(struct coord_blk_t));
                packet_ptr += sizeof(struct coord_blk_t);
            }
        }

        if (radio_telem->buff->alt_n > 0) {
            header->type_count++;
            blk_hdr_t blk_hdr = {
                .type = DATA_ALT_SEA,
                .count = radio_telem->buff->alt_n,
            };
            memcpy(packet_ptr, &blk_hdr, sizeof(blk_hdr));
            packet_ptr += sizeof(blk_hdr);
            for (int i = 0; i < radio_telem->buff->alt_n; i++) {
                struct alt_blk_t alt_blk;
                if (orb_alt_pkt(&radio_telem->buff->alt[i], &alt_blk, header->timestamp)) {
                    inerr("Failed to create Alt block %d\n", i);
                    continue;
                }
                memcpy(packet_ptr, &alt_blk, sizeof(struct alt_blk_t));
                packet_ptr += sizeof(struct alt_blk_t);
            }
        }

        if (radio_telem->buff->mag_n > 0) {
            header->type_count++;
            blk_hdr_t blk_hdr = {
                .type = DATA_MAGNETIC,
                .count = radio_telem->buff->mag_n,
            };
            memcpy(packet_ptr, &blk_hdr, sizeof(blk_hdr));
            packet_ptr += sizeof(blk_hdr);
            for (int i = 0; i < radio_telem->buff->mag_n; i++) {
                struct mag_blk_t mag_blk;
                if (orb_mag_pkt(&radio_telem->buff->mag[i], &mag_blk, header->timestamp)) {
                    inerr("Failed to create Mag block %d\n", i);
                    continue;
                }
                memcpy(packet_ptr, &mag_blk, sizeof(struct mag_blk_t));
                packet_ptr += sizeof(struct mag_blk_t);
            }
        }

        if (radio_telem->buff->accel_n > 0) {
            header->type_count++;
            blk_hdr_t blk_hdr = {
                .type = DATA_ACCEL_REL,
                .count = radio_telem->buff->accel_n,
            };
            memcpy(packet_ptr, &blk_hdr, sizeof(blk_hdr));
            packet_ptr += sizeof(blk_hdr);
            for (int i = 0; i < radio_telem->buff->accel_n; i++) {
                struct accel_blk_t accel_blk;
                if (orb_accel_pkt(&radio_telem->buff->accel[i], &accel_blk, header->timestamp)) {
                    inerr("Failed to create Accel block %d\n", i);
                    continue;
                }
                memcpy(packet_ptr, &accel_blk, sizeof(struct accel_blk_t));
                packet_ptr += sizeof(struct accel_blk_t);
            }
        }

        if (radio_telem->buff->gyro_n > 0) {
            header->type_count++;
            blk_hdr_t blk_hdr = {
                .type = DATA_ANGULAR_VEL,
                .count = radio_telem->buff->gyro_n,
            };
            memcpy(packet_ptr, &blk_hdr, sizeof(blk_hdr));
            packet_ptr += sizeof(blk_hdr);
            for (int i = 0; i < radio_telem->buff->gyro_n; i++) {
                struct ang_vel_blk_t ang_vel_blk;
                if (orb_ang_vel_pkt(&radio_telem->buff->gyro[i], &ang_vel_blk, header->timestamp)) {
                    inerr("Failed to create Ang vel block %d\n", i);
                    continue;
                }
                memcpy(packet_ptr, &ang_vel_blk, sizeof(struct ang_vel_blk_t));
                packet_ptr += sizeof(struct ang_vel_blk_t);
            }
        }

        size_t packet_size = packet_ptr - packet_buffer;
        if (packet_size > PACKET_MAX_SIZE) {
            inerr("Packet size is too large: %zu\n", packet_size);
        } else if (packet_size > sizeof(pkt_hdr_t)) {
            ininfo("Transmitting packet #%u of size %zu bytes. Accel: %d, Gyro: %d, Mag: %d, GNSS: %d, Alt: %d\n",
                   header->packet_num, packet_size, radio_telem->buff->accel_n, radio_telem->buff->gyro_n,
                   radio_telem->buff->mag_n, radio_telem->buff->gnss_n, radio_telem->buff->alt_n);
            err = transmit(radio, packet_buffer, packet_size);
            if (err < 0) {
                inerr("Error transmitting packet: %d\n", -err);
            }
        }

        /* for the sake of making the dynamic rate adjustment work, we need at least one static thing as an anchor, imo
         * the choice should be transmission time */
        struct timespec cycle_end;
        clock_gettime(CLOCK_MONOTONIC, &cycle_end);
        long elapsed_ms =
            (cycle_end.tv_sec - cycle_start.tv_sec) * 1000 + (cycle_end.tv_nsec - cycle_start.tv_nsec) / 1000000;
        ininfo("Transmission cycle time: %ld ms\n", elapsed_ms);
        long remaining_ms = TRANSMIT_PERIOD_MS - elapsed_ms;
        if (remaining_ms > 0) {
            struct timespec sleep_time = {
                .tv_sec = remaining_ms / 1000,
                .tv_nsec = (remaining_ms % 1000) * 1000000,
            };
            nanosleep(&sleep_time, NULL);
        }
    }

err_cleanup:
    if (radio != -1 && close(radio) < 0) {
        inerr("Error closing radio handle: %d\n", err);
    }
    publish_error(PROC_ID_TRANSMIT, ERROR_PROCESS_DEAD);
    pthread_exit(err_to_ptr(err));
}

/* Transmits a packet over the radio with a fake delay
 *
 * @param radio The radio to transmit to
 * @param packet The completed packet to transmit
 * @param packet_size The size of the packet to transmit
 * @return The number of bytes written or a negative error code
 */
static int transmit(int radio, uint8_t *packet, size_t packet_size) {
    int written = write(radio, packet, packet_size);
    int err = 0;
    if (written == -1) {
        err = errno;
        inerr("Error transmitting: %d\n", err);
        return -err;
    }
    indebug("Completed transmission of packet #%u of %zu bytes.\n", ((pkt_hdr_t *)packet)->packet_num, packet_size);
    return written;
}

static int configure_radio(int fd, struct radio_options const *config) {
    int err = 0;

#if defined(CONFIG_LPWAN_RN2XX3)
    int32_t txpwr = config->txpwr * 100;
    uint64_t sync = config->sync;

    err = ioctl(fd, WLIOC_SETRADIOFREQ, config->freq);
    config_error(err);
    ininfo("RADIO: Set frequency to %d\n", config->freq);
    err = ioctl(fd, WLIOC_SETTXPOWERF, &txpwr);
    config_error(err);
    err = ioctl(fd, WLIOC_SETSPREAD, config->spread);
    ininfo("RADIO: Set spread to %d\n", config->spread);
    config_error(err);
    err = ioctl(fd, WLIOC_SETCODERATE, config->cr);
    config_error(err);
    ininfo("RADIO: Set code rate to %d\n", config->cr);
    err = ioctl(fd, WLIOC_SETBANDWIDTH, config->bw);
    config_error(err);
    ininfo("RADIO: Set bandwidth to %d\n", config->bw);
    err = ioctl(fd, WLIOC_CRCEN, config->crc);
    config_error(err);
    err = ioctl(fd, WLIOC_IQIEN, config->iqi);
    config_error(err);
    ininfo("RADIO: Set IQ inversion to %d\n", config->iqi);
    err = ioctl(fd, WLIOC_SETSYNC, &sync);
    config_error(err);
    ininfo("RADIO: Set sync to %llu\n", (unsigned long long)sync);
    err = ioctl(fd, WLIOC_SETPRLEN, config->preamble);
    config_error(err);
    ininfo("RADIO: Set preamble to %d\n", config->preamble);
#endif /* defined(CONFIG_LPWAN_RN2XX3) */

    return err;
}
