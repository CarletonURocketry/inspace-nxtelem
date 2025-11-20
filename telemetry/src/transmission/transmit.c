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
        pthread_mutex_lock(&radio_telem->empty_mux);
        pthread_mutex_lock(&radio_telem->full_mux);

        /* Swap the pointers of the empty and full data */
        radio_raw_data *temp = radio_telem->empty;
        radio_telem->empty = radio_telem->full;
        radio_telem->full = temp;

        /* Reset the new empty buffer's counters */
        radio_telem->empty->gnss_n = -1;
        radio_telem->empty->alt_n = -1;
        radio_telem->empty->mag_n = -1;
        radio_telem->empty->accel_n = -1;
        radio_telem->empty->gyro_n = -1;

        /* release the empty mutex, collector can lock it and start collecting again */
        pthread_mutex_unlock(&radio_telem->empty_mux);

        /* create a new packet buffer */
        uint8_t packet_buffer[PACKET_MAX_SIZE];
        uint8_t *packet_ptr = packet_buffer;

        /* use mission time as current time for now, this does not account for reboots */
        struct timespec current_time;
        clock_gettime(CLOCK_REALTIME, &current_time);
        uint32_t mission_time_ms = current_time.tv_sec * 1000 + current_time.tv_nsec / 1000000;
        pkt_hdr_t *header = (pkt_hdr_t *)packet_ptr;
        packet_ptr = pkt_init(packet_ptr, seq_num++, mission_time_ms);

        if (radio_telem->full->gnss_n > 0) {
            header->type_count++;
            blk_hdr_t blk_hdr = {
                .type = DATA_LAT_LONG,
                .count = radio_telem->full->gnss_n,
            };
            memcpy(packet_ptr, &blk_hdr, sizeof(blk_hdr));
            packet_ptr += sizeof(blk_hdr);
            for (int i = 0; i < radio_telem->full->gnss_n; i++) {
                struct coord_blk_t coord_blk;
                if (orb_gnss_pkt(&radio_telem->full->gnss[i], &coord_blk, header->timestamp)) {
                    inerr("Failed to create GNSS block %d\n", i);
                    continue;
                }

                memcpy(packet_ptr, &coord_blk, sizeof(struct coord_blk_t));
                packet_ptr += sizeof(struct coord_blk_t);
            }
        }

        if (radio_telem->full->alt_n > 0) {
            header->type_count++;
            blk_hdr_t blk_hdr = {
                .type = DATA_ALT_LAUNCH,
                .count = radio_telem->full->alt_n,
            };
            memcpy(packet_ptr, &blk_hdr, sizeof(blk_hdr));
            packet_ptr += sizeof(blk_hdr);
            for (int i = 0; i < radio_telem->full->alt_n; i++) {
                struct alt_blk_t alt_blk;
                if (orb_alt_pkt(&radio_telem->full->alt[i], &alt_blk, header->timestamp)) {
                    inerr("Failed to create Alt block %d\n", i);
                    continue;
                }
                memcpy(packet_ptr, &alt_blk, sizeof(struct alt_blk_t));
                packet_ptr += sizeof(struct alt_blk_t);
            }
        }

        if (radio_telem->full->mag_n > 0) {
            header->type_count++;
            blk_hdr_t blk_hdr = {
                .type = DATA_MAGNETIC,
                .count = radio_telem->full->mag_n,
            };
            memcpy(packet_ptr, &blk_hdr, sizeof(blk_hdr));
            packet_ptr += sizeof(blk_hdr);
            for (int i = 0; i < radio_telem->full->mag_n; i++) {
                struct mag_blk_t mag_blk;
                if (orb_mag_pkt(&radio_telem->full->mag[i], &mag_blk, header->timestamp)) {
                    inerr("Failed to create Mag block %d\n", i);
                    continue;
                }
                memcpy(packet_ptr, &mag_blk, sizeof(struct mag_blk_t));
                packet_ptr += sizeof(struct mag_blk_t);
            }
        }

        if (radio_telem->full->accel_n > 0) {
            header->type_count++;
            blk_hdr_t blk_hdr = {
                .type = DATA_ACCEL_REL,
                .count = radio_telem->full->accel_n,
            };
            memcpy(packet_ptr, &blk_hdr, sizeof(blk_hdr));
            packet_ptr += sizeof(blk_hdr);
            for (int i = 0; i < radio_telem->full->accel_n; i++) {
                struct accel_blk_t accel_blk;
                if (orb_accel_pkt(&radio_telem->full->accel[i], &accel_blk, header->timestamp)) {
                    inerr("Failed to create Accel block %d\n", i);
                    continue;
                }
                memcpy(packet_ptr, &accel_blk, sizeof(struct accel_blk_t));
                packet_ptr += sizeof(struct accel_blk_t);
            }
        }

        if (radio_telem->full->gyro_n > 0) {
            header->type_count++;
            blk_hdr_t blk_hdr = {
                .type = DATA_ANGULAR_VEL,
                .count = radio_telem->full->gyro_n,
            };
            memcpy(packet_ptr, &blk_hdr, sizeof(blk_hdr));
            packet_ptr += sizeof(blk_hdr);
            for (int i = 0; i < radio_telem->full->gyro_n; i++) {
                struct ang_vel_blk_t ang_vel_blk;
                if (orb_ang_vel_pkt(&radio_telem->full->gyro[i], &ang_vel_blk, header->timestamp)) {
                    inerr("Failed to create Ang vel block %d\n", i);
                    continue;
                }
                memcpy(packet_ptr, &ang_vel_blk, sizeof(struct ang_vel_blk_t));
                packet_ptr += sizeof(struct ang_vel_blk_t);
            }
        }

        pthread_mutex_unlock(&radio_telem->full_mux);
        size_t packet_size = packet_ptr - packet_buffer;
        if (packet_size > PACKET_MAX_SIZE) {
            /* if the packet size is too large we are cooked, this should never happen if the downsampling variable is
             * set correctly */
            inerr("Packet size is too large: %zu\n", packet_size);
            continue;
        } else if (packet_size == sizeof(pkt_hdr_t)) {
            inerr("Packet does not contain any data\n");
            /* this sleep is needed to avoid priority inversion between the 2 threads, this is a problem I didn't find a
             * good soluition to */
            sleep(1);
            continue;
        }
        ininfo("Packet size: %zu\n", packet_size);
        transmit(radio, packet_buffer, packet_size);
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
