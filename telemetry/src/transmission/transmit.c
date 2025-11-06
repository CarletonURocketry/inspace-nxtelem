#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#if defined(CONFIG_LPWAN_RN2XX3)
#include <nuttx/wireless/ioctl.h>
#include <nuttx/wireless/lpwan/rn2xx3.h>
#endif

#include "../packets/packets.h"
#include "../collection/status-update.h"
#include "../syslogging.h"
#include "transmit.h"
#include "../packets/packets.h"

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
    packet_buffer_t *buffer = unpacked_args->buffer;
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
        radio_telem->empty->ang_vel_n = -1;

        /* release the empty mutex, collector can lock it and start collecting again */
        pthread_mutex_unlock(&radio_telem->empty_mux);

        /* create a new packet buffer */
        uint8_t packet_buffer[PACKET_MAX_SIZE];
        uint8_t *packet_ptr = packet_buffer;

        /* use mission time as current time for now */
        struct timespec current_time;
        clock_gettime(CLOCK_REALTIME, &current_time);
        pkt_hdr_init((pkt_hdr_t *)packet_ptr, seq_num, current_time.tv_sec);
        pkt_hdr_t *header = (pkt_hdr_t *)packet_ptr;
        packet_ptr = pkt_init(packet_ptr, 0, current_time.tv_sec);

        if(radio_telem->full->gnss_n > 0) {
            ininfo("GNSS blocks: %d\n", radio_telem->full->gnss_n);
            blk_hdr_t blk_hdr = {
                .type = DATA_LAT_LONG,
                .count = radio_telem->full->gnss_n,
            };
            memcpy(packet_ptr, &blk_hdr, sizeof(blk_hdr));
            packet_ptr += sizeof(blk_hdr);
            for(int i = 0; i < radio_telem->full->gnss_n; i++) {
                struct coord_blk_t coord_blk = radio_telem->full->gnss[i];
                int16_t time_offset;
                uint32_t mission_time = (uint32_t)coord_blk.time_offset;
                if(pkt_blk_calc_time(mission_time, header->timestamp, &time_offset)) {
                    inerr("Failed to calculate time offset for GNSS block %d\n", i);
                    continue;
                }
                coord_blk.time_offset = time_offset;
                memcpy(packet_ptr, &coord_blk, sizeof(struct coord_blk_t));
                packet_ptr += sizeof(struct coord_blk_t);
            }
        }

        if(radio_telem->full->alt_n > 0) {
            ininfo("Alt blocks: %d\n", radio_telem->full->alt_n);
            blk_hdr_t blk_hdr = {
                .type = DATA_ALT_LAUNCH,
                .count = radio_telem->full->alt_n,
            };
            memcpy(packet_ptr, &blk_hdr, sizeof(blk_hdr));
            packet_ptr += sizeof(blk_hdr);
            for(int i = 0; i < radio_telem->full->alt_n; i++) {
                struct alt_blk_t alt_blk = radio_telem->full->alt[i];
                int16_t time_offset;
                uint32_t mission_time = (uint32_t)alt_blk.time_offset;
                if(pkt_blk_calc_time(mission_time, header->timestamp, &time_offset)) {
                    inerr("Failed to calculate time offset for Alt block %d\n", i);
                    continue;
                }
                alt_blk.time_offset = time_offset;
                memcpy(packet_ptr, &alt_blk, sizeof(struct alt_blk_t));
                packet_ptr += sizeof(struct alt_blk_t);
            }
        }

        if(radio_telem->full->mag_n > 0) {
            ininfo("Mag blocks: %d\n", radio_telem->full->mag_n);
            blk_hdr_t blk_hdr = {
                .type = DATA_MAGNETIC,
                .count = radio_telem->full->mag_n,
            };
            memcpy(packet_ptr, &blk_hdr, sizeof(blk_hdr));
            packet_ptr += sizeof(blk_hdr);
            for(int i = 0; i < radio_telem->full->mag_n; i++) {
                struct mag_blk_t mag_blk = radio_telem->full->mag[i];
                int16_t time_offset;
                uint32_t mission_time = (uint32_t)mag_blk.time_offset;
                if(pkt_blk_calc_time(mission_time, header->timestamp, &time_offset)) {
                    inerr("Failed to calculate time offset for Mag block %d\n", i);
                    continue;
                }
                mag_blk.time_offset = time_offset;
                memcpy(packet_ptr, &mag_blk, sizeof(struct mag_blk_t));
                packet_ptr += sizeof(struct mag_blk_t);
            }
        }

        if(radio_telem->full->accel_n > 0) {
            ininfo("Accel blocks: %d\n", radio_telem->full->accel_n);
            blk_hdr_t blk_hdr = {
                .type = DATA_ACCEL_REL,
                .count = radio_telem->full->accel_n,
            };
            memcpy(packet_ptr, &blk_hdr, sizeof(blk_hdr));
            packet_ptr += sizeof(blk_hdr);
            for(int i = 0; i < radio_telem->full->accel_n; i++) {
                struct accel_blk_t accel_blk = radio_telem->full->accel[i];
                int16_t time_offset;
                uint32_t mission_time = (uint32_t)accel_blk.time_offset;
                if(pkt_blk_calc_time(mission_time, header->timestamp, &time_offset)) {
                    inerr("Failed to calculate time offset for Accel block %d\n", i);
                    continue;
                }
                accel_blk.time_offset = time_offset;
                memcpy(packet_ptr, &accel_blk, sizeof(struct accel_blk_t));
                packet_ptr += sizeof(struct accel_blk_t);
            }
        }

        if(radio_telem->full->ang_vel_n > 0) {
            ininfo("Ang vel blocks: %d\n", radio_telem->full->ang_vel_n);
            blk_hdr_t blk_hdr = {
                .type = DATA_ANGULAR_VEL,
                .count = radio_telem->full->ang_vel_n,
            };
            memcpy(packet_ptr, &blk_hdr, sizeof(blk_hdr));
            packet_ptr += sizeof(blk_hdr);
            for(int i = 0; i < radio_telem->full->ang_vel_n; i++) {
                struct ang_vel_blk_t ang_vel_blk = radio_telem->full->ang_vel[i];
                int16_t time_offset;
                uint32_t mission_time = (uint32_t)ang_vel_blk.time_offset;
                if(pkt_blk_calc_time(mission_time, header->timestamp, &time_offset)) {
                    inerr("Failed to calculate time offset for Ang vel block %d\n", i);
                    continue;
                }
                ang_vel_blk.time_offset = time_offset;
                memcpy(packet_ptr, &ang_vel_blk, sizeof(struct ang_vel_blk_t));
                packet_ptr += sizeof(struct ang_vel_blk_t);
            }
        }

        pthread_mutex_unlock(&radio_telem->full_mux);
        size_t packet_size = packet_ptr - packet_buffer;
        if(packet_size > PACKET_MAX_SIZE) {
            inerr("Packet size is too large: %zu\n", packet_size);
            continue;
        } else if (packet_size == sizeof(pkt_hdr_t)) {
            inerr("Packet does not contain any data\n");
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
    err = ioctl(fd, WLIOC_SETTXPOWERF, &txpwr);
    config_error(err);
    err = ioctl(fd, WLIOC_SETSPREAD, config->spread);
    config_error(err);
    err = ioctl(fd, WLIOC_SETCODERATE, config->cr);
    config_error(err);
    err = ioctl(fd, WLIOC_SETBANDWIDTH, config->bw);
    config_error(err);
    err = ioctl(fd, WLIOC_CRCEN, config->crc);
    config_error(err);
    err = ioctl(fd, WLIOC_IQIEN, config->iqi);
    config_error(err);
    err = ioctl(fd, WLIOC_SETSYNC, &sync);
    config_error(err);
    err = ioctl(fd, WLIOC_SETPRLEN, config->preamble);
    config_error(err);
#endif /* defined(CONFIG_LPWAN_RN2XX3) */

    return err;
}
