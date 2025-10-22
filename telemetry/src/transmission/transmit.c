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

        printf("Accel in packet: %d\n", radio_telem->full->accel_n);

        /* Reset the new empty buffer's counters */
        radio_telem->empty->gnss_n = 0;
        radio_telem->empty->alt_n = 0;
        radio_telem->empty->mag_n = 0;
        radio_telem->empty->accel_n = 0;
        radio_telem->empty->ang_vel_n = 0;

        pthread_mutex_unlock(&radio_telem->empty_mux);


        /* create a new packet buffer */
        uint8_t packet_buffer[PACKET_MAX_SIZE];
        uint8_t *packet_ptr = packet_buffer;

        /* use mission time as current time for now */
        struct timespec current_time;
        clock_gettime(CLOCK_REALTIME, &current_time);
        pkt_hdr_init((pkt_hdr_t *)packet_ptr, 0, current_time.tv_sec);
        packet_ptr = pkt_init(packet_ptr, 0, current_time.tv_sec);

        /* downsample the data to get ~3hz from each sensor */
        int accel_skip = (radio_telem->full->accel_n > 7) ? (radio_telem->full->accel_n / 7) : 1;
        int gyro_skip = (radio_telem->full->ang_vel_n > 7) ? (radio_telem->full->ang_vel_n / 7) : 1;
        int mag_skip = (radio_telem->full->mag_n > 7) ? (radio_telem->full->mag_n / 7) : 1;
        int alt_skip = (radio_telem->full->alt_n > 7) ? (radio_telem->full->alt_n / 7) : 1;
        int gnss_skip = (radio_telem->full->gnss_n > 7) ? (radio_telem->full->gnss_n / 7) : 1;

        /* Add GNSS coordinate blocks */
        if (radio_telem->full->gnss_n > 0) {
            uint8_t num_blocks = (uint8_t)(radio_telem->full->gnss_n / gnss_skip);
            blk_hdr_init((blk_hdr_t *)packet_ptr, DATA_LAT_LONG, num_blocks);
            packet_ptr += sizeof(blk_hdr_t);
            for(int i = 0; i < radio_telem->full->gnss_n; i += gnss_skip) {
                memcpy(packet_ptr, &radio_telem->full->gnss[i], sizeof(struct coord_blk_t));
                packet_ptr += sizeof(struct coord_blk_t);
            }
            ((pkt_hdr_t *)packet_buffer)->blocks++;
        }

        if (radio_telem->full->alt_n > 0) {
            uint8_t num_blocks = (uint8_t)(radio_telem->full->alt_n / alt_skip);
            blk_hdr_init((blk_hdr_t *)packet_ptr, DATA_ALT_LAUNCH, num_blocks);
            packet_ptr += sizeof(blk_hdr_t);

            for(int i = 0; i < radio_telem->full->alt_n; i += alt_skip) {
                memcpy(packet_ptr, &radio_telem->full->alt[i], sizeof(struct alt_blk_t));
                packet_ptr += sizeof(struct alt_blk_t);
            }
            ((pkt_hdr_t *)packet_buffer)->blocks++;
        }

        if (radio_telem->full->mag_n > 0) {
            uint8_t num_blocks = (uint8_t)(radio_telem->full->mag_n / mag_skip);
            blk_hdr_init((blk_hdr_t *)packet_ptr, DATA_MAGNETIC, num_blocks);

            packet_ptr += sizeof(blk_hdr_t);
            for(int i = 0; i < radio_telem->full->mag_n; i += mag_skip) {
                memcpy(packet_ptr, &radio_telem->full->mag[i], sizeof(struct mag_blk_t));
                packet_ptr += sizeof(struct mag_blk_t);
            }
        }

        if (radio_telem->full->accel_n > 0) {
            uint8_t num_blocks = (uint8_t)(radio_telem->full->accel_n / accel_skip);
            blk_hdr_init((blk_hdr_t *)packet_ptr, DATA_ACCEL_REL, num_blocks);

            packet_ptr += sizeof(blk_hdr_t);
            for(int i = 0; i < radio_telem->full->accel_n; i += accel_skip) {
                memcpy(packet_ptr, &radio_telem->full->accel[i], sizeof(struct accel_blk_t));
                packet_ptr += sizeof(struct accel_blk_t);
            }
            ((pkt_hdr_t *)packet_buffer)->blocks++;
        }

        if (radio_telem->full->ang_vel_n > 0) {
            uint8_t num_blocks = (uint8_t)(radio_telem->full->ang_vel_n / gyro_skip);
            blk_hdr_init((blk_hdr_t *)packet_ptr, DATA_ANGULAR_VEL, num_blocks);

            packet_ptr += sizeof(blk_hdr_t);
            for(int i = 0; i < radio_telem->full->ang_vel_n; i += gyro_skip) {
                memcpy(packet_ptr, &radio_telem->full->ang_vel[i], sizeof(struct ang_vel_blk_t));
                packet_ptr += sizeof(struct ang_vel_blk_t);
            }
            ((pkt_hdr_t *)packet_buffer)->blocks++;
        }

        pthread_mutex_unlock(&radio_telem->full_mux);
        size_t packet_size = packet_ptr - packet_buffer;
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
