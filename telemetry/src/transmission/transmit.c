#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
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

static ssize_t transmit(int radio, uint8_t *packet, size_t packet_size);
static int configure_radio(int fd, struct radio_options const *config);

/* Main thread for data transmission over radio. */
void *transmit_main(void *arg) {

    int err;
    int radio = -1; /* Radio device file descriptor */
    struct transmit_args *unpacked_args = (struct transmit_args *)arg;
    packet_buffer_t *buffer = unpacked_args->buffer;
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
        packet_node_t *next_packet = packet_buffer_get_full(buffer);
        ((pkt_hdr_t *)next_packet->packet)->packet_num = seq_num++;
        transmit(radio, next_packet->packet, next_packet->end - next_packet->packet);
        packet_buffer_put_empty(buffer, next_packet);
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
static ssize_t transmit(int radio, uint8_t *packet, size_t packet_size) {
    ssize_t written = write(radio, packet, packet_size);
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
