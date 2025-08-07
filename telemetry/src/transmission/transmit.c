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

/* InSpace chosen radio settings - data types are as expected by radio driver */

#if defined(CONFIG_LPWAN_RN2XX3)
#define RN2483_FREQ (uint32_t)433050000
#define RN2483_TXPWR (int32_t)15
#define RN2483_SPREAD_FACTOR (uint8_t)7
#define RN2483_CODING_RATE (enum rn2xx3_cr_e) RN2XX3_CR_4_5
#define RN2483_BANDWIDTH (uint32_t)125
#define RN2483_CRC 1
#define RN2483_IQI 0
#define RN2483_SYNC (uint64_t)67
#define RN2483_PREAMBLE (uint16_t)6
#endif /* defined(CONFIG_LPWAN_RN2XX3) */

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
static int configure_radio(int fd);

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

    err = configure_radio(radio);
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

static int configure_radio(int fd) {
    int err = 0;

#if defined(CONFIG_LPWAN_RN2XX3)
    int32_t txpwr = RN2483_TXPWR * 100;
    uint64_t sync = RN2483_SYNC;

    err = ioctl(fd, WLIOC_SETRADIOFREQ, RN2483_FREQ);
    config_error(err);
    err = ioctl(fd, WLIOC_SETTXPOWERF, &txpwr);
    config_error(err);
    err = ioctl(fd, WLIOC_SETSPREAD, RN2483_SPREAD_FACTOR);
    config_error(err);
    err = ioctl(fd, WLIOC_SETCODERATE, RN2483_CODING_RATE);
    config_error(err);
    err = ioctl(fd, WLIOC_SETBANDWIDTH, RN2483_BANDWIDTH);
    config_error(err);
    err = ioctl(fd, WLIOC_CRCEN, RN2483_CRC);
    config_error(err);
    err = ioctl(fd, WLIOC_IQIEN, RN2483_IQI);
    config_error(err);
    err = ioctl(fd, WLIOC_SETSYNC, &sync);
    config_error(err);
    err = ioctl(fd, WLIOC_SETPRLEN, RN2483_PREAMBLE);
    config_error(err);
#endif /* defined(CONFIG_LPWAN_RN2XX3) */

    return err;
}
