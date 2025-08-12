/* Controls the configuration shell which sets parameters of operation in the EEPROM and provides some maintenance
 * controls to the flight computer.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#include <nuttx/usb/cdcacm.h>
#include <sys/boardctl.h>

#include "../rocket-state/rocket-state.h"
#include "../syslogging.h"
#include "helptext.h"
#include "shell.h"

#ifndef CONFIG_CDCACM
#error "CONFIG_CDCACM must be enabled, as it is required for USB configuration"
#endif

/* Cast an error to a void pointer */

#define err_to_ptr(err) ((void *)((err)))

/* The path used for the USB console device */

#define USB_PATH "/dev/ttyACM0"

/* Maximum size of the input command */

#define COMMAND_IN_SIZE 128

static int usb_init(void);
static int read_command(int usbfd, char *buf, size_t n);
static void print_config(int usbfd, struct config_options const *config);
static char *get_first_arg(char *command);

/* Main shell thread for configuring parameters in the EEPROM and controlling the operation of the flight computer.
 * @param arg The arguments struct for this shell, of type `struct shell_args`
 * @return 0 on successful exit, error code otherwise
 */
void *shell_main(void *arg) {
    int err;
    int usbfd;
    struct config_options modified;
    struct config_options disk;
    char command_in[COMMAND_IN_SIZE];

    (void)(arg);

    /* Set up the USB device */

    err = usb_init();
    if (err) {
        inerr("Couldn't initialize USB device: %d\n", err);
        return err_to_ptr(err);
    }
    ininfo("USB device initialized.");

    /* Get access to the USB device */

    usbfd = open(USB_PATH, O_RDWR);
    if (usbfd < 0) {
        inerr("Couldn't open %s: %d\n", USB_PATH, errno);
        return err_to_ptr(err);
    }

    sleep(1); /* Makes sure that all following prints are captured. */

    /* Load disk for modification */

    if (config_get(&modified)) {
        inerr("Couldn't get initial disk contents\n");
    }

    /* Now start doing shell things, like reading input handling commands. */

    dprintf(usbfd, "Hello world, this is the configuration shell on %s!\n", USB_PATH);

    for (;;) {

        /* Indicate waiting for command */

        dprintf(usbfd, "Waiting for command...\n");

        /* Get an input command */

        err = read_command(usbfd, command_in, sizeof(command_in));
        if (err) {
            inerr("Couldn't read input command: %d\n", err);
            continue;
        }

        /* Generic commands */

        if (strstr(command_in, "reboot")) {
            /* Reboot the board, causing changes to come into effect */

            dprintf(usbfd, "Rebooting...\n");
            boardctl(BOARDIOC_RESET, 0);
        } else if (strstr(command_in, "disk")) {
            /* Shows the current configuration stored in EEPROM */

            if (config_get(&disk)) {
                dprintf(usbfd, "Couldn't read EEPROM\n");
            }
            print_config(usbfd, &disk);
        } else if (strstr(command_in, "current")) {
            /* Shows the current configuration stored in RAM (modified) */

            print_config(usbfd, &modified);
        } else if (strstr(command_in, "load")) {
            /* Loads the disk configuration into the modifiable copy */

            if (config_get(&modified)) {
                dprintf(usbfd, "Couldn't read EEPROM\n");
            }
            dprintf(usbfd, "Configuration loaded!\n");
        } else if (strstr(command_in, "save")) {
            /* Saves the entire modified configuration to EEPROM */

            if (config_set(&modified)) {
                dprintf(usbfd, "Couldn't write to EEPROM\n");
            }
            dprintf(usbfd, "Configuration saved!\n");
        } else if (strstr(command_in, "help")) {
            /* Print out the help text for the shell */

            dprintf(usbfd, HELP_TEXT);
        }

        /* Commands for setting radio parameters */

        else if (strstr(command_in, "frequency")) {
            modified.radio.freq = strtoul(get_first_arg(command_in), NULL, 10);
            print_config(usbfd, &modified);
        } else if (strstr(command_in, "preamble")) {
            modified.radio.preamble = strtoul(get_first_arg(command_in), NULL, 10);
            print_config(usbfd, &modified);
        } else if (strstr(command_in, "spread")) {
            modified.radio.spread = strtoul(get_first_arg(command_in), NULL, 10);
            print_config(usbfd, &modified);
        } else if (strstr(command_in, "txpwr")) {
            modified.radio.txpwr = atoi(get_first_arg(command_in));
            print_config(usbfd, &modified);
        } else if (strstr(command_in, "bandwidth")) {
            modified.radio.bw = strtoul(get_first_arg(command_in), NULL, 10);
            print_config(usbfd, &modified);
        } else if (strstr(command_in, "sync")) {
            modified.radio.sync = strtoul(get_first_arg(command_in), NULL, 16);
            print_config(usbfd, &modified);
        } else if (strstr(command_in, "sync")) {
            modified.radio.sync = strtoul(get_first_arg(command_in), NULL, 16);
            print_config(usbfd, &modified);
        } else if (strstr(command_in, "crc")) {
            modified.radio.crc = atoi(get_first_arg(command_in));
            print_config(usbfd, &modified);
        } else if (strstr(command_in, "iqi")) {
            modified.radio.iqi = atoi(get_first_arg(command_in));
            print_config(usbfd, &modified);
        } else if (strstr(command_in, "coder")) {
            char *firstarg = get_first_arg(command_in);
            if (strstr(firstarg, "4/5")) {
                modified.radio.cr = RN2XX3_CR_4_5;
            } else if (strstr(firstarg, "4/6")) {
                modified.radio.cr = RN2XX3_CR_4_6;
            } else if (strstr(firstarg, "4/7")) {
                modified.radio.cr = RN2XX3_CR_4_7;
            } else if (strstr(firstarg, "4/8")) {
                modified.radio.cr = RN2XX3_CR_4_8;
            } else {
                dprintf(usbfd, "Unknown coding rate: %s\n", firstarg);
            }
            print_config(usbfd, &modified);
        }

        /* Default case */

        else {
            dprintf(usbfd, "Unkown command: %s\n", command_in);
        }
    }

    return err_to_ptr(ENOTRECOVERABLE);
}

/* Initializes the USB CDCACM console device and waits for it to be brought up.
 * @return 0 on success, an error code on error.
 */
static int usb_init(void) {
    void *handle;
    int err;
    int usbfd;
    struct boardioc_usbdev_ctrl_s ctrl = {
        .usbdev = BOARDIOC_USBDEV_CDCACM,
        .action = BOARDIOC_USBDEV_CONNECT,
        .instance = 0,
        .handle = &handle,
    };

    /* Try to connect the USB device */

    err = boardctl(BOARDIOC_USBDEV_CONTROL, (uintptr_t)&ctrl);
    if (err < 0) {
        inerr("boardctl(BOARDIOC_USBDEV_CONTROL) failed: %d\n", errno);
        return err;
    }

    /* Now that the USB device initialization has started, wait to see if it comes up */

    do {
        usbfd = open(USB_PATH, O_RDWR);

        /* If we get anything besides an indication that the USB is not yet connected, that's a failure */

        if (errno != ENOTCONN) {
            inerr("Got unexpected error code: %d\n", errno);
            return errno;
        }

    } while (usbfd < 0);

    /* If we are here, the USB device is up and running. Close it to pass along control. */

    close(usbfd);
    return 0;
}

static int read_command(int usbfd, char *buf, size_t n) {
    ssize_t bread;
    char *end = buf;

    /* Read one character at a time until we hit a newline character, indicating end of command */

    while (end - buf < n) {
        bread = read(usbfd, end, 1);
        if (bread < 0) {
            return errno;
        }
        if (*end == '\n') break;

        end += bread; /* Increment end pointer */
    }

    /* We found a newline character, replace the newline with null terminator and pass along command */

    if (*end == '\n') {
        *end = '\0';
        return 0;
    }

    /* Ran out of space */

    return ENOMEM;
}

/* Prints the radio configuration parameter struct in a user legible way */
static void print_radio_config(int usbfd, struct radio_options const *config) {
    char *coding_rate;

    dprintf(usbfd, "radio {\n");
    dprintf(usbfd, "\tFrequency: %lu Hz\n", config->freq);
    dprintf(usbfd, "\tTransmit power: %ld dBm\n", config->txpwr);
    dprintf(usbfd, "\tSync word: %016llX\n", config->sync);
    dprintf(usbfd, "\tBandwidth: %lu kHz\n", config->bw);
    dprintf(usbfd, "\tPreamble length: %u\n", config->preamble);
    dprintf(usbfd, "\tSpread factor: %u\n", config->spread);
    dprintf(usbfd, "\tCRC: %s\n", config->crc ? "on" : "off");
    dprintf(usbfd, "\tIQI: %s\n", config->iqi ? "on" : "off");

    switch (config->cr) {
    case RN2XX3_CR_4_5:
        coding_rate = "4/5";
        break;
    case RN2XX3_CR_4_6:
        coding_rate = "4/6";
        break;
    case RN2XX3_CR_4_7:
        coding_rate = "4/7";
        break;
    case RN2XX3_CR_4_8:
        coding_rate = "4/8";
        break;
    default:
        coding_rate = "ERR";
        break;
    }

    dprintf(usbfd, "\tCoding rate: %s\n", coding_rate);
    dprintf(usbfd, "}\n");
}

/* Prints the configuration parameter struct in a user legible way */
static void print_config(int usbfd, struct config_options const *config) { print_radio_config(usbfd, &config->radio); }

/* Gets the first argument in the command (based on space separation). */
static char *get_first_arg(char *command) {
    strtok(command, " ");
    return strtok(NULL, " ");
}
