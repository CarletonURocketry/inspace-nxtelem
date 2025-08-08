/* Controls the configuration shell which sets parameters of operation in the EEPROM and provides some maintenance
 * controls to the flight computer.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include <nuttx/usb/cdcacm.h>
#include <sys/boardctl.h>

#include "../syslogging.h"
#include "shell.h"

#ifndef CONFIG_CDCACM
#error "CONFIG_CDCACM must be enabled, as it is required for USB configuration"
#endif

/* Cast an error to a void pointer */

#define err_to_ptr(err) ((void *)((err)))

/* The path used for the USB console device */

#define USB_PATH "/dev/ttyACM0"

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

/* Main shell thread for configuring parameters in the EEPROM and controlling the operation of the flight computer.
 * @param arg The arguments struct for this shell, of type `struct shell_args`
 * @return 0 on successful exit, error code otherwise
 */
void *shell_main(void *arg) {
    int err;
    int usbfd;

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

    /* Now start doing shell things, like reading input handling commands. */

    dprintf(usbfd, "Hello world, this is the configuration shell on %s!\n", USB_PATH);

    for (;;) {
        // TODO
    }

    return err_to_ptr(ENOTRECOVERABLE);
}
