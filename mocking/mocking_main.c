#include <nuttx/config.h>

#include <nuttx/sensors/fakesensor.h>
#include <nuttx/sensors/sensor.h>
#include <nuttx/drivers/ramdisk.h>

#include <sys/mount.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include "data.img.h"

#define CONFIG_MOCK_ROMFS_DEVNO 1
#define CONFIG_MOCK_ROMFS_MOUNTPOINT "/usr/local/share"
#define CONFIG_MOCK_ROMFS_SECTORSIZE 512


#define CONFIG_MOCK
#define STR_RAMDEVNO(m)    #m
#define MKMOUNT_DEVNAME(m) "/dev/ram" STR_RAMDEVNO(m)
#define NSECTORS(b)        (((b)+CONFIG_MOCK_ROMFS_SECTORSIZE-1)/CONFIG_MOCK_ROMFS_SECTORSIZE)

#define MOUNT_DEVNAME MKMOUNT_DEVNAME(CONFIG_MOCK_ROMFS_DEVNO)
#define MOCK_BARO_FILE CONFIG_MOCK_ROMFS_MOUNTPOINT "baro.csv"

//
int mount_mock_fs() {
    // Created a ROMDISK using genromfs, then converted it into a header file earlier
    int ret;
    ret = romdisk_register(CONFIG_MOCK_ROMFS_DEVNO, data_img, NSECTORS(data_img_len), CONFIG_MOCK_ROMFS_SECTORSIZE);
    if (ret < 0) {
        printf("ERROR: Failed to create RAM disk: %s\n", strerror(errno));
        return -1
    }
    printf("Mounting ROMFS filesystem at %s with source /dev/ram1\n",
        CONFIG_MOCK_ROMFS_MOUNTPOINT);
    ret = mount(MOUNT_DEVNAME, MOUNT_CONFIG_MOCK_ROMFS_MOUNTPOINT, "romfs", MS_RDONLY, NULL);
    if (ret < 0) {
        printF("ERROR: Failed to mount: %s", strerror(errno));
    }
}

int activate_mock_sensor() {
    fakesensor_init(SENSOR_TYPE_BAROMETER, MOCK_BARO_FILE, 0, 0);
}

int main(int argc, FAR char *argv[]) {
    mount_mock_fs();
    activate_mock_sensor();
}
