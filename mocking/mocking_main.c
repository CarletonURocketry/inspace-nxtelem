#include <nuttx/config.h>
#include <stdint.h>
#include <nuttx/sensors/fakesensor.h>
#include <nuttx/sensors/sensor.h>
#include <sys/mount.h>
#include <sys/boardctl.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "data_romfs.h"

#define CONFIG_MOCKING_ROMFS_DEVNO 4
#define CONFIG_MOCKING_ROMFS_SECTORSIZE 512


#define STR_RAMDEVNO(m)    #m
#define MKMOUNT_DEVNAME(m) "/dev/ram" STR_RAMDEVNO(m)
#define NSECTORS(b)        (((b)+CONFIG_MOCKING_ROMFS_SECTORSIZE-1)/CONFIG_MOCKING_ROMFS_SECTORSIZE)

#define MOUNT_DEVNAME MKMOUNT_DEVNAME(CONFIG_MOCKING_ROMFS_DEVNO)

/**
 * Mounts a ROM file system, which has already been created by genromfs and xxd
 * @return 0 if the ROM file system mounted correctly
 */
int mount_mock_fs(void) {
    int ret;
    struct boardioc_romdisk_s desc;

    desc.minor    = CONFIG_MOCKING_ROMFS_DEVNO;         /* Minor device number of the ROM disk. */
    desc.nsectors = NSECTORS(data_img_len);              /* The number of sectors in the ROM disk */
    desc.sectsize = CONFIG_MOCKING_ROMFS_SECTORSIZE;       /* The size of one sector in bytes */
    desc.image    = (FAR uint8_t *)data_img;             /* File system image */

    ret = boardctl(BOARDIOC_ROMDISK, (uintptr_t)&desc);
    if (ret < 0) {
        printf("ERROR: Failed to create RAM disk: %s\n", strerror(errno));
        return 1;
    }

    printf("Mounting ROMFS filesystem at target=%s with source=%s\n",
            CONFIG_INSPACE_MOCKING_MOUNT, MOUNT_DEVNAME);
    ret = mount(MOUNT_DEVNAME, CONFIG_INSPACE_MOCKING_MOUNT, "romfs",
                MS_RDONLY, NULL);
    if (ret < 0) {
        printf("ERROR: Mount failed: %s\n", strerror(errno));
        return 1;
    }
    return 0;
}

/**
 * Creates fakesensors as was configured, only works with a flat build because we're registering drivers
 * in entirely the wrong place
 * @return 0 if the sensors were mounted correctly
 */
int register_fakesensors(void) {
    int ret = 0;
#if defined(CONFIG_INSPACE_FAKE_BARO)
    printf("Mounting a fake barometer with csv %s\n", CONFIG_INSPACE_FAKE_BARO_FILENAME);
    ret = fakesensor_init(SENSOR_TYPE_BAROMETER, CONFIG_INSPACE_FAKE_BARO_FILENAME, 0, CONFIG_INSPACE_FAKE_BARO_MAX_BATCH);
    if (ret < 0)
    {
        fprintf(stderr, "ERROR: fakesensor_init() failed: %d\n", ret);
        return ret;
    }
#endif

#if defined(CONFIG_INSPACE_FAKE_ACCEL)
    printf("Mounting a fake accelerometer with csv %s\n", CONFIG_INSPACE_FAKE_ACCEL_FILENAME);
    ret = fakesensor_init(SENSOR_TYPE_ACCELEROMETER, CONFIG_INSPACE_FAKE_ACCEL_FILENAME, 0, CONFIG_INSPACE_FAKE_ACCEL_MAX_BATCH);
    if (ret < 0)
    {
        fprintf(stderr, "ERROR: fakesensor_init() failed: %d\n", ret);
        return ret;
    }
#endif

    return ret;
}
int main(int argc, char **argv) {
    int ret = 0;
    ret = mount_mock_fs();
    if (ret < 0) {
        return ret;
    } 
    return register_fakesensors();
}
