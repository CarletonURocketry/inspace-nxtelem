#include <nuttx/config.h>

#include <sys/mount.h>
#include <sys/boardctl.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "data_romfs.h"

#define CONFIG_MOCK_ROMFS_DEVNO 1
#define CONFIG_MOCK_ROMFS_SECTORSIZE 512


#define STR_RAMDEVNO(m)    #m
#define MKMOUNT_DEVNAME(m) "/dev/ram" STR_RAMDEVNO(m)
#define NSECTORS(b)        (((b)+CONFIG_MOCK_ROMFS_SECTORSIZE-1)/CONFIG_MOCK_ROMFS_SECTORSIZE)

#define MOUNT_DEVNAME MKMOUNT_DEVNAME(CONFIG_MOCK_ROMFS_DEVNO)

int mount_mock_fs(void) {
    int ret;
    struct boardioc_romdisk_s desc;

    desc.minor    = CONFIG_MOCK_ROMFS_DEVNO;         /* Minor device number of the ROM disk. */
    desc.nsectors = NSECTORS(data_img_len);              /* The number of sectors in the ROM disk */
    desc.sectsize = CONFIG_MOCK_ROMFS_SECTORSIZE;       /* The size of one sector in bytes */
    desc.image    = (FAR uint8_t *)data_img;             /* File system image */

    ret = boardctl(BOARDIOC_ROMDISK, (uintptr_t)&desc);
    if (ret < 0) {
        printf("ERROR: Failed to create RAM disk: %s\n", strerror(errno));
        return 1;
    }

    printf("Mounting ROMFS filesystem at target=%s with source=%s\n",
            CONFIG_INSPACE_MOCK_MOUNT, MOUNT_DEVNAME);
    ret = mount(MOUNT_DEVNAME, CONFIG_INSPACE_MOCK_MOUNT, "romfs",
                MS_RDONLY, NULL);
    if (ret < 0) {
        printf("ERROR: Mount failed: %s\n", strerror(errno));
        return 1;
    }
    return 0;
}

int main(int argc, char **argv) {
    return mount_mock_fs();
}
