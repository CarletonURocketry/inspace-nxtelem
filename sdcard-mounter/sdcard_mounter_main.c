#include <debug.h>
#include <nuttx/config.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/partition.h>
#include <stdio.h>

typedef struct {
  int partition_num;
  uint8_t err;
} partition_state_t;

static void partition_handler(struct partition_s *part, void *arg) {
  partition_state_t *partition_handler_state = (partition_state_t *)arg;

  char devname[] = "/dev/mmcsd0p0";

  if (partition_handler_state->partition_num < 10 &&
      part->index == partition_handler_state->partition_num) {
    finfo("Num of sectors: %d \n", part->nblocks);
    devname[sizeof(devname) - 2] = partition_handler_state->partition_num + 48;
    register_blockpartition(devname, 0, "/dev/mmcsd0", part->firstblock,
                            part->nblocks);
    partition_handler_state->err = 0;
  }
}

int main(int argc, char **argv) {
  /*Partitioning and mounting code*/
  int ret = 0;

  static partition_state_t partitions[] = {
      {.partition_num = 0, .err = ENOENT},
      {.partition_num = 1, .err = ENOENT},
  };

  for (int i = 0; i < 2; i++) {
    parse_block_partition("/dev/mmcsd0", partition_handler, &partitions[i]);
    if (partitions[i].err == ENOENT) {
      fwarn("Partition %d did not register \n", partitions[i].partition_num);
    } else {
      finfo("Partition %d registered! \n", partitions[i].partition_num);
    }
  }

  ret = nx_mount("/dev/mmcsd0p0", "/mnt/sd0p0", "vfat", 666, NULL);
  /*Invalid Superblock, means it's not formatted as vfat*/
  if (ret == -EINVAL) {
    // for now do nothing, in the future try to format it as fat 32.
    ;
  }
  if (ret) {
    ferr("ERROR: Could not mount fat partition %d: \n", ret);
    return ret;
  }

  /*Mount our littlefs partition, and format it if it's corrupted*/
  ret = nx_mount("dev/mmcsd0p1", "/mnt/sd0p1", "littlefs", 666, "autoformat");
  if (ret) {
    ferr("ERROR: could not mount littlefs partition %d: \n", ret);
    return ret;
  }

  return OK;
}
