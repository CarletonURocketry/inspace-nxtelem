#!/bin/bash

command -v sgdisk &>/dev/null || { echo "sgdisk not found. Run: brew install gptfdisk"; exit 1; }

DISK="disk${1:?Usage: $0 <disk number>}"

echo "Formatting /dev/$DISK — this will destroy all data."
read -p "Confirm? (y/N) " confirm
[[ "$confirm" != "y" ]] && exit 1

sudo sgdisk --zap-all /dev/$DISK \
  && sudo sgdisk -n 1:2048:+31G -t 1:0700 -c 1:PART1 /dev/$DISK \
  && sudo sgdisk -n 2:0:0 -t 2:0700 -c 2:PART2 /dev/$DISK \
  && sudo diskutil unmountDisk force /dev/$DISK \
  && sudo newfs_msdos -F 32 -v PART1 /dev/r${DISK}s1 \
  && echo "Done."

echo "--- diskutil ---"
diskutil list /dev/$DISK