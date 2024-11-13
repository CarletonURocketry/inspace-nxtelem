#include <string.h>

#include "packets.h"

/* Get the absolute timestamp that should be used for a packet created
 * at the given mission time
 */
static uint16_t calc_timestamp(uint32_t mission_time) {
  uint16_t timestamp = (mission_time / 1000) / 30;
  // If we're over halfway to the next abs_timestamp, rollover to it now
  if ((mission_time - (timestamp * 1000 * 30)) > 15000) {
    timestamp++;
  }
  return timestamp;
}

/* Calculate the offset timestamp
 * @param mission_time The absolute mission time in milliseconds
 * @param abs_timestamp The abs time to get the offset from
 * @param result The place to store the calculated value
 * @return If mission_time could be represented as an offset from the
 * given timestamp returns 0, otherwise returns 1
 */
static int calc_offset(uint32_t mission_time, uint16_t abs_timestamp,
                       int16_t *result) {
  /* Offset from abs_timestamp to time zero */
  int64_t offset = (int64_t)abs_timestamp * 30 * -1000;
  offset += mission_time;
  if (offset <= INT16_MAX && offset >= INT16_MIN) {
    *result = offset;
    return 0;
  }
  return 1;
}

/* Check if a block has an offset that needs to be set
 * @param b The block header
 * @returns 1 if the block has an offset, 0 otherwise
 */
static int has_offset(blk_hdr_t *b) {
  /* All blocks have offsets right now */
  switch ((enum block_type_e)b->type) {
  default:
    return 1;
  }
}

/* Initialize the packet header.
 * @param p The packet header to initialize.
 * @param packet_number The sequence number of this packet in the stream of
 * transmissions.
 */
void pkt_hdr_init(pkt_hdr_t *p, uint8_t packet_number, uint32_t mission_time) {
  memcpy(&p->call_sign, CONFIG_INSPACE_TELEMETRY_CALLSIGN,
         sizeof(CONFIG_INSPACE_TELEMETRY_CALLSIGN));
  p->packet_num = packet_number;
  p->timestamp = calc_timestamp(mission_time);
  p->blocks = 0;
}

/* Initialize the block header.
 * @param b The block header to initialize.
 * @param type The block type of the block this header will be associated with.
 * @param subtype The sub-type of the block this header will be associated with.
 */
void blk_hdr_init(blk_hdr_t *b, const enum block_type_e type) {
  b->type = type;
}

/* Return the length of the block that this header preceeds
 * @param b The header of the block to get the length of
 * @return The length of the block that follows this header
 */
int blk_len(blk_hdr_t *b) {
  switch ((enum block_type_e)b->type) {
  case DATA_TEMP:
    return sizeof(struct temp_blk_t);
    break;
  case DATA_HUMIDITY:
    return sizeof(struct hum_blk_t);
    break;
  case DATA_VOLTAGE:
    return sizeof(struct volt_blk_t);
    break;
  case DATA_LAT_LONG:
    return sizeof(struct coord_blk_t);
    break;
  case DATA_PRESSURE:
    return sizeof(struct pres_blk_t);
    break;
  case DATA_ANGULAR_VEL:
    return sizeof(struct ang_vel_blk_t);
    break;
  case DATA_ACCEL_REL:
    return sizeof(struct accel_blk_t);
    break;
  case DATA_ALT_LAUNCH:
    return sizeof(struct alt_blk_t);
    break;
  default:
    return 0;
    break;
  }
}

/* Prepare a block to be included with a packet
 * @param p The header of the packet to be sent, initialized
 * @param b The block to be sent, initialized
 * @param mission_time The number of milliseconds since the start of the mission
 * @returns 0 if the block can now be included with the packet, 1 otherwise
 */
int pkt_add_blk(pkt_hdr_t *p, blk_hdr_t *b, void *blk, uint32_t mission_time) {
  p->blocks++;
  if (has_offset(b)) {
    return calc_offset(mission_time, p->timestamp,
                       &((offset_blk *)blk)->time_offset);
  }
  return 0;
}

/*
 * Construct an altitude block.
 * @param b The altitude block to initialize.
 * @param altitude The altitude value measured in millimetres above launch
 * height.
 */
void alt_blk_init(struct alt_blk_t *b, const int32_t altitude) {
  b->altitude = altitude;
}

/*
 * Construct a temperature block.
 * @param b The temperature block to initialize.
 * @param temperature The temperature measured in millidegrees Celsius.
 */
void temp_blk_init(struct temp_blk_t *b, const int32_t temperature) {
  b->temperature = temperature;
}

/* TODO: other block types */
