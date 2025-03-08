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
 * @return 1 if the block has an offset, 0 otherwise
 */
static int has_offset(enum block_type_e type) {
  /* All blocks have offsets right now */
  switch (type) {
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
size_t blk_len(enum block_type_e type) {
  switch (type) {
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

/**
 * Add a block to a packet, both with initialized headers
 * 
 * @param packet The packet to add the block to with an initialized header in the first sizeof(pkt_hdr_t) bytes
 * @param len The length of the packet including the header
 * @param b_header The initialized block header to add to the packet
 * @param b_body The initialized block body to add to the packet
 * @param mission_time The mission time
 * @return The number of bytes added or 0 if the block cannot be added to the packet
 */
size_t pkt_add_blk(uint8_t *packet, uint8_t len, blk_hdr_t *b_header, uint8_t *b_body, uint32_t mission_time) {
  pkt_hdr_t *p_header = (pkt_hdr_t *)packet;
  if (len < sizeof(pkt_hdr_t)) {
    return 0;
  } 
  if ((len + blk_len(b_header) + sizeof(blk_hdr_t)) > PACKET_MAX_SIZE) {
    return 0;
  }
  if (has_offset(b_header) && (!calc_offset(mission_time, p_header->timestamp, &((offset_blk *)b_body)->time_offset))) {
    return 0;
  }
  memcpy((packet + len), b_header, sizeof(blk_hdr_t)); 
  len += sizeof(b_header);
  memcpy((packet + len), b_body, blk_len(b_header));
  p_header->blocks++;
  return sizeof(blk_hdr_t) + blk_len(b_header);
}

uint8_t *pkt_allocate_block(uint8_t *packet, uint8_t *write_pointer, enum block_type_e type, uint32_t mission_time) {
  pkt_hdr_t *header = (pkt_hdr_t *)packet;
  size_t packet_size = packet - write_pointer;
  size_t block_size = sizeof(blk_hdr_t) + blk_len(type);

  if (packet_size < sizeof(pkt_hdr_t)) {
    return NULL;
  }
  if ((packet_size + block_size) > PACKET_MAX_SIZE) {
    return NULL;
  }
  if (has_offset(type)) {
    uint8_t *block_body = write_pointer + sizeof(blk_hdr_t); 
    if (!calc_offset(mission_time, header->timestamp, &((offset_blk *)block_body)->time_offset)) {
      return NULL;
    }
  } 
  return write_pointer + block_size;
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

/*
 * Construct a acceleration block
 * @param b The acceleration block to initialize
 * @param x_axis Linear acceleration in the x-axis measured in centimetres per second
 * @param y_axis Linear acceleration in the y-axis measured in centimetres per second
 * @param z_axis Linear acceleration in the z-axis measured in centimetres per second
 */
void accel_blk_init(struct accel_blk_t *b, const int16_t x_axis,
                    const int16_t y_axis, const int16_t z_axis) {
  b->x = x_axis;
  b->y = y_axis;
  b->z = z_axis;
}
/* TODO: other block types */
