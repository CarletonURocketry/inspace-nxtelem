#include <string.h>

#include <stdio.h>

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

static uint32_t expand_abs_timestamp(uint16_t abs_timestamp) {
  return abs_timestamp * 30 * 1000;
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

/* Return the length of a block of this type's body, excluding the header
 * @param type The type of block to get the length of
 * @return The number of bytes in the block body
 */
size_t blk_body_len(enum block_type_e type) {
  switch (type) {
  case DATA_TEMP:
    return sizeof(struct temp_blk_t);
  case DATA_HUMIDITY:
    return sizeof(struct hum_blk_t);
  case DATA_VOLTAGE:
    return sizeof(struct volt_blk_t);
  case DATA_LAT_LONG:
    return sizeof(struct coord_blk_t);
  case DATA_PRESSURE:
    return sizeof(struct pres_blk_t);
  case DATA_ANGULAR_VEL:
    return sizeof(struct ang_vel_blk_t);
  case DATA_ACCEL_REL:
    return sizeof(struct accel_blk_t);
  case DATA_ACCEL_ABS:
    return sizeof(struct accel_blk_t);
  case DATA_ALT_LAUNCH:
    return sizeof(struct alt_blk_t);
  default:
    return 0;
  }
}

/* Initialize a packet with a header and return a pointer to the first byte of its body
 * @param packet The packet to initialize
 * @param packet_num The sequence number of the packet
 * @param mission_time The mission time of the packet
 * @returns A pointer to the first byte of the packet body
 */
uint8_t *init_pkt(uint8_t *packet, uint8_t packet_num, uint32_t mission_time) {
  pkt_hdr_t *header = (pkt_hdr_t *)packet;
  pkt_hdr_init(header, packet_num, mission_time);
  return packet + sizeof(pkt_hdr_t);
}

/**
 * Return the location of the block's body
 * 
 * @param block The block to get the body of
 * @return The first byte after the block's header
 */
uint8_t *block_body(uint8_t *block) {
  return block + sizeof(blk_hdr_t);
}

/**
 * Creates a block in a packet if it is possible to do so
 * 
 * @param packet The packet to add a block to
 * @param block Where to add the block, if one were able to be added
 * @param type The type of block to add
 * @param mission_time If the block type has an offset, set the offset using this time
 * @returns The location to add the next block, or NULL if the block can not be added
 */
uint8_t *pkt_create_blk(uint8_t *packet, uint8_t *block, enum block_type_e type, uint32_t mission_time) {
  pkt_hdr_t *header = (pkt_hdr_t *)packet;
  size_t packet_size = block - packet;
  size_t block_size = sizeof(blk_hdr_t) + blk_body_len(type);

  if (packet_size < sizeof(pkt_hdr_t)) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Packet is too small to contain a header\n");
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    return NULL;
  }
  if ((packet_size + block_size) > PACKET_MAX_SIZE) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
    fprintf(stderr, "Packet is too large to contain another block, packet size is %d, block size is %d\n", packet_size, block_size);
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
    return NULL;
  }
  if (has_offset(type)) {
    if (calc_offset(mission_time, header->timestamp, &((offset_blk *)block_body(block))->time_offset)) {
#if defined(CONFIG_INSPACE_TELEMETRY_DEBUG)
      fprintf(stderr, "Could not fit time into packet, time was %d and packet header had %d\n", mission_time, expand_abs_timestamp(header->timestamp));
#endif /* defined(CONFIG_INSPACE_TELEMETRY_DEBUG) */
      return NULL;
    }
  }
  blk_hdr_init((blk_hdr_t *)block, type);
  return block + block_size;
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
 * Construct a pressure block
 * @param b The pressure block to initialize
 * @param pressure The pressure measured in Pascals
 */
void pres_blk_init(struct pres_blk_t *b, const int32_t pressure) {
  b->pressure = pressure;
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
