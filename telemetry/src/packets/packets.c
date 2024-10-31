#include <string.h>

#include "packets.h"

/* Initialize the packet header.
 * @param p The packet header to initialize.
 * @param packet_number The sequence number of this packet in the stream of
 * transmissions.
 */
void pkt_hdr_init(pkt_hdr_t *p, const uint32_t packet_number) {
  p->version = PACKET_SPEC_VERSION;
  memcpy(&p->call_sign, CONFIG_INSPACE_TELEMETRY_CALLSIGN,
         sizeof(CONFIG_INSPACE_TELEMETRY_CALLSIGN));
  p->src_addr = ADDR_ROCKET;
  p->packet_num = packet_number;
  pkt_hdr_set_len(p, 0);
}

/* Initialize the block header.
 * @param b The block header to initialize.
 * @param type The block type of the block this header will be associated with.
 * @param subtype The sub-type of the block this header will be associated with.
 */
void blk_hdr_init(blk_hdr_t *b, const enum block_type_e type,
                  const block_subtype_t subtype) {
  b->type = type;
  b->subtype = subtype;
  b->dest_addr = ADDR_GROUNDSTATION;

  switch (type) {
  case TYPE_DATA:
    switch ((enum data_block_type_e)subtype) {
    case DATA_TEMP:
      blk_hdr_set_len(b, sizeof(struct temp_blk_t));
      break;
    case DATA_HUMIDITY:
      blk_hdr_set_len(b, sizeof(struct hum_blk_t));
      break;
    case DATA_VOLTAGE:
      blk_hdr_set_len(b, sizeof(struct volt_blk_t));
      break;
    case DATA_LAT_LONG:
      blk_hdr_set_len(b, sizeof(struct coord_blk_t));
      break;
    case DATA_PRESSURE:
      blk_hdr_set_len(b, sizeof(struct pres_blk_t));
      break;
    case DATA_ANGULAR_VEL:
      blk_hdr_set_len(b, sizeof(struct ang_vel_blk_t));
      break;
    case DATA_ACCEL_REL:
      blk_hdr_set_len(b, sizeof(struct accel_blk_t));
      break;
    case DATA_ALT_LAUNCH:
      blk_hdr_set_len(b, sizeof(struct alt_blk_t));
      break;
    }
    break;
  }
}

/*
 * Construct an altitude block.
 * @param b The altitude block to initialize.
 * @param mission_time The mission time at which the altitude was measured in ms
 * since launch.
 * @param altitude The altitude value measured in millimetres above launch
 * height.
 */
void alt_blk_init(struct alt_blk_t *b, const uint32_t mission_time,
                  const int32_t altitude) {
  b->mission_time = mission_time;
  b->altitude = altitude;
}

/*
 * Construct a temperature block.
 * @param b The temperature block to initialize.
 * @param mission_time The mission time at which the temperature was measured in
 * ms since launch.
 * @param temperature The temperature measured in millidegrees Celsius.
 */
void temp_blk_init(struct temp_blk_t *b, const uint32_t mission_time,
                   const int32_t temperature) {
  b->mission_time = mission_time;
  b->temperature = temperature;
}

/* TODO: other block types */
