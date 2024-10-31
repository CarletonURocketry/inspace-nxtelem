#ifndef _INSPACE_TELEMETRY_PACKETS_H_
#define _INSPACE_TELEMETRY_PACKETS_H_

#include <stdint.h>

/* The CU InSpace packet specification version implemented here */

#define PACKET_SPEC_VERSION 1

/* The maximum size a packet can be in bytes. */

#define PACKET_MAX_SIZE 256

/* The maximum size a block can be in bytes. */

#define BLOCK_MAX_SIZE 128

/* Makes a struct/unions members be aligned as tightly as possible. */

#define TIGHTLY_PACKED __attribute__((packed, aligned(1)))

/* Possible devices from which a packet could originate or be sent to. */
enum dev_addr_e {
  ADDR_GROUNDSTATION = 0x0, /* Ground station */
  ADDR_ROCKET = 0x1,        /* The rocket */
  ADDR_MULTICAST = 0xFF,    /* Any device which is listening */
};

/* Possible types of radio packet blocks that could be sent. */
enum block_type_e {
  TYPE_DATA = 0x0, /* Data block */
};

/* Possible sub-types of data blocks that can be sent. */
enum data_block_type_e {
  DATA_DBG_MSG = 0x0,    /* Debug message */
  DATA_ALT_SEA = 0x1,    /* Altitude above sea level */
  DATA_ALT_LAUNCH = 0x2, /* Altitude above launch level */
  DATA_TEMP = 0x3,       /* Temperature data */
  DATA_PRESSURE = 0x4,   /* Pressure data */
  DATA_ACCEL_REL = 0x5,  /* Relative linear acceleration data */
  DATA_ACCEL_ABS =
      0x6, /* Absolute linear acceleration data (relative to ground) */
  DATA_ANGULAR_VEL = 0x7, /* Angular velocity data */
  DATA_HUMIDITY = 0x8,    /* Humidity data */
  DATA_LAT_LONG = 0x9,    /* Latitude and longitude coordinates */
  DATA_VOLTAGE = 0xA,     /* Voltage in millivolts with a unique ID. */
};

/* Any block sub-type from DataBlockType, CtrlBlockType or CmdBlockType. */
typedef uint8_t block_subtype_t;

/* Each radio packet will have a header in this format. */
typedef struct {
  /* The HAM radio call sign with trailing null characters. */
  char call_sign[9];
  /* The packet length in multiples of 4 bytes. */
  uint8_t len;
  /* The version of InSpace radio packet encoding being used. */
  uint8_t version;
  /* The source address of the packet. */
  uint8_t src_addr;
  /* Which number this packet is in the stream of sent packets. */
  uint32_t packet_num;
} pkt_hdr_t;

void pkt_hdr_init(pkt_hdr_t *p, const uint32_t packet_number);

/* Each block in the radio packet will have a header in this format. */
typedef struct {
  /* The block header accessed as a bytes array. */
  uint8_t len;
  /* The type of this block. */
  uint8_t type;
  /* The sub type of this block. */
  uint8_t subtype;
  /* The address of this blocks destination device. */
  uint8_t dest_addr;
} blk_hdr_t;

void blk_hdr_init(blk_hdr_t *b, const enum block_type_e type,
                  const block_subtype_t subtype);

/* A data block containing information about altitude. */
struct alt_blk_t {
  /* Mission time in milliseconds since launch. */
  uint32_t mission_time;
  /* Altitude in units of millimetres above/below the launch height. */
  int32_t altitude;
};

void alt_blk_init(struct alt_blk_t *b, const uint32_t mission_time,
                  const int32_t altitude);

/* A data block containing information about temperature. */
struct temp_blk_t {
  /* Mission time in milliseconds since launch. */
  uint32_t mission_time;
  /* Temperature in millidegrees Celsius. */
  int32_t temperature;
};

void temp_blk_init(struct temp_blk_t *b, const uint32_t mission_time,
                   const int32_t temperature);

/* A data block containing information about humidity. */
struct hum_blk_t {
  /* Mission time in milliseconds since launch. */
  uint32_t mission_time;
  /* Relative humidity in ten thousandths of a percent. */
  uint32_t humidity;
};

void hum_blk_init(struct hum_blk_t *b, const uint32_t mission_time,
                  const uint32_t humidity);

/* A data block containing information about pressure. */
struct pres_blk_t {
  /* Mission time in milliseconds since launch. */
  uint32_t mission_time;
  /* Pressure measured in Pascals. */
  uint32_t pressure;
};

void pres_blk_init(struct pres_blk_t *b, const uint32_t mission_time,
                   const int32_t pressure);

/* A data block containing information about angular velocity. */
struct ang_vel_blk_t {
  /* Mission time in milliseconds since launch. */
  uint32_t mission_time;
  /* Angular velocity in the x-axis measured in tenths of degrees per second.
   */
  int16_t x;
  /* Angular velocity in the y-axis measured in tenths of degrees per second.
   */
  int16_t y;
  /* Angular velocity in the z-axis measured in tenths of degrees per second.
   */
  int16_t z;
  /* 0 padding to fill the 4 byte multiple requirement of the packet spec. */
  int16_t _padding;
};

void ang_vel_blk_init(struct ang_vel_blk_t *b, const uint32_t mission_time,
                      const int16_t x_axis, const int16_t y_axis,
                      const int16_t z_axis);

/* A data block containing information about acceleration. */
struct accel_blk_t {
  /* Mission time in milliseconds since launch. */
  uint32_t mission_time;
  /* Linear acceleration in the x-axis measured in centimetres per second
   * squared. */
  int16_t x;
  /* Linear acceleration in the y-axis measured in centimetres per second
   * squared. */
  int16_t y;
  /* Linear acceleration in the z-axis measured in centimetres per second
   * squared. */
  int16_t z;
  /* 0 padding to fill the 4 byte multiple requirement of the packet spec. */
  int16_t _padding;
};

void accel_blk_init(struct accel_blk_t *b, const uint32_t mission_time,
                    const int16_t x_axis, const int16_t y_axis,
                    const int16_t z_axis);

/* A data block containing latitude and longitude coordinates. */
struct coord_blk_t {
  /* Mission time in milliseconds since launch. */
  uint32_t mission_time;
  /* Latitude in 0.1 microdegrees/LSB. */
  int32_t latitude;
  /* Longitude in 0.1 microdegrees/LSB. */
  int32_t longitude;
};

void coord_blk_init(struct coord_blk_t *b, const uint32_t mission_time,
                    const int32_t lat, const int32_t lon);

/* A data block containing voltage measurements and an ID for the sensor with
 * the associated voltage. */
struct volt_blk_t {
  /* Mission time in milliseconds since launch. */
  uint32_t mission_time;
  /* Unique sensor ID. */
  uint16_t id;
  /* Voltage in millivolts. */
  int16_t voltage;
};

void volt_blk_init(struct volt_blk_t *b, const uint32_t mission_time,
                   const uint16_t id, const int16_t voltage);

/* Sets the length of the packet the header is associated with.
 * @param p The packet header to store the length in.
 * @param length The length of the packet in bytes, not including the packet
 * header itself.
 */
static inline void pkt_hdr_set_len(pkt_hdr_t *p, const uint16_t length) {
  p->len = ((length + sizeof(pkt_hdr_t)) / 4) - 1;
}

/* Gets the length of the packet header.
 * @param p The packet header to read the length from.
 * @return The length of the packet header in bytes, including itself.
 */
static inline uint16_t pkt_hdr_get_len(const pkt_hdr_t *p) {
  return (p->len + 1) * 4;
}

/* Increments the length of the packet header.
 * @param p The packet header change the length of.
 * @param l The additional length (in bytes) to add to the packet length.
 */
static inline void pkt_hdr_inc_len(pkt_hdr_t *p, const uint16_t l) {
  pkt_hdr_set_len(p, pkt_hdr_get_len(p) + l - sizeof(pkt_hdr_t));
}

/* Sets the length of the block the block header is associated with.
 * @param b The block header to store the length in.
 * @param length The length of the block in bytes, not including the header
 * itself. Must be a multiple of 4.
 */
static inline void blk_hdr_set_len(blk_hdr_t *b, const uint16_t length) {
  b->len = ((length + sizeof(blk_hdr_t)) / 4) - 1;
}

/* Gets the length of the block header.
 * @param p The block header to read the length from.
 * @return The length of the block header in bytes, including itself.
 */
static inline uint16_t blk_hdr_get_len(const blk_hdr_t *b) {
  return (b->len + 1) * 4;
}

#endif // _INSPACE_TELEMETRY_PACKET_H_
