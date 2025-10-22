#ifndef _INSPACE_TELEMETRY_PACKETS_H_
#define _INSPACE_TELEMETRY_PACKETS_H_

#include <stdint.h>
#include <stdlib.h>

/* The CU InSpace packet specification version implemented here */

/* The maximum size a packet can be in bytes. */

#define PACKET_MAX_SIZE 255

/* The maximum size a block can be in bytes. */

#define BLOCK_MAX_SIZE 128

/* Makes a struct/unions members be aligned as tightly as possible. */

#define TIGHTLY_PACKED __attribute__((packed, aligned(1)))

/* Possible sub-types of data blocks that can be sent. */
enum block_type_e {
    DATA_ALT_SEA = 0x0,     /* Altitude above sea level */
    DATA_ALT_LAUNCH = 0x1,  /* Altitude above launch level */
    DATA_TEMP = 0x2,        /* Temperature data */
    DATA_PRESSURE = 0x3,    /* Pressure data */
    DATA_ACCEL_REL = 0x4,   /* Relative linear acceleration data */
    DATA_ANGULAR_VEL = 0x5, /* Angular velocity data */
    DATA_HUMIDITY = 0x6,    /* Humidity data */
    DATA_LAT_LONG = 0x7,    /* Latitude and longitude coordinates */
    DATA_VOLTAGE = 0x8,     /* Voltage in millivolts with a unique ID. */
    DATA_MAGNETIC = 0x9,    /* Magnetic field data */
    DATA_STATUS = 0xA,      /* Status information */
    DATA_ERROR = 0xB,       /* Error information */
    DATA_RES_ABOVE = 0xC,   /* Types unused above this value */
};

/* Each radio packet will have a header in this format. */
typedef struct {
    /* The HAM radio call sign with trailing null characters. */
    char call_sign[9];
    /* The measurement time that blocks in this packet are offset from in
     * half-minutes
     */
    uint16_t timestamp;
    /* The number of blocks in this packet*/
    uint8_t blocks;
    /* Which number this packet is in the stream of sent packets. */
    uint8_t packet_num;
} TIGHTLY_PACKED pkt_hdr_t;

void pkt_hdr_init(pkt_hdr_t *p, uint8_t packet_number, uint32_t mission_time);

/* Each block in the radio packet will have a header in this format. */
typedef struct {
    /* The type of this block. */
    uint8_t type;
    uint8_t count;
} TIGHTLY_PACKED blk_hdr_t;

void blk_hdr_init(blk_hdr_t *b, const enum block_type_e type, const uint8_t count);

size_t blk_body_len(enum block_type_e type);

uint8_t *block_body(uint8_t *block);
uint8_t *pkt_init(uint8_t *packet, uint8_t packet_num, uint32_t mission_time);
uint8_t *pkt_create_blk(uint8_t *packet, uint8_t *block, enum block_type_e type, uint32_t mission_time);

/* A data block containing information about altitude. */
struct alt_blk_t {
    /* The offset from the absolute time in the header in milliseconds */
    int16_t time_offset;
    /* Altitude in units of millimetres above/below the launch height. */
    int32_t altitude;
} TIGHTLY_PACKED;

void alt_blk_init(struct alt_blk_t *b, const int32_t altitude);

/* A data block containing information about temperature. */
struct temp_blk_t {
    /* The offset from the absolute time in the header in milliseconds */
    int16_t time_offset;
    /* Temperature in millidegrees Celsius. */
    int32_t temperature;
} TIGHTLY_PACKED;

void temp_blk_init(struct temp_blk_t *b, const int32_t temperature);

/* A data block containing information about humidity. */
struct hum_blk_t {
    /* The offset from the absolute time in the header in milliseconds */
    int16_t time_offset;
    /* Relative humidity in ten thousandths of a percent. */
    uint32_t humidity;
} TIGHTLY_PACKED;

void hum_blk_init(struct hum_blk_t *b, const uint32_t humidity);

/* A data block containing information about pressure. */
struct pres_blk_t {
    /* The offset from the absolute time in the header in milliseconds */
    int16_t time_offset;
    /* Pressure measured in Pascals. */
    uint32_t pressure;
} TIGHTLY_PACKED;

void pres_blk_init(struct pres_blk_t *b, const int32_t pressure);

/* A data block containing information about angular velocity. */
struct ang_vel_blk_t {
    /* The offset from the absolute time in the header in milliseconds */
    int16_t time_offset;
    /* Angular velocity in the x-axis measured in tenths of degrees per second.
     */
    int16_t x;
    /* Angular velocity in the y-axis measured in tenths of degrees per second.
     */
    int16_t y;
    /* Angular velocity in the z-axis measured in tenths of degrees per second.
     */
    int16_t z;
} TIGHTLY_PACKED;

void ang_vel_blk_init(struct ang_vel_blk_t *b, const int16_t x_axis, const int16_t y_axis, const int16_t z_axis);

/* A data block containing information about acceleration. */
struct accel_blk_t {
    /* The offset from the absolute time in the header in milliseconds */
    int16_t time_offset;
    /* Linear acceleration in the x-axis measured in centimetres per second
     * squared. */
    int16_t x;
    /* Linear acceleration in the y-axis measured in centimetres per second
     * squared. */
    int16_t y;
    /* Linear acceleration in the z-axis measured in centimetres per second
     * squared. */
    int16_t z;
} TIGHTLY_PACKED;

void accel_blk_init(struct accel_blk_t *b, const int16_t x_axis, const int16_t y_axis, const int16_t z_axis);

/* A data block containing information about acceleration. */
struct mag_blk_t {
    /* The offset from the absolute time in the header in 0.1 microtesla */
    int16_t time_offset;
    /* Magnetic field in the x-axis measured in 0.1 microtesla */
    int16_t x;
    /* Magnetic field in the y-axis measured in 0.1 microtesla */
    int16_t y;
    /* Magnetic field in the z-axis measured in 0.1 microtesla */
    int16_t z;
} TIGHTLY_PACKED;

void mag_blk_init(struct mag_blk_t *b, const int16_t x_axis, const int16_t y_axis, const int16_t z_axis);

/* A data block containing latitude and longitude coordinates. */
struct coord_blk_t {
    /* The offset from the absolute time in the header in milliseconds */
    int16_t time_offset;
    /* Latitude in 0.1 microdegrees/LSB. */
    int32_t latitude;
    /* Longitude in 0.1 microdegrees/LSB. */
    int32_t longitude;
} TIGHTLY_PACKED;

void coord_blk_init(struct coord_blk_t *b, const int32_t lat, const int32_t lon);

/* A data block containing voltage measurements and an ID for the sensor with
 * the associated voltage. */
struct volt_blk_t {
    /* The offset from the absolute time in the header in milliseconds */
    int16_t time_offset;
    /* Voltage in millivolts. */
    int16_t voltage;
    /* Unique sensor ID. */
    uint8_t id;
} TIGHTLY_PACKED;

void volt_blk_init(struct volt_blk_t *b, const uint8_t id, const int16_t voltage);

/* A data block containing information about the rocket's current status */
struct status_blk_t {
    /* The offset from the absolute time in the header in milliseconds */
    int16_t time_offset;
    /* A status code, one of the values in status_blk_code_e */
    uint8_t status_code;
} TIGHTLY_PACKED;

void status_blk_init(struct status_blk_t *b, const uint8_t status_code);

/* A data block containing information about an error that occured */
struct error_blk_t {
    /* The offset from the absolute time in the header in milliseconds */
    int16_t time_offset;
    /* The originating process, must be a value less than 32, as top 3 bits are reserved */
    uint8_t originating_process;
    /* An error code, one of the values in error_blk_code_e */
    uint8_t error_code;
} TIGHTLY_PACKED;

void error_blk_init(struct error_blk_t *b, const uint8_t proc_id, const uint8_t error_code);

#endif // _INSPACE_TELEMETRY_PACKET_H_
