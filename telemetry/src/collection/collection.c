#include <math.h>
#include <pthread.h>
#include <sys/ioctl.h>

#include <nuttx/sensors/sensor.h>

#include "../fusion/fusion.h"
#include "../rocket-state/rocket-state.h"
#include "../sensors/sensors.h"
#include "../syslogging.h"
#include "uORB/uORB.h"
#include "collection.h"

/* Cast an error to a void pointer */

#define err_to_ptr(err) ((void *)((err)))

typedef struct {
    packet_buffer_t *buffer;
    packet_node_t *current;
    int block_count[DATA_RES_ABOVE];
} collection_info_t;

typedef struct {
    collection_info_t logging;
    collection_info_t transmit;
} processing_context_t;

static int setup_collection(collection_info_t *collection, packet_buffer_t *packet_buffer);
static void reset_block_count(collection_info_t *collection);
static uint8_t *add_block(collection_info_t *collection, enum block_type_e type, uint32_t mission_time);
static uint8_t *add_or_new(collection_info_t *collection, enum block_type_e type, uint32_t mission_time);
static void baro_handler(void *ctx, uint8_t *data);
static void accel_handler(void *ctx, uint8_t *data);
static void mag_handler(void *ctx, uint8_t *data);
static void gnss_handler(void *ctx, uint8_t *data);
static void gyro_handler(void *ctx, uint8_t *data);
static void alt_handler(void *ctx, uint8_t *data);

/* Convert microseconds to milliseconds */

#define us_to_ms(us) (us / 1000)

/* Convert millibar to pascals */

#define pascals(millibar) (millibar * 100)

/* Convert meters to millimeters */

#define millimeters(meters) (meters * 1000)

/* Convert a degree to a tenth of a microdegree */

#define point_one_microdegrees(degrees) (1E7f * degrees)

/* Convert a radian to a tenth of a degree */

#define tenth_degree(radian) (radian * 18 / M_PI)

/* Convert gauss to milligauss */

#define tenth_microtesla(microtesla) (microtesla * 1000)

/* Convert meters per second squared to centimeters per second squared */

#define cm_per_sec_squared(meters_per_sec_squared) (meters_per_sec_squared * 100)

/* Convert a degrees celsius to millidegrees */

#define millidegrees(celsius) (celsius * 1000)

/* How many readings of each type of lower-priority data to add to each packet */

#define TRANSMIT_NUM_LOW_PRIORITY_READINGS 2

/* Describes information needed to use a sensor in the collection thread */
struct sensor_desc {
    struct pollfd *fd;                /* Pointer to the pollfd in struct uorb_inputs of this sensor */
    const struct orb_metadata *meta;  /* The metadata of this sensors uORB topic */
    uorb_data_callback_t handler;     /* Handler for this sensors data */
    void *data_buffer;                /* The buffer to read data from this sensor into */
    size_t buffer_size;               /* The size of data_buffer in bytes */
    size_t elem_size;                 /* The size of each element the handler takes in bytes */
};

/* A structure that can be polled to wait for new sensor data */
static struct uorb_inputs sensors;

/* How many measurements to read from sensors at a time (match to size of internal buffers) */
#define ACCEL_READ_SIZE 5
#define BARO_READ_SIZE 5
#define MAG_READ_SIZE 5
#define GNSS_READ_SIZE 5
#define GYRO_READ_SIZE 5
#define ALT_READ_SIZE 5

/* Space to read sensor data into */
static struct sensor_accel accel_buffer[ACCEL_READ_SIZE];
static struct sensor_baro baro_buffer[BARO_READ_SIZE];
static struct sensor_mag mag_buffer[MAG_READ_SIZE];
static struct sensor_gyro gyro_buffer[GYRO_READ_SIZE];
static struct sensor_gnss gnss_buffer[GNSS_READ_SIZE];
static struct fusion_altitude alt_buffer[ALT_READ_SIZE];

/* Information about sensors to help with setting them up or reading from them */
static struct sensor_desc sensor_descs[NUM_SENSORS] = {{.fd = &sensors.accel,
                                                        .handler = accel_handler,
                                                        .data_buffer = accel_buffer,
                                                        .buffer_size = sizeof(accel_buffer),
                                                        .elem_size = sizeof(struct sensor_accel),
                                                        .meta = ORB_ID(sensor_accel)},
                                                       {.fd = &sensors.baro,
                                                        .handler = baro_handler,
                                                        .data_buffer = baro_buffer,
                                                        .buffer_size = sizeof(baro_buffer),
                                                        .elem_size = sizeof(struct sensor_baro),
                                                        .meta = ORB_ID(sensor_baro)},
                                                       {.fd = &sensors.mag,
                                                        .handler = mag_handler,
                                                        .data_buffer = mag_buffer,
                                                        .buffer_size = sizeof(mag_buffer),
                                                        .elem_size = sizeof(struct sensor_mag),
                                                        .meta = ORB_ID(sensor_mag)},
                                                       {.fd = &sensors.gyro,
                                                        .handler = gyro_handler,
                                                        .data_buffer = gyro_buffer,
                                                        .buffer_size = sizeof(gyro_buffer),
                                                        .elem_size = sizeof(struct sensor_gyro),
                                                        .meta = ORB_ID(sensor_gyro)},
                                                       {.fd = &sensors.gnss,
                                                        .handler = gnss_handler,
                                                        .data_buffer = gnss_buffer,
                                                        .buffer_size = sizeof(gnss_buffer),
                                                        .elem_size = sizeof(struct sensor_gnss),
                                                        .meta = ORB_ID(sensor_gnss)},
                                                       {.fd = &sensors.alt,
                                                        .handler = alt_handler,
                                                        .data_buffer = alt_buffer,
                                                        .buffer_size = sizeof(alt_buffer),
                                                        .elem_size = sizeof(struct fusion_altitude),
                                                        .meta = ORB_ID(fusion_altitude)}};

/*
 * Collection thread which runs to collect data.
 */
void *collection_main(void *arg) {
    int err;
    enum flight_state_e flight_state;
    struct collection_args *unpacked_args = (struct collection_args *)(arg);
    rocket_state_t *state = unpacked_args->state;

    static processing_context_t callback_context;
    err = setup_collection(&callback_context.logging, unpacked_args->logging_buffer);
    if (err < 0) {
        inerr("Could not get an initial empty packet for collection\n");
        pthread_exit(err_to_ptr(err));
    }
    err = setup_collection(&callback_context.transmit, unpacked_args->transmit_buffer);
    if (err < 0) {
        inerr("Could not get an initial empty packet for collection\n");
        pthread_exit(err_to_ptr(err));
    }

    clear_uorb_inputs(&sensors);
    for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
        setup_sensor(sensor_descs[sensor].fd, sensor_descs[sensor].meta);
    }

    /* Separate buffers use more memory but allow us to process in pieces while still reading only once */

    ininfo("Collection thread started.\n");

    /* Measure data forever */

    for (;;) {

        /* Get the current flight state */

        err = state_get_flightstate(state, &flight_state); // TODO: error handling
        if (err) {
            inerr("Could not get flight state: %d\n", err);
        }

        /* Wait for new data */

        poll_sensors(&sensors);

        // Note the current read position and the final read position
        static void *read_pos[NUM_SENSORS];
        static void *data_end[NUM_SENSORS];

        /* Read data all at once to avoid as much locking as possible */
        for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
            data_end[sensor] = get_sensor_data_end(sensor_descs[sensor].fd, sensor_descs[sensor].data_buffer,
                                                   sensor_descs[sensor].buffer_size);
            read_pos[sensor] = sensor_descs[sensor].data_buffer;
        }

        /* Process one piece of data of each type at a time to get a more even mix of things in the packets */
        int processed;
        do {
            processed = 0;
            for (int sensor = 0; sensor < NUM_SENSORS; sensor++) {
                process_one(sensor_descs[sensor].handler, &callback_context, &read_pos[sensor], data_end[sensor],
                            sensor_descs[sensor].elem_size);
            }
        } while (processed);

        /* Decide whether to move to lift-off state. TODO: real logic */

        switch (flight_state) {
        case STATE_IDLE: {
            // Perform lift-off detection operations
            // If lift-off:
            // state_set_flightstate(state, STATE_AIRBORNE);
        } break;
        case STATE_AIRBORNE: {
            // Perform landing detection operations
            // If landed
            // state_set_flightstate(state, STATE_LANDED);
        } break;
        case STATE_LANDED:
            break; /* Do nothing, just continue collecting */
        }
    }

    pthread_exit(0);
}

/**
 * Sets up a collection_info_t struct with a packet buffer and gets an initially empty node to work on
 *
 * @param collection The collection_info_t struct to set up
 * @param packet_buffer The buffer to get packets from a put into
 * @return 0 on success, or a negative error code on failure
 */
static int setup_collection(collection_info_t *collection, packet_buffer_t *packet_buffer) {
    collection->buffer = packet_buffer;
    collection->current = packet_buffer_get_empty(packet_buffer);
    reset_block_count(collection);
    if (!collection->current) {
        return -1;
    }
    return 0;
}

/**
 * Resets the current block count for the collected data
 *
 * @param collection Information about collected data
 */
static void reset_block_count(collection_info_t *collection) {
    for (int i = 0; i < sizeof(collection->block_count) / sizeof(collection->block_count[0]); i++) {
        collection->block_count[i] = 0;
    }
}

/**
 * Adds a block to the current packet being worked on
 *
 * @param collection Collection information containing a packet to add the block to
 * @param type The type of block to add
 * @param mission_time The time of measurement for the block, if it has one
 * @return The location to write the block, or NULL if the block could not be added
 */
static uint8_t *add_block(collection_info_t *collection, enum block_type_e type, uint32_t mission_time) {
    uint8_t *block = pkt_create_blk(collection->current->packet, collection->current->end, type, mission_time);
    if (block) {
        collection->block_count[type]++;
    }
    return block;
}

/**
 * Allocates a block in the current packet or swaps out the current packet if the block can't be added
 *
 * @param collection Collection information to add the block to
 * @param type The type of block being allocated
 * @param mission_time The time of the measurement, if this block type has one
 * @return The location to write the requested type of block
 */
static uint8_t *add_or_new(collection_info_t *collection, enum block_type_e type, uint32_t mission_time) {
    // The last byte of the packet will be where the block is allocated, but we need to know where it will end to update
    // (*node)->end
    uint8_t *next_block = add_block(collection, type, mission_time);
    // Can't add to this packet, it's full or we can just assume its done being asssembled
    if (next_block == NULL) {
        indebug("Completed a packet length %ld\n", collection->current->end - collection->current->packet);
        packet_buffer_put_full(collection->buffer, collection->current);
        collection->current = packet_buffer_get_empty(collection->buffer);
        reset_block_count(collection);
        if (collection->current == NULL) {
            inerr("Couldn't get an empty packet or overwrite a full one - not enough packets in buffer\n");
            return NULL;
        }

        // Leave seq num up to the logger/transmitter (don't know if or in what order packets get transmitted)
        collection->current->end = init_pkt(collection->current->packet, 0, mission_time);
        next_block = pkt_create_blk(collection->current->packet, collection->current->end, type, mission_time);
        if (next_block == NULL) {
            inerr("Couldn't add a block to a new packet\n");
            return NULL;
        }
    }
    uint8_t *write_to = collection->current->end;
    collection->current->end = next_block;
    return write_to;
}

/**
 * Add a pressure block to the packet being assembled
 *
 * @param collection Collection information where the block should be added
 * @param node The packet currently being assembled
 * @param baro_data The baro data to add
 */
static void add_pres_blk(collection_info_t *collection, struct sensor_baro *baro_data) {
    uint8_t *block = add_or_new(collection, DATA_PRESSURE, us_to_ms(baro_data->timestamp));
    if (block) {
        pres_blk_init((struct pres_blk_t *)block_body(block), pascals(baro_data->pressure));
    }
}

/**
 * Add a temperature block to the packet being assembled
 *
 * @param collection Collection information where the block should be added
 * @param node The packet currently being assembled
 * @param baro_data The baro data to add
 */
static void add_temp_blk(collection_info_t *collection, struct sensor_baro *baro_data) {
    uint8_t *block = add_or_new(collection, DATA_TEMP, us_to_ms(baro_data->timestamp));
    if (block) {
        temp_blk_init((struct temp_blk_t *)block_body(block), millidegrees(baro_data->temperature));
    }
}

/**
 * A uorb_data_callback_t function - adds barometric data to the required packets
 *
 * @param ctx Context information, type processing_context_t
 * @param data Barometric data to add, type struct sensor_baro
 */
static void baro_handler(void *ctx, uint8_t *data) {
    struct sensor_baro *baro_data = (struct sensor_baro *)data;
    processing_context_t *context = (processing_context_t *)ctx;
    add_pres_blk(&context->logging, baro_data);
    add_temp_blk(&context->logging, baro_data);

    if (context->transmit.block_count[DATA_PRESSURE] < TRANSMIT_NUM_LOW_PRIORITY_READINGS) {
        add_pres_blk(&context->transmit, baro_data);
    }
    if (context->transmit.block_count[DATA_TEMP] < TRANSMIT_NUM_LOW_PRIORITY_READINGS) {
        add_temp_blk(&context->transmit, baro_data);
    }
}

/**
 * Add an acceleration block to the packet being assembled
 *
 * @param collection Collection information where the block should be added
 * @param node The packet currently being assembled
 * @param accel_data The accel data to add
 */
static void add_accel_blk(collection_info_t *collection, struct sensor_accel *accel_data) {
    uint8_t *block = add_or_new(collection, DATA_ACCEL_REL, us_to_ms(accel_data->timestamp));
    if (block) {
        accel_blk_init((struct accel_blk_t *)block_body(block), cm_per_sec_squared(accel_data->x),
                       cm_per_sec_squared(accel_data->y), cm_per_sec_squared(accel_data->z));
    }
}

/**
 * A uorb_data_callback_t function - adds acceleration data to the required packets
 *
 * @param ctx Context information, type processing_context_t
 * @param data Acceleration data to add, type struct sensor_accel
 */
static void accel_handler(void *ctx, uint8_t *data) {
    struct sensor_accel *accel_data = (struct sensor_accel *)data;
    processing_context_t *context = (processing_context_t *)ctx;
    add_accel_blk(&context->logging, accel_data);
    add_accel_blk(&context->transmit, accel_data);
}

/**
 * Add a magnetometer block to the packet being assembled
 *
 * @param collection Collection information where the block should be added
 * @param node The packet currently being assembled
 * @param mag_data The magnetic field data to add
 */
static void add_mag_blk(collection_info_t *collection, struct sensor_mag *mag_data) {
    uint8_t *block = add_or_new(collection, DATA_MAGNETIC, us_to_ms(mag_data->timestamp));
    if (block) {
        mag_blk_init((struct mag_blk_t *)block_body(block), tenth_microtesla(mag_data->x),
                     tenth_microtesla(mag_data->y), tenth_microtesla(mag_data->z));
    }
}

/**
 * A uorb_data_callback_t function - adds magnetometer data to the required packets
 *
 * @param ctx Context information, type processing_context_t
 * @param data magnetometer data to add, type struct sensor_mag
 */
static void mag_handler(void *ctx, uint8_t *data) {
    struct sensor_mag *mag_data = (struct sensor_mag *)data;
    processing_context_t *context = (processing_context_t *)ctx;
    add_mag_blk(&context->logging, mag_data);
    add_mag_blk(&context->transmit, mag_data);
}

/**
 * Add an gyro block to the packet being assembled
 *
 * @param collection Collection information where the block should be added
 * @param node The packet currently being assembled
 * @param gyro_data The gyro data to add
 */
static void add_gyro_blk(collection_info_t *collection, struct sensor_gyro *gyro_data) {
    uint8_t *block = add_or_new(collection, DATA_ANGULAR_VEL, us_to_ms(gyro_data->timestamp));
    if (block) {
        ang_vel_blk_init((struct ang_vel_blk_t *)block_body(block), tenth_degree(gyro_data->x),
                         tenth_degree(gyro_data->y), tenth_degree(gyro_data->z));
    }
}

/**
 * A uorb_data_callback_t function - adds gyro data to the required packets
 *
 * @param ctx Context information, type processing_context_t
 * @param data Acceleration data to add, type struct sensor_gyro
 */
static void gyro_handler(void *ctx, uint8_t *data) {
    struct sensor_gyro *gyro_data = (struct sensor_gyro *)data;
    processing_context_t *context = (processing_context_t *)ctx;
    add_gyro_blk(&context->logging, gyro_data);
    add_gyro_blk(&context->transmit, gyro_data);
}

/**
 * Add an gnss block to the packet being assembled
 *
 * @param collection Collection information where the block should be added
 * @param node The packet currently being assembled
 * @param gnss_data The gnss data to add
 */
static void add_gnss_block(collection_info_t *collection, struct sensor_gnss *gnss_data) {
    uint8_t *block = add_or_new(collection, DATA_LAT_LONG, us_to_ms(gnss_data->timestamp));
    if (block) {
        coord_blk_init((struct coord_blk_t *)block_body(block), point_one_microdegrees(gnss_data->latitude),
                       point_one_microdegrees(gnss_data->longitude));
    }
}

/**
 * Add a gnss mean sea level altitude block to the packet being assembled
 *
 * @param collection Collection information where the block should be added
 * @param node The packet currently being assembled
 * @param alt_data The altitude data to add
 */
static void add_gnss_msl_block(collection_info_t *collection, struct sensor_gnss *alt_data) {
    uint8_t *block = add_or_new(collection, DATA_ALT_SEA, us_to_ms(alt_data->timestamp));
    if (block) {
        alt_blk_init((struct alt_blk_t *)block_body(block), millimeters(alt_data->altitude));
    }
}

/**
 * Add a mean sea level altitude block to the packet being assembled
 *
 * @param collection Collection information where the block should be added
 * @param node The packet currently being assembled
 * @param alt_data The altitude data to add
 */
static void add_msl_block(collection_info_t *collection, struct fusion_altitude *alt_data) {
    uint8_t *block = add_or_new(collection, DATA_ALT_SEA, us_to_ms(alt_data->timestamp));
    if (block) {
        alt_blk_init((struct alt_blk_t *)block_body(block), millimeters(alt_data->altitude));
    }
}

/**
 * A uorb_data_callback_t function - adds gnss data to the required packets
 *
 * @param ctx Context information, type processing_context_t
 * @param data GNSS data to add, type struct sensor_gnss
 */
static void gnss_handler(void *ctx, uint8_t *data) {
    struct sensor_gnss *gnss_data = (struct sensor_gnss *)data;
    processing_context_t *context = (processing_context_t *)ctx;
    if (gnss_data->latitude == (int)NULL && gnss_data->longitude == (int)NULL)
        return; // Don't send packets with no sat fix

    add_gnss_block(&context->logging, gnss_data);
    add_gnss_msl_block(&context->logging, gnss_data);

    add_gnss_block(&context->transmit, gnss_data);
    add_gnss_msl_block(&context->transmit, gnss_data);
}

/**
 * A uorb_data_callback_t function - adds altitude data to the required packets
 *
 * @param ctx Context information, type processing_context_t
 * @param data Altitude data to add, type struct fusion_altitude
 */
static void alt_handler(void *ctx, uint8_t *data) {
    struct fusion_altitude *alt_data = (struct fusion_altitude *)data;
    processing_context_t *context = (processing_context_t *)ctx;
    add_msl_block(&context->logging, alt_data);
    add_msl_block(&context->transmit, alt_data);
}
