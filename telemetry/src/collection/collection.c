/* The collection thread is responsible for collecting all the sensor measurements available and packaging them into the
 * correct packet format. Packets are then published to both the logging and transmission sinks so that they can be
 * output properly.
 */

#include <math.h>
#include <pthread.h>
#include <sys/ioctl.h>

#include <nuttx/sensors/sensor.h>

#include "../fusion/fusion.h"
#include "../rocket-state/rocket-state.h"
#include "../sensors/sensors.h"
#include "../syslogging.h"
#include "collection.h"
#include "uORB/uORB.h"

/* Cast an error to a void pointer */

#define err_to_ptr(err) ((void *)((err)))

/* Unit conversion helpers */

#define us_to_ms(us) (us / 1000)
#define pascals(millibar) (millibar * 100)
#define millimeters(meters) (meters * 1000)
#define point_one_microdegrees(degrees) (1E7f * degrees)
#define tenth_degree(radian) (radian * 18 / M_PI)
#define tenth_microtesla(microtesla) (microtesla * 1000)
#define cm_per_sec_squared(meters_per_sec_squared) (meters_per_sec_squared * 100)
#define millidegrees(celsius) (celsius * 1000)

/* How many measurements to read from sensors at a time (match to size of internal buffers) */

#define DATA_BUFFER_SIZE 1

/* How many readings of each type of lower-priority data to add to each packet */

#define TRANSMIT_NUM_LOW_PRIORITY_READINGS 2

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
#ifdef CONFIG_SENSORS_L86XXX
static void gnss_handler(void *ctx, uint8_t *data);
#endif
static void gyro_handler(void *ctx, uint8_t *data);
static void alt_handler(void *ctx, uint8_t *data);

/* uORB polling file descriptors */

static struct pollfd uorb_fds[] = {
#ifdef CONFIG_SENSORS_LSM6DSO32
    [SENSOR_ACCEL] = {.fd = -1, .events = POLLIN, .revents = 0},
    [SENSOR_GYRO] = {.fd = -1, .events = POLLIN, .revents = 0},
#endif
#ifdef CONFIG_SENSORS_MS56XX
    [SENSOR_BARO] = {.fd = -1, .events = POLLIN, .revents = 0},
#endif
#ifdef CONFIG_SENSORS_LIS2MDL
    [SENSOR_MAG] = {.fd = -1, .events = POLLIN, .revents = 0},
#endif
#ifdef CONFIG_SENSORS_L86XXX
    [SENSOR_GNSS] = {.fd = -1, .events = POLLIN, .revents = 0},
#endif
    [SENSOR_ALT] = {.fd = -1, .events = POLLIN, .revents = 0},
};

/* uORB sensor metadatas */

static struct orb_metadata const *uorb_metas[] = {
#ifdef CONFIG_SENSORS_LSM6DSO32
    [SENSOR_ACCEL] = NULL, [SENSOR_GYRO] = NULL,
#endif
#ifdef CONFIG_SENSORS_MS56XX
    [SENSOR_BARO] = NULL,
#endif
#ifdef CONFIG_SENSORS_LIS2MDL
    [SENSOR_MAG] = NULL,
#endif
#ifdef CONFIG_SENSORS_L86XXX
    [SENSOR_GNSS] = NULL,
#endif
    [SENSOR_ALT] = NULL,
};

/* Sensor desired sampling rates in Hz*/

static const uint32_t sample_freqs[] = {
#ifdef CONFIG_SENSORS_LSM6DSO32
    [SENSOR_ACCEL] = CONFIG_INSPACE_TELEMETRY_ACCEL_SF, [SENSOR_GYRO] = CONFIG_INSPACE_TELEMETRY_GYRO_SF,
#endif
#ifdef CONFIG_SENSORS_MS56XX
    [SENSOR_BARO] = CONFIG_INSPACE_TELEMETRY_BARO_SF,
#endif
#ifdef CONFIG_SENSORS_LIS2MDL
    [SENSOR_MAG] = CONFIG_INSPACE_TELEMETRY_MAG_SF,
#endif
#ifdef CONFIG_SENSORS_L86XXX
    [SENSOR_GNSS] = CONFIG_INSPACE_TELEMETRY_GPS_SF,
#endif
    [SENSOR_ALT] = CONFIG_INSPACE_TELEMETRY_ALT_SF,
};

/* Data handlers for different sensors */

static const uorb_data_callback_t uorb_handlers[] = {
#ifdef CONFIG_SENSORS_MS56XX
    [SENSOR_BARO] = baro_handler,
#endif
#ifdef CONFIG_SENSORS_LSM6DSO32
    [SENSOR_GYRO] = gyro_handler, [SENSOR_ACCEL] = accel_handler,
#endif
#ifdef CONFIG_SENSORS_LIS2MDL
    [SENSOR_MAG] = mag_handler,
#endif
#ifdef CONFIG_SENSORS_L86_XXX
    [SENSOR_GPS] = gnss_handler,
#endif
    [SENSOR_ALT] = alt_handler,
};

/* Separate buffers use more memory but allow us to process in pieces while still reading only once */

#ifdef CONFIG_SENSORS_LSM6DSO32
static struct sensor_accel accel_data_buf[DATA_BUFFER_SIZE];
static struct sensor_gyro gyro_data_buf[DATA_BUFFER_SIZE];
#endif
#ifdef CONFIG_SENSORS_MS56XX
static struct sensor_baro baro_data_buf[DATA_BUFFER_SIZE];
#endif
#ifdef CONFIG_SENSORS_LIS2MDL
static struct sensor_mag mag_data_buf[DATA_BUFFER_SIZE];
#endif
#ifdef CONFIG_SENSORS_L86XXX
struct sensor_gnss gnss_data_buf[DATA_BUFFER_SIZE];
#endif
static struct fusion_altitude alt_data_buf[DATA_BUFFER_SIZE];

/* Put the buffers in array so that agnostic code can read into the write buffer. */

static void *uorb_data_buffers[] = {
#ifdef CONFIG_SENSORS_LSM6DSO32
    [SENSOR_ACCEL] = &accel_data_buf, [SENSOR_GYRO] = &gyro_data_buf,
#endif
#ifdef CONFIG_SENSORS_MS56XX
    [SENSOR_BARO] = &baro_data_buf,
#endif
#ifdef CONFIG_SENSORS_LIS2MDL
    [SENSOR_MAG] = &mag_data_buf,
#endif
#ifdef CONFIG_SENSORS_L86XXX
    [SENSOR_GNSS] = &gnss_data_buf,
#endif
    [SENSOR_ALT] = &alt_data_buf,
};

/* The numbers of sensors that are available to be polled */

#define NUM_SENSORS (sizeof(uorb_fds) / sizeof(uorb_fds[0]))

/*
 * Collection thread.
 *
 * Runs to collect data from sensors and battery ADC, packaging all measurements into packets.
 */
void *collection_main(void *arg) {
    int err;
    enum flight_state_e flight_state;
    struct collection_args *unpacked_args = (struct collection_args *)(arg);
    rocket_state_t *state = unpacked_args->state;
    processing_context_t context;

    ininfo("Collection thread started.\n");

    ininfo("Setting up collection context.\n");

    err = setup_collection(&context.logging, unpacked_args->logging_buffer);

    if (err < 0) {
        inerr("Could not get an initial empty packet for collection\n");
        pthread_exit(err_to_ptr(err));
    }

    err = setup_collection(&context.transmit, unpacked_args->transmit_buffer);

    if (err < 0) {
        inerr("Could not get an initial empty packet for collection\n");
        pthread_exit(err_to_ptr(err));
    }

    /* Get metadata of all sensors */

    ininfo("Getting sensor metadatas.\n");

#ifdef CONFIG_SENSORS_MS56XX
    uorb_metas[SENSOR_BARO] = orb_get_meta("sensor_baro");
    if (uorb_metas[SENSOR_BARO] == NULL) {
        inerr("Couldn't get metadata for sensor_baro: %d\n", errno);
    }
#endif
#ifdef CONFIG_SENSORS_LSM6DSO32
    uorb_metas[SENSOR_ACCEL] = orb_get_meta("sensor_accel");
    if (uorb_metas[SENSOR_ACCEL] == NULL) {
        inerr("Couldn't get metadata for sensor_accel: %d\n", errno);
    }
    uorb_metas[SENSOR_GYRO] = orb_get_meta("sensor_gyro");
    if (uorb_metas[SENSOR_GYRO] == NULL) {
        inerr("Couldn't get metadata for sensor_gyro: %d\n", errno);
    }
#endif
#ifdef CONFIG_SENSORS_LIS2MDL
    uorb_metas[SENSOR_MAG] = orb_get_meta("sensor_mag");
    if (uorb_metas[SENSOR_MAG] == NULL) {
        inerr("Couldn't get metadata for sensor_mag: %d\n", errno);
    }
#endif
#ifdef CONFIG_SENSORS_L86_XXX
    uorb_metas[SENSOR_GPS] = orb_get_meta("sensor_gnss");
    if (uorb_metas[SENSOR_GPS] == NULL) {
        inerr("Couldn't get metadata for sensor_gnss: %d\n", errno);
    }
#endif
    uorb_metas[SENSOR_ALT] = orb_get_meta("fusion_altitude");
    if (uorb_metas[SENSOR_ALT] == NULL) {
        inerr("Couldn't get metadata for fusion_altitude: %d\n", errno);
    }

    /* Subscribe to all sensors */

    ininfo("Subscribing to all sensors.\n");

    for (int i = 0; i < NUM_SENSORS; i++) {

        /* Skip metadata that couldn't be found */

        if (uorb_metas[i] == NULL) {
            inwarn("Missing metadata for sensor %d\n", i);
            continue;
        }

        ininfo("Subscribing to '%s'\n", uorb_metas[i]->o_name);
        uorb_fds[i].fd = orb_subscribe(uorb_metas[i]);
        if (uorb_fds[i].fd < 0) {
            inerr("Failed to subscribe to '%s': %d\n", uorb_metas[i]->o_name, errno);
        }
    }

    ininfo("Sensors subscribed.\n");

    /* Set sensor specific requirements */

    ininfo("Configuring sensors with their specific requirements.\n");

#ifdef CONFIG_SENSORS_LSM6DSO32
    ininfo("Configuring accelerometer FSR to +/-32g.\n");
    err = orb_ioctl(uorb_fds[SENSOR_ACCEL].fd, SNIOC_SETFULLSCALE, 32);
    if (err < 0) {
        inerr("Couldn't set FSR of sensor_accel: %d\n", errno);
    }

    ininfo("Configuring gyro FSR to +/-2000dps.\n");
    err = orb_ioctl(uorb_fds[SENSOR_GYRO].fd, SNIOC_SETFULLSCALE, 2000);
    if (err < 0) {
        inerr("Couldn't set FSR of sensor_gyro: %d\n", errno);
    }
#endif
#ifdef CONFIG_SENSORS_LIS2MDL
    /* TODO: maybe low pass filter? */
#endif

    ininfo("Sensors configured with specific settings.\n");

    /* Set sample frequencies for all sensors */

    ininfo("Setting sensor sample frequencies.\n");

    for (int i = 0; i < NUM_SENSORS; i++) {

        /* Skip invalid sensors */

        if (uorb_fds[i].fd < 0) {
            continue;
        }

        ininfo("Setting frequency of '%s' to %luHz\n", uorb_metas[i]->o_name, sample_freqs[i]);
        err = orb_set_frequency(uorb_fds[i].fd, sample_freqs[i]);
        if (err < 0) {
            inerr("Failed to set frequency of '%s' to %luHz: %d\n", uorb_metas[i]->o_name, sample_freqs[i], errno);
        }
    }

    ininfo("Sensor frequencies set.\n");

    /* Measure data forever */

    for (;;) {

        /* Get the current flight state */

        err = state_get_flightstate(state, &flight_state); // TODO: error handling
        if (err) {
            inerr("Could not get flight state: %d\n", err);
        }

        /* Wait for new data */

        poll(uorb_fds, NUM_SENSORS, -1);

        /* Read one thing per data read and process it */

        for (int i = 0; i < NUM_SENSORS; i++) {

            /* Skip invalid sensors and sensors without new data */

            if (uorb_fds[i].fd < 0 || !(uorb_fds[i].revents & POLL_IN)) {
                continue;
            }

            /* If we are here, a valid sensor has some data ready to be read. Add it to a packet. */

            uorb_fds[i].revents = 0; /* Mark the event as handled */

            err = orb_copy(uorb_metas[i], uorb_fds[i].fd, uorb_data_buffers[i]);
            if (err < 0) {
                inerr("Error reading data from %s: %d\n", uorb_metas[i]->o_name, errno);
                continue;
            }
            uorb_handlers[i](&context, uorb_data_buffers[i]); /* Add the data to a packet */
        }
    }

    pthread_exit(0);
}

/* Sets up a collection_info_t struct with a packet buffer and gets an initially empty node to work on
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

/* Resets the current block count for the collected data
 *
 * @param collection Information about collected data
 */
static void reset_block_count(collection_info_t *collection) {
    for (int i = 0; i < sizeof(collection->block_count) / sizeof(collection->block_count[0]); i++) {
        collection->block_count[i] = 0;
    }
}

/* Adds a block to the current packet being worked on
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
    // Can't add to this packet, it's full or we can just assume its done being assembled
    if (next_block == NULL) {
        indebug("Completed a packet length %d\n", collection->current->end - collection->current->packet);
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

#ifdef CONFIG_SENSORS_L86XXX
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
#endif

#ifdef CONFIG_SENSORS_L86XXX
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
#endif

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

#ifdef CONFIG_SENSORS_L86XXX
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
#endif

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
