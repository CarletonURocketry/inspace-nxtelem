#include <float.h>
#include <math.h>
#include <stdint.h>

#include "../syslogging.h"
#include "detector.h"

#define us_to_s(us) ((float)(us) / 1000000.0f)

#define LANDED_ALT_WINDOW_SIZE ((float)(CONFIG_INSPACE_TELEMETRY_LANDED_ALT_WINDOW / 10))
#define AIRBORNE_ACCEL_THRESHOLD ((float)(CONFIG_INSPACE_TELEMETRY_AIRBORNE_ACCEL_THRESH / 10))
#define AIRBORNE_ALT_THRESHOLD ((float)(CONFIG_INSPACE_TELEMETRY_AIRBORNE_ALT_THRESH / 10))
#define APOGEE_ALT_THRESHOLD ((float)(CONFIG_INSPACE_TELEMETRY_APOGEE_ALT_THRESH) / 10)
#define APOGEE_ACCEL_THRESHOLD ((float)(CONFIG_INSPACE_TELEMETRY_APOGEE_ACCEL_THRESH) / 10)
#define LANDED_ACCEL_THRESHOLD_MIN (6.0f)

/* The maximum speed with which our altitude could decrease at apogee, to differentiate mach lockout */

#define APOGEE_ALT_THRESHOLD_MAX_SPEED 250.0f

/* Check if the current altitude is valid and can be used for detection purposes
 *
 * @param detector The detector to use
 * @return 1 if the detector's current altitude measurement can be used in detection
 */
static bool detector_alt_valid(struct detector *detector) {
    /* Currently, only check if the measurement as generated too long ago. Could also so sanity checks on its value */
    return detector->current_time - detector->last_alt_update < CONFIG_INSPACE_TELEMETRY_STALETIME;
}

/* Check if the current acceleration is valid and can be used for detection purposes
 *
 * @param detector The detector to use
 * @return 1 if the detector's current acceleration measurement can be used in detection
 */
static bool detector_accel_valid(struct detector *detector) {
    /* Only check if the measurement is too old */
    return detector->current_time - detector->last_accel_update < CONFIG_INSPACE_TELEMETRY_STALETIME;
}

/* Reset the apogee variables, so that apogee can be detected again
 *
 * @param detector The detector to use
 */
static void detector_reset_apogee(struct detector *detector) {
    detector->apogee = -FLT_MAX;
    detector->apogee_time = detector->current_time;
}
/**
 * Check if the conditions for being airborne are satisfied
 *
 * @param detector The detector to use
 * @return 1 if the rocket satisfies the conditions to be airborne, 0 otherwise
 */
static bool detector_is_airborne(struct detector *detector) {
    /* Check for an absolute change in altitude from landing, or a high acceleration. If the elevation is set wrong,
     * we may detect being airborne when set on the pad. Ideally, set the elevation correctly before starting to detect
     * events, but if we do detect based on an incorrect elevation, allow landing detections in airborne so that the
     * consequences of the false detection aren't severe. As long as the rocket is on for a reasonable amount of time (1
     * min), going through idle -> airborne -> landed -> idle shouldn't cause us to miss the real launch
     *
     * Alternatively, we can also require the altitude criteria to be set AND the acceleration criteria, but we should
     * check the reliability of the accelerometer first
     */
    return (detector->elevation_set && detector_alt_valid(detector) &&
            fabs(detector_get_alt(detector) - detector->elevation) > AIRBORNE_ALT_THRESHOLD) ||
           (detector_accel_valid(detector) && detector_get_accel(detector) > AIRBORNE_ACCEL_THRESHOLD);
}

/**
 * Check if the conditions for being landed are satisfied
 *
 * @param detector The detector to use
 * @return 1 if the rocket satisfies the conditions to be landed, 0 otherwise
 */
static bool detector_is_landed(struct detector *detector) {
    /* Use an altitude window to make sure there isn't too much variation in the altitude
     * and then check that acceleration is below launch levels. Acceleration check is necessary in case
     * the barometer's readings are unreliable when airborne.
     */
    return detector_alt_valid(detector) && window_criteria_satisfied(&detector->land_alt_window) &&
           detector_accel_valid(detector) && detector_get_accel(detector) < AIRBORNE_ACCEL_THRESHOLD &&
           detector_get_accel(detector) > LANDED_ACCEL_THRESHOLD_MIN;
}

/**
 * Check if the conditions for apogee are satisfied
 *
 * @param detector The detector to use
 * @return 1 if the rocket satisfies the conditions for having passed apogee, 0 otherwise
 */
static bool detector_is_apogee(struct detector *detector) {
    /* At transonic speeds the barometer is unreliable, so require acceleration to be less than what we
     * get during the burning of the motor. With okay filtering, we should be able to trust our barometer
     * at non-transonic speeds enough to compare apogee against the current height directly
     */
    float alt_change = detector->apogee - detector_get_alt(detector);
    uint64_t time_diff = detector->last_alt_update - detector->apogee_time;

    if (detector_alt_valid(detector) && alt_change > APOGEE_ALT_THRESHOLD && time_diff != 0 &&
        detector_accel_valid(detector) && detector_get_accel(detector) < APOGEE_ACCEL_THRESHOLD) {
        float current_speed = alt_change / us_to_s(time_diff);
        if (current_speed > APOGEE_ALT_THRESHOLD_MAX_SPEED) {
            /* We're going too fast for this to actually be an apogee - let us find a new value for apogee */
            detector_reset_apogee(detector);
            return false;
        } else {
            return true;
        }
    }
    return false;
}

/**
 * Initialize a detector with a number of defaults. Its best not to use the detector before setting some of these values
 * and providng a number of samples to populate the internal buffers with
 *
 * @param detector The detector to initialize
 */
void detector_init(struct detector *detector, uint64_t time) {
    median_filter_init(&detector->alts.median, detector->alts.median_backing_sorted,
                       detector->alts.median_backing_time_ordered, sizeof(detector->alts.median_backing_sorted));
    average_filter_init(&detector->alts.average, detector->alts.average_backing,
                        sizeof(detector->alts.average_backing));

    median_filter_init(&detector->accels.median, detector->accels.median_backing_sorted,
                       detector->accels.median_backing_time_ordered, sizeof(detector->accels.median_backing_sorted));
    average_filter_init(&detector->accels.average, detector->accels.average_backing,
                        sizeof(detector->accels.average_backing));
    window_criteria_init(&detector->land_alt_window, LANDED_ALT_WINDOW_SIZE,
                         CONFIG_INSPACE_TELEMETRY_LANDED_ALT_DURATION);

    detector->init_time = time;
    detector->current_time = time;
    detector->last_alt_update = 0;
    detector->last_accel_update = 0;
    detector->current_alt = 0.0f;
    detector->current_accel = 0.0f;

    detector_reset_apogee(detector);

    /* This can be set manually, or will be set by the detector automatically */
    detector->elevation_set = 0;
    detector->elevation = 0.0f;

    /* These should ideally be set manually before the detector is used, but these defaults may work */
    detector_set_state(detector, STATE_AIRBORNE, SUBSTATE_UNKNOWN);
}

/**
 * Add an altitude sample to the detector to perform filtering on, and then to update internal state with
 *
 * @param detector The detector to use
 * @param sample The altitude sample to use
 */
void detector_add_alt(struct detector *detector, struct altitude_sample *sample) {
    /* Time decreasing would not make sense in the detector, but possible with order of provided samples */
    if (sample->time > detector->current_time) {
        detector->current_time = sample->time;
    }

    // Filtering step to prevent false readings
    float median = median_filter_add(&detector->alts.median, sample->altitude);
    detector->current_alt = average_filter_add(&detector->alts.average, median);

    /* Keep track of apogee, could also add other validation to make sure we don't keep a bad apogee */
    if (detector->current_alt > detector->apogee) {
        detector->apogee = detector->current_alt;
        detector->apogee_time = sample->time;
    }

    /* Could limit use of the altitude window to states that need it */
    window_criteria_add(&detector->land_alt_window, detector->current_alt, sample->time - detector->last_alt_update);

    /* If we just powered on and elevation hasn't been set */
    if (!detector->elevation_set) {
        if (detector->current_time - detector->init_time > CONFIG_INSPACE_TELEMETRY_ELEV_DELAY) {
            /* Hopefully, the filters should be full and this should be a very sensible value */
            detector->elevation = detector->current_alt;
            detector->elevation_set = 1;
        }
    }

    detector->last_alt_update = sample->time;
}

/**
 * Get the current altitude value being used by the detector, after filtering
 *
 * @param detector The detector to use
 * @return The current altitude that the detector is using
 */
float detector_get_alt(struct detector *detector) { return detector->current_alt; }

/**
 * Add an acceleration value (magnitude) to the detector to perform filtering on, and then to update internal state with
 *
 * @param detector The detector to use
 * @param sample the altitude sample to use
 */
void detector_add_accel(struct detector *detector, struct accel_sample *sample) {
    if (sample->time > detector->current_time) {
        detector->current_time = sample->time;
    }
    /* Note - get rid of the sign on the provided acceleration value since in case it has one */
    float median = median_filter_add(&detector->accels.median, fabs(sample->acceleration));
    detector->current_accel = average_filter_add(&detector->accels.average, median);

    detector->last_accel_update = sample->time;
}

/**
 * Get the current acceleration value being used by the detector, after filtering. Note: this
 * doesn't make use of altitude data. Only direct acceleration measurements are used
 *
 * @param detector The detector to use
 * @return The current acceleration that the detector is using
 */
float detector_get_accel(struct detector *detector) { return detector->current_accel; }

/**
 * Detects events based on the internal state of the detector, like samples and rocket state information
 *
 * @param detector The detector to use
 * @return The detected event, or DETECTOR_NO_EVENT if none were detected
 */
enum detector_event detector_detect(struct detector *detector) {

    /* We are doing detection events based on state because the checks we perform otherwise
     * might not make sense.
     */

    switch (detector->state) {

    case STATE_IDLE: {
        if (detector_is_airborne(detector)) {
            ininfo("Detected airborne event from the idle state\n");
            return DETECTOR_AIRBORNE_EVENT;
        }
    } break;

    case STATE_AIRBORNE: {
        switch (detector->substate) {

        case SUBSTATE_UNKNOWN:
        case SUBSTATE_ASCENT:
            if (detector_is_apogee(detector)) {
                ininfo("Detected apogee from the airborne state\n");
                return DETECTOR_APOGEE_EVENT;
            } else if (detector_is_landed(detector)) {
                ininfo("Detected landing from the airborne state\n");
                return DETECTOR_LANDING_EVENT;
            }
            break;

        case SUBSTATE_DESCENT:
            if (detector_is_landed(detector)) {
                ininfo("Detected a landing event from the descent state\n");
                return DETECTOR_LANDING_EVENT;
            }
            break;
        }

    } break;

    default:
        /* Ignore states like landing */
        break;
    }

    return DETECTOR_NO_EVENT;
}

/**
 * Set the current flight state of the detector. This should be set whenever the rocket's flight state changes,
 * otherwise the detector can provide events that aren't relevant and miss others
 *
 * @param detector The detector to set the state for
 * @param state The flight state of the rocket
 * @param substate The flight substate of the rocket
 */
void detector_set_state(struct detector *detector, enum flight_state_e state, enum flight_substate_e substate) {
    if (state != detector->state) {
        switch (state) {
        case STATE_LANDED:
            detector_set_elevation(detector, detector_get_alt(detector));
            detector_reset_apogee(detector);
            break;
        case STATE_AIRBORNE:
            window_criteria_init(&detector->land_alt_window, LANDED_ALT_WINDOW_SIZE,
                                 CONFIG_INSPACE_TELEMETRY_LANDED_ALT_DURATION);
            break;
        default:
            break;
        }
    }
    detector->state = state;
    detector->substate = substate;
}

/**
 * Set the elevation (landing altitude) of the detector. The detector will take a reading after a certain amount
 * of time if the elevation isn't set
 *
 * @param detector The detector to use
 * @param elevation The elevation to set, which should be collected anytime the rocket is in a new location
 */
void detector_set_elevation(struct detector *detector, float elevation) {
    detector->elevation_set = 1;
    detector->elevation = elevation;
}
