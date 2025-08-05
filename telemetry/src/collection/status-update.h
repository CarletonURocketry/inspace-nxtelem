#ifndef _UORB_DATA_H
#define _UORB_DATA_H

#include <stdint.h>

#include <uORB/uORB.h>

#include "../syslogging.h"

/* Possible status codes */
enum status_code_e {
  STATUS_SYSTEMS_NOMINAL = 0x00,            /* All systems nominal */

  STATUS_TELEMETRY_CHANGED_IDLE = 0x01,     /* The telemetry system just changed to the idle state */
  STATUS_TELEMETRY_CHANGED_AIRBORNE = 0x02, /* The telemetry system just changed to the airborne state */
  STATUS_TELEMETRY_CHANGED_ASCENT = 0x03,   /* The telemetry system just changed to the ascent state */
  STATUS_TELEMETRY_CHANGED_APOGEE = 0x04,   /* The telemetry system just detected apogee */
  STATUS_TELEMETRY_CHANGED_LANDED = 0x04,   /* The telemetry system just changed to the landed state */

  STATUS_TELEMETRY_UPDATE_IDLE = 0x05,      /* The telemetry system is still in the idle state */
  STATUS_TELEMETRY_UPDATE_AIRBORNE = 0x06,  /* The telemetry system is still in the airborne state */
  STATUS_TELEMETRY_UPDATE_ASCENT = 0x07,    /* The telemetry system is still in the ascent stage */
  STATUS_TELEMETRY_UPDATE_DESCENT = 0x08,   /* The telemetry system is still in the descent stage */
  STATUS_TELEMETRY_UPDATE_LANDED = 0x09,    /* The telemetry system is still in the landed state */

  STATUS_RES_ABOVE = 0x0a                   /* Reserved values for status codes above this value */
};

/* Possible error codes */
enum error_code_e {
    ERROR_GENERAL = 0x00,      /* A general error has occured */
    ERROR_PROCESS_DEAD = 0x01, /* The process in proc_id has died */
};

/* Process IDs for error messages */
enum process_id_e {
  PROC_ID_GENERAL = 0x00,    /* General error process ID */
  PROC_ID_COLLECTION = 0x01, /* Collection thread */
  PROC_ID_FUSION = 0x02,     /* Fusion thread */
  PROC_ID_LOGGING = 0x03,    /* Logging thread */
  PROC_ID_TRANSMIT = 0x04,   /* Transmit thread */
};

struct error_message {
    uint64_t timestamp;
    enum process_id_e proc_id;
    enum error_code_e error_code;
};

struct status_message {
    uint64_t timestamp;
    enum status_code_e status_code;
};

/* UORB declarations */
ORB_DECLARE(error_message);
ORB_DECLARE(status_message);

/**
 * Publish a status message
 *
 * @param outputs An outputs struct that has been set up
 * @return 0 if the code was published successfully, or a negative error code on failure
 */
static int publish_status(enum status_code_e status_code) {
    struct status_message status = {.timestamp = orb_absolute_time(), .status_code = status_code};
    indebug("Publishing a status message with code %d", status_code);
    return orb_publish_auto(ORB_ID(status_message), NULL, &status, NULL);
}

/**
 * Publish an error message
 *
 * @param outputs An outputs struct that has been set up
 * @return 0 if the error was published successfully, or a negative error code on failure
 */
static int publish_error(enum process_id_e proc_id, enum error_code_e error_code) {
    struct error_message error = {.timestamp = orb_absolute_time(), .proc_id = proc_id, .error_code = error_code};
    indebug("Publishing an error message for process %d with code %d", proc_id, error_code);
    return orb_publish_auto(ORB_ID(error_message), NULL, &error, NULL);
}

#endif // _UORB_DATA_
