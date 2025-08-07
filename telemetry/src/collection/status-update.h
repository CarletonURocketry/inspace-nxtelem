#ifndef _STATUS_UPDATE_H_
#define _STATUS_UPDATE_H_

#include <stdint.h>

#include "uORB/uORB.h"

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

int publish_status(enum status_code_e status_code);
int publish_error(enum process_id_e proc_id, enum error_code_e error_code);

#endif // _STATUS_UPDATE_H_
