
#include <uORB/uORB.h>

#include "../syslogging.h"
#include "status-update.h"

/* uORB metadata definitions */
#if defined(CONFIG_DEBUG_UORB)
static const char error_message_format[] = "error message - timestamp:%" PRIu64 ", proc_id %d error %d";
static const char status_message_format[] = "status message - timestamp%" PRIu64 ", status %d";
ORB_DEFINE(error_message, struct error_message, error_message_format);
ORB_DEFINE(status_message, struct status_message, status_message_format);
#else
ORB_DEFINE(error_message, struct error_message, 0);
ORB_DEFINE(status_message, struct status_message, 0);
#endif

/**
 * Publish a status message
 *
 * @param outputs An outputs struct that has been set up
 * @return 0 if the code was published successfully, or a negative error code on failure
 */
int publish_status(enum status_code_e status_code) {
    struct status_message status = {.timestamp = orb_absolute_time(), .status_code = status_code};
    return orb_publish_auto(ORB_ID(status_message), NULL, &status, NULL);
}

/**
 * Publish an error message
 *
 * @param outputs An outputs struct that has been set up
 * @return 0 if the error was published successfully, or a negative error code on failure
 */
int publish_error(enum process_id_e proc_id, enum error_code_e error_code) {
    struct error_message error = {.timestamp = orb_absolute_time(), .proc_id = proc_id, .error_code = error_code};
    ininfo("Publishing an error message for process %d with code %d\n", proc_id, error_code);
    return orb_publish_auto(ORB_ID(error_message), NULL, &error, NULL);
}
