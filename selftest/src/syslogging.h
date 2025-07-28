#include <stdio.h>
#include <syslog.h>

#define __HLOGSTR(fstring) "%s::" fstring

#ifdef CONFIG_SYSLOG
#define swarn(fstring, ...)                                                                                            \
    syslog(LOG_WARNING | LOG_USER, "WARNING::" __HLOGSTR(fstring), __FUNCTION__ __VA_OPT__(, ) __VA_ARGS__)
#define serr(fstring, ...)                                                                                             \
    syslog(LOG_ERR | LOG_USER, "ERROR::" __HLOGSTR(fstring), __FUNCTION__ __VA_OPT__(, ) __VA_ARGS__)
#define ssuc(fstring, ...)                                                                                             \
    syslog(LOG_INFO | LOG_USER, "SUCCESS::" __HLOGSTR(fstring), __FUNCTION__ __VA_OPT__(, ) __VA_ARGS__)
#else
#define swarn(fstring, ...) fprintf(stderr, "WARNING::" __HLOGSTR(fstring), __FUNCTION__ __VA_OPT__(, ) __VA_ARGS__)
#define serr(fstring, ...) fprintf(stderr, "ERROR::" __HLOGSTR(fstring), __FUNCTION__ __VA_OPT__(, ) __VA_ARGS__)
#define ssuc(fstring, ...) fprintf(stdout, "SUCCESS::" __HLOGSTR(fstring), __FUNCTION__ __VA_OPT__(, ) __VA_ARGS__)
#endif
