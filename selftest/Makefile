include $(APPDIR)/Make.defs

# Mocking built-in application info

PROGNAME  = $(CONFIG_INSPACE_SELFTEST_PROGNAME)
PRIORITY  = $(CONFIG_INSPACE_SELFTEST_PRIORITY)
STACKSIZE = $(CONFIG_INSPACE_SELFTEST_STACKSIZE)
MODULE    = $(CONFIG_INSPACE_SELFTEST)

# Sensor mocking application

MAINSRC = src/selftest_main.c
CSRCS += $(wildcard src/*.c)

include $(APPDIR)/Application.mk
