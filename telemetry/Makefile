include $(APPDIR)/Make.defs

# Application information for telemetry system

PROGNAME = $(CONFIG_INSPACE_TELEMETRY_PROGNAME)
PRIORITY = $(CONFIG_INSPACE_TELEMETRY_PRIORITY)
STACKSIZE = $(CONFIG_INSPACE_TELEMETRY_STACKSIZE)
MODULE = $(CONFIG_INSPACE_TELEMETRY)

# Telemetry system

MAINSRC = src/telemetry_main.c

CSRCS += $(wildcard src/collection/*.c)
CSRCS += $(wildcard src/rocket-state/*.c)
CSRCS += $(wildcard src/transmission/*.c)
CSRCS += $(wildcard src/logging/*.c)
CSRCS += $(wildcard src/packets/*.c)
CSRCS += $(wildcard src/pwm/*.c)
CSRCS += $(wildcard src/fusion/*.c)
CSRCS += $(wildcard src/sensors/*.c)
CSRCS += $(wildcard src/shell/*.c)

include $(APPDIR)/Application.mk
