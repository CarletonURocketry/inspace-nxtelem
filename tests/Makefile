include $(APPDIR)/Make.defs

# Application information

PROGNAME = $(CONFIG_INSPACE_TESTS_PROGNAME)
PRIORITY = 100
STACKSIZE = $(CONFIG_DEFAULT_TASK_STACKSIZE)
MODULE = $(CONFIG_INSPACE_TESTS)

MAINSRC = tests_main.c
CSRCS += $(filter-out $(MAINSRC), $(wildcard *.c))

include $(APPDIR)/Application.mk
