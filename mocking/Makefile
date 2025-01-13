

include $(APPDIR)/Make.defs

# Mocking built-in application info

PROGNAME  = $(CONFIG_INSPACE_MOCKING_PROGNAME)
PRIORITY  = $(CONFIG_INSPACE_MOCKING_PRIORITY)
STACKSIZE = $(CONFIG_INSPACE_MOCKING_STACKSIZE)
MODULE    = $(CONFIG_INSPACE_MOCKING)

# Sensor mocking application

MAINSRC = mocking_main.c

# Common build

checkgenromfs:
	@genromfs -h 1>/dev/null 2>&1 || { \
 echo "Host executable genromfs not available in PATH"; \
 echo "You may need to download in from http://romfs.sourceforge.net/"; \
 exit 1; \
	}

data.img: checkgenromfs
	@genromfs -f $@ -d test-data -V "mocking-romfs" || { echo "genromfs failed" ; exit 1 ; }

data_romfs.h: data.img
	@xxd -i $< >$@ || { echo "xxd of $< failed" ; exit 1 ; }

context:: data_romfs.h

distclean::
	$(call DELFILE, data.img)
	$(call DELFILE, data.img.h)

include $(APPDIR)/Application.mk
