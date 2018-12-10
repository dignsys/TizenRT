###########################################################################
#
# Copyright 2017 Samsung Electronics All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
# either express or implied. See the License for the specific
# language governing permissions and limitations under the License.
#
###########################################################################

-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
include $(APPDIR)/Make.defs

# built-in application info

APPNAME = st_things_sample
FUNCNAME = $(APPNAME)_main
THREADEXEC = TASH_EXECMD_SYNC

# SmartThings Things example

ASRCS =
CSRCS =
ifeq ($(CONFIG_EXAMPLES_ST_THINGS_LIGHT),y)
CSRCS += light/sample_light_things.c light/sample_light_user.c
endif
ifeq ($(CONFIG_EXAMPLES_ST_THINGS_BLINK),y)
CSRCS += blink/sample_blink_things.c blink/sample_blink_user.c blink/iotbus_util.c
endif
ifeq ($(CONFIG_EXAMPLES_ST_THINGS_MSG_HANDLING),y)
CSRCS += msghandling/sample_msghandling_things.c msghandling/sample_msghandling_user.c
endif
ifeq ($(CONFIG_EXAMPLES_ST_THINGS_LED),y)
CSRCS += led/things.c led/common_handlers.c 
CSRCS += led/resource_capability_switch_main_0.c
endif
ifeq ($(CONFIG_EXAMPLES_ST_THINGS_CO2),y)
CSRCS += co2/things.c co2/common_handlers.c 
CSRCS += co2/resource_capability_switch_main_0.c
CSRCS += co2/resource_capability_switchlevel_main_0.c
CSRCS += co2/resource_capability_airqualitysensor_main_0.c
endif
ifeq ($(CONFIG_EXAMPLES_ST_THINGS_SMARTPLUG),y)
CSRCS += smartplug/things.c smartplug/common_handlers.c 
CSRCS += smartplug/resource_capability_switch_main_0.c
CSRCS += smartplug/resource_capability_powermeter_main_0.c
CSRCS += smartplug/resource_capability_energymeter_main_0.c
CSRCS += smartplug/resource_capability_dustsensor_main_0.c
CSRCS += smartplug/resource_capability_colorcontrol_main_0.c
CSRCS += smartplug/resource_capability_doorcontrol_main_0.c
endif
MAINSRC = st_things_sample_main.c

# Comment out the following line to disable debug messages
# CFLAGS += -DDEBUG

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))
MAINOBJ = $(MAINSRC:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS) $(MAINSRC)
OBJS = $(AOBJS) $(COBJS)

ifneq ($(CONFIG_BUILD_KERNEL),y)
  OBJS += $(MAINOBJ)
endif

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  BIN = ..\..\libapps$(LIBEXT)
else
ifeq ($(WINTOOL),y)
  BIN = ..\\..\\libapps$(LIBEXT)
else
  BIN = ../../libapps$(LIBEXT)
endif
endif

ifeq ($(WINTOOL),y)
  INSTALL_DIR = "${shell cygpath -w $(BIN_DIR)}"
else
  INSTALL_DIR = $(BIN_DIR)
endif

CONFIG_EXAMPLES_ST_THINGS_PROGNAME ?= st_things$(EXEEXT)
PROGNAME = $(CONFIG_EXAMPLES_ST_THINGS_PROGNAME)

ROOTDEPPATH = --dep-path .

# Common build

all: .built
.PHONY: clean depend distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS) $(MAINOBJ): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

.built: $(OBJS)
	$(call ARCHIVE, $(BIN), $(OBJS))
	@touch .built

ifeq ($(CONFIG_BUILD_KERNEL),y)
$(BIN_DIR)$(DELIM)$(PROGNAME): $(OBJS) $(MAINOBJ)
	@echo "LD: $(PROGNAME)"
	$(Q) $(LD) $(LDELFFLAGS) $(LDLIBPATH) -o $(INSTALL_DIR)$(DELIM)$(PROGNAME) $(ARCHCRT0OBJ) $(MAINOBJ) $(LDLIBS)
	$(Q) $(NM) -u  $(INSTALL_DIR)$(DELIM)$(PROGNAME)

install: $(BIN_DIR)$(DELIM)$(PROGNAME)

else
install:

endif

ifeq ($(CONFIG_BUILTIN_APPS)$(CONFIG_EXAMPLES_ST_THINGS),yy)
$(BUILTIN_REGISTRY)$(DELIM)$(FUNCNAME).bdat: $(DEPCONFIG) Makefile
	$(call REGISTER,$(APPNAME),$(FUNCNAME),$(THREADEXEC))

context: $(BUILTIN_REGISTRY)$(DELIM)$(FUNCNAME).bdat

else
context:

endif

.depend: Makefile $(SRCS)
	@$(MKDEP) $(ROOTDEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	$(call DELFILE, .built)
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
.PHONY: preconfig
preconfig:
