-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
include $(APPDIR)/Make.defs

# built-in application info

APPNAME = simpleServer
FUNCNAME = $(APPNAME)_main
PRIORITY = 100
STACKSIZE = 4096
THREADEXEC = TASH_EXECMD_SYNC

# iotivity Constrained Simple Server example
IOTIVITY_CONSTRAINED_BASE_DIR = $(APPDIR)/../external/iotivity-constrained

CFLAGS=-fPIC -fno-asynchronous-unwind-tables -fno-omit-frame-pointer -ffreestanding -Os -fno-stack-protector -ffunction-sections -fdata-sections -fno-reorder-functions -fno-defer-pop -fno-strict-overflow -std=gnu99

ifeq ($(CONFIG_ARCH_CHIP_STM32L4),y)
CFLAGS+=-mcpu=cortex-m4 -mthumb -march=armv7e-m -mfloat-abi=soft
endif

# must set this option
CFLAGS+=-DOC_DYNAMIC_ALLOCATION

ifeq ($(CONFIG_IOTIVITY_CONSTRAINED_SECURED),y)
  CFLAGS+=-DOC_SECURITY
endif

ifeq ($(CONFIG_IOTIVITY_CONSTRAINED_DEBUG),y)
	CFLAGS+=-DOC_DEBUG -g -O0
else
	CFLAGS+=-Wl,--gc-sections
endif

ifeq ($(CONFIG_IOTIVITY_CONSTRAINED_IPV4),y)
	CFLAGS+=-DOC_IPV4
endif

ifeq ($(CONFIG_IOTIVITY_CONTRAINED_TCP),y)
	CFLAGS+=-DOC_TCP
endif

CFLAGS+=-DOC_SERVER
CFLAGS+=-D__TIZENRT__=1
CFLAGS+=-I${TOPDIR}/include/
CFLAGS+=-I${IOTIVITY_CONSTRAINED_BASE_DIR}/
CFLAGS+=-I${IOTIVITY_CONSTRAINED_BASE_DIR}/include/
CFLAGS+=-I${IOTIVITY_CONSTRAINED_BASE_DIR}/port/tizenrt/

CSRCS = port_tinyara.c

MAINSRC = simpleServer.c

COBJS = $(CSRCS:.c=$(OBJEXT))
MAINOBJ = $(MAINSRC:.c=$(OBJEXT))

SRCS = $(CSRCS) $(MAINSRC)
OBJS = $(COBJS)

ifneq ($(CONFIG_BUILD_KERNEL),y)
  OBJS += $(MAINOBJ)
endif

BIN = ../../libapps$(LIBEXT)

INSTALL_DIR = $(BIN_DIR)

CONFIG_EXAMPLES_IOTIVITY_CONSTRAINED_PROGNAME ?= iotivityConstrainedSimServ$(EXEEXT)
PROGNAME = $(CONFIG_EXAMPLES_IOTIVITY_CONSTRAINED_PROGNAME)

ROOTDEPPATH = --dep-path .

all: .built
.PHONY: clean depend distclean preconfig

$(COBJS) $(MAINOBJ): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

.built: $(OBJS)
	$(call ARCHIVE, $(BIN), $(OBJS))
	@touch .built

ifeq ($(CONFIG_BUILD_KERNEL),y)
$(BIN_DIR)$(DELIM)$(PROGNAME): $(OBJS)
	@echo "LD: $(PROGNAME)"
	$(LD) $(LDELFFLAGS) $(LDLIBPATH) -o $(INSTALL_DIR)$(DELIM)$(PROGNAME) $(ARCHCRT0OBJ) $(MAINOBJ) $(LDLIBS)
	$(NM) -u  $(INSTALL_DIR)$(DELIM)$(PROGNAME)

install: $(BIN_DIR)$(DELIM)$(PROGNAME)

else
install:

endif

ifeq ($(CONFIG_BUILTIN_APPS)$(CONFIG_EXAMPLES_IOTIVITY_CONSTRAINED_SIMPLESERVER),yy)
$(BUILTIN_REGISTRY)$(DELIM)$(FUNCNAME).bdat: $(DEPCONFIG) Makefile
	$(call REGISTER,$(APPNAME),$(FUNCNAME),$(THREADEXEC),$(PRIORITY),$(STACKSIZE))

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
preconfig:

