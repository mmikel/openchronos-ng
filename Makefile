SUBDIRS = drivers modules

include Common.mk

PYTHON := $(shell which python2 || which python)
BASH := $(shell which bash || which bash)

.PHONY: all
.PHONY: clean
.PHONY: install
.PHONY: config
.PHONY: depend
.PHONY: doc
.PHONY: httpdoc
.PHONY: force

all: depend config.h openchronos.txt

#
# Build list of sources and objects to build
SRCS := $(wildcard *.c)
$(foreach subdir,$(SUBDIRS), \
	$(eval SRCS := $(SRCS) $(wildcard $(subdir)/*.c)) \
)
OBJS := $(patsubst %.c,%.o,$(SRCS))
HEADERS := $(wildcard *.h)

#
# Dependencies rules
depend: openchronos.dep

openchronos.dep: $(SRCS) $(HEADERS) drivers/rtca_now.h
	@echo "Generating dependencies.."
	@touch $@
	@makedepend $(CFLAGS) $(INCLUDES) -Y -f $@ $^ > /dev/null
	@rm -f $@.bak

#
# Append specific CFLAGS/LDFLAGS
DEBUG := $(shell grep "^\#define CONFIG_DEBUG" config.h)
ifeq ($(DEBUG),)
TARGET	:= RELEASE
CFLAGS	+= $(CFLAGS_REL)
LDFLAGS	+= $(LDFLAGS_REL)
else
TARGET	:= DEBUG
CFLAGS	+= $(CFLAGS_DBG)
LDFLAGS	+= $(LDFLAGS_DBG)
endif

# rebuild if CFLAGS changed, as suggested in:
# http://stackoverflow.com/questions/3236145/force-gnu-make-to-rebuild-objects-affected-by-compiler-definition/3237349#3237349
openchronos.cflags: force
	@echo "$(CFLAGS)" | cmp -s - $@ || echo "$(CFLAGS)" > $@

$(OBJS): openchronos.cflags
#
# Top rules

openchronos.elf: even_in_range.o $(OBJS)
	@echo -e "\n>> Building $@ as target $(TARGET)"
	@$(CC) $(CFLAGS) $(LDFLAGS) $(INCLUDES) -o $@ $+

openchronos.txt: openchronos.elf
	$(PYTHON) tools/memory.py -i $< -o $@

even_in_range.o: even_in_range.s
	$(AS) $< -o $@

modinit.o: modinit.c
	@echo "CC $<"
	@$(CC) $(CFLAGS) -Wno-implicit-function-declaration $(INCLUDES) -c $< -o $@

%.o: %.c
	@echo "CC $<"
	@$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

modinit.c:
	@echo "Please do a 'make config' first!" && false

config.h:
	@echo "Please do a 'make config' first!" && false

drivers/rtca_now.h:
	@echo "Generating $@"
	@$(BASH) ./tools/update_rtca_now.sh

config:
	$(PYTHON) tools/config.py
	$(PYTHON) tools/make_modinit.py

install: openchronos.txt
	contrib/ChronosTool.py rfbsl $<

clean: $(SUBDIRS)
	@for subdir in $(SUBDIRS); do \
		echo "Cleaning $$subdir .."; rm -f $$subdir/*.o; \
	done
	@rm -f *.o openchronos.{elf,txt,cflags,dep} output.map
	@rm -f drivers/rtca_now.h

upload: openchronos.txt
	mspdebug rf2500 "prog openchronos.elf"

debug: openchronos.txt
	mspdebug rf2500 gdb

run: openchronos.txt
	mspdebug rf2500 run

-include openchronos.dep
# DO NOT DELETE
