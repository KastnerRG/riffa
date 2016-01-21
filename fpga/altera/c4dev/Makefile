BOARD=c4dev
BOARD_PROJECTS:=C4DevGen1x4If64
BOARD_PATH:=$(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))
BOARD_HDL:= $(BOARD_PATH)/riffa_wrapper_$(BOARD).v
SUBDIRS = $(BOARD_PROJECTS)

all: $(SUBDIRS)

$(SUBDIRS)::
	$(MAKE) -C $@ $(MAKECMDGOALS) BOARD_HDL=$(BOARD_HDL) BOARD=$(BOARD)

.PHONY:clean $(SUBDIRS)
clean: $(SUBDIRS)
	rm -rf *~




