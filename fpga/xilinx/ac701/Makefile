BOARD:=ac701
BOARD_PROJECTS:=AC701_Gen1x4If64 AC701_Gen2x4If128
BOARD_PATH:=$(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))
BOARD_HDL:= $(BOARD_PATH)/riffa_wrapper_$(BOARD).v
SUBDIRS = $(BOARD_PROJECTS)

all: $(SUBDIRS)

$(SUBDIRS)::
	$(MAKE) -C $@ $(MAKECMDGOALS) BOARD_HDL=$(BOARD_HDL)

.PHONY:clean $(SUBDIRS)
clean: $(SUBDIRS)
	rm -rf *.log *.jou .Xil *~




