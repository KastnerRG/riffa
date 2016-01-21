BOARD:=NetFPGA
BOARD_PROJECTS:=NetFPGA_Gen1x8If64 NetFPGA_Gen2x8If128 NetFPGA_Gen3x4If128
BOARD_PATH:=$(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))
BOARD_HDL:= $(BOARD_PATH)/riffa_wrapper_$(BOARD).v
SUBDIRS = $(BOARD_PROJECTS)

all: $(SUBDIRS)

$(SUBDIRS)::
	$(MAKE) -C $@ $(MAKECMDGOALS) BOARD_HDL=$(BOARD_HDL)

.PHONY:clean $(SUBDIRS)
clean: $(SUBDIRS)
	rm -rf *.log *.jou .Xil *~




