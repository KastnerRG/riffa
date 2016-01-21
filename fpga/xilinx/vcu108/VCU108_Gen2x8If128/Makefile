WIDTH=128
TYPE=ultrascale
CURRENT_PATH := $(notdir $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST))))))
PROJECT=$(shell basename $(CURRENT_PATH))

ifndef RIFFA_PATH
	RIFFA_PATH:=../../../riffa_hdl
endif
ifndef BOARD_PATH
	BOARD_PATH:=..
endif
ifndef JOBS
	JOBS=1
endif
include $(RIFFA_PATH)/riffa.mk
include $(BOARD_PATH)/board.mk

PROJECT_IP+=PCIeGen2x8If128.xci
