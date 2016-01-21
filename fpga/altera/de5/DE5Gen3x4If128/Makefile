# This make file expects the following variables to be set:
# RIFFA_PATH -- Path to the riffa_hdl directory in the corresponding RIFFA directory.
# BOARD_PATH -- Path to the $(BOARD) directory, the board this project corresponds to
# BOARD_HDL -- A list of an board-specific HDL files not in the riffa_hdl directory

WIDTH=128
TYPE=classic
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

PROJECT_IP+=ip/XCVRCtrlGen3x4.sip ip/XCVRCtrlGen3x4.qip ip/PCIeGen3x4If128.sip ip/PCIeGen3x4If128.qip
