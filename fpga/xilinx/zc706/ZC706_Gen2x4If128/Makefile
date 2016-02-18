# ----------------------------------------------------------------------
# Copyright (c) 2016, The Regents of the University of California All
# rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
# 
#     * Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials provided
#       with the distribution.
# 
#     * Neither the name of The Regents of the University of California
#       nor the names of its contributors may be used to endorse or
#       promote products derived from this software without specific
#       prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL REGENTS OF THE
# UNIVERSITY OF CALIFORNIA BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
# TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
# USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
# ----------------------------------------------------------------------
#-----------------------------------------------------------------------
# Filename:            Makefile
# Version:             1.0
# Description:         Project-level makefile for building an example project
# Author:              Dustin Richmond (@darichmond)
#-----------------------------------------------------------------------
# This make file expects the following variables to be set:
# RIFFA_HDL_PATH -- Path to the riffa_hdl directory in the corresponding RIFFA directory.
# BOARD_PATH -- Path to the $(BOARD) directory, the board this project corresponds to
# BOARD_HDL -- A list of an board-specific HDL files not in the riffa_hdl directory
WIDTH=128
TYPE=classic
CURRENT_PATH := $(notdir $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST))))))
PROJECT=$(shell basename $(CURRENT_PATH))

ifndef RIFFA_HDL_PATH
	RIFFA_HDL_PATH:=../../../riffa_hdl
endif
ifndef BOARD_PATH
	BOARD_PATH:=..
endif
ifndef JOBS
	JOBS=1
endif
include $(RIFFA_HDL_PATH)/riffa.mk
include $(BOARD_PATH)/board.mk

PROJECT_IP+=ip/PCIeGen2x4If128.xci
