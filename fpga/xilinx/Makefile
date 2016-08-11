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
# Description:         Vendor-level makefile for building all boards
# Author:              Dustin Richmond (@darichmond)
#-----------------------------------------------------------------------
VENDOR:=xilinx
ULTRASCALE:=NetFPGA adm7V3 vc709 
CLASSIC:=vc707 zc706 ac701 kc705
BOARDS:=$(CLASSIC) $(ULTRASCALE)
CURRENT_PATH := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
.DEFAULT_GOAL=all

RIFFA_ROOT_PATH=$(CURRENT_PATH)/../../
RIFFA_HDL_PATH=$(CURRENT_PATH)/../riffa_hdl
include $(RIFFA_ROOT_PATH)/release.mk
include $(RIFFA_ROOT_PATH)/fpga/fpga.mk

all $(VENDOR): $(BOARDS)

classic:$(CLASSIC)
ultrascale:$(ULTRASCALE)

$(CLASSIC) $(ULTRASCALE)::
	$(MAKE) -C $@ $(MAKECMDGOALS) VENDOR_PATH=$(CURRENT_PATH) VENDOR=$(VENDOR) RIFFA_ROOT_PATH=$(RIFFA_ROOT_PATH)

.PHONY:clean $(BOARDS) classic all clobber
clean clobber: $(BOARDS)
	rm -rf *.log *.jou .Xil *~
