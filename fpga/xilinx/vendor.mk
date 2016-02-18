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
# Filename:            vendor.mk
# Version:             1.0
# Description:         Vendor-specific include makefile
# Author:              Dustin Richmond (@darichmond)
#-----------------------------------------------------------------------

BOARD_PATH:=$(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))/$(BOARD)
BOARD_HDL:= $(BOARD_PATH)/riffa_wrapper_$(BOARD).v
RIFFA_ROOT_PATH:=$(BOARD_PATH)/../../../
RIFFA_HDL_PATH:=$(BOARD_PATH)/../../riffa_hdl
include $(RIFFA_ROOT_PATH)/release.mk

RELEASE_BOARD_PATH=$(RELEASE_SRC_PATH)/fpga/$(VENDOR)/$(BOARD)
SUBDIRS = $(BOARD_PROJECTS)
.DEFAULT_GOAL=all

.PHONY:clean clobber $(SUBDIRS) all $(VENDOR) $(BOARD) $(BOARD_TYPE)
all $(VENDOR) $(BOARD) $(BOARD_TYPE): $(SUBDIRS)

$(SUBDIRS)::
	$(MAKE) -C $@ $(MAKECMDGOALS) BOARD=$(BOARD) TYPE=$(BOARD_TYPE) VENDOR=$(VENDOR) BOARD_HDL=$(BOARD_HDL) RIFFA_ROOT_PATH=$(RIFFA_ROOT_PATH)
clean clobber: $(SUBDIRS)
	rm -rf *~

release: destination $(SUBDIRS)

destination: $(RELEASE_BOARD_PATH)
$(RELEASE_BOARD_PATH): check-release-src
	mkdir $@
	cp $(BOARD_HDL) $(RELEASE_BOARD_PATH)

