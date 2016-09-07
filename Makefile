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
# Description:         Top-level makefile for building a RIFFA distribution
# Author:              Dustin Richmond (@darichmond)
#-----------------------------------------------------------------------
include release.mk
CURRENT_PATH := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
RIFFA_ROOT_PATH := $(CURRENT_PATH)

RELEASE_VER=2.2.2
RELEASE_DIR=riffa_$(RELEASE_VER)
RELEASE_PATH=$(CURRENT_PATH)/$(RELEASE_DIR)
RELEASE_SRC_DIR=$(RELEASE_DIR)/source
RELEASE_SRC_PATH=$(CURRENT_PATH)/$(RELEASE_SRC_DIR)
RELEASE_DOC_DIR=$(RELEASE_DIR)/documentation
RELEASE_DOC_PATH=$(CURRENT_PATH)/$(RELEASE_DOC_DIR)
RELEASE_INSTALL_DIR=$(RELEASE_DIR)/install
RELEASE_INSTALL_PATH=$(CURRENT_PATH)/$(RELEASE_INSTALL_DIR)

VENDORS=altera xilinx
SUBDIRS=c_c++ docs driver fpga java matlab python #install

all-boards: 
	$(MAKE) -C fpga $(VENDORS)

$(RELEASE_DIR):
	mkdir $@

$(RELEASE_SRC_DIR): $(RELEASE_DIR) check-release
	mkdir $@

$(RELEASE_DOC_DIR): $(RELEASE_DIR) check-release
	mkdir $@

release: clean $(RELEASE_DIR) $(RELEASE_SRC_DIR) $(RELEASE_DOC_DIR) $(SUBDIRS) 

$(SUBDIRS)::
	make -C $@ $(MAKECMDGOALS) RELEASE_SRC_PATH=$(RELEASE_SRC_PATH) RELEASE_VER=$(RELEASE_VER) RIFFA_ROOT_PATH=$(RIFFA_ROOT_PATH) RELEASE_DOC_PATH=$(RELEASE_DOC_PATH)

clean:
	rm -rf $(RELEASE_DIR)
