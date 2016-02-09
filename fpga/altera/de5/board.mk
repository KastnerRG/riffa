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
# Filename:            board.mk
# Version:             1.0
# Description:         Board-specific include makefile
# Author:              Dustin Richmond (@darichmond)
#-----------------------------------------------------------------------
BOARD_HDL:= $(BOARD_PATH)/riffa_wrapper_$(BOARD).v

PROJECT_IP=
PROJECT_HDL=hdl/$(PROJECT).v $(BOARD_HDL) $(patsubst %, $(RIFFA_PATH)/%,$(RIFFA_HDL))
PROJECT_CONSTR=constr/$(PROJECT).sdc
PROJECT_FILE=prj/$(PROJECT).qsf prj/$(PROJECT).qpf
PROJECT_FILES=$(PROJECT_IP) $(PROJECT_CONSTR) $(PROJECT_QSRCS) $(PROJECT_HDL)

.PHONY:$(PROJECT) all synthesis implementation clean clobber $(TYPE) $(VENDOR) $(BOARD)
$(PROJECT): bit/$(PROJECT).sof 
	@echo Compiling Project $@

bit/$(PROJECT).sof: $(PROJECT_FILES)
	quartus_sh --flow compile prj/$(PROJECT).qpf

synthesis: bit/$(PROJECT).map.rpt
bit/$(PROJECT).map.rpt: $(PROJECT_FILES)
	quartus_sh --flow analysis_and_synthesis prj/$(PROJECT).qpf


implementation:bit/$(PROJECT).fit.rpt
bit/$(PROJECT).fit.rpt: $(PROJECT_FILES)
	quartus_sh --flow fitter prj/$(PROJECT).qpf

all $(TYPE) $(VENDOR) $(BOARD):$(PROJECT)
clean:
	rm -rf ip/.qsys_edit ip/*~
	rm -rf prj/db prj/incremental_db prj/*txt prj/*.sopcinfo prj/*.qws prj/*~
	rm -rf bit/*.done bit/*.smsg bit/*.rpt bit/*.summary bit/*.sld bit/*.pin bit/*.jdi bit/*~
	rm -rf hdl/*~
	rm -rf constr/*~
	rm -rf *~

clobber:
	rm -rf bit/*.sof
