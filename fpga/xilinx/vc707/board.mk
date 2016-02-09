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
PROJECT_HDL=hdl/$(PROJECT).v $(BOARD_HDL) $(patsubst %, $(RIFFA_PATH)/%,$(RIFFA_HDL)) $(PROJECT_FILES) 
PROJECT_CONSTR=constr/$(PROJECT).xdc
PROJECT_FILE=prj/$(PROJECT).xpr
PROJECT_FILES=$(PROJECT_IP) $(PROJECT_CONSTR) $(PROJECT_QSRCS) $(PROJECT_HDL)

.PHONY:$(PROJECT) all synthesis implementation clean clobber $(TYPE) $(VENDOR) $(BOARD)
$(PROJECT): bit/$(PROJECT).bit 
	@echo Compiling Project $@

bit/$(PROJECT).bit: $($(PROJECT)_FILES)
	echo "launch_runs impl_1 -to_step write_bitstream -jobs $(JOBS); wait_on_run impl_1" | vivado -mode tcl prj/$(PROJECT).xpr
	mv prj/$(PROJECT).runs/impl_1/$(PROJECT).bit bit/

synthesis: prj/$(PROJECT).runs/synth_1
prj/$(PROJECT).runs/synth_1: $($(PROJECT)_FILES)
	echo "launch_runs synth_1 -jobs $(JOBS); wait_on_run synth_1" | vivado -mode tcl prj/$(PROJECT).xpr

implementation:prj/$(PROJECT).runs/impl_1
prj/$(PROJECT).runs/impl_1: $($(PROJECT)_FILES)
	echo "launch_runs impl_1 -jobs $(JOBS); wait_on_run impl1" | vivado -mode tcl prj/$(PROJECT).xpr

all $(TYPE) $(VENDOR) $(BOARD):$(PROJECT)
clean:
	echo "reset_run impl_1; reset_run synth_1;" | vivado -mode tcl prj/$(PROJECT).xpr
	rm -rf *.log *.jou *~ .Xil 
	rm -rf ip/doc ip/sim ip/source ip/synth ip/*.dcp ip/*.v ip/*.xml ip/*.vhdl ip/*.veo ip/*~
	rm -rf prj/*.hw prj/*.runs prj/*.cache prj/*~

clobber:
	rm -rf bit/*.bit
