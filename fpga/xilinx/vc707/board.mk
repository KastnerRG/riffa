BOARD_HDL:= $(BOARD_PATH)/riffa_wrapper_$(BOARD).v

PROJECT_IP=
PROJECT_HDL=hdl/$(PROJECT).v $(BOARD_HDL) $(patsubst %, $(RIFFA_PATH)/%,$(RIFFA_HDL)) $(PROJECT_FILES) 
PROJECT_CONSTR=constr/$(PROJECT).xdc
PROJECT_FILE=prj/$(PROJECT).xpr
PROJECT_FILES=$(PROJECT_IP) $(PROJECT_CONSTR) $(PROJECT_QSRCS) $(PROJECT_HDL)

.PHONY:$(PROJECT)
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

all:$(PROJECT)
clean:
	rm -rf *.log *.jou *~ .Xil 
	rm -rf ip/doc ip/sim ip/source ip/synth ip/*.dcp ip/*.v ip/*.xml ip/*.vhdl ip/*.veo *~
	rm -rf prj/*.hw prj/*.runs prj/*.cache prj/*~

clean-bit:
	rm -rf bit/*.bit 
