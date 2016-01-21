BOARD_HDL:= $(BOARD_PATH)/riffa_wrapper_$(BOARD).v

# These rules impact 
PROJECT_IP=
PROJECT_HDL=hdl/$(PROJECT).v $(BOARD_HDL) $(patsubst %, $(RIFFA_PATH)/%,$(RIFFA_HDL))
PROJECT_CONSTR=constr/$(PROJECT).sdc
PROJECT_FILE=prj/$(PROJECT).qsf prj/$(PROJECT).qpf
PROJECT_FILES=$(PROJECT_IP) $(PROJECT_CONSTR) $(PROJECT_QSRCS) $(PROJECT_HDL)

.PHONY:$(PROJECT)
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

all:$(PROJECT)
clean:
	rm -rf ip/.qsys_edit ip/*~
	rm -rf prj/db prj/incremental_db prj/*txt prj/*.sopcinfo prj/*.qws prj/*~
	rm -rf bit/*.done bit/*.smsg bit/*.rpt bit/*.summary bit/*.sld bit/*.pin bit/*.jdi bit/*~
	rm -rf hdl/*~
	rm -rf constr/*~
	rm -rf *~

clean-bit:
	rm -rf bit/*.sof
