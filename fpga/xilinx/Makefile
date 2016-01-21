VENDOR:=xilinx
ULTRASCALE:=NetFPGA adm7V3 kcu105 vc709 vcu108
CLASSIC:=ac701 kc705 vc707 zc706
CURRENT_PATH := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
RIFFA_PATH=$(CURRENT_PATH)/../riffa_hdl

all: $(VENDOR)

classic:$(CLASSIC)
ultrascale:$(ULTRASCALE)

$(VENDOR): $(CLASSIC) $(ULTRASCALE)
$(CLASSIC) $(ULTRASCALE)::
	$(MAKE) -C $@ $(MAKECMDGOALS) RIFFA_PATH=$(RIFFA_PATH)

.PHONY:clean $(SUBDIRS)
clean: $(SUBDIRS) $(VENDOR)
	rm -rf *.log *.jou .Xil *~
