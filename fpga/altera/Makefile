VENDOR:=altera
CLASSIC:=de5 de4 de2i c4dev
CURRENT_PATH := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
RIFFA_PATH=$(CURRENT_PATH)/../riffa_hdl

all: $(VENDOR)

classic:$(CLASSIC)

$(VENDOR): $(CLASSIC)
$(CLASSIC)::
	$(MAKE) -C $@ $(MAKECMDGOALS) RIFFA_PATH=$(RIFFA_PATH)

.PHONY:clean $(SUBDIRS)
clean: $(VENDOR)
	rm -rf *~ .qsys_edit
