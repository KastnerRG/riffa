RELEASE_VENDOR_PATH=$(RELEASE_SRC_PATH)/fpga/$(VENDOR)

destination: $(RELEASE_VENDOR_PATH)
$(RELEASE_VENDOR_PATH): check-release-src
	mkdir $@

release: destination $(BOARDS)
