# Check that given variables are set and all have non-empty values,
# die with an error otherwise.
#
# Params:
#   1. Variable name(s) to test.
#   2. (optional) Error message to print.
check_defined = \
    $(foreach 1,$1,$(__check_defined))
__check_defined = \
    $(if $(value $1),, \
      $(error Undefined $1$(if $(value 2), ($(strip $2)))))

check-release:
	$(call check_defined, RELEASE_PATH)

check-release-src:
	$(call check_defined, RELEASE_SRC_PATH)

check-release-doc:
	$(call check_defined, RELEASE_DOC_PATH)

check-root:
	$(call check_defined, RIFFA_ROOT_PATH)

check-hdl:
	$(call check_defined, RIFFA_HDL_PATH)



