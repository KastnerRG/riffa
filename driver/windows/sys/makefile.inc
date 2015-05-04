_LNG=$(LANGUAGE)
_INX=.
STAMP=stampinf -f $@ -a $(_BUILDARCH) -k $(KMDF_VERSION_MAJOR).$(KMDF_VERSION_MINOR)


$(OBJ_PATH)\$(O)\$(INF_NAME).inf: $(_INX)\$(INF_NAME).inx
    copy $(_INX)\$(@B).inx $@
    $(STAMP)
!   if "$(DDKBUILDENV)" == "chk"
        echo RIFFA.DEBUG=" (Debug)" >> $@
!   else    
        echo RIFFA.DEBUG="" >> $@
!   endif    


POST:
    copy $(BASEDIR)\redist\wdf\$(_BUILDARCH)\WdfCoInstaller*.dll $(OBJ_PATH)\$(O)
!   if "$(DDK_TARGET_OS)" == "Win7"
!       if "$(_BUILDARCH)" == "x86"
            inf2cat /driver:$(OBJ_PATH)\$(O) /os:7_x86
!       else 
            inf2cat /driver:$(OBJ_PATH)\$(O) /os:7_x64
!       endif 
!   endif 
    signtool sign /v /ac "$(_INX)\GlobalSign Root CA.crt" /s my /n "University of California, San Diego" /t http://timestamp.verisign.com/scripts/timestamp.dll $(OBJ_PATH)\$(O)\$(INF_NAME).cat
    signtool sign /v /ac "$(_INX)\GlobalSign Root CA.crt" /s my /n "University of California, San Diego" /t http://timestamp.verisign.com/scripts/timestamp.dll $(OBJ_PATH)\$(O)\$(INF_NAME).sys

    