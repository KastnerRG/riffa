#define WIN9X_COMPAT_SPINLOCK
#include <ntddk.h>
#pragma warning(disable:4201)  // nameless struct/union warning

#include <stdarg.h>
#include <wdf.h>
#include <ntstrsafe.h>
#include <stdlib.h>
#include <stdio.h>

#pragma warning(default:4201)

#include <initguid.h> // required for GUID definitions
#include <wdmguid.h>  // required for WMILIB_CONTEXT


#include "riffa_driver.h"
#include "riffa_private.h"
#include "trace.h"


