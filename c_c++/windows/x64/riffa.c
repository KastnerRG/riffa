// ----------------------------------------------------------------------
// Copyright (c) 2016, The Regents of the University of California All
// rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
// 
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
// 
//     * Redistributions in binary form must reproduce the above
//       copyright notice, this list of conditions and the following
//       disclaimer in the documentation and/or other materials provided
//       with the distribution.
// 
//     * Neither the name of The Regents of the University of California
//       nor the names of its contributors may be used to endorse or
//       promote products derived from this software without specific
//       prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL REGENTS OF THE
// UNIVERSITY OF CALIFORNIA BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
// OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
// TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
// USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
// ----------------------------------------------------------------------
/*
 * Filename: riffa.c
 * Version: 2.0
 * Description: Windows PCIe communications API for RIFFA.
 * Author: Matthew Jacobsen
 * History: @mattj: Initial release. Version 2.0.
 */

#define INITGUID

#include <windows.h>
#include <setupapi.h>
#include <stdio.h>
#include <stdlib.h>
#include "riffa_driver.h"
#include "riffa.h"

// The structure used to hold data transfer information
typedef struct RIFFA_FPGA_CHNL_IO {
	UINT32					Id;
	UINT32					Chnl;
	UINT32					Length;
	UINT32					Offset;
	UINT32					Last;
	UINT64					Timeout;
} RIFFA_FPGA_CHNL_IO, * PRIFFA_FPGA_CHNL_IO;

// Represents the FPGA device
struct fpga_t
{
	HANDLE dev;
	int id;
};

HANDLE get_device(UINT32 index, BOOLEAN overlapped);
DWORD fill_device_info(fpga_info_list * info);

fpga_t * RIFFACALL fpga_open(int id) {
	fpga_t * fpga;

	// Allocate space for the fpga_dev
	fpga = (fpga_t *)malloc(sizeof(fpga_t));
	if (fpga == NULL)
		return NULL;
	fpga->id = id;

	// Open the device handle.
	fpga->dev = get_device(id, TRUE);
	if (fpga->dev == INVALID_HANDLE_VALUE) {
		free(fpga);
		return NULL;
	}

	return fpga;
}

void RIFFACALL fpga_close(fpga_t * fpga) {
	// Validate the device handle
	if (fpga->dev == NULL || fpga->dev == INVALID_HANDLE_VALUE) {
		printf("Invalid fpga_t device handle\n");
		return;
	}

	// Close the device handle.
	CloseHandle(fpga->dev);
	fpga->dev = NULL;

	// Free the fpga_t struct
	free(fpga);
}

int RIFFACALL fpga_send(fpga_t * fpga, int chnl, void * data, int len,
	int destoff, int last, long long timeout) {
	RIFFA_FPGA_CHNL_IO io;
	OVERLAPPED overlapStruct = {0};
	HANDLE evt;
	BOOLEAN status;
	ULONG wordsReturned;

	// Validate the device handle
	if (fpga->dev == NULL || fpga->dev == INVALID_HANDLE_VALUE) {
		printf("Invalid fpga_t device handle\n");
		return 0;
	}

	// Initialize the RIFFA_FPGA_CHNL_IO struct
	io.Id = fpga->id;
	io.Chnl = chnl;
	io.Length = len;
	io.Offset = destoff;
	io.Last = last;
	io.Timeout = timeout;

	// Create a thread specific event to wait on.
	evt = CreateEvent(NULL, TRUE, TRUE, NULL);
	overlapStruct.hEvent = evt;

	// Call IOCTL with IOCTL_RIFFA_SEND
	status = DeviceIoControl(fpga->dev, IOCTL_RIFFA_SEND, (LPVOID)&io,
		sizeof(io), data, (len<<2), &wordsReturned, &overlapStruct);
	if(!status) {
		// Should be the IO Pending error
		if(GetLastError() == ERROR_IO_PENDING) {
			// Wait for the IOCTL to complete and get the return value
			WaitForSingleObject(evt, INFINITE);
			status = GetOverlappedResult(fpga->dev, &overlapStruct,
				&wordsReturned, FALSE);
			if(!status) {
				if (GetLastError() == ERROR_OPERATION_ABORTED)
					printf("Operation timed out or was aborted\n");
				else
					printf("Error in GetOverlappedResult: %d\n", GetLastError());
			}
		}
		else {
			printf("Error in DeviceIoControl: %d\n", GetLastError());
		}
	}
	return wordsReturned;
}

int RIFFACALL fpga_recv(fpga_t * fpga, int chnl, void * data, int len,
	long long timeout) {
	RIFFA_FPGA_CHNL_IO io;
	OVERLAPPED overlapStruct = {0};
	HANDLE evt;
	BOOLEAN status;
	ULONG wordsReturned;

	// Validate the device handle
	if (fpga->dev == NULL || fpga->dev == INVALID_HANDLE_VALUE) {
		printf("Invalid fpga_t device handle\n");
		return 0;
	}

	// Initialize the RIFFA_FPGA_CHNL_IO struct
	io.Id = fpga->id;
	io.Chnl = chnl;
	io.Length = len;
	io.Timeout = timeout;

	// Create a thread specific event to wait on.
	evt = CreateEvent(NULL, TRUE, TRUE, NULL);
	overlapStruct.hEvent = evt;

	// Call IOCTL with IOCTL_RIFFA_RECV
	status = DeviceIoControl(fpga->dev, IOCTL_RIFFA_RECV, (LPVOID)&io,
		sizeof(io), data, (len<<2), &wordsReturned, &overlapStruct);
	if(!status) {
		// Should be the IO Pending error
		if(GetLastError() == ERROR_IO_PENDING) {
			// Wait for the IOCTL to complete and get the return value
			WaitForSingleObject(evt, INFINITE);
			status = GetOverlappedResult(fpga->dev, &overlapStruct,
				&wordsReturned, FALSE);
			if(!status) {
				if (GetLastError() == ERROR_OPERATION_ABORTED)
					printf("Operation timed out or was aborted\n");
				else
					printf("Error in GetOverlappedResult: %d\n", GetLastError());
			}
		}
		else {
			printf("Error in DeviceIoControl: %d\n", GetLastError());
		}
	}
	return wordsReturned;
}

void RIFFACALL fpga_reset(fpga_t * fpga) {
	BOOLEAN status;
	OVERLAPPED overlapStruct = {0};
	ULONG wordsReturned;

	// Validate the device handle
	if (fpga->dev == NULL || fpga->dev == INVALID_HANDLE_VALUE) {
		printf("Invalid fpga_t device handle\n");
		return;
	}

	// Call IOCTL with IOCTL_RIFFA_RESET. Must use the overlapped struct as the
	// device was opened with overlap support.
	status = DeviceIoControl(fpga->dev, IOCTL_RIFFA_RESET, NULL, 0, NULL, 0,
		&wordsReturned, &overlapStruct);
	if(!status) {
		// Should be the IO Pending error
		if(GetLastError() == ERROR_IO_PENDING) {
			// Wait for the IOCTL to complete and get the return value
			status = GetOverlappedResult(fpga->dev, &overlapStruct,
				&wordsReturned, TRUE);
			if(!status)
				printf("Error in GetOverlappedResult: %d\n", GetLastError());
		}
		else {
			printf("Error in DeviceIoControl: %d\n", GetLastError());
		}
	}
}

int RIFFACALL fpga_list(fpga_info_list * list) {
	// Populate the fpga_info_list struct
	list->num_fpgas = 0;
	return (int)fill_device_info(list);
}

/**
 * Returns a handle to the FPGA device specified by index. On failure, returns
 * an INVALID_HANDLE_VALUE. If the value of overlapped is TRUE, the device
 * handle will be opened with flag FILE_FLAG_OVERLAPPED to signal that future
 * operations will use overlapped IO. If the value of overlapped is FALSE, the
 * device handle will be opened with flag FILE_ATTRIBUTE_NORMAL and no
 * overlapped IO can be used with the returned device handle.
 */
HANDLE get_device(UINT32 index, BOOLEAN overlapped) {
    PSP_DEVICE_INTERFACE_DETAIL_DATA devIntfDetail;
    SP_DEVICE_INTERFACE_DATA devIntfData;
    SP_DEVINFO_DATA devInfoData;
    SECURITY_ATTRIBUTES secAttr;
    HDEVINFO devInfo;
    BOOLEAN status = TRUE;
    ULONG size;
    HANDLE dev;
    DWORD flags;

    //  Retreive the device information for all RIFFA devices.
    devInfo = SetupDiGetClassDevs(&GUID_RIFFA_INTERFACE, NULL, NULL,
    	DIGCF_DEVICEINTERFACE | DIGCF_PRESENT);
	if (devInfo == INVALID_HANDLE_VALUE) {
        printf("SetupDiGetClassDevs failed, Error: %d\n", GetLastError());
        return INVALID_HANDLE_VALUE;
	}

    //  Initialize the appropriate data structures for the SetupDi calls.
    devIntfData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
    devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

    //  Get information for specific device.
    status = SetupDiEnumDeviceInterfaces(devInfo, NULL, (LPGUID)&GUID_RIFFA_INTERFACE,
    	index, &devIntfData);
    if (!status) {
        printf("SetupDiEnumDeviceInterfaces failed, Error: %d\n", GetLastError());
		SetupDiDestroyDeviceInfoList(devInfo);
        return INVALID_HANDLE_VALUE;
    }

    // Determine the size required for the devIntfData
    SetupDiGetDeviceInterfaceDetail(devInfo, &devIntfData, NULL, 0, &size, NULL);
    if (GetLastError() != ERROR_INSUFFICIENT_BUFFER) {
        printf("SetupDiGetDeviceInterfaceDetail failed, Error: %d\n", GetLastError());
		SetupDiDestroyDeviceInfoList(devInfo);
        return INVALID_HANDLE_VALUE;
    }

    devIntfDetail = (PSP_DEVICE_INTERFACE_DETAIL_DATA)malloc(size);
    if (!devIntfDetail) {
        printf("Insufficient memory.\n");
		SetupDiDestroyDeviceInfoList(devInfo);
        return INVALID_HANDLE_VALUE;
    }

    // Initialize structure and retrieve data.
    devIntfDetail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);
    status = SetupDiGetDeviceInterfaceDetail(devInfo, &devIntfData, devIntfDetail,
    	size, NULL, &devInfoData);
    if (!status) {
        printf("SetupDiGetDeviceInterfaceDetail failed, Error: %d\n", GetLastError());
        free(devIntfDetail);
		SetupDiDestroyDeviceInfoList(devInfo);
        return INVALID_HANDLE_VALUE;
    }

	// Prepare the security attributes for interitance.
	secAttr.nLength = sizeof(SECURITY_ATTRIBUTES);
	secAttr.bInheritHandle = TRUE;
	secAttr.lpSecurityDescriptor = NULL;

	// Get the device handle itself (finally!)
	flags = (overlapped ? FILE_FLAG_OVERLAPPED : FILE_ATTRIBUTE_NORMAL);
	flags = flags | FILE_FLAG_NO_BUFFERING;
	dev = CreateFile(devIntfDetail->DevicePath, GENERIC_READ|GENERIC_WRITE,
		FILE_SHARE_READ | FILE_SHARE_WRITE, &secAttr, OPEN_EXISTING, flags, NULL);
	if (dev == INVALID_HANDLE_VALUE) {
		printf("CreateFile failed.  Error:%d\n", GetLastError());
        free(devIntfDetail);
		SetupDiDestroyDeviceInfoList(devInfo);
		return INVALID_HANDLE_VALUE;
	}
	free(devIntfDetail);
	SetupDiDestroyDeviceInfoList(devInfo);

    return dev;
}

/**
 * Populates the specified fpga_info_list struct with FPGA information. Returns
 * zero on success. On failure, returns an error code (positive value).
 */
DWORD fill_device_info(fpga_info_list * info) {
    PSP_DEVICE_INTERFACE_DETAIL_DATA devIntfDetail;
    SP_DEVICE_INTERFACE_DATA devIntfData;
    SP_DEVINFO_DATA devInfoData;
    HDEVINFO devInfo;
    BOOLEAN status = TRUE;
    ULONG size;
    UINT32 i;
    ULONG wordsReturned;
    HANDLE dev;

    //  Retreive the device information for all RIFFA devices.
    devInfo = SetupDiGetClassDevs(&GUID_RIFFA_INTERFACE, NULL, NULL,
    	DIGCF_DEVICEINTERFACE | DIGCF_PRESENT);
	if (devInfo == INVALID_HANDLE_VALUE) {
        printf("SetupDiGetClassDevs failed, Error: %d\n", GetLastError());
        return GetLastError();
	}

    //  Initialize the appropriate data structures for the SetupDi calls.
    devIntfData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
    devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

    //  Loop through the device list.
    i = 0;
    while (SetupDiEnumDeviceInterfaces(devInfo, NULL, (LPGUID)&GUID_RIFFA_INTERFACE,
    	i, &devIntfData)) {
        // Determine the size required for the devIntfData
        SetupDiGetDeviceInterfaceDetail(devInfo, &devIntfData, NULL, 0, &size, NULL);
        if (GetLastError() != ERROR_INSUFFICIENT_BUFFER) {
            printf("SetupDiGetDeviceInterfaceDetail failed, Error: %d\n", GetLastError());
			SetupDiDestroyDeviceInfoList(devInfo);
            return GetLastError();
        }

		// Create the SP_DEVICE_INTERFACE_DETAIL_DATA
        devIntfDetail = (PSP_DEVICE_INTERFACE_DETAIL_DATA)malloc(size);
        if (!devIntfDetail) {
            printf("Insufficient memory.\n");
			SetupDiDestroyDeviceInfoList(devInfo);
            return ERROR_NOT_ENOUGH_MEMORY;
        }

		// Initialize structure and retrieve data.
		devIntfDetail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);
		status = SetupDiGetDeviceInterfaceDetail(devInfo, &devIntfData, devIntfDetail,
			size, NULL, &devInfoData);
		if (!status) {
			printf("SetupDiGetDeviceInterfaceDetail failed, Error: %d\n", GetLastError());
	        free(devIntfDetail);
			SetupDiDestroyDeviceInfoList(devInfo);
			return GetLastError();
		}

		// Get the device handle itself (no overlap)
		dev = CreateFile(devIntfDetail->DevicePath, GENERIC_READ|GENERIC_WRITE,
			FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
		if (dev == INVALID_HANDLE_VALUE) {
			printf("CreateFile failed.  Error:%d\n", GetLastError());
	        free(devIntfDetail);
			SetupDiDestroyDeviceInfoList(devInfo);
			return GetLastError();
		}

		// Call IOCTL with IOCTL_RIFFA_LIST
		status = DeviceIoControl(dev, IOCTL_RIFFA_LIST, (LPVOID)&i, sizeof(i),
			(LPVOID)info, sizeof(*info), &wordsReturned, NULL);
		if (!status) {
			printf("Error in DeviceIoControl: %d\n", GetLastError());
	        CloseHandle(dev);
	        free(devIntfDetail);
			SetupDiDestroyDeviceInfoList(devInfo);
			return GetLastError();
		}
		info->num_fpgas = info->num_fpgas + 1;

		// Done with the device
		CloseHandle(dev);
		free(devIntfDetail);

        // Cycle through the available devices.
        i++;
    }

	// Done with the DeviceInfo
	SetupDiDestroyDeviceInfoList(devInfo);

	return 0;
}



