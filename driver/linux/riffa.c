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
 * Description: Linux PCIe communications API for RIFFA.
 * Author: Matthew Jacobsen
 * History: @mattj: Initial release. Version 2.0.
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include "riffa.h"

struct fpga_t
{
	int fd;
	int id;
};

fpga_t * fpga_open(int id) 
{
	fpga_t * fpga;

	// Allocate space for the fpga_dev
	fpga = (fpga_t *)malloc(sizeof(fpga_t));
	if (fpga == NULL)
		return NULL;
	fpga->id = id;	

	// Open the device file.
	fpga->fd = open("/dev/" DEVICE_NAME, O_RDWR | O_SYNC);
	if (fpga->fd < 0) {
		free(fpga); 
		return NULL;
	}
	
	return fpga;
}

void fpga_close(fpga_t * fpga) 
{
	// Close the device file.
	close(fpga->fd);
	free(fpga);
}

int fpga_send(fpga_t * fpga, int chnl, void * data, int len, int destoff, 
	int last, long long timeout)
{
	fpga_chnl_io io;

	io.id = fpga->id;
	io.chnl = chnl;
	io.len = len;
	io.offset = destoff;
	io.last = last;
	io.timeout = timeout;
	io.data = (char *)data;

	return ioctl(fpga->fd, IOCTL_SEND, &io);
}

int fpga_recv(fpga_t * fpga, int chnl, void * data, int len, long long timeout)
{
	fpga_chnl_io io;

	io.id = fpga->id;
	io.chnl = chnl;
	io.len = len;
	io.timeout = timeout;
	io.data = (char *)data;

	return ioctl(fpga->fd, IOCTL_RECV, &io);
}

void fpga_reset(fpga_t * fpga)
{
	ioctl(fpga->fd, IOCTL_RESET, fpga->id);
}

int fpga_list(fpga_info_list * list) {
	int fd;
	int rc;

	fd = open("/dev/" DEVICE_NAME, O_RDWR | O_SYNC);
	if (fd < 0)
		return fd;
	rc = ioctl(fd, IOCTL_LIST, list);
	close(fd);
	return rc;
}



