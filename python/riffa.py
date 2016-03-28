# ----------------------------------------------------------------------
# Copyright (c) 2016, The Regents of the University of California All
# rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
# 
#     * Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials provided
#       with the distribution.
# 
#     * Neither the name of The Regents of the University of California
#       nor the names of its contributors may be used to endorse or
#       promote products derived from this software without specific
#       prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL REGENTS OF THE
# UNIVERSITY OF CALIFORNIA BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
# TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
# USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
# ----------------------------------------------------------------------
# Filename: riffa.py
# Version: 2.0
# Description: Python module for RIFFA.
# Author: Matthew Jacobsen
# History: @mattj: Initial release. Version 2.0.

import ctypes
import platform

NUM_FPGAS = 5

if platform.system() == "Linux":
	libriffa = ctypes.CDLL("libriffa.so.1")
else:
	libriffa = ctypes.CDLL("riffa.dll")

libriffa.fpga_recv.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_int, ctypes.c_longlong]
libriffa.fpga_send.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_longlong]

class FpgaInfoList(ctypes.Structure):
	_fields_ = [("num_fpgas", ctypes.c_int),
				("id", ctypes.c_int * NUM_FPGAS),
				("num_chnls", ctypes.c_int * NUM_FPGAS),
				("name", (ctypes.c_char * 16) * NUM_FPGAS),
				("vendor_id", ctypes.c_int * NUM_FPGAS),
				("device_id", ctypes.c_int * NUM_FPGAS)]
	def __str__(self):
		strl = []
		strl.append("num fpgas: %d" % self.num_fpgas) 
		for i in range(self.num_fpgas):
			strl.append("fpga (%d) id: %d" % (i, self.id[i]))
			strl.append("fpga (%d) name: %s" % (i, self.name[i].value))
			strl.append("fpga (%d) num_chnls: %d" % (i, self.num_chnls[i]))
			strl.append("fpga (%d) vendor_id: %x" % (i, self.vendor_id[i]))
			strl.append("fpga (%d) device_id: %x" % (i, self.device_id[i]))
		return '\n'.join(strl)


# Populates and returns a FpgaInfoList with all FPGAs registered in the system.
# Returns None on error.
def fpga_list():
	s = FpgaInfoList()
	rc = libriffa.fpga_list(ctypes.byref(s))
	if (rc == 0):
		return s
	else:
		return None


# Initializes the FPGA specified by id. On success, returns a descriptor. On 
# error, returns None. Each FPGA must be opened before any channels can be 
# accessed. Once opened, any number of threads can use the descriptor.
def fpga_open(id):
	return libriffa.fpga_open(id)

# Cleans up memory/resources for the FPGA specified by the fd descriptor.
def fpga_close(fd):
	libriffa.fpga_close(fd)


# Sends length words (4 byte words) from data to FPGA channel chnl using the 
# fd descriptor. The data parameter must implement the buffer protocol 
# interface so that a memoryview can be created from it. The FPGA channel will 
# be sent length, destoff, and last. If last is True, the channel should 
# interpret the end of this send as the end of a transaction. If last is False, 
# the channel should wait for additional sends before the end of the 
# transaction. If timeout is non-zero, this call will send data and wait up to 
# timeout ms for the FPGA to respond (between packets) before timing out. If 
# timeout is zero, this call may block indefinitely. Multiple threads sending 
# on the same channel may result in corrupt data or error. This function is 
# thread safe across channels. Returns the number of words sent. 
def fpga_send(fd, chnl, data, length, destoff, last, timeout):
	ptr = 0
	datalen = 0
	ilast = 0
	if last:
		ilast = 1
	if length > 0:
		
		# Pre 3.0 workaround for array types (they don't implement the 3.0 
		# buffer protocol interface despite documentation to the contrary).
		if hasattr(data, 'buffer_info') and hasattr(data, 'itemsize'):
			(ptr, datalen) = data.buffer_info()
			datalen = datalen * data.itemsize
		else:
			m = memoryview(data)
			obj = ctypes.py_object(m)
			a = ctypes.c_void_p()
			l = ctypes.c_size_t()
			ctypes.pythonapi.PyObject_AsReadBuffer(obj, ctypes.byref(a), ctypes.byref(l))
			ptr = a.value
			datalen = l.value
		if length > (datalen/4):
			length = (datalen/4)
	return libriffa.fpga_send(fd, chnl, ptr, length, destoff, ilast, timeout)


# Receives data from the FPGA channel chnl to the supplied data object, using 
# the fd descriptor. The supplied data object must implement the buffer 
# protocol interface so that a memoryview can be created from it. The FPGA 
# channel can send any amount of data, so the data object's backing array must
# be large enough to accommodate. The FPGA channel will specify an offset which 
# will determine where in the data object's backing array the data will start
# being written. If the amount of data (plus offset) exceed the size of the 
# data object's backing array, then additional data will be discarded. If 
# timeout is non-zero, this call will wait up to timeout ms for the FPGA to 
# respond (between packets) before timing out. If timeout is zero, this call 
# may block indefinitely. Multiple threads receiving on the same channel may 
# result in corrupt data or error. This function is thread safe across channels.
# Returns the number of words received to the data array. 
def fpga_recv(fd, chnl, data, timeout):
	ptr = 0
	datalen = 0
	if data is not None:
		# Pre 3.0 workaround for array types (they don't implement the 3.0 
		# buffer protocol interface despite documentation to the contrary).
		if hasattr(data, 'buffer_info') and hasattr(data, 'itemsize'):
			(ptr, datalen) = data.buffer_info()
			datalen = datalen * data.itemsize
		else:
			m = memoryview(data)
			obj = ctypes.py_object(m)
			a = ctypes.c_void_p()
			l = ctypes.c_size_t()
			ctypes.pythonapi.PyObject_AsReadBuffer(obj, ctypes.byref(a), ctypes.byref(l))
			ptr = a.value
			datalen = l.value
	return libriffa.fpga_recv(fd, chnl, ptr, datalen//4, timeout)


# Resets the state of the FPGA and all transfers across all channels. This is
# meant to be used as an alternative to rebooting if an error occurs while 
# sending/receiving. Calling this function while other threads are sending or
# receiving will result in unexpected behavior.
def fpga_reset(fd):
	libriffa.fpga_reset(fd)


