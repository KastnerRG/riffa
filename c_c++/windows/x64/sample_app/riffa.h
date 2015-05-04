/*******************************************************************************
 * This software is Copyright © 2012 The Regents of the University of
 * California. All Rights Reserved.
 *
 * Permission to copy, modify, and distribute this software and its
 * documentation for educational, research and non-profit purposes, without fee,
 * and without a written agreement is hereby granted, provided that the above
 * copyright notice, this paragraph and the following three paragraphs appear in
 * all copies.
 *
 * Permission to make commercial use of this software may be obtained by
 * contacting:
 * Technology Transfer Office
 * 9500 Gilman Drive, Mail Code 0910
 * University of California
 * La Jolla, CA 92093-0910
 * (858) 534-5815
 * invent@ucsd.edu
 *
 * This software program and documentation are copyrighted by The Regents of the
 * University of California. The software program and documentation are supplied
 * "as is", without any accompanying services from The Regents. The Regents does
 * not warrant that the operation of the program will be uninterrupted or error-
 * free. The end-user understands that the program was developed for research
 * purposes and is advised not to rely exclusively on the program for any
 * reason.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO
 * ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING
 * OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE UNIVERSITY OF CALIFORNIA HAS BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. THE UNIVERSITY OF
 * CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * THE SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS" BASIS,
 * AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATIONS TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR
 * MODIFICATIONS.
 */

/*
 * Filename: riffa.h
 * Version: 2.0
 * Description: Windows PCIe communications API for RIFFA.
 * Author: Matthew Jacobsen
 * History: @mattj: Initial release. Version 2.0.
 */

#ifndef RIFFA_H
#define RIFFA_H

// Must RIFFA_EXPORTS *only* when building the DLL.
#ifdef RIFFA_EXPORTS
  #define RIFFAAPI __declspec(dllexport)
#else
  #define RIFFAAPI __declspec(dllimport)
#endif

// Define calling convention in one place, for convenience.
#define RIFFACALL __cdecl

#ifdef __cplusplus
extern "C" {
#endif

// Maximum number of RIFFA FPGAs
#define RIFFA_MAX_NUM_FPGAS (5)

// Holds FPGA information for installed RIFFA FPGAs
struct fpga_info_list
{
	int num_fpgas;
	int id[RIFFA_MAX_NUM_FPGAS];
	int num_chnls[RIFFA_MAX_NUM_FPGAS];
	char name[RIFFA_MAX_NUM_FPGAS][16];
	int vendor_id[RIFFA_MAX_NUM_FPGAS];
	int device_id[RIFFA_MAX_NUM_FPGAS];
};
typedef struct fpga_info_list fpga_info_list;

// Represents the FPGA device
struct fpga_t;
typedef struct fpga_t fpga_t;

/**
 * Populates the fpga_info_list pointer with all FPGAs registered in the system.
 * Returns 0 on success, non-zero on error.
 */
RIFFAAPI int RIFFACALL fpga_list(fpga_info_list * list);

/**
 * Initializes the FPGA specified by id. On success, returns a pointer to a
 * fpga_t struct. On error, returns NULL. Each FPGA must be opened before any
 * channels can be accessed. Once opened, any number of threads can use the
 * fpga_t struct.
 */
RIFFAAPI fpga_t * RIFFACALL fpga_open(int id);

/**
 * Cleans up memory/resources for the FPGA specified by the fd descriptor.
 */
RIFFAAPI void RIFFACALL fpga_close(fpga_t * fpga);

/**
 * Sends len words (4 byte words) from data to FPGA channel chnl using the
 * fpga_t struct. The FPGA channel will be sent len, destoff, and last. If last
 * is 1, the channel should interpret the end of this send as the end of a
 * transaction. If last is 0, the channel should wait for additional sends
 * before the end of the transaction. If timeout is non-zero, this call will
 * send data and wait up to timeout ms for the FPGA to respond (between
 * packets) before timing out. If timeout is zero, this call may block
 * indefinitely. Multiple threads sending on the same channel may result in
 * corrupt data or error. This function is thread safe across channels.
 * Returns the number of words sent.
 */
RIFFAAPI int RIFFACALL fpga_send(fpga_t * fpga, int chnl, void * data, int len,
	int destoff, int last, long long timeout);

/**
 * Receives data from the FPGA channel chnl to the data pointer, using the
 * fpga_t struct. The FPGA channel can send any amount of data, so the data
 * array should be large enough to accommodate. The len parameter specifies the
 * actual size of the data buffer in words (4 byte words). The FPGA channel will
 * specify an offset which will determine where in the data array the data will
 * start being written. If the amount of data (plus offset) exceed the size of
 * the data array (len), then that data will be discarded. If timeout is
 * non-zero, this call will wait up to timeout ms for the FPGA to respond
 * (between packets) before timing out. If timeout is zero, this call may block
 * indefinitely. Multiple threads receiving on the same channel may result in
 * corrupt data or error. This function is thread safe across channels.
 * Returns the number of words received to the data array.
 */
RIFFAAPI int RIFFACALL fpga_recv(fpga_t * fpga, int chnl, void * data, int len,
	long long timeout);

/**
 * Resets the state of the FPGA and all transfers across all channels. This is
 * meant to be used as an alternative to rebooting if an error occurs while
 * sending/receiving. Calling this function while other threads are sending or
 * receiving will result in unexpected behavior.
 */
RIFFAAPI void RIFFACALL fpga_reset(fpga_t * fpga);

#ifdef __cplusplus
}
#endif

#endif
