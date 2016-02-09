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
//----------------------------------------------------------------------------
// Filename:            trellis.vh
// Version:             1.0
// Verilog Standard:    Verilog-2001
// Description:         The trellis.vh file is a header file with many interface
// width definitions for the Trellis stack
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`ifndef __TRELLIS_VH
`define __TRELLIS_VH 1
`include "widths.vh"
`include "types.vh"
`include "functions.vh"

// PCIe Signals
`define SIG_BARDECODE_W `BARDECODE_W
`define SIG_OFFSET_W `OFFSET_W
`define SIG_TC_W `TC_W
`define SIG_ATTR_W `ATTR_W
`define SIG_LEN_W `LEN_W
`define SIG_TD_W `TD_W
`define SIG_TYPE_W `EXT_TYPE_W
`define SIG_FMT_W `FMT_W
`define SIG_FBE_W `FBE_W
`define SIG_LBE_W `LBE_W
`define SIG_TAG_W `TAG_W
`define SIG_REQID_W `REQID_W
`define SIG_ADDR_W `ADDR_W
`define SIG_BYTECNT_W `BYTECNT_W
`define SIG_STAT_W `STAT_W
`define SIG_CPLID_W `CPLID_W
`define SIG_LOWADDR_W `LOWADDR_W

`define SIG_CFGREG_W `PCIE_CONFIGURATION_REGISTER_WIDTH
`define SIG_BUSID_W `PCIE_BUS_ID_WIDTH
`define SIG_DEVID_W `PCIE_DEVICE_ID_WIDTH // Device ID Width
`define SIG_FNID_W `PCIE_FUNCTION_ID_WIDTH // Function Number

`define SIG_LINKWIDTH_W `LINKWIDTH_W
`define SIG_LINKRATE_W `LINKRATE_W
`define SIG_MAXREAD_W `MAXREAD_W
`define SIG_MAXPAYLOAD_W `MAXPAYLOAD_W

`define SIG_FC_CPLD_W 12
`define SIG_FC_CPLH_W 8

// The maximum number of alignment blanks that can be inserted in a packet is 7
`define SIG_NONPAY_W 4
`define SIG_PACKETLEN_W (clog2s(4096/4) + `SIG_NONPAY_W + 1)
`define SIG_ALIGN_W 3
`define SIG_HDRLEN_W 3
`define SIG_MAXHDR_W 128

`endif
