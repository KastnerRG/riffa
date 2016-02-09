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
// ----------------------------------------------------------------------
// Filename:            Filename: widths.vh
// Version:             Version: 1.0
// Verilog Standard:    Verilog-2005
// Description: This header file contains several convenient widths that
// are used in the engine interface and PCIe TLP packets.
// Author: Dustin Richmond (@darichmond) 
// ----------------------------------------------------------------------
`ifndef __WIDTHS_VH
`define __WIDTHS_VH 1

`define LEN_W 10
`define TD_W 1

`define BARDECODE_W 8
`define OFFSET_W 4
`define EP_W 1
`define TC_W 3
`define TYPE_W 5
`define ATTR_W 3
`define FMT_W 3
`define FBE_W 4
`define LBE_W 4
`define TAG_W 8
`define ADDR_W 64
`define REQID_W 16
`define CPLID_W 16
`define BYTECNT_W 12
`define STAT_W 3
`define LOWADDR_W 7

`define EXT_TYPE_W 3

`define LINKWIDTH_W 6
`define LINKRATE_W 4
`define MAXREAD_W 3
`define MAXPAYLOAD_W 3

`define PCIE_CONFIGURATION_REGISTER_WIDTH 16
`define PCIE_BUS_ID_WIDTH 8
`define PCIE_DEVICE_ID_WIDTH 5
`define PCIE_FUNCTION_ID_WIDTH 3

`endif

