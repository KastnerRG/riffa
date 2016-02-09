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
// Filename:            riffa.vh
// Version:             1.0
// Verilog Standard:    Verilog-2001
// Description:         The riffa.vh file is a header file that defines
// various RIFFA-specific primitives.
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`ifndef __RIFFA_VH
`define __RIFFA_VH 1
`include "widths.vh"

// User Interface Signals
`define SIG_CHNL_OFFSET_W 31
`define SIG_CHNL_LENGTH_W 32
`define SIG_CHNL_LAST_W 1

// Engine/Channel interface signals
`define SIG_TXRLEN_W 32
`define SIG_OFFLAST_W 32
`define SIG_LAST_W 1
`define SIG_TXDONELEN_W 32
`define SIG_RXDONELEN_W 32
`define SIG_CORESETTINGS_W 32

// Writable addresses
`define ADDR_SGRX_LEN 4'b0000
`define ADDR_SGRX_ADDRLO 4'b0001
`define ADDR_SGRX_ADDRHI 4'b0010
`define ADDR_RX_LEN 4'b0011
`define ADDR_RX_OFFLAST 4'b0100
`define ADDR_SGTX_LEN 4'b0101
`define ADDR_SGTX_ADDRLO 4'b0110
`define ADDR_SGTX_ADDRHI 4'b0111
// Readable Addresses
`define ADDR_TX_LEN 4'b1000
`define ADDR_TX_OFFLAST 4'b1001
`define ADDR_CORESETTINGS 4'b1010
`define ADDR_INTR_VECTOR_0 4'b1011
`define ADDR_INTR_VECTOR_1 4'b1100
`define ADDR_RX_LEN_XFERD 4'b1101
`define ADDR_TX_LEN_XFERD 4'b1110
`define ADDR_FPGA_NAME 4'b1111

`endif
