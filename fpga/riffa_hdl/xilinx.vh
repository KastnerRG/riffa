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
// Filename:            xilinx.vh
// Version:             1.0
// Verilog Standard:    Verilog-2001
// Description:         The xilinx.vh file is a header file that defines
// various Xilinx-specific primitives.
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`ifndef __XILINX_VH
`define __XILINX_VH 1

`define SIG_XIL_TX_TUSER_W 4
`define SIG_XIL_RX_TUSER_W 22

`define SIG_FC_SEL_W 3 // Xilinx specific

`define SIG_FC_SEL_RX_BUF_AVAIL_V 3'b000
`define SIG_FC_SEL_RX_MAXALLOC_V 3'b001
`define SIG_FC_SEL_RX_CONSUMED_V 3'b010
`define SIG_FC_SEL_TX_CRED_AVAIL_V 3'b100
`define SIG_FC_SEL_TX_MAXALLOC_V 3'b101
`define SIG_FC_SEL_TX_CONSUMED_V 3'b110

`define CFG_COMMAND_BUSMSTR_R 2
`define CFG_LSTATUS_LWIDTH_R 9:4
`define CFG_LSTATUS_LRATE_R 3:0
`define CFG_DCOMMAND_MAXREQ_R 14:12
`define CFG_DCOMMAND_MAXPAY_R 7:5
`define CFG_LCOMMAND_RCB_R 3

`endif
