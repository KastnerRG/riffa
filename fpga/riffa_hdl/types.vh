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
// Filename:            Filename: types.vh
// Version:             Version: 1.0
// Verilog Standard:    Verilog-2005
// Description: This header file contains several convenient types that
// are used in the engine interface.
// Author: Dustin Richmond (@darichmond) 
// ----------------------------------------------------------------------
`ifndef __TYPES_VH
`define __TYPES_VH 1

`define TRLS_REQ_RD `EXT_TYPE_W'b000 
`define TRLS_REQ_WR `EXT_TYPE_W'b001 
`define TRLS_CPL_ND `EXT_TYPE_W'b010
`define TRLS_CPL_WD `EXT_TYPE_W'b011
`define TRLS_MSG_ND `EXT_TYPE_W'b100
`define TRLS_MSG_WD `EXT_TYPE_W'b101

`define TRLS_TYPE_PAY_I 0 // Payload Bit Index. If 1, packet has a payload, else 0
`define TRLS_TYPE_CPL_I 1 // Completion Bit Index. If 1, packet is a Completion
`define TRLS_TYPE_MSG_I 2 // Message Bit Index. If 1, packet is a message

`endif
