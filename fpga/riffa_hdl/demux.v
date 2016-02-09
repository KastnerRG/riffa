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
// Filename:			demux.v
// Version:				1.00.a
// Verilog Standard:	Verilog-2001
// Description:			A simple demultiplexer
// Author:				Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
`include "functions.vh"
module demux
    #(
      parameter C_OUTPUTS = 12,
      parameter C_WIDTH = 1
      )
    (
     input [C_WIDTH-1:0]            WR_DATA,// Inputs
     input [clog2s(C_OUTPUTS)-1:0]  WR_SEL,// Selector
     output [C_OUTPUTS*C_WIDTH-1:0] RD_DATA// Outputs
     );
    genvar                          i;
    reg [C_OUTPUTS*C_WIDTH-1:0]     _rOut;
    assign RD_DATA = _rOut;
    always @(*) begin
        _rOut = 0;
        _rOut[C_WIDTH*WR_SEL +: C_WIDTH] = WR_DATA;
    end
endmodule
