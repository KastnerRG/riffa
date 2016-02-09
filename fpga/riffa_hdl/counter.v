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
// Filename:			counter.v
// Version:				1.00.a
// Verilog Standard:	Verilog-2001
// Description: A simple up-counter. The maximum value is the largest expected
// value. The counter will not pass the SAT_VALUE. If the SAT_VALUE > MAX_VALUE,
// the counter will roll over and never stop. On RST_IN, the counter
// synchronously resets to the RST_VALUE
// Author:				Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
`include "functions.vh"
module counter
    #(parameter C_MAX_VALUE = 10,
      parameter C_SAT_VALUE = 10,
      parameter C_RST_VALUE = 0)
    (
     input                              CLK,
     input                              RST_IN,

     input                              ENABLE,
     output [clog2s(C_MAX_VALUE+1)-1:0] VALUE
     );
    wire                                wEnable;
    reg [clog2s(C_MAX_VALUE+1)-1:0]     wCtrValue;
    reg [clog2s(C_MAX_VALUE+1)-1:0]     rCtrValue;
    /* verilator lint_off WIDTH */
    assign wEnable = ENABLE & (C_SAT_VALUE > rCtrValue);
    /* verilator lint_on WIDTH */
    assign VALUE = rCtrValue;
    always @(posedge CLK) begin
        if(RST_IN) begin
            rCtrValue <= C_RST_VALUE[clog2s(C_MAX_VALUE+1)-1:0];
        end else if(wEnable) begin
            rCtrValue <= rCtrValue + 1;
        end
    end
endmodule
