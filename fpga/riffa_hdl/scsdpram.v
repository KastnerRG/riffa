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
 Filename: scsdp.v
 Version: 1.0
 Verilog Standard: Verilog-2001

 Description: A simple, single clock, simple dual port (SCSDP) ram
 
 Notes: Any modifications to this file should meet the conditions set
 forth in the "Trellis Style Guide"
 
 Author: Dustin Richmond (@darichmond) 
 Co-Authors:
 */
`timescale 1ns/1ns
`include "functions.vh"
module scsdpram
    #(
      parameter C_WIDTH = 32,
      parameter C_DEPTH = 1024
      )
    (
     input                       CLK,

     input                       RD1_EN,
     input [clog2s(C_DEPTH)-1:0] RD1_ADDR,
     output [C_WIDTH-1:0]        RD1_DATA,

     input                       WR1_EN,
     input [clog2s(C_DEPTH)-1:0] WR1_ADDR,
     input [C_WIDTH-1:0]         WR1_DATA
     );

    reg [C_WIDTH-1:0]            rMemory [C_DEPTH-1:0];
    reg [C_WIDTH-1:0]            rDataOut;   

    assign RD1_DATA = rDataOut;

    always @(posedge CLK) begin
        if (WR1_EN) begin
            rMemory[WR1_ADDR] <= #1 WR1_DATA;
        end
        if(RD1_EN) begin
            rDataOut <= #1 rMemory[RD1_ADDR];
        end
    end   
endmodule
