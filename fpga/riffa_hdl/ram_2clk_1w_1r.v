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
// Filename:            ram_2clk_1w_1r.v
// Version:             1.00.a
// Verilog Standard:    Verilog-2001
// Description:         An inferrable RAM module. Dual clocks, 1 write port, 1 
//                      read port. In Xilinx designs, specify RAM_STYLE="BLOCK" 
//                      to use BRAM memory or RAM_STYLE="DISTRIBUTED" to use 
//                      LUT memory.
// Author:              Matt Jacobsen
// History:             @mattj: Version 2.0
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
`include "functions.vh"
module ram_2clk_1w_1r 
    #(
      parameter C_RAM_WIDTH = 32,
      parameter C_RAM_DEPTH = 1024
      )
    (
     input                           CLKA,
     input                           CLKB,
     input                           WEA,
     input [clog2s(C_RAM_DEPTH)-1:0] ADDRA,
     input [clog2s(C_RAM_DEPTH)-1:0] ADDRB,
     input [C_RAM_WIDTH-1:0]         DINA,
     output [C_RAM_WIDTH-1:0]        DOUTB
     );
    //Local parameters
    localparam C_RAM_ADDR_BITS = clog2s(C_RAM_DEPTH);
    reg [C_RAM_WIDTH-1:0]            rRAM [C_RAM_DEPTH-1:0];
    reg [C_RAM_WIDTH-1:0]            rDout;   
    assign DOUTB = rDout;
    always @(posedge CLKA) begin
        if (WEA)
            rRAM[ADDRA] <= #1 DINA;
    end
    always @(posedge CLKB) begin
        rDout <= #1 rRAM[ADDRB];
    end
endmodule
