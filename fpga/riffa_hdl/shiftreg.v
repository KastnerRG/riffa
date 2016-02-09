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
 Filename: shiftreg.v
 Version: 1.0
 Verilog Standard: Verilog-2001

 Description: A simple parameterized shift register. 
 
 Notes: Any modifications to this file should meet the conditions set
 forth in the "Trellis Style Guide"
 
 Author: Dustin Richmond (@darichmond) 
 Co-Authors:
 */
`timescale 1ns/1ns
module shiftreg
    #(parameter C_DEPTH=10,
      parameter C_WIDTH=32,
      parameter C_VALUE=0
      )
    (input                            CLK,
     input                            RST_IN,
     input [C_WIDTH-1:0]              WR_DATA,
     output [(C_DEPTH+1)*C_WIDTH-1:0] RD_DATA);

    // Start Flag Shift Register. Data enables are derived from the 
    // taps on this shift register.

    wire [(C_DEPTH+1)*C_WIDTH-1:0]    wDataShift;
    reg [C_WIDTH-1:0]                 rDataShift[C_DEPTH:0];

    assign wDataShift[(C_WIDTH*0)+:C_WIDTH] = WR_DATA;
    always @(posedge CLK) begin
        rDataShift[0] <= WR_DATA;
    end
    
    genvar                                     i;
    generate
        for (i = 1 ; i <= C_DEPTH; i = i + 1) begin : gen_sr_registers
            assign wDataShift[(C_WIDTH*i)+:C_WIDTH] = rDataShift[i-1];
            always @(posedge CLK) begin
                if(RST_IN)
                    rDataShift[i] <= C_VALUE;
                else
                    rDataShift[i] <= rDataShift[i-1];
            end
        end
    endgenerate
    assign RD_DATA = wDataShift;
    
endmodule
