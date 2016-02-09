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
// Filename:			rotate.v
// Version:				1.00
// Verilog Standard:	Verilog-2001
// Description:			A simple module to perform to rotate the input data
// Author:				Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
`include "functions.vh"
module rotate
    #(
      parameter C_DIRECTION = "LEFT",
      parameter C_WIDTH = 4
      )
    (
     input [C_WIDTH-1:0]        WR_DATA,
     input [clog2s(C_WIDTH)-1:0] WR_SHIFTAMT,
     output [C_WIDTH-1:0]       RD_DATA
     );

    wire [2*C_WIDTH-1:0]        wPreShiftR;
    wire [2*C_WIDTH-1:0]        wPreShiftL;

    wire [2*C_WIDTH-1:0]        wShiftR;
    wire [2*C_WIDTH-1:0]        wShiftL;

    assign wPreShiftL = {WR_DATA,WR_DATA};
    assign wPreShiftR = {WR_DATA,WR_DATA};

    assign wShiftL = wPreShiftL << WR_SHIFTAMT;
    assign wShiftR = wPreShiftR >> WR_SHIFTAMT;

    generate
        if(C_DIRECTION == "LEFT") begin
            assign RD_DATA = wShiftL[2*C_WIDTH-1:C_WIDTH];
        end else if (C_DIRECTION == "RIGHT") begin
            assign RD_DATA = wShiftR[C_WIDTH-1:0];
        end
    endgenerate
endmodule
