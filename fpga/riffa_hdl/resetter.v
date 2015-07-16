// ----------------------------------------------------------------------
// Copyright (c) 2015, The Regents of the University of California All
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
// Filename:			resetter.v
// Version:				1.00.a
// Verilog Standard:	Verilog-2001
// Description: A simple reset controller. 
// Author:				Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
`include "functions.vh"
module resetter
    #(parameter C_RST_COUNT = 10,
      parameter C_RST_USE_SHREG = 0)
    (input CLK,
     input RST_IN,
     output RST_OUT);
    localparam C_CLOG2_RST_COUNT = clog2s(C_RST_COUNT);
    localparam C_CEIL2_RST_COUNT = 1 << C_CLOG2_RST_COUNT;

    generate 
        wire [C_RST_COUNT-1:0] wRstShift;
        if(C_RST_USE_SHREG > 0) begin : rst_shreg
            shiftreg
                #(// Parameters
                  .C_DEPTH              (C_RST_COUNT),
                  .C_WIDTH              (1),
                  .C_VALUE              (1'b1))
            rst_shreg
                (// Outputs
                 .RD_DATA               (wRstShift),
                 // Inputs
                 .WR_DATA               (0),
                 /*AUTOINST*/
                 // Inputs
                 .CLK                   (CLK),
                 .RST_IN                (RST_IN));
            assign RST_OUT = wRstShift[C_RST_COUNT-1];
        end else begin : rst_counter // block: rst_shreg
            wire [C_CLOG2_RST_COUNT-1:0] wRstCount;
            counter
                #(// Parameters
                  .C_MAX_VALUE          (C_CEIL2_RST_COUNT - 1),
                  .C_SAT_VALUE          (C_CEIL2_RST_COUNT - 1),
                  .C_RST_VALUE          (C_CEIL2_RST_COUNT - C_RST_COUNT)
                  /*AUTOINSTPARAM*/)
            rst_counter
                (// Outputs
                 .VALUE                 (wRstCount),
                 // Inputs
                 .ENABLE                (1'b1),
                 /*AUTOINST*/
                 // Inputs
                 .CLK                   (CLK),
                 .RST_IN                (RST_IN));
            assign RST_OUT = wRstCount[C_CLOG2_RST_COUNT-1];
        end
    endgenerate
endmodule
