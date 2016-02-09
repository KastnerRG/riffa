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
// Filename:			mux.v
// Version:				1.00.a
// Verilog Standard:	Verilog-2001
// Description:			A simple multiplexer
// Author:				Dustin Richmond (@darichmond)
// TODO:                Remove C_CLOG_NUM_INPUTS
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
`include "functions.vh"
module mux
    #(
      parameter C_NUM_INPUTS = 4,
      parameter C_CLOG_NUM_INPUTS = 2,
      parameter C_WIDTH = 32,
      parameter C_MUX_TYPE = "SELECT"
      )
    (
     input [(C_NUM_INPUTS)*C_WIDTH-1:0] MUX_INPUTS,
     input [C_CLOG_NUM_INPUTS-1:0]      MUX_SELECT,
     output [C_WIDTH-1:0]               MUX_OUTPUT
     );
    generate
        if(C_MUX_TYPE == "SELECT") begin
            mux_select
                #(/*AUTOINSTPARAM*/
                  // Parameters
                  .C_NUM_INPUTS             (C_NUM_INPUTS),
                  .C_CLOG_NUM_INPUTS        (C_CLOG_NUM_INPUTS),
                  .C_WIDTH                  (C_WIDTH))
            mux_select_inst
                (/*AUTOINST*/
                 // Outputs
                 .MUX_OUTPUT            (MUX_OUTPUT[C_WIDTH-1:0]),
                 // Inputs
                 .MUX_INPUTS            (MUX_INPUTS[(C_NUM_INPUTS)*C_WIDTH-1:0]),
                 .MUX_SELECT            (MUX_SELECT[C_CLOG_NUM_INPUTS-1:0]));
        end else if (C_MUX_TYPE == "SHIFT") begin
            mux_shift
                #(/*AUTOINSTPARAM*/
                  // Parameters
                  .C_NUM_INPUTS             (C_NUM_INPUTS),
                  .C_CLOG_NUM_INPUTS        (C_CLOG_NUM_INPUTS),
                  .C_WIDTH                  (C_WIDTH))
            mux_shift_inst
                (/*AUTOINST*/
                 // Outputs
                 .MUX_OUTPUT            (MUX_OUTPUT[C_WIDTH-1:0]),
                 // Inputs
                 .MUX_INPUTS            (MUX_INPUTS[(C_NUM_INPUTS)*C_WIDTH-1:0]),
                 .MUX_SELECT            (MUX_SELECT[C_CLOG_NUM_INPUTS-1:0]));
        end
    endgenerate
endmodule

module mux_select
    #(
      parameter C_NUM_INPUTS = 4,
      parameter C_CLOG_NUM_INPUTS = 2,
      parameter C_WIDTH = 32
      )
    (
     input [(C_NUM_INPUTS)*C_WIDTH-1:0] MUX_INPUTS,
     input [C_CLOG_NUM_INPUTS-1:0]      MUX_SELECT,
     output [C_WIDTH-1:0]               MUX_OUTPUT
     );
    genvar                              i;
    wire [C_WIDTH-1:0]                  wMuxInputs[C_NUM_INPUTS-1:0];
    assign MUX_OUTPUT = wMuxInputs[MUX_SELECT];
    generate
        for (i = 0; i < C_NUM_INPUTS ; i = i + 1) begin : gen_muxInputs_array
            assign wMuxInputs[i] = MUX_INPUTS[i*C_WIDTH +: C_WIDTH];
        end
    endgenerate
endmodule

module mux_shift
    #(
      parameter C_NUM_INPUTS = 4,
      parameter C_CLOG_NUM_INPUTS = 2,
      parameter C_WIDTH = 32
      )
    (
     input [(C_NUM_INPUTS)*C_WIDTH-1:0] MUX_INPUTS,
     input [C_CLOG_NUM_INPUTS-1:0]      MUX_SELECT,
     output [C_WIDTH-1:0]               MUX_OUTPUT
     );
    genvar                              i;
    wire [C_WIDTH*C_NUM_INPUTS-1:0]     wMuxInputs;
    assign wMuxInputs = MUX_INPUTS >> MUX_SELECT;   
    assign MUX_OUTPUT = wMuxInputs[C_WIDTH-1:0];
endmodule
