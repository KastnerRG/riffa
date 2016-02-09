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
// Filename:			one_hot_mux.v
// Version:				1.00.a
// Verilog Standard:	Verilog-2001
// Description:			A mux module, where the output select is a one-hot bus
// Author:				Dustin Richmond
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
`include "functions.vh"
module one_hot_mux
    #(parameter C_DATA_WIDTH = 1,
      parameter C_SELECT_WIDTH = 2,
      parameter C_AGGREGATE_WIDTH = C_SELECT_WIDTH*C_DATA_WIDTH
      )
    (
     input [C_SELECT_WIDTH-1:0]    ONE_HOT_SELECT,
     input [C_AGGREGATE_WIDTH-1:0] ONE_HOT_INPUTS,
     output [C_DATA_WIDTH-1:0]     ONE_HOT_OUTPUT);

    genvar                         i;

    wire [C_DATA_WIDTH-1:0]        wOneHotInputs[(1<<C_SELECT_WIDTH):1];
    reg [C_DATA_WIDTH-1:0]         _rOneHotOutput;

    assign ONE_HOT_OUTPUT = _rOneHotOutput;
    generate
        for( i = 0 ; i < C_SELECT_WIDTH; i = i + 1 ) begin : gen_input_array
            assign wOneHotInputs[(1<<i)] = ONE_HOT_INPUTS[C_DATA_WIDTH*i +: C_DATA_WIDTH];
        end
        if(C_SELECT_WIDTH == 1) begin
            always @(*) begin
                _rOneHotOutput = wOneHotInputs[1];
            end
        end else if(C_SELECT_WIDTH == 2) begin
            always @(*) begin
                case(ONE_HOT_SELECT)
                    2'b01: _rOneHotOutput = wOneHotInputs[1];
                    2'b10: _rOneHotOutput = wOneHotInputs[2];
                    default:_rOneHotOutput = wOneHotInputs[1];
                endcase // case (ONE_HOT_SELECT)
            end
        end else if( C_SELECT_WIDTH == 4) begin
            always @(*) begin
                case(ONE_HOT_SELECT)
                    4'b0001: _rOneHotOutput = wOneHotInputs[1];
                    4'b0010: _rOneHotOutput = wOneHotInputs[2];
                    4'b0100: _rOneHotOutput = wOneHotInputs[4];
                    4'b1000: _rOneHotOutput = wOneHotInputs[8];
                    default:_rOneHotOutput = wOneHotInputs[1];
                endcase // case (ONE_HOT_SELECT)
            end
        end else if( C_SELECT_WIDTH == 8) begin
            always @(*) begin
                case(ONE_HOT_SELECT)
                    8'b00000001: _rOneHotOutput = wOneHotInputs[1];
                    8'b00000010: _rOneHotOutput = wOneHotInputs[2];
                    8'b00000100: _rOneHotOutput = wOneHotInputs[4];
                    8'b00001000: _rOneHotOutput = wOneHotInputs[8];
                    8'b00010000: _rOneHotOutput = wOneHotInputs[16];
                    8'b00100000: _rOneHotOutput = wOneHotInputs[32];
                    8'b01000000: _rOneHotOutput = wOneHotInputs[64];
                    8'b10000000: _rOneHotOutput = wOneHotInputs[128];
                    default:_rOneHotOutput = wOneHotInputs[1];
                endcase // case (ONE_HOT_SELECT)
            end
        end 
    endgenerate
endmodule
