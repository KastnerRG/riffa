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
// Filename:            trellis.vh
// Version:             1.0
// Verilog Standard:    Verilog-2001
// Description: The reset_controller module will safely reset a single stage
// pipeline without using an asychronous reset (bleh). It is intended for use in
// the TX engines, where it will control the output stage of the engine, and
// provide a gracefull end-of-packet reset
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`define S_RC_IDLE 3'b001
`define S_RC_WAIT 3'b010
`define S_RC_ACTIVE 3'b100

`include "trellis.vh"
module reset_controller
    #(parameter C_RST_COUNT = 10)
    (
     input  CLK,
     input  RST_IN,

     output DONE_RST,
     output WAITING_RESET,
     output RST_OUT,
     
     input  SIGNAL_RST,
     input  WAIT_RST,
     input  NEXT_CYC_RST);

    localparam C_CLOG2_RST_COUNT = clog2s(C_RST_COUNT);
    localparam C_CEIL2_RST_COUNT = 1 << C_CLOG2_RST_COUNT;

    reg [2:0] _rState,rState;

    wire [C_CLOG2_RST_COUNT:0] wRstCount;

    assign DONE_RST = rState[0];
    assign WAITING_RESET = rState[1] & NEXT_CYC_RST;
    assign RST_OUT = rState[2];

    counter
        #(// Parameters
          .C_MAX_VALUE          (C_CEIL2_RST_COUNT),
          .C_SAT_VALUE          (C_CEIL2_RST_COUNT),
          .C_RST_VALUE          (C_CEIL2_RST_COUNT - C_RST_COUNT)
          /*AUTOINSTPARAM*/)
    rst_counter
        (// Outputs
         .VALUE                 (wRstCount),
         // Inputs
         .ENABLE                (1'b1),
         .RST_IN                (~rState[2] | RST_IN),
         /*AUTOINST*/
         // Inputs
         .CLK                   (CLK));

    always @(posedge CLK) begin
        if(RST_IN) begin
            rState <= `S_RC_ACTIVE;
        end else begin
            rState <= _rState;
        end
    end
    
    always @(*) begin
        _rState = rState;
        case(rState)
            `S_RC_IDLE:begin
                if(SIGNAL_RST & WAIT_RST) begin
                    _rState = `S_RC_WAIT;
                end else if(SIGNAL_RST) begin
                    _rState = `S_RC_ACTIVE;
                end
            end
            `S_RC_WAIT:begin
                if(NEXT_CYC_RST) begin
                    _rState = `S_RC_ACTIVE;
                end
            end
            `S_RC_ACTIVE:begin
                if(wRstCount[C_CLOG2_RST_COUNT] & ~SIGNAL_RST) begin
                    _rState = `S_RC_IDLE;
                end
            end
            default: _rState = rState;
        endcase
    end
endmodule
