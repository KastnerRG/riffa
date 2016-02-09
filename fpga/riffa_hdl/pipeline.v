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
// Filename:			pipeline.v
// Version:				1.00
// Verilog Standard:	Verilog-2001
// Description: Standard 0-delay pipeline implementation. Takes WR_DATA on
// WR_READY and WR_VALID. RD_DATA is read on RD_READY and
// RD_VALID. C_DEPTH specifies the latency between RD and WR ports
// Author:				Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
module pipeline
    #(
      parameter C_DEPTH = 10,
      parameter C_WIDTH = 10,
      parameter C_USE_MEMORY = 1
      )
    (
     input                CLK,
     input                RST_IN,

     input [C_WIDTH-1:0]  WR_DATA,
     input                WR_DATA_VALID,
     output               WR_DATA_READY,

     output [C_WIDTH-1:0] RD_DATA,
     output               RD_DATA_VALID,
     input                RD_DATA_READY
     );

    generate
        if (C_USE_MEMORY & C_DEPTH > 2) begin
            mem_pipeline
                #(
                  .C_PIPELINE_INPUT     (1),
                  .C_PIPELINE_OUTPUT    (1),
                  /*AUTOINSTPARAM*/
                  // Parameters
                  .C_DEPTH              (C_DEPTH),
                  .C_WIDTH              (C_WIDTH))
            pipeline_inst
                (/*AUTOINST*/
                 // Outputs
                 .WR_DATA_READY         (WR_DATA_READY),
                 .RD_DATA               (RD_DATA[C_WIDTH-1:0]),
                 .RD_DATA_VALID         (RD_DATA_VALID),
                 // Inputs
                 .CLK                   (CLK),
                 .RST_IN                (RST_IN),
                 .WR_DATA               (WR_DATA[C_WIDTH-1:0]),
                 .WR_DATA_VALID         (WR_DATA_VALID),
                 .RD_DATA_READY         (RD_DATA_READY));
            
        end else begin
            reg_pipeline
                #(/*AUTOINSTPARAM*/
                  // Parameters
                  .C_DEPTH              (C_DEPTH),
                  .C_WIDTH              (C_WIDTH))
            pipeline_inst
                (/*AUTOINST*/
                 // Outputs
                 .WR_DATA_READY         (WR_DATA_READY),
                 .RD_DATA               (RD_DATA[C_WIDTH-1:0]),
                 .RD_DATA_VALID         (RD_DATA_VALID),
                 // Inputs
                 .CLK                   (CLK),
                 .RST_IN                (RST_IN),
                 .WR_DATA               (WR_DATA[C_WIDTH-1:0]),
                 .WR_DATA_VALID         (WR_DATA_VALID),
                 .RD_DATA_READY         (RD_DATA_READY));
        end
    endgenerate
endmodule // pipeline

module mem_pipeline
    #(
      parameter C_DEPTH = 10,
      parameter C_WIDTH = 10,
      parameter C_PIPELINE_INPUT = 0,
      parameter C_PIPELINE_OUTPUT = 1
      )
    (
     input                CLK,
     input                RST_IN,

     input [C_WIDTH-1:0]  WR_DATA,
     input                WR_DATA_VALID,
     output               WR_DATA_READY,

     output [C_WIDTH-1:0] RD_DATA,
     output               RD_DATA_VALID,
     input                RD_DATA_READY
     );

    localparam C_INPUT_REGISTERS = C_PIPELINE_INPUT?1:0;
    localparam C_OUTPUT_REGISTERS = C_PIPELINE_OUTPUT?1:0;
    
    wire                  RST;
    
    wire [C_WIDTH-1:0]    wRdData;
    wire                  wRdDataValid;
    wire                  wRdDataReady;

    wire [C_WIDTH-1:0]    wWrData;
    wire                  wWrDataValid;
    wire                  wWrDataReady;

    assign RST = RST_IN;

    reg_pipeline
        #(
          // Parameters
          .C_DEPTH                      (C_INPUT_REGISTERS),
          /*AUTOINSTPARAM*/
          // Parameters
          .C_WIDTH                      (C_WIDTH))
    reg_in
        (
         // Outputs
         .RD_DATA                       (wRdData),
         .RD_DATA_VALID                 (wRdDataValid),
         // Inputs
         .RD_DATA_READY                 (wRdDataReady),
         /*AUTOINST*/
         // Outputs
         .WR_DATA_READY                 (WR_DATA_READY),
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN),
         .WR_DATA                       (WR_DATA[C_WIDTH-1:0]),
         .WR_DATA_VALID                 (WR_DATA_VALID));
    
    fifo
        #(
          // Parameters
          .C_WIDTH                      (C_WIDTH),
          .C_DEPTH                      (C_DEPTH - C_PIPELINE_INPUT - C_PIPELINE_OUTPUT),
          .C_DELAY                      (C_DEPTH - C_PIPELINE_INPUT - C_PIPELINE_OUTPUT)
          /*AUTOINSTPARAM*/)
    fifo_inst
        (
         // Outputs
         .RD_DATA                       (wWrData),
         .WR_READY                      (wRdDataReady),
         .RD_VALID                      (wWrDataValid),
         // Inputs
         .WR_DATA                       (wRdData),
         .WR_VALID                      (wRdDataValid),
         .RD_READY                      (wWrDataReady),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST                           (RST));

    reg_pipeline
        #(
          // Parameters
          .C_DEPTH                      (C_OUTPUT_REGISTERS),
          .C_WIDTH                      (C_WIDTH)
          /*AUTOINSTPARAM*/)
    reg_OUT
        (
         // Outputs
         .WR_DATA_READY                 (wWrDataReady),
         // Inputs
         .WR_DATA                       (wWrData),
         .WR_DATA_VALID                 (wWrDataValid),
         /*AUTOINST*/
         // Outputs
         .RD_DATA                       (RD_DATA[C_WIDTH-1:0]),
         .RD_DATA_VALID                 (RD_DATA_VALID),
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN),
         .RD_DATA_READY                 (RD_DATA_READY));

endmodule // mem_pipeline

/* verilator lint_off UNOPTFLAT */
module reg_pipeline
    #(
      parameter C_DEPTH = 10,
      parameter C_WIDTH = 10
      )
    (
     input                CLK,
     input                RST_IN,

     input [C_WIDTH-1:0]  WR_DATA,
     input                WR_DATA_VALID,
     output               WR_DATA_READY,

     output [C_WIDTH-1:0] RD_DATA,
     output               RD_DATA_VALID,
     input                RD_DATA_READY
     );

    genvar                i;

    wire                  wReady [C_DEPTH:0];
    
    reg [C_WIDTH-1:0]     _rData [C_DEPTH:1], rData [C_DEPTH:0];
    reg                   _rValid [C_DEPTH:1], rValid [C_DEPTH:0];

    // Read interface
    assign wReady[C_DEPTH] = RD_DATA_READY;
    assign RD_DATA = rData[C_DEPTH];
    assign RD_DATA_VALID = rValid[C_DEPTH];

    // Write interface
    assign WR_DATA_READY = wReady[0];
    always @(*) begin
        rData[0] = WR_DATA;
        rValid[0] = WR_DATA_VALID;
    end

    generate
        for( i = 1 ; i <= C_DEPTH; i = i + 1 ) begin : gen_stages
            assign #1 wReady[i-1] =  ~rValid[i] | wReady[i];

            // Data Registers
            always @(*) begin
                _rData[i] = rData[i-1];
            end

            // Enable the data register when the corresponding stage is ready
            always @(posedge CLK) begin
                if(wReady[i-1]) begin
                    rData[i] <= #1 _rData[i];
                end
            end

            // Valid Registers
            always @(*) begin
                if(RST_IN) begin
                    _rValid[i] = 1'b0;
                end else begin
                    _rValid[i] = rValid[i-1] | (rValid[i] & ~wReady[i]);
                end
            end

            // Always enable the valid registers
            always @(posedge CLK) begin
                rValid[i] <= #1 _rValid[i];
            end

        end
    endgenerate
endmodule
/* verilator lint_on UNOPTFLAT */
