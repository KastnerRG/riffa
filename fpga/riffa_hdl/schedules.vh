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
// ----------------------------------------------------------------------
// Filename:            Filename: schedules.vh
// Version:             Version: 1.0
// Verilog Standard:    Verilog-2005
// Description: This file defines the schedules for each output mux used
// in the tx_alignment alignment pipeline and the connections for each
// mux. The schedule depends on the width of the PCI bus.
// 
// The wSchedule array is intended to be a multiported ROM implemented
// as small memory slices adjacent to the corresponding output
// multiplexer. The wSchedule array is first indexed by the MUX id,
// then the concatenation of the header select bit (3 or 4 DW Header),
// whether or not the packet is Quad word aligned, and a saturating
// counter that counts up from the start of the packet to the maximum
// schedule index for that particular data bus width. 
// 
// For a 256-bit PCIe interface, the schedule is 2 cycles long. 
// For a 128-bit PCIe interface, the schedule is 3 cycles long. 
// For a 64-bit PCIe interface, the schedule is 4 cycles long. 
// For a 32-bit PCIe interface, the schedule is 5 cycles long. 
// 
// This file also contains the array wTxMuxInputs, which defines the
// connections for each output multiplexer. The wTxMuxInputs array is
// indexed by the output multiplexer ID, which provides a concatenated
// bus of all of the DW inputs for that particular mux.
// 
// These schedules are derived from the .csv files:
// SchedulesTable_256.csv SchedulesTable_128.csv SchedulesTable_64.csv
// and SchedulesTable_32.csv
// 
// Author: Dustin Richmond (@darichmond) 
// ----------------------------------------------------------------------
`ifndef SCHEDULES_VH

    `define SCHEDULES_VH

    `define TXA_HDR0_INDEX 0
    `define TXA_HDR1_INDEX 1
    `define TXA_HDR2_INDEX 2
    `define TXA_HDR3_INDEX 3
    `define TXA_DW0_INDEX 4
    `define TXA_DW1_INDEX 5
    `define TXA_DW2_INDEX 6
    `define TXA_DW3_INDEX 7
    `define TXA_DW4_INDEX 8
    `define TXA_DW5_INDEX 9
    `define TXA_DW6_INDEX 10
    `define TXA_DW7_INDEX 11

    `define TXA_5DW 3'b101
    `define TXA_4DW 3'b100
    `define TXA_3DW 3'b011

    `define TXA_3DWH 1'b0
    `define TXA_4DWH 1'b1
    `define TXA_NOB  1'b0
    `define TXA_INSB 1'b1

    `define TXA_READ 1'b1
// 256 Schedule

// Ugly as sin, but works
generate 
    if(C_DATA_WIDTH == 256) begin
        assign wTxMuxInputs[0] = {wAggregate[`TXA_DW5_INDEX],
                                  wAggregate[`TXA_DW4_INDEX],
                                  wAggregate[`TXA_DW3_INDEX],
                                  wAggregate[`TXA_HDR0_INDEX]};
        assign wSchedule[0][{`TXA_3DW,1'b0}] = 2'd0;
        assign wSchedule[0][{`TXA_3DW,1'b1}] = 2'd3;

        assign wSchedule[0][{`TXA_4DW,1'b0}] = 2'd0;
        assign wSchedule[0][{`TXA_4DW,1'b1}] = 2'd2;

        assign wSchedule[0][{`TXA_5DW,1'b0}] = 2'd0;
        assign wSchedule[0][{`TXA_5DW,1'b1}] = 2'd1;

        
        assign wTxMuxInputs[1] = {wAggregate[`TXA_DW6_INDEX],
                                  wAggregate[`TXA_DW5_INDEX],
                                  wAggregate[`TXA_DW4_INDEX],
                                  wAggregate[`TXA_HDR1_INDEX]};

        assign wSchedule[1][{`TXA_3DW,1'b0}] = 2'd0;
        assign wSchedule[1][{`TXA_3DW,1'b1}] = 2'd3;

        assign wSchedule[1][{`TXA_4DW,1'b0}] = 2'd0;
        assign wSchedule[1][{`TXA_4DW,1'b1}] = 2'd2;

        assign wSchedule[1][{`TXA_5DW,1'b0}] = 2'd0;
        assign wSchedule[1][{`TXA_5DW,1'b1}] = 2'd1;

        assign wTxMuxInputs[2] = {wAggregate[`TXA_DW7_INDEX],
                                  wAggregate[`TXA_DW6_INDEX],
                                  wAggregate[`TXA_DW5_INDEX],
                                  wAggregate[`TXA_HDR2_INDEX]};

        assign wSchedule[2][{`TXA_3DW,1'b0}] = 2'd0;
        assign wSchedule[2][{`TXA_3DW,1'b1}] = 2'd3;

        assign wSchedule[2][{`TXA_4DW,1'b0}] = 2'd0;
        assign wSchedule[2][{`TXA_4DW,1'b1}] = 2'd2;

        assign wSchedule[2][{`TXA_5DW,1'b0}] = 2'd0;
        assign wSchedule[2][{`TXA_5DW,1'b1}] = 2'd1;

        assign wTxMuxInputs[3] = {wAggregate[`TXA_DW7_INDEX],
                                  wAggregate[`TXA_DW6_INDEX],
                                  wAggregate[`TXA_DW0_INDEX],
                                  wAggregate[`TXA_HDR3_INDEX]};

        assign wSchedule[3][{`TXA_3DW,1'b0}] = 2'd1;
        assign wSchedule[3][{`TXA_3DW,1'b1}] = 2'd1;

        assign wSchedule[3][{`TXA_4DW,1'b0}] = 2'd0;
        assign wSchedule[3][{`TXA_4DW,1'b1}] = 2'd3;

        assign wSchedule[3][{`TXA_5DW,1'b0}] = 2'd0;
        assign wSchedule[3][{`TXA_5DW,1'b1}] = 2'd2;

        assign wTxMuxInputs[4] = {wAggregate[`TXA_DW7_INDEX],
                                  wAggregate[`TXA_DW1_INDEX],
                                  wAggregate[`TXA_DW0_INDEX],
                                  wAggregate[`TXA_DW0_INDEX]};

        assign wSchedule[4][{`TXA_3DW,1'b0}] = 2'd2;
        assign wSchedule[4][{`TXA_3DW,1'b1}] = 2'd2;

        assign wSchedule[4][{`TXA_4DW,1'b0}] = 2'd0;
        assign wSchedule[4][{`TXA_4DW,1'b1}] = 2'd0;

        assign wSchedule[4][{`TXA_5DW,1'b0}] = 2'd3;
        assign wSchedule[4][{`TXA_5DW,1'b1}] = 2'd3;

        assign wTxMuxInputs[5] = {wAggregate[`TXA_DW2_INDEX],
                                  wAggregate[`TXA_DW1_INDEX],
                                  wAggregate[`TXA_DW1_INDEX],
                                  wAggregate[`TXA_DW0_INDEX]};

        assign wSchedule[5][{`TXA_3DW,1'b0}] = 2'd3;
        assign wSchedule[5][{`TXA_3DW,1'b1}] = 2'd3;

        assign wSchedule[5][{`TXA_4DW,1'b0}] = 2'd1;
        assign wSchedule[5][{`TXA_4DW,1'b1}] = 2'd1;

        assign wSchedule[5][{`TXA_5DW,1'b0}] = 2'd0;
        assign wSchedule[5][{`TXA_5DW,1'b1}] = 2'd0;

        assign wTxMuxInputs[6] = {wAggregate[`TXA_DW3_INDEX],
                                  wAggregate[`TXA_DW2_INDEX],
                                  wAggregate[`TXA_DW2_INDEX],
                                  wAggregate[`TXA_DW1_INDEX]};

        assign wSchedule[6][{`TXA_3DW,1'b0}] = 2'd3;
        assign wSchedule[6][{`TXA_3DW,1'b1}] = 2'd3;

        assign wSchedule[6][{`TXA_4DW,1'b0}] = 2'd1;
        assign wSchedule[6][{`TXA_4DW,1'b1}] = 2'd1;

        assign wSchedule[6][{`TXA_5DW,1'b0}] = 2'd0;
        assign wSchedule[6][{`TXA_5DW,1'b1}] = 2'd0;

        assign wTxMuxInputs[7] = {wAggregate[`TXA_DW4_INDEX],
                                  wAggregate[`TXA_DW3_INDEX],
                                  wAggregate[`TXA_DW3_INDEX],
                                  wAggregate[`TXA_DW2_INDEX]};

        assign wSchedule[7][{`TXA_3DW,1'b0}] = 2'd3;
        assign wSchedule[7][{`TXA_3DW,1'b1}] = 2'd3;

        assign wSchedule[7][{`TXA_4DW,1'b0}] = 2'd1;
        assign wSchedule[7][{`TXA_4DW,1'b1}] = 2'd1;

        assign wSchedule[7][{`TXA_5DW,1'b0}] = 2'd0;
        assign wSchedule[7][{`TXA_5DW,1'b1}] = 2'd0;

    end else if (C_DATA_WIDTH == 128) begin

        assign wTxMuxInputs[0] = {wAggregate[`TXA_DW3_INDEX],
                                  wAggregate[`TXA_DW1_INDEX],
                                  wAggregate[`TXA_DW0_INDEX],
                                  wAggregate[`TXA_HDR0_INDEX]};

        assign wSchedule[0][{`TXA_3DW,1'b0,1'b0}] = 2'd0;
        assign wSchedule[0][{`TXA_3DW,1'b0,1'b1}] = 2'd2;
        assign wSchedule[0][{`TXA_3DW,1'b1,1'b0}] = 2'd2;
        
        assign wSchedule[0][{`TXA_4DW,1'b0,1'b0}] = 2'd0;
        assign wSchedule[0][{`TXA_4DW,1'b0,1'b1}] = 2'd1;
        assign wSchedule[0][{`TXA_4DW,1'b1,1'b0}] = 2'd1;

        assign wSchedule[0][{`TXA_5DW,1'b0,1'b0}] = 2'd0;
        assign wSchedule[0][{`TXA_5DW,1'b0,1'b1}] = 2'd3;
        assign wSchedule[0][{`TXA_5DW,1'b1,1'b0}] = 2'd3;

        assign wTxMuxInputs[1] = {wAggregate[`TXA_DW2_INDEX],
                                  wAggregate[`TXA_DW1_INDEX],
                                  wAggregate[`TXA_DW0_INDEX],
                                  wAggregate[`TXA_HDR1_INDEX]};

        assign wSchedule[1][{`TXA_3DW,1'b0,1'b0}] = 2'd0;
        assign wSchedule[1][{`TXA_3DW,1'b0,1'b1}] = 2'd3;
        assign wSchedule[1][{`TXA_3DW,1'b1,1'b0}] = 2'd3;

        assign wSchedule[1][{`TXA_4DW,1'b0,1'b0}] = 2'd0;
        assign wSchedule[1][{`TXA_4DW,1'b0,1'b1}] = 2'd2;
        assign wSchedule[1][{`TXA_4DW,1'b1,1'b0}] = 2'd2;

        assign wSchedule[1][{`TXA_5DW,1'b0,1'b0}] = 2'd0;
        assign wSchedule[1][{`TXA_5DW,1'b0,1'b1}] = 2'd1;
        assign wSchedule[1][{`TXA_5DW,1'b1,1'b0}] = 2'd1;

        assign wTxMuxInputs[2] = {wAggregate[`TXA_DW3_INDEX],
                                  wAggregate[`TXA_DW2_INDEX],
                                  wAggregate[`TXA_DW1_INDEX],
                                  wAggregate[`TXA_HDR2_INDEX]};

        assign wSchedule[2][{`TXA_3DW,1'b0,1'b0}] = 2'd0;
        assign wSchedule[2][{`TXA_3DW,1'b0,1'b1}] = 2'd3;
        assign wSchedule[2][{`TXA_3DW,1'b1,1'b0}] = 2'd3;

        assign wSchedule[2][{`TXA_4DW,1'b0,1'b0}] = 2'd0;
        assign wSchedule[2][{`TXA_4DW,1'b0,1'b1}] = 2'd2;
        assign wSchedule[2][{`TXA_4DW,1'b1,1'b0}] = 2'd2;

        assign wSchedule[2][{`TXA_5DW,1'b0,1'b0}] = 2'd0;
        assign wSchedule[2][{`TXA_5DW,1'b0,1'b1}] = 2'd1;
        assign wSchedule[2][{`TXA_5DW,1'b1,1'b0}] = 2'd1;

        assign wTxMuxInputs[3] = {wAggregate[`TXA_DW3_INDEX],
                                  wAggregate[`TXA_DW2_INDEX],
                                  wAggregate[`TXA_DW0_INDEX],
                                  wAggregate[`TXA_HDR3_INDEX]};

        assign wSchedule[3][{`TXA_3DW,1'b0,1'b0}] = 2'd1;
        assign wSchedule[3][{`TXA_3DW,1'b0,1'b1}] = 2'd1;
        assign wSchedule[3][{`TXA_3DW,1'b1,1'b0}] = 2'd1;

        assign wSchedule[3][{`TXA_4DW,1'b0,1'b0}] = 2'd0;
        assign wSchedule[3][{`TXA_4DW,1'b0,1'b1}] = 2'd3;
        assign wSchedule[3][{`TXA_4DW,1'b1,1'b0}] = 2'd3;

        assign wSchedule[3][{`TXA_5DW,1'b0,1'b0}] = 2'd0;
        assign wSchedule[3][{`TXA_5DW,1'b0,1'b1}] = 2'd2;
        assign wSchedule[3][{`TXA_5DW,1'b1,1'b0}] = 2'd2;

    end else if (C_DATA_WIDTH == 64) begin

        assign wTxMuxInputs[0] = {wAggregate[`TXA_DW1_INDEX],
                                  wAggregate[`TXA_DW0_INDEX],
                                  wAggregate[`TXA_HDR2_INDEX],
                                  wAggregate[`TXA_HDR0_INDEX]};

        assign wSchedule[0][{`TXA_3DW,2'b00}] = 2'd0;
        assign wSchedule[0][{`TXA_3DW,2'b01}] = 2'd1;
        assign wSchedule[0][{`TXA_3DW,2'b10}] = 2'd3;
        assign wSchedule[0][{`TXA_3DW,2'b11}] = 2'd3;
        
        assign wSchedule[0][{`TXA_4DW,2'b00}] = 2'd0;
        assign wSchedule[0][{`TXA_4DW,2'b01}] = 2'd1;
        assign wSchedule[0][{`TXA_4DW,2'b10}] = 2'd2;
        assign wSchedule[0][{`TXA_4DW,2'b11}] = 2'd2;

        assign wSchedule[0][{`TXA_5DW,2'b00}] = 2'd0;
        assign wSchedule[0][{`TXA_5DW,2'b01}] = 2'd1;
        assign wSchedule[0][{`TXA_5DW,2'b10}] = 2'd3;
        assign wSchedule[0][{`TXA_5DW,2'b11}] = 2'd3;

        assign wTxMuxInputs[1] = {wAggregate[`TXA_DW1_INDEX],
                                  wAggregate[`TXA_DW0_INDEX],
                                  wAggregate[`TXA_HDR3_INDEX],
                                  wAggregate[`TXA_HDR1_INDEX]};

        assign wSchedule[1][{`TXA_3DW,2'b00}] = 2'd0;
        assign wSchedule[1][{`TXA_3DW,2'b01}] = 2'd2;
        assign wSchedule[1][{`TXA_3DW,2'b10}] = 2'd2;
        assign wSchedule[1][{`TXA_3DW,2'b11}] = 2'd2;

        assign wSchedule[1][{`TXA_4DW,2'b00}] = 2'd0;
        assign wSchedule[1][{`TXA_4DW,2'b01}] = 2'd1;
        assign wSchedule[1][{`TXA_4DW,2'b10}] = 2'd3;
        assign wSchedule[1][{`TXA_4DW,2'b11}] = 2'd3;
        
        assign wSchedule[1][{`TXA_5DW,2'b00}] = 2'd0;
        assign wSchedule[1][{`TXA_5DW,2'b01}] = 2'd1;
        assign wSchedule[1][{`TXA_5DW,2'b10}] = 2'd2;
        assign wSchedule[1][{`TXA_5DW,2'b11}] = 2'd2;

    end else if (C_DATA_WIDTH == 32) begin

        assign wTxMuxInputs[0] = {wAggregate[`TXA_DW0_INDEX],
                                  wAggregate[`TXA_HDR3_INDEX],
                                  wAggregate[`TXA_HDR2_INDEX],
                                  wAggregate[`TXA_HDR1_INDEX],
                                  wAggregate[`TXA_HDR0_INDEX]};

        assign wSchedule[0][{`TXA_3DW,3'b000}] = 3'd0;
        assign wSchedule[0][{`TXA_3DW,3'b001}] = 3'd1;
        assign wSchedule[0][{`TXA_3DW,3'b010}] = 3'd2;
        assign wSchedule[0][{`TXA_3DW,3'b011}] = 3'd4;
        assign wSchedule[0][{`TXA_3DW,3'b100}] = 3'd4;
        assign wSchedule[0][{`TXA_3DW,3'b101}] = 3'd4;
        
        assign wSchedule[0][{`TXA_4DW,3'b000}] = 3'd0;
        assign wSchedule[0][{`TXA_4DW,3'b001}] = 3'd1;
        assign wSchedule[0][{`TXA_4DW,3'b010}] = 3'd2;
        assign wSchedule[0][{`TXA_4DW,3'b011}] = 3'd3;
        assign wSchedule[0][{`TXA_4DW,3'b100}] = 3'd4;
        assign wSchedule[0][{`TXA_4DW,3'b101}] = 3'd4;

        assign wSchedule[0][{`TXA_5DW,3'b000}] = 3'd0;
        assign wSchedule[0][{`TXA_5DW,3'b001}] = 3'd1;
        assign wSchedule[0][{`TXA_5DW,3'b010}] = 3'd2;
        assign wSchedule[0][{`TXA_5DW,3'b011}] = 3'd3;
        assign wSchedule[0][{`TXA_5DW,3'b100}] = 3'd4;
        assign wSchedule[0][{`TXA_5DW,3'b101}] = 3'd4;
    end else begin
        // Error!!!
    end
endgenerate
`endif
