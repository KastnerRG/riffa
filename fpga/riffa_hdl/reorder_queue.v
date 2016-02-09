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
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    19:27:32 06/14/2012 
// Design Name: 
// Module Name:    reorder_queue
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description:
// Reorders downstream TLPs to output in increasing tag sequence. Input packets
// are stored in RAM and then read out when all previous sequence numbers have
// arrived and been read out. This module also provides the next available tag
// for the TX engine to use when sending memory request TLPs.
//
// Dependencies: 
// reorder_queue_input.v
// reorder_queue_output.v
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
`include "trellis.vh"
`timescale 1ns / 1ps
module reorder_queue 
    #(
      parameter C_PCI_DATA_WIDTH = 9'd128,
      parameter C_NUM_CHNL = 4'd12,
      parameter C_MAX_READ_REQ_BYTES = 512,            // Max size of read requests (in bytes)
      parameter C_TAG_WIDTH = 5,                       // Number of outstanding requests 
      // Local parameters
      parameter C_PCI_DATA_WORD = C_PCI_DATA_WIDTH/32,
      parameter C_PCI_DATA_COUNT_WIDTH = clog2(C_PCI_DATA_WORD+1),
      parameter C_NUM_TAGS = 2**C_TAG_WIDTH,
      parameter C_DW_PER_TAG = C_MAX_READ_REQ_BYTES/4,
      parameter C_TAG_DW_COUNT_WIDTH = clog2s(C_DW_PER_TAG+1),
      parameter C_DATA_ADDR_STRIDE_WIDTH = clog2s(C_DW_PER_TAG/C_PCI_DATA_WORD), // div by C_PCI_DATA_WORD b/c there are C_PCI_DATA_WORD RAMs
      parameter C_DATA_ADDR_WIDTH = C_TAG_WIDTH + C_DATA_ADDR_STRIDE_WIDTH
      )
    (
     input                                            CLK, // Clock
     input                                            RST, // Synchronous reset
     input                                            VALID, // Valid input packet
     input [C_PCI_DATA_WIDTH-1:0]                     DATA, // Input packet payload
     input [(C_PCI_DATA_WIDTH/32)-1:0]                DATA_EN, // Input packet payload data enable
     input                                            DATA_START_FLAG, // Input packet payload
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]          DATA_START_OFFSET, // Input packet payload data enable count
     input                                            DATA_END_FLAG, // Input packet payload
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]          DATA_END_OFFSET, // Input packet payload data enable count
     input                                            DONE, // Input packet done
     input                                            ERR, // Input packet has error
     input [C_TAG_WIDTH-1:0]                          TAG, // Input packet tag (external tag)

     input [5:0]                                      INT_TAG, // Internal tag to exchange with external
     input                                            INT_TAG_VALID, // High to signal tag exchange 
     output [C_TAG_WIDTH-1:0]                         EXT_TAG, // External tag to provide in exchange for internal tag
     output                                           EXT_TAG_VALID, // High to signal external tag is valid

     output [C_PCI_DATA_WIDTH-1:0]                    ENG_DATA, // Engine data 
     output [(C_NUM_CHNL*C_PCI_DATA_COUNT_WIDTH)-1:0] MAIN_DATA_EN, // Main data enable
     output [C_NUM_CHNL-1:0]                          MAIN_DONE, // Main data complete
     output [C_NUM_CHNL-1:0]                          MAIN_ERR, // Main data completed with error
     output [(C_NUM_CHNL*C_PCI_DATA_COUNT_WIDTH)-1:0] SG_RX_DATA_EN, // Scatter gather for RX data enable
     output [C_NUM_CHNL-1:0]                          SG_RX_DONE, // Scatter gather for RX data complete
     output [C_NUM_CHNL-1:0]                          SG_RX_ERR, // Scatter gather for RX data completed with error
     output [(C_NUM_CHNL*C_PCI_DATA_COUNT_WIDTH)-1:0] SG_TX_DATA_EN, // Scatter gather for TX data enable
     output [C_NUM_CHNL-1:0]                          SG_TX_DONE, // Scatter gather for TX data complete
     output [C_NUM_CHNL-1:0]                          SG_TX_ERR                             // Scatter gather for TX data completed with error
     );


    wire [(C_DATA_ADDR_WIDTH*C_PCI_DATA_WORD)-1:0]    wWrDataAddr;
    wire [C_PCI_DATA_WIDTH-1:0]                       wWrData;
    wire [C_PCI_DATA_WORD-1:0]                        wWrDataEn;

    wire [C_TAG_WIDTH-1:0]                            wWrPktTag;
    wire [C_TAG_DW_COUNT_WIDTH-1:0]                   wWrPktWords;
    wire                                              wWrPktWordsLTE1;
    wire                                              wWrPktWordsLTE2;
    wire                                              wWrPktValid;
    wire                                              wWrPktDone;
    wire                                              wWrPktErr;

    wire [C_DATA_ADDR_WIDTH-1:0]                      wRdDataAddr;
    wire [C_PCI_DATA_WIDTH-1:0]                       wRdData;

    wire [C_TAG_WIDTH-1:0]                            wRdPktTag;
    wire [(1+1+1+1+C_TAG_DW_COUNT_WIDTH)-1:0]         wRdPktInfo;

    wire [5:0]                                        wRdTagMap;

    wire [C_NUM_TAGS-1:0]                             wFinish;
    wire [C_NUM_TAGS-1:0]                             wClear;

    reg [C_TAG_WIDTH-1:0]                             rPos=0;
    reg                                               rValid=0;
    reg [C_NUM_TAGS-1:0]                              rFinished=0;
    reg [C_NUM_TAGS-1:0]                              rUse=0;
    reg [C_NUM_TAGS-1:0]                              rUsing=0;

    assign EXT_TAG = rPos;
    assign EXT_TAG_VALID = rValid;

    // Move through tag/slot/bucket space.
    always @ (posedge CLK) begin
        if (RST) begin
            rPos <= #1 0;
            rUse <= #1 0;
            rValid <= #1 0;
        end
        else begin
            if (INT_TAG_VALID & EXT_TAG_VALID) begin
                rPos <= #1 rPos + 1'd1;
                rUse <= #1 1<<rPos;
                rValid <= #1 !rUsing[rPos + 1'd1];
            end
            else begin
                rUse <= #1 0;
                rValid <= #1 !rUsing[rPos];
            end
        end
    end


    // Update tag/slot/bucket status.
    always @ (posedge CLK) begin
        if (RST) begin
            rUsing <= #1 0;
            rFinished <= #1 0;
        end
        else begin
            rUsing <= #1 (rUsing | rUse) & ~wClear;
            rFinished <= #1 (rFinished | wFinish) & ~wClear;
        end
    end


    genvar r;
    generate
        for (r = 0; r < C_PCI_DATA_WORD; r = r + 1) begin : rams
            // RAMs for packet reordering.
            (* RAM_STYLE="BLOCK" *)
            ram_1clk_1w_1r 
                 #(.C_RAM_WIDTH(32), 
                   .C_RAM_DEPTH(C_NUM_TAGS*C_DW_PER_TAG/C_PCI_DATA_WORD)
                   ) 
            ram 
                 (
                  .CLK(CLK),
                  .ADDRA(wWrDataAddr[C_DATA_ADDR_WIDTH*r +:C_DATA_ADDR_WIDTH]),
                  .WEA(wWrDataEn[r]),
                  .DINA(wWrData[32*r +:32]),
                  .ADDRB(wRdDataAddr),
                  .DOUTB(wRdData[32*r +:32])
                  );
        end
    endgenerate


    // RAM for bucket done, err, final DW count
    (* RAM_STYLE="DISTRIBUTED" *)
    ram_1clk_1w_1r 
        #(.C_RAM_WIDTH(1 + 1 + 1 + 1 + C_TAG_DW_COUNT_WIDTH), 
          .C_RAM_DEPTH(C_NUM_TAGS)) 
    pktRam 
        (
         .CLK(CLK),
         .ADDRA(wWrPktTag),
         .WEA((wWrPktDone | wWrPktErr) & wWrPktValid),
         .DINA({wWrPktDone, wWrPktErr, wWrPktWordsLTE2, wWrPktWordsLTE1, wWrPktWords}),
         .ADDRB(wRdPktTag),
         .DOUTB(wRdPktInfo)
         );


    // RAM for tag map
    (* RAM_STYLE="DISTRIBUTED" *)
    ram_1clk_1w_1r 
        #(.C_RAM_WIDTH(6), 
          .C_RAM_DEPTH(C_NUM_TAGS)) 
    mapRam 
        (
         .CLK(CLK),
         .ADDRA(rPos),
         .WEA(INT_TAG_VALID & EXT_TAG_VALID),
         .DINA(INT_TAG),
         .ADDRB(wRdPktTag),
         .DOUTB(wRdTagMap)
         );


    // Demux input data into the correct slot/bucket.
    reorder_queue_input 
        #(
          .C_PCI_DATA_WIDTH(C_PCI_DATA_WIDTH),
          .C_TAG_WIDTH(C_TAG_WIDTH),
          .C_TAG_DW_COUNT_WIDTH(C_TAG_DW_COUNT_WIDTH),
          .C_DATA_ADDR_STRIDE_WIDTH(C_DATA_ADDR_STRIDE_WIDTH),
          .C_DATA_ADDR_WIDTH(C_DATA_ADDR_WIDTH)
          ) 
    data_input 
        (
         .CLK(CLK),
         .RST(RST),
         .VALID(VALID),
         .DATA_START_FLAG               (DATA_START_FLAG),
         .DATA_START_OFFSET             (DATA_START_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .DATA_END_FLAG                 (DATA_END_FLAG),
         .DATA_END_OFFSET               (DATA_END_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .DATA                          (DATA),
         .DATA_EN                       (DATA_EN),
         .DONE(DONE),
         .ERR(ERR),
         .TAG(TAG),
         .TAG_FINISH(wFinish),
         .TAG_CLEAR(wClear),
         .STORED_DATA_ADDR(wWrDataAddr),
         .STORED_DATA(wWrData),
         .STORED_DATA_EN(wWrDataEn),
         .PKT_VALID(wWrPktValid),
         .PKT_TAG(wWrPktTag),
         .PKT_WORDS(wWrPktWords),
         .PKT_WORDS_LTE1(wWrPktWordsLTE1),
         .PKT_WORDS_LTE2(wWrPktWordsLTE2),
         .PKT_DONE(wWrPktDone),
         .PKT_ERR(wWrPktErr)
         );


    // Output packets in increasing tag order.
    reorder_queue_output 
        #(
          .C_PCI_DATA_WIDTH(C_PCI_DATA_WIDTH),
          .C_NUM_CHNL(C_NUM_CHNL),
          .C_TAG_WIDTH(C_TAG_WIDTH),
          .C_TAG_DW_COUNT_WIDTH(C_TAG_DW_COUNT_WIDTH),
          .C_DATA_ADDR_STRIDE_WIDTH(C_DATA_ADDR_STRIDE_WIDTH),
          .C_DATA_ADDR_WIDTH(C_DATA_ADDR_WIDTH)
          ) 
    data_output 
        (
         .CLK(CLK),
         .RST(RST),
         .DATA_ADDR(wRdDataAddr),
         .DATA(wRdData),
         .TAG_FINISHED(rFinished),
         .TAG_CLEAR(wClear),
         .TAG(wRdPktTag),
         .TAG_MAPPED(wRdTagMap),
         .PKT_WORDS(wRdPktInfo[0 +:C_TAG_DW_COUNT_WIDTH]),
         .PKT_WORDS_LTE1(wRdPktInfo[C_TAG_DW_COUNT_WIDTH]),
         .PKT_WORDS_LTE2(wRdPktInfo[C_TAG_DW_COUNT_WIDTH+1]),
         .PKT_ERR(wRdPktInfo[C_TAG_DW_COUNT_WIDTH+2]),
         .PKT_DONE(wRdPktInfo[C_TAG_DW_COUNT_WIDTH+3]),
         .ENG_DATA(ENG_DATA),
         .MAIN_DATA_EN(MAIN_DATA_EN),
         .MAIN_DONE(MAIN_DONE),
         .MAIN_ERR(MAIN_ERR),
         .SG_RX_DATA_EN(SG_RX_DATA_EN),
         .SG_RX_DONE(SG_RX_DONE),
         .SG_RX_ERR(SG_RX_ERR),
         .SG_TX_DATA_EN(SG_TX_DATA_EN),
         .SG_TX_DONE(SG_TX_DONE),
         .SG_TX_ERR(SG_TX_ERR)
         );
endmodule
