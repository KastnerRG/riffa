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
// Filename:            reorder_queue_input.v
// Version:             1.00
// Verilog Standard:    Verilog-2005
// Description:         Input stage to the reorder-queue. 
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`include "trellis.vh"
`timescale 1ns / 1ps
module reorder_queue_input 
    #(parameter C_PCI_DATA_WIDTH = 9'd128,
      parameter C_TAG_WIDTH = 5, // Number of outstanding requests 
      parameter C_TAG_DW_COUNT_WIDTH = 8,// Width of max count DWs per packet
      parameter C_DATA_ADDR_STRIDE_WIDTH = 5,// Width of max num stored data addr positions per tag
      parameter C_DATA_ADDR_WIDTH = 10, // Width of stored data address
      // Local parameters
      parameter C_PCI_DATA_WORD = C_PCI_DATA_WIDTH/32,
      parameter C_PCI_DATA_WORD_WIDTH = clog2s(C_PCI_DATA_WORD),
      parameter C_PCI_DATA_COUNT_WIDTH = clog2s(C_PCI_DATA_WORD+1),
      parameter C_NUM_TAGS = 2**C_TAG_WIDTH)
    (input                                            CLK, // Clock
     input                                            RST, // Synchronous reset
     input                                            VALID, // Valid input packet
     input [C_PCI_DATA_WIDTH-1:0]                     DATA, // Input packet payload data enable
     input [(C_PCI_DATA_WIDTH/32)-1:0]                DATA_EN, // Input packet payload data enable
     input                                            DATA_START_FLAG, // Input packet payload
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]          DATA_START_OFFSET, // Input packet payload data enable count
     input                                            DATA_END_FLAG, // Input packet payload
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]          DATA_END_OFFSET, // Input packet payload data enable count
     input                                            DONE, // Input packet done
     input                                            ERR, // Input packet has error
     input [C_TAG_WIDTH-1:0]                          TAG, // Input packet tag (external tag)

     output [C_NUM_TAGS-1:0]                          TAG_FINISH, // Bitmap of tags to finish
     input [C_NUM_TAGS-1:0]                           TAG_CLEAR, // Bitmap of tags to clear

     output [(C_DATA_ADDR_WIDTH*C_PCI_DATA_WORD)-1:0] STORED_DATA_ADDR, // Address of stored packet data for RAMs
     output [C_PCI_DATA_WIDTH-1:0]                    STORED_DATA, // Stored packet data for RAMs
     output [C_PCI_DATA_WORD-1:0]                     STORED_DATA_EN, // Stored packet data enable for RAMs
     output                                           PKT_VALID, // Valid flag for packet data
     output [C_TAG_WIDTH-1:0]                         PKT_TAG, // Tag for stored packet data
     output [C_TAG_DW_COUNT_WIDTH-1:0]                PKT_WORDS, // Total count of stored packet payload in DWs
     output                                           PKT_WORDS_LTE1, // True if total count of stored packet payload is <= 4 DWs
     output                                           PKT_WORDS_LTE2, // True if total count of stored packet payload is <= 8 DWs
     output                                           PKT_DONE, // Stored packet done flag
     output                                           PKT_ERR); // Stored packet error flag
     

    wire [C_PCI_DATA_COUNT_WIDTH-1:0]                 wDECount;
    wire [C_PCI_DATA_WORD-1:0]                        wDE;
    wire [C_PCI_DATA_WIDTH-1:0]                       wData;
    wire [C_PCI_DATA_WORD-1:0]                        wStartMask;
    wire [C_PCI_DATA_WORD-1:0]                        wEndMask;

    reg [5:0]                                         rValid=0;
    reg [(C_PCI_DATA_WIDTH*5)-1:0]                    rData=0;
    reg [(C_PCI_DATA_WORD*3)-1:0]                     rDE=0;
    reg [(C_PCI_DATA_COUNT_WIDTH*2)-1:0]              rDECount=0;
    reg [5:0]                                         rDone=0;
    reg [5:0]                                         rErr=0;
    reg [(C_TAG_WIDTH*6)-1:0]                         rTag=0;

    reg [C_PCI_DATA_WORD-1:0]                         rDEShift=0;
    reg [(C_PCI_DATA_WORD*2)-1:0]                     rDEShifted=0;

    reg                                               rCountValid=0;
    reg [C_NUM_TAGS-1:0]                              rCountRst=0;
    reg [C_NUM_TAGS-1:0]                              rValidCount=0;

    reg                                               rUseCurrCount=0;
    reg                                               rUsePrevCount=0;
    reg [C_TAG_DW_COUNT_WIDTH-1:0]                    rPrevCount=0;
    reg [C_TAG_DW_COUNT_WIDTH-1:0]                    rCount=0;
    wire [C_TAG_DW_COUNT_WIDTH-1:0]                   wCount;
    wire [C_TAG_DW_COUNT_WIDTH-1:0]                   wCountClr = wCount & {C_TAG_DW_COUNT_WIDTH{rCountValid}};
    reg [(C_TAG_DW_COUNT_WIDTH*3)-1:0]                rWords=0;
    reg [C_PCI_DATA_WORD_WIDTH-1:0]                   rShift=0;
    reg [C_PCI_DATA_WORD_WIDTH-1:0]                   rShifted=0;

    reg                                               rPosValid=0;
    reg [C_NUM_TAGS-1:0]                              rPosRst=0;
    reg [C_NUM_TAGS-1:0]                              rValidPos=0;

    reg                                               rUseCurrPos=0;
    reg                                               rUsePrevPos=0;
    reg [(C_DATA_ADDR_STRIDE_WIDTH*C_PCI_DATA_WORD)-1:0] rPrevPos=0;
    reg [(C_DATA_ADDR_STRIDE_WIDTH*C_PCI_DATA_WORD)-1:0] rPosNow=0;
    reg [(C_DATA_ADDR_STRIDE_WIDTH*C_PCI_DATA_WORD)-1:0] rPos=0;
    wire [(C_DATA_ADDR_STRIDE_WIDTH*C_PCI_DATA_WORD)-1:0] wPos;
    wire [(C_DATA_ADDR_STRIDE_WIDTH*C_PCI_DATA_WORD)-1:0] wPosClr = wPos & {C_DATA_ADDR_STRIDE_WIDTH*C_PCI_DATA_WORD{rPosValid}};

    reg [(C_DATA_ADDR_WIDTH*C_PCI_DATA_WORD)-1:0]         rAddr=0;
    reg [(C_PCI_DATA_WORD_WIDTH+5)-1:0]                   rShiftUp=0;
    reg [(C_PCI_DATA_WORD_WIDTH+5)-1:0]                   rShiftDown=0;
    reg [C_DATA_ADDR_WIDTH-1:0]                           rBaseAddr=0;
    reg [C_PCI_DATA_WIDTH-1:0]                            rDataShifted=0;
    reg                                                   rLTE1Pkt=0;
    reg                                                   rLTE2Pkt=0;

    reg [C_NUM_TAGS-1:0]                                  rFinish=0;

    wire [31:0]                                           wZero=32'd0;
    integer                                               i;
    
    assign wDE = DATA_EN >> (DATA_START_FLAG ? DATA_START_OFFSET : 0);/* TODO: Could move this to the RX Engine*/
    assign wData = DATA >> (DATA_START_FLAG ? {DATA_START_OFFSET,5'b0} : 0);
    generate
        if(C_PCI_DATA_WIDTH == 32) begin
            assign wDECount = VALID ? 1 : 0;
        end
        if(C_PCI_DATA_WIDTH == 64) begin
            assign wDECount = VALID ? DATA_EN[1] + DATA_EN[0] : 0;
        end
        if(C_PCI_DATA_WIDTH == 128) begin
            assign wDECount = VALID ? DATA_EN[3] + DATA_EN[2] + DATA_EN[1] + DATA_EN[0] : 0;
        end
        if(C_PCI_DATA_WIDTH == 256) begin
            assign wDECount = VALID ? DATA_EN[7] + DATA_EN[6] + DATA_EN[5] + DATA_EN[4] +
                              DATA_EN[3] + DATA_EN[2] + DATA_EN[1] + DATA_EN[0] : 0;
        end
    endgenerate
    
    assign TAG_FINISH = rFinish;

    assign STORED_DATA_ADDR = rAddr;
    assign STORED_DATA = rDataShifted;
    assign STORED_DATA_EN = rDEShifted[1*C_PCI_DATA_WORD +:C_PCI_DATA_WORD];

    assign PKT_VALID = rValid[5];
    assign PKT_TAG = rTag[5*C_TAG_WIDTH +:C_TAG_WIDTH];
    assign PKT_WORDS = rWords[2*C_TAG_DW_COUNT_WIDTH +:C_TAG_DW_COUNT_WIDTH];
    assign PKT_WORDS_LTE1 = rLTE1Pkt;
    assign PKT_WORDS_LTE2 = rLTE2Pkt;
    assign PKT_DONE = rDone[5];
    assign PKT_ERR = rErr[5];


    // Pipeline the input and intermediate data
    always @ (posedge CLK) begin
        if (RST) begin
            rValid <= #1 0;
            rTag <= #1 0;
        end
        else begin
            rValid <= #1 (rValid<<1) | VALID;
            rTag <= #1 (rTag<<C_TAG_WIDTH) | TAG;
        end
        rData <= #1 (rData<<C_PCI_DATA_WIDTH) | wData;
        rDE <= #1 (rDE<<C_PCI_DATA_WORD) | wDE;//DATA_EN;
        rDECount <= #1 (rDECount<<C_PCI_DATA_COUNT_WIDTH) | wDECount;//DATA_EN_COUNT;
        rDone <= #1 (rDone<<1) | DONE;
        rErr <= #1 (rErr<<1) | ERR;
        
        rDEShifted <= #1 (rDEShifted<<C_PCI_DATA_WORD) | rDEShift;
        rWords <= #1 (rWords<<C_TAG_DW_COUNT_WIDTH) | rCount;
        rShifted <= #1 (rShifted<<C_PCI_DATA_WORD_WIDTH) | rShift;
    end


    // Input processing pipeline
    always @ (posedge CLK) begin
        // STAGE 0: Register the incoming data

        // STAGE 1: Request existing count from RAM
        // To cover the gap b/t reads and writes to RAM, next cycle we might need 
        // to use the existing or even the previous rCount value if the tags match.
        rUseCurrCount <= #1 (rTag[0*C_TAG_WIDTH +:C_TAG_WIDTH] == rTag[1*C_TAG_WIDTH +:C_TAG_WIDTH] && rValid[1]);
        rUsePrevCount <= #1 (rTag[0*C_TAG_WIDTH +:C_TAG_WIDTH] == rTag[2*C_TAG_WIDTH +:C_TAG_WIDTH] && rValid[2]);
        rPrevCount <= #1 rCount;
        // See if we need to reset the count
        rCountValid <= #1 (RST ? 1'd0 : rCountRst>>rTag[0*C_TAG_WIDTH +:C_TAG_WIDTH]);
        rValidCount <= #1 (RST ? 0 : rValid[0]<<rTag[0*C_TAG_WIDTH +:C_TAG_WIDTH]);
        
        // STAGE 2: Calculate new count (saves next cycle)
        if (rUseCurrCount) begin
            rShift <= #1 rCount[0 +:C_PCI_DATA_WORD_WIDTH];
            rCount <= #1 rCount + rDECount[1*C_PCI_DATA_COUNT_WIDTH +:C_PCI_DATA_COUNT_WIDTH];
        end
        else if (rUsePrevCount) begin
            rShift <= #1 rPrevCount[0 +:C_PCI_DATA_WORD_WIDTH];
            rCount <= #1 rPrevCount + rDECount[1*C_PCI_DATA_COUNT_WIDTH +:C_PCI_DATA_COUNT_WIDTH];
        end
        else begin
            rShift <= #1 wCountClr[0 +:C_PCI_DATA_WORD_WIDTH];
            rCount <= #1 wCountClr + rDECount[1*C_PCI_DATA_COUNT_WIDTH +:C_PCI_DATA_COUNT_WIDTH];
        end
        
        // STAGE 3: Request existing positions from RAM
        // Barrel shift the DE
        rDEShift <= #1 (rDE[2*C_PCI_DATA_WORD +:C_PCI_DATA_WORD]<<rShift) | 
                    (rDE[2*C_PCI_DATA_WORD +:C_PCI_DATA_WORD]>>(C_PCI_DATA_WORD-rShift));
        // To cover the gap b/t reads and writes to RAM, next cycle we might need 
        // to use the existing or even the previous rPos values if the tags match.
        rUseCurrPos <= #1 (rTag[2*C_TAG_WIDTH +:C_TAG_WIDTH] == rTag[3*C_TAG_WIDTH +:C_TAG_WIDTH] && rValid[3]);
        rUsePrevPos <= #1 (rTag[2*C_TAG_WIDTH +:C_TAG_WIDTH] == rTag[4*C_TAG_WIDTH +:C_TAG_WIDTH] && rValid[4]);
        for (i = 0; i < C_PCI_DATA_WORD; i = i + 1) begin
            rPrevPos[C_DATA_ADDR_STRIDE_WIDTH*i +:C_DATA_ADDR_STRIDE_WIDTH] <= #1 
                 (RST ? wZero[C_DATA_ADDR_STRIDE_WIDTH-1:0] : rPos[C_DATA_ADDR_STRIDE_WIDTH*i +:C_DATA_ADDR_STRIDE_WIDTH]);
        end
        // See if we need to reset the positions
        rPosValid <= #1 (RST ? 1'd0 : rPosRst>>rTag[2*C_TAG_WIDTH +:C_TAG_WIDTH]);
        rValidPos <= #1 (RST ? 0 : rValid[2]<<rTag[2*C_TAG_WIDTH +:C_TAG_WIDTH]);

        // STAGE 4: Calculate new positions (saves next cycle)
        if (rUseCurrPos) begin
            for (i = 0; i < C_PCI_DATA_WORD; i = i + 1) begin
                rPosNow[C_DATA_ADDR_STRIDE_WIDTH*i +:C_DATA_ADDR_STRIDE_WIDTH] <= #1 
                     (RST ? 
                      wZero[C_DATA_ADDR_STRIDE_WIDTH-1:0] : 
                      rPos[C_DATA_ADDR_STRIDE_WIDTH*i +:C_DATA_ADDR_STRIDE_WIDTH]);
                
                rPos[C_DATA_ADDR_STRIDE_WIDTH*i +:C_DATA_ADDR_STRIDE_WIDTH] <= #1 
                     (RST ? 
                      wZero[C_DATA_ADDR_STRIDE_WIDTH-1:0] : 
                      rPos[C_DATA_ADDR_STRIDE_WIDTH*i +:C_DATA_ADDR_STRIDE_WIDTH] + rDEShift[i]);
            end
        end
        else if (rUsePrevPos) begin
            for (i = 0; i < C_PCI_DATA_WORD; i = i + 1) begin
                rPosNow[C_DATA_ADDR_STRIDE_WIDTH*i +:C_DATA_ADDR_STRIDE_WIDTH] <= #1 
                     (RST ? 
                      wZero[C_DATA_ADDR_STRIDE_WIDTH-1:0] : 
                      rPrevPos[C_DATA_ADDR_STRIDE_WIDTH*i +:C_DATA_ADDR_STRIDE_WIDTH]);
                rPos[C_DATA_ADDR_STRIDE_WIDTH*i +:C_DATA_ADDR_STRIDE_WIDTH] <= #1 
                     (RST ? 
                      wZero[C_DATA_ADDR_STRIDE_WIDTH-1:0] : 
                      rPrevPos[C_DATA_ADDR_STRIDE_WIDTH*i +:C_DATA_ADDR_STRIDE_WIDTH] + rDEShift[i]);
            end
        end
        else begin
            for (i = 0; i < C_PCI_DATA_WORD; i = i + 1) begin
                rPosNow[C_DATA_ADDR_STRIDE_WIDTH*i +:C_DATA_ADDR_STRIDE_WIDTH] <= #1 
                     (RST ? 
                      wZero[C_DATA_ADDR_STRIDE_WIDTH-1:0] : 
                      wPosClr[i*C_DATA_ADDR_STRIDE_WIDTH +:C_DATA_ADDR_STRIDE_WIDTH]);
                rPos[C_DATA_ADDR_STRIDE_WIDTH*i +:C_DATA_ADDR_STRIDE_WIDTH] <= #1 
                      (RST ? 
                       wZero[C_DATA_ADDR_STRIDE_WIDTH-1:0] : 
                       wPosClr[i*C_DATA_ADDR_STRIDE_WIDTH +:C_DATA_ADDR_STRIDE_WIDTH] + rDEShift[i]);
            end
        end
        // Calculate the base address offset
        rBaseAddr <= #1 rTag[3*C_TAG_WIDTH +:C_TAG_WIDTH]<<C_DATA_ADDR_STRIDE_WIDTH;
        // Calculate the shift amounts for barrel shifting payload data
        rShiftUp <= #1 rShifted[0*C_PCI_DATA_WORD_WIDTH +:C_PCI_DATA_WORD_WIDTH]<<5;
        rShiftDown <= #1 (C_PCI_DATA_WORD[C_PCI_DATA_WORD_WIDTH:0] - rShifted[0*C_PCI_DATA_WORD_WIDTH +:C_PCI_DATA_WORD_WIDTH])<<5;

        // STAGE 5: Prepare to write data, final info
        for (i = 0; i < C_PCI_DATA_WORD; i = i + 1) begin
            rAddr[C_DATA_ADDR_WIDTH*i +:C_DATA_ADDR_WIDTH] <= #1 
                 rPosNow[C_DATA_ADDR_STRIDE_WIDTH*i +:C_DATA_ADDR_STRIDE_WIDTH] + rBaseAddr;
        end
        rDataShifted <= #1 (rData[4*C_PCI_DATA_WIDTH +:C_PCI_DATA_WIDTH]<<rShiftUp) | 
                        (rData[4*C_PCI_DATA_WIDTH +:C_PCI_DATA_WIDTH]>>rShiftDown);
        rLTE1Pkt <= #1 (rWords[1*C_TAG_DW_COUNT_WIDTH +:C_TAG_DW_COUNT_WIDTH] <= C_PCI_DATA_WORD);
        rLTE2Pkt <= #1 (rWords[1*C_TAG_DW_COUNT_WIDTH +:C_TAG_DW_COUNT_WIDTH] <= (C_PCI_DATA_WORD*2));
        rFinish <= #1 (rValid[4] & (rDone[4] | rErr[4]))<<rTag[4*C_TAG_WIDTH +:C_TAG_WIDTH];

        // STAGE 6: Write data, final info
    end


    // Reset the count and positions when needed
    always @ (posedge CLK) begin
        if (RST) begin
            rCountRst <= #1 0;
            rPosRst <= #1 0;
        end
        else begin
            rCountRst <= #1 (rCountRst | rValidCount) & ~TAG_CLEAR;
            rPosRst <= #1 (rPosRst | rValidPos) & ~TAG_CLEAR;
        end
    end


    // RAM for counts
    (* RAM_STYLE="DISTRIBUTED" *)
    ram_1clk_1w_1r 
        #(
          .C_RAM_WIDTH(C_TAG_DW_COUNT_WIDTH), 
          .C_RAM_DEPTH(C_NUM_TAGS)) 
    countRam 
        (
         .CLK(CLK),
         .ADDRA(rTag[2*C_TAG_WIDTH +:C_TAG_WIDTH]),
         .WEA(rValid[2]),
         .DINA(rCount),
         .ADDRB(rTag[0*C_TAG_WIDTH +:C_TAG_WIDTH]),
         .DOUTB(wCount)
         );


    // RAM for positions
    (* RAM_STYLE="DISTRIBUTED" *)
    ram_1clk_1w_1r 
        #(
          .C_RAM_WIDTH(C_PCI_DATA_WORD*C_DATA_ADDR_STRIDE_WIDTH), 
          .C_RAM_DEPTH(C_NUM_TAGS)) 
    posRam 
        (
         .CLK(CLK),
         .ADDRA(rTag[4*C_TAG_WIDTH +:C_TAG_WIDTH]),
         .WEA(rValid[4]),
         .DINA(rPos),
         .ADDRB(rTag[2*C_TAG_WIDTH +:C_TAG_WIDTH]),
         .DOUTB(wPos)
         );

endmodule
// Local Variables:
// verilog-library-directories:("." "registers/" "../common/")
// End:
