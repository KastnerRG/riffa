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
`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    19:27:32 06/14/2012 
// Design Name: 
// Module Name:    reorder_queue_output
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description:
// Outputs stored TLPs in increasing tag order.
//
// Dependencies: 
// reorder_queue.v
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
`include "trellis.vh"
module reorder_queue_output #(
    parameter C_PCI_DATA_WIDTH = 9'd128,
    parameter C_NUM_CHNL = 4'd12,
    parameter C_TAG_WIDTH = 5, 						// Number of outstanding requests 
    parameter C_TAG_DW_COUNT_WIDTH = 8,				// Width of max count DWs per packet
    parameter C_DATA_ADDR_STRIDE_WIDTH = 5,			// Width of max num stored data addr positions per tag
    parameter C_DATA_ADDR_WIDTH = 10,				// Width of stored data address
    // Local parameters
    parameter C_PCI_DATA_WORD = C_PCI_DATA_WIDTH/32,
    parameter C_PCI_DATA_WORD_WIDTH = clog2s(C_PCI_DATA_WORD),
    parameter C_PCI_DATA_COUNT_WIDTH = clog2s(C_PCI_DATA_WORD+1),
    parameter C_NUM_TAGS = 2**C_TAG_WIDTH
    )
   (
    input 					     CLK, // Clock
    input 					     RST, // Synchronous reset
    output [C_DATA_ADDR_WIDTH-1:0] 		     DATA_ADDR, // Address of stored packet data
    input [C_PCI_DATA_WIDTH-1:0] 		     DATA, // Stored packet data
    input [C_NUM_TAGS-1:0] 			     TAG_FINISHED, // Bitmap of finished tags
    output [C_NUM_TAGS-1:0] 			     TAG_CLEAR, // Bitmap of tags to clear
    output [C_TAG_WIDTH-1:0] 			     TAG, // Tag for which to retrieve packet data
    input [5:0] 				     TAG_MAPPED, // Mapped tag (i.e. internal tag)
    input [C_TAG_DW_COUNT_WIDTH-1:0] 		     PKT_WORDS, // Total count of packet payload in DWs
    input 					     PKT_WORDS_LTE1, // True if total count of packet payload is <= 4 DWs
    input 					     PKT_WORDS_LTE2, // True if total count of packet payload is <= 8 DWs
    input 					     PKT_DONE, // Packet done flag
    input 					     PKT_ERR, // Packet error flag

    output [C_PCI_DATA_WIDTH-1:0] 		     ENG_DATA, // Engine data 
    output [(C_NUM_CHNL*C_PCI_DATA_COUNT_WIDTH)-1:0] MAIN_DATA_EN, // Main data enable
    output [C_NUM_CHNL-1:0] 			     MAIN_DONE, // Main data complete
    output [C_NUM_CHNL-1:0] 			     MAIN_ERR, // Main data completed with error
    output [(C_NUM_CHNL*C_PCI_DATA_COUNT_WIDTH)-1:0] SG_RX_DATA_EN, // Scatter gather for RX data enable
    output [C_NUM_CHNL-1:0] 			     SG_RX_DONE, // Scatter gather for RX data complete
    output [C_NUM_CHNL-1:0] 			     SG_RX_ERR, // Scatter gather for RX data completed with error
    output [(C_NUM_CHNL*C_PCI_DATA_COUNT_WIDTH)-1:0] SG_TX_DATA_EN, // Scatter gather for TX data enable
    output [C_NUM_CHNL-1:0] 			     SG_TX_DONE, // Scatter gather for TX data complete
    output [C_NUM_CHNL-1:0] 			     SG_TX_ERR								// Scatter gather for TX data completed with error
    );


   reg [1:0] 					     rState=0;
   reg [C_DATA_ADDR_WIDTH-1:0] 			     rDataAddr=0;
   reg [C_PCI_DATA_WIDTH-1:0] 			     rData=0;

   reg 						     rTagFinished=0;
   reg [C_NUM_TAGS-1:0] 			     rClear=0;

   reg [C_TAG_WIDTH-1:0] 			     rTag=0;
   reg [C_TAG_WIDTH-1:0] 			     rTagCurr=0;
   wire [C_TAG_WIDTH-1:0] 			     wTagNext = rTag + 1'd1;

   reg [5:0] 					     rShift;
   reg 						     rDone=0;
   reg 						     rDoneLast=0;
   reg 						     rErr=0;
   reg 						     rErrLast=0;
   reg [C_PCI_DATA_COUNT_WIDTH-1:0] 		     rDE=0;
   reg [C_TAG_DW_COUNT_WIDTH-1:0] 		     rWords=0;
   reg 						     rLTE2Pkts=0;

   reg [C_PCI_DATA_WIDTH-1:0] 			     rDataOut={C_PCI_DATA_WIDTH{1'b0}};
   reg [(3*16*C_PCI_DATA_COUNT_WIDTH)-1:0] 	     rDEOut={3*16*C_PCI_DATA_COUNT_WIDTH{1'd0}};
   reg [(3*16)-1:0] 				     rDoneOut={3*16{1'd0}};
   reg [(3*16)-1:0] 				     rErrOut={3*16{1'd0}};


   assign DATA_ADDR = rDataAddr;
   assign TAG = rTag;
   assign TAG_CLEAR = rClear;

   assign ENG_DATA = rDataOut;

   assign MAIN_DATA_EN = rDEOut[(0*16*C_PCI_DATA_COUNT_WIDTH) +:(C_NUM_CHNL*C_PCI_DATA_COUNT_WIDTH)];
   assign MAIN_DONE = rDoneOut[(0*16) +:C_NUM_CHNL];
   assign MAIN_ERR = rErrOut[(0*16) +:C_NUM_CHNL];

   assign SG_RX_DATA_EN = rDEOut[(1*16*C_PCI_DATA_COUNT_WIDTH) +:(C_NUM_CHNL*C_PCI_DATA_COUNT_WIDTH)];
   assign SG_RX_DONE = rDoneOut[(1*16) +:C_NUM_CHNL];
   assign SG_RX_ERR = rErrOut[(1*16) +:C_NUM_CHNL];

   assign SG_TX_DATA_EN = rDEOut[(2*16*C_PCI_DATA_COUNT_WIDTH) +:(C_NUM_CHNL*C_PCI_DATA_COUNT_WIDTH)];
   assign SG_TX_DONE = rDoneOut[(2*16) +:C_NUM_CHNL];
   assign SG_TX_ERR = rErrOut[(2*16) +:C_NUM_CHNL];


   // Output completed data in increasing tag order, avoid stalls if possible
   always @ (posedge CLK) begin
      if (RST) begin
	      rState <= #1 0;
	      rTag <= #1 0;
	      rDataAddr <= #1 0;
	      rDone <= #1 0;
	      rErr <= #1 0;
	      rDE <= #1 0;
	      rClear <= #1 0;
	      rTagFinished <= #1 0;
          rShift <= 0; // Added
      end
      else begin
	 rTagFinished <= #1 TAG_FINISHED[rTag];
	 case (rState)
	   
	   2'd0: begin // Request initial data and final info, output nothing
	      rDone <= #1 0;
	      rErr <= #1 0;
	      rDE <= #1 0;
	      rClear <= #1 0;
	      if (rTagFinished) begin
		 rTag <= #1 wTagNext;
		 rTagCurr <= #1 rTag;
		 rDataAddr <= #1 rDataAddr + 1'd1;
		 rState <= #1 2'd2;
	      end
	      else begin
		 rState <= #1 2'd0;
	      end
	   end

	   2'd1: begin // Request initial data and final info, output last data
	      rDone <= #1 rDoneLast;
	      rErr <= #1 rErrLast;
	      rDE <= #1 rWords[C_PCI_DATA_COUNT_WIDTH-1:0];
	      rClear <= #1 1<<rTagCurr; // Clear the tag
	      if (rTagFinished) begin
		 rTag <= #1 wTagNext;
		 rTagCurr <= #1 rTag;
		 rDataAddr <= #1 rDataAddr + 1'd1;
		 rState <= #1 2'd2;
	      end
	      else begin
		 rState <= #1 2'd0;
	      end
	   end

	   2'd2: begin // Initial data now available, output data
	      rShift <= #1 TAG_MAPPED;
	      rDoneLast <= #1 PKT_DONE;
	      rErrLast <= #1 PKT_ERR;
	      rWords <= #1 PKT_WORDS - C_PCI_DATA_WORD[C_PCI_DATA_WORD_WIDTH:0];
	      rLTE2Pkts <= #1 (PKT_WORDS <= (C_PCI_DATA_WORD*3));
	      if (PKT_WORDS_LTE1) begin // Guessed wrong, no addl data, need to reset
		 rDone <= #1 PKT_DONE;
		 rErr <= #1 PKT_ERR;
		 rDE <= #1 PKT_WORDS[C_PCI_DATA_COUNT_WIDTH-1:0];
		 rClear <= #1 1<<rTagCurr; // Clear the tag
		 rDataAddr <= #1 rTag<<C_DATA_ADDR_STRIDE_WIDTH; // rTag is already on the next
		 rState <= #1 2'd0;
	      end
	      else if (PKT_WORDS_LTE2) begin // Guessed right, end of data, output last and continue
		 rDone <= #1 0;
		 rErr <= #1 0;
		 rDE <= #1 C_PCI_DATA_WORD[C_PCI_DATA_WORD_WIDTH:0];
		 rClear <= #1 0;
		 rDataAddr <= #1 rTag<<C_DATA_ADDR_STRIDE_WIDTH; // rTag is already on the next
		 rState <= #1 2'd1;
	      end
	      else begin // Guessed right, more data, output it and continue
		 rDone <= #1 0;
		 rErr <= #1 0;
		 rDE <= #1 C_PCI_DATA_WORD[C_PCI_DATA_WORD_WIDTH:0];
		 rClear <= #1 0;
		 rDataAddr <= #1 rDataAddr + 1'd1;
		 rState <= #1 2'd3;
	      end
	   end

	   2'd3: begin // Next data now available, output data
	      rDone <= #1 0;
	      rErr <= #1 0;
	      rDE <= #1 C_PCI_DATA_WORD[C_PCI_DATA_WORD_WIDTH:0];
	      rWords <= #1 rWords - C_PCI_DATA_WORD[C_PCI_DATA_WORD_WIDTH:0];
	      rLTE2Pkts <= #1 (rWords <= (C_PCI_DATA_WORD*3));
	      if (rLTE2Pkts) begin // End of data, output last and continue
		 rDataAddr <= #1 rTag<<C_DATA_ADDR_STRIDE_WIDTH; // rTag is already on the next
		 rState <= #1 2'd1;
	      end
	      else begin // More data, output it and continue
		 rDataAddr <= #1 rDataAddr + 1'd1;
		 rState <= #1 2'd3;
	      end
	   end

	 endcase
      end
   end


   // Output the data
   always @ (posedge CLK) begin
      rData <= #1 DATA;
      rDataOut <= #1 rData;
      if (RST) begin
	 rDEOut <= #1 0;
	 rDoneOut <= #1 0;
	 rErrOut <= #1 0;
      end
      else begin
	 rDEOut <= #1 rDE<<(C_PCI_DATA_COUNT_WIDTH*rShift);
	 rDoneOut <= #1 (rDone | rErr)<<rShift;
	 rErrOut <= #1 rErr<<rShift;
      end
   end


   /*
    wire [35:0] wControl0;
    chipscope_icon_1 cs_icon(
    .CONTROL0(wControl0)
    );

    chipscope_ila_t8_512 a0(
    .CLK(DVI_CLK), 
    .CONTROL(wControl0), 
    .TRIG0({wFifoFull, wFifoEmpty, rState, rPrevDVI_VS, DVI_DE, DVI_HS, DVI_VS}),
    .DATA({457'd0,
    rCount, // 21
    rFrameCount, // 21
    wCapture, // 1
    RD_EN, // 1
    RD_EMPTY, // 1
    EOF, // 1
    wPackerFull, // 1
    wFifoFull, // 1
    wFifoEmpty, // 1
    rState, // 2
    rPrevDVI_VS, // 1
    DVI_DE, // 1
    DVI_HS, // 1
    DVI_VS}) // 1
    );
    */

endmodule
