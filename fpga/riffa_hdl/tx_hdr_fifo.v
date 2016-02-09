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
// Filename: tx_hdr_fifo.v
// Version: 1.0
// Verilog Standard: Verilog-2001
//
// Description: The tx_hdr_fifo module implements a simple fifo for a packet
// (WR_TX_HDR) header and three metadata signals: WR_TX_HDR_ABLANKS,
// WR_TX_HDR_LEN, WR_TX_HDR_NOPAYLOAD. NOPAYLOAD indicates that the header is not
// followed by a payload. HDR_LEN indicates the length of the header in
// dwords. The ABLANKS signal indicates how many dwords should be inserted between
// the header and payload.
// 
// The intended use for this module is between the interface specific tx formatter
// (TXC or TXR) and the alignment pipeline, in parallel with the tx_data_pipeline
// which contains a fifo for payloads.
// 
// Author: Dustin Richmond (@darichmond) 
// Co-Authors:
//----------------------------------------------------------------------------
`timescale 1ns/1ns
`include "trellis.vh" // Defines the user-facing signal widths.
module tx_hdr_fifo
    #(parameter C_DEPTH_PACKETS = 10,
      parameter C_MAX_HDR_WIDTH = 128,
      parameter C_PIPELINE_OUTPUT = 1,
      parameter C_PIPELINE_INPUT = 1,
      parameter C_VENDOR = "ALTERA"
      )
    (
     // Interface: Clocks
     input                          CLK,

     // Interface: Reset
     input                          RST_IN,

     // Interface: WR_TX_HDR
     input                          WR_TX_HDR_VALID,
     input [(C_MAX_HDR_WIDTH)-1:0]  WR_TX_HDR,
     input [`SIG_LEN_W-1:0]         WR_TX_HDR_PAYLOAD_LEN,
     input [`SIG_NONPAY_W-1:0]      WR_TX_HDR_NONPAY_LEN,
     input [`SIG_PACKETLEN_W-1:0]   WR_TX_HDR_PACKET_LEN,
     input                          WR_TX_HDR_NOPAYLOAD,
     output                         WR_TX_HDR_READY,

     // Interface: RD_TX_HDR
     output                         RD_TX_HDR_VALID,
     output [(C_MAX_HDR_WIDTH)-1:0] RD_TX_HDR,
     output [`SIG_LEN_W-1:0]        RD_TX_HDR_PAYLOAD_LEN,
     output [`SIG_NONPAY_W-1:0]     RD_TX_HDR_NONPAY_LEN,
     output [`SIG_PACKETLEN_W-1:0]  RD_TX_HDR_PACKET_LEN,
     output                         RD_TX_HDR_NOPAYLOAD,
     input                          RD_TX_HDR_READY
     );

    // Size of the header, plus the three metadata signals
    localparam C_WIDTH = (C_MAX_HDR_WIDTH) + `SIG_NONPAY_W + `SIG_PACKETLEN_W + 1 + `SIG_LEN_W;

    wire                            RST;

    wire                            wWrTxHdrReady;
    wire                            wWrTxHdrValid;
    wire [(C_MAX_HDR_WIDTH)-1:0]    wWrTxHdr;
    wire [`SIG_NONPAY_W-1:0]        wWrTxHdrNonpayLen;
    wire [`SIG_PACKETLEN_W-1:0]     wWrTxHdrPacketLen;
    wire [`SIG_LEN_W-1:0]           wWrTxHdrPayloadLen; 
    wire                            wWrTxHdrNoPayload;

    wire                            wRdTxHdrReady;
    wire                            wRdTxHdrValid;
    wire [C_MAX_HDR_WIDTH-1:0]      wRdTxHdr;
    wire [`SIG_NONPAY_W-1:0]        wRdTxHdrNonpayLen;
    wire [`SIG_PACKETLEN_W-1:0]     wRdTxHdrPacketLen;
    wire [`SIG_LEN_W-1:0]           wRdTxHdrPayloadLen; 
    wire                            wRdTxHdrNoPayload;

    assign RST = RST_IN;

    pipeline
        #(
          .C_DEPTH              (C_PIPELINE_INPUT?1:0),
          .C_USE_MEMORY         (0),
          /*AUTOINSTPARAM*/
          // Parameters
          .C_WIDTH                      (C_WIDTH))
    input_pipeline_inst
        (
         // Outputs
         .WR_DATA_READY         (WR_TX_HDR_READY),
         .RD_DATA               ({wWrTxHdr,wWrTxHdrNonpayLen,wWrTxHdrPacketLen,wWrTxHdrPayloadLen,wWrTxHdrNoPayload}),
         .RD_DATA_VALID         (wWrTxHdrValid),
         // Inputs
         .WR_DATA               ({WR_TX_HDR,WR_TX_HDR_NONPAY_LEN,WR_TX_HDR_PACKET_LEN,WR_TX_HDR_PAYLOAD_LEN,WR_TX_HDR_NOPAYLOAD}),
         .WR_DATA_VALID         (WR_TX_HDR_VALID),
         .RD_DATA_READY         (wWrTxHdrReady),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));

    fifo
        #(
          // Parameters
          .C_DELAY             (0),
          /*AUTOINSTPARAM*/
          // Parameters
          .C_WIDTH                      (C_WIDTH),
          .C_DEPTH                      (C_DEPTH_PACKETS))
    fifo_inst
        (
         // Outputs
         .RD_DATA              ({wRdTxHdr,wRdTxHdrNonpayLen,wRdTxHdrPacketLen,wRdTxHdrPayloadLen,wRdTxHdrNoPayload}),
         .WR_READY             (wWrTxHdrReady),
         .RD_VALID             (wRdTxHdrValid),
         // Inputs
         .WR_DATA              ({wWrTxHdr,wWrTxHdrNonpayLen,wWrTxHdrPacketLen,wWrTxHdrPayloadLen,wWrTxHdrNoPayload}),
         .WR_VALID             (wWrTxHdrValid),
         .RD_READY             (wRdTxHdrReady),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST                           (RST));

    pipeline
        #(
          .C_DEPTH              (C_PIPELINE_OUTPUT?1:0),
          .C_USE_MEMORY         (0),
          /*AUTOINSTPARAM*/
          // Parameters
          .C_WIDTH                      (C_WIDTH))
    output_pipeline_inst
        (
         // Outputs
         .WR_DATA_READY         (wRdTxHdrReady),
         .RD_DATA               ({RD_TX_HDR,RD_TX_HDR_NONPAY_LEN,RD_TX_HDR_PACKET_LEN,RD_TX_HDR_PAYLOAD_LEN,RD_TX_HDR_NOPAYLOAD}),
         .RD_DATA_VALID         (RD_TX_HDR_VALID),
         // Inputs
         .WR_DATA              ({wRdTxHdr,wRdTxHdrNonpayLen,wRdTxHdrPacketLen,wRdTxHdrPayloadLen,wRdTxHdrNoPayload}),
         .WR_DATA_VALID         (wRdTxHdrValid),
         .RD_DATA_READY         (RD_TX_HDR_READY),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));
endmodule
// Local Variables:
// verilog-library-directories:("." "../../common/")
// End:
