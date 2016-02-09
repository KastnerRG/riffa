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
// Filename:            txc_engine_ultrascale.v
// Version:             1.0
// Verilog Standard:    Verilog-2001
// Description:         The TXC Engine takes unformatted completions, formats
// these packets into AXI-style packets. These packets must meet max-request,
// max-payload, and payload termination requirements (see Read Completion
// Boundary). The TXC Engine does not check these requirements during operation,
// but may do so during simulation.
// 
// This Engine is capable of operating at "line rate".
// 
// Author: Dustin Richmond (@darichmond) 
//-----------------------------------------------------------------------------
`include "trellis.vh"
`include "ultrascale.vh"
module txc_engine_ultrascale
    #(parameter C_PCI_DATA_WIDTH = 128,
      parameter C_PIPELINE_INPUT = 1,
      parameter C_PIPELINE_OUTPUT = 1,
      parameter C_DEPTH_PACKETS = 10,
      parameter C_MAX_PAYLOAD_DWORDS = 256)
    (// Interface: Clocks
     input                                   CLK,

     // Interface: Resets
     input                                   RST_BUS, // Replacement for generic RST_IN
     input                                   RST_LOGIC, // Addition for RIFFA_RST
     output                                  DONE_TXC_RST,

     // Interface: Configuration
     input [`SIG_CPLID_W-1:0]                CONFIG_COMPLETER_ID,

     // Interface: CC
     input                                   S_AXIS_CC_TREADY,
     output                                  S_AXIS_CC_TVALID,
     output                                  S_AXIS_CC_TLAST,
     output [C_PCI_DATA_WIDTH-1:0]           S_AXIS_CC_TDATA,
     output [(C_PCI_DATA_WIDTH/32)-1:0]      S_AXIS_CC_TKEEP,
     output [`SIG_CC_TUSER_W-1:0]            S_AXIS_CC_TUSER,

     // Interface: TXC Engine
     input                                   TXC_DATA_VALID,
     input [C_PCI_DATA_WIDTH-1:0]            TXC_DATA,
     input                                   TXC_DATA_START_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0] TXC_DATA_START_OFFSET,
     input                                   TXC_DATA_END_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0] TXC_DATA_END_OFFSET,
     output                                  TXC_DATA_READY,

     input                                   TXC_META_VALID,
     input [`SIG_FBE_W-1:0]                  TXC_META_FDWBE,
     input [`SIG_LBE_W-1:0]                  TXC_META_LDWBE,
     input [`SIG_LOWADDR_W-1:0]              TXC_META_ADDR,
     input [`SIG_TYPE_W-1:0]                 TXC_META_TYPE,
     input [`SIG_LEN_W-1:0]                  TXC_META_LENGTH,
     input [`SIG_BYTECNT_W-1:0]              TXC_META_BYTE_COUNT,
     input [`SIG_TAG_W-1:0]                  TXC_META_TAG,
     input [`SIG_REQID_W-1:0]                TXC_META_REQUESTER_ID,
     input [`SIG_TC_W-1:0]                   TXC_META_TC,
     input [`SIG_ATTR_W-1:0]                 TXC_META_ATTR,
     input                                   TXC_META_EP,
     output                                  TXC_META_READY
     );

    localparam C_VENDOR = "XILINX";
    localparam C_DATA_WIDTH = C_PCI_DATA_WIDTH;
    localparam C_MAX_HDR_WIDTH = 128; // It's really 96... But it gets trimmed
    localparam C_MAX_HDR_DWORDS = C_MAX_HDR_WIDTH/32;
    localparam C_MAX_ALIGN_DWORDS = 0;
    localparam C_MAX_NONPAY_DWORDS = C_MAX_HDR_DWORDS + C_MAX_ALIGN_DWORDS;
    //
    localparam C_PIPELINE_FORMATTER_INPUT = C_PIPELINE_INPUT;
    localparam C_PIPELINE_FORMATTER_OUTPUT = 1;
    localparam C_FORMATTER_DELAY = 1 + C_PIPELINE_FORMATTER_INPUT;
    localparam C_RST_COUNT = 10;
    /*AUTOWIRE*/
    // Beginning of automatic wires (for undeclared instantiated-module outputs)
    wire                RST_OUT;                // From txc_trans_inst of txc_translation_layer.v
    // End of automatics
    /*AUTOINPUT*/
    ///*AUTOOUTPUT*/

    wire                                     wTxHdrReady;
    wire                                     wTxHdrValid;
    wire [C_MAX_HDR_WIDTH-1:0]               wTxHdr;
    wire [`SIG_NONPAY_W-1:0]                 wTxHdrNonpayLen;
    wire [`SIG_PACKETLEN_W-1:0]              wTxHdrPacketLen;
    wire [`SIG_LEN_W-1:0]                    wTxHdrPayloadLen; 
    wire                                     wTxHdrNopayload;

    wire                                     wTxDataReady;
    wire [C_PCI_DATA_WIDTH-1:0]              wTxData;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]   wTxDataEndOffset;
    wire                                     wTxDataStartFlag;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]         wTxDataEndFlags;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]         wTxDataWordValid;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]         wTxDataWordReady;

    wire [C_PCI_DATA_WIDTH-1:0]              wTxcPkt;
    wire                                     wTxcPktEndFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]   wTxcPktEndOffset;
    wire                                     wTxcPktStartFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]   wTxcPktStartOffset;
    wire                                     wTxcPktValid;
    wire                                     wTxcPktReady;

    wire                                     wTransDoneRst;
    wire                                     wTransRstOut;
    wire                                     wDoneEngRst;
    wire                                     wRst;
    wire [C_RST_COUNT:0]                     wShiftRegRst;
    
    assign DONE_TXC_RST = wTransDoneRst & wDoneEngRst;
    assign wRst = wShiftRegRst[C_RST_COUNT-3];
    assign wDoneEngRst = ~wShiftRegRst[C_RST_COUNT];

    shiftreg
        #(// Parameters
          .C_DEPTH                      (C_RST_COUNT),
          .C_WIDTH                      (1),
          .C_VALUE                      (1)
          /*AUTOINSTPARAM*/)
    rst_shiftreg
        (// Outputs
         .RD_DATA                       (wShiftRegRst),
         // Inputs
         .RST_IN                        (RST_BUS),
         .WR_DATA                       (wTransRstOut),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    txc_formatter_ultrascale
        #(// Parameters
          .C_PIPELINE_OUTPUT            (C_PIPELINE_FORMATTER_OUTPUT),
          .C_PIPELINE_INPUT             (C_PIPELINE_FORMATTER_INPUT),
          /*AUTOINSTPARAM*/
          // Parameters
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
          .C_MAX_HDR_WIDTH              (C_MAX_HDR_WIDTH))
    txc_formatter_inst
        (// Outputs
         .TX_HDR_VALID                  (wTxHdrValid),
         .TX_HDR                        (wTxHdr[C_MAX_HDR_WIDTH-1:0]),
         .TX_HDR_NOPAYLOAD              (wTxHdrNopayload),
         .TX_HDR_PAYLOAD_LEN            (wTxHdrPayloadLen[`SIG_LEN_W-1:0]),
         .TX_HDR_NONPAY_LEN             (wTxHdrNonpayLen[`SIG_NONPAY_W-1:0]),
         .TX_HDR_PACKET_LEN             (wTxHdrPacketLen[`SIG_PACKETLEN_W-1:0]),
         // Inputs
         .TX_HDR_READY                  (wTxHdrReady),
         .RST_IN                        (wRst),
         /*AUTOINST*/
         // Outputs
         .TXC_META_READY                (TXC_META_READY),
         // Inputs
         .CLK                           (CLK),
         .CONFIG_COMPLETER_ID           (CONFIG_COMPLETER_ID[`SIG_CPLID_W-1:0]),
         .TXC_META_VALID                (TXC_META_VALID),
         .TXC_META_FDWBE                (TXC_META_FDWBE[`SIG_FBE_W-1:0]),
         .TXC_META_LDWBE                (TXC_META_LDWBE[`SIG_LBE_W-1:0]),
         .TXC_META_ADDR                 (TXC_META_ADDR[`SIG_LOWADDR_W-1:0]),
         .TXC_META_LENGTH               (TXC_META_LENGTH[`SIG_LEN_W-1:0]),
         .TXC_META_TYPE                 (TXC_META_TYPE[`SIG_TYPE_W-1:0]),
         .TXC_META_BYTE_COUNT           (TXC_META_BYTE_COUNT[`SIG_BYTECNT_W-1:0]),
         .TXC_META_TAG                  (TXC_META_TAG[`SIG_TAG_W-1:0]),
         .TXC_META_REQUESTER_ID         (TXC_META_REQUESTER_ID[`SIG_REQID_W-1:0]),
         .TXC_META_TC                   (TXC_META_TC[`SIG_TC_W-1:0]),
         .TXC_META_ATTR                 (TXC_META_ATTR[`SIG_ATTR_W-1:0]),
         .TXC_META_EP                   (TXC_META_EP));

    tx_engine
        #(.C_DATA_WIDTH                 (C_PCI_DATA_WIDTH),
          /*AUTOINSTPARAM*/
          // Parameters
          .C_DEPTH_PACKETS              (C_DEPTH_PACKETS),
          .C_PIPELINE_INPUT             (C_PIPELINE_INPUT),
          .C_PIPELINE_OUTPUT            (C_PIPELINE_OUTPUT),
          .C_FORMATTER_DELAY            (C_FORMATTER_DELAY),
          .C_MAX_HDR_WIDTH              (C_MAX_HDR_WIDTH),
          .C_MAX_PAYLOAD_DWORDS         (C_MAX_PAYLOAD_DWORDS),
          .C_VENDOR                     (C_VENDOR))
    txc_engine_inst
        (// Outputs
         .TX_HDR_READY                  (wTxHdrReady),
         .TX_DATA_READY                 (TXC_DATA_READY),
         .TX_PKT                        (wTxcPkt[C_DATA_WIDTH-1:0]),
         .TX_PKT_START_FLAG             (wTxcPktStartFlag),
         .TX_PKT_START_OFFSET           (wTxcPktStartOffset[clog2s(C_DATA_WIDTH/32)-1:0]),
         .TX_PKT_END_FLAG               (wTxcPktEndFlag),
         .TX_PKT_END_OFFSET             (wTxcPktEndOffset[clog2s(C_DATA_WIDTH/32)-1:0]),
         .TX_PKT_VALID                  (wTxcPktValid),
         // Inputs
         .TX_HDR_VALID                  (wTxHdrValid),
         .TX_HDR                        (wTxHdr[C_MAX_HDR_WIDTH-1:0]),
         .TX_HDR_NOPAYLOAD              (wTxHdrNopayload),
         .TX_HDR_PAYLOAD_LEN            (wTxHdrPayloadLen[`SIG_LEN_W-1:0]),
         .TX_HDR_NONPAY_LEN             (wTxHdrNonpayLen[`SIG_NONPAY_W-1:0]),
         .TX_HDR_PACKET_LEN             (wTxHdrPacketLen[`SIG_PACKETLEN_W-1:0]),
         .TX_DATA_VALID                 (TXC_DATA_VALID),
         .TX_DATA                       (TXC_DATA[C_DATA_WIDTH-1:0]),
         .TX_DATA_START_FLAG            (TXC_DATA_START_FLAG),
         .TX_DATA_START_OFFSET          (TXC_DATA_START_OFFSET[clog2s(C_DATA_WIDTH/32)-1:0]),
         .TX_DATA_END_FLAG              (TXC_DATA_END_FLAG),
         .TX_DATA_END_OFFSET            (TXC_DATA_END_OFFSET[clog2s(C_DATA_WIDTH/32)-1:0]),
         .TX_PKT_READY                  (wTxcPktReady),
         .RST_IN                        (wRst),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    txc_translation_layer
        #(/*AUTOINSTPARAM*/
          // Parameters
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
          .C_PIPELINE_INPUT             (C_PIPELINE_INPUT))
    txc_trans_inst
        (// Outputs
         .TXC_PKT_READY                 (wTxcPktReady),
         .DONE_RST                      (wTransDoneRst),
         .RST_OUT                       (wTransRstOut),
         // Inputs
         .TXC_PKT                       (wTxcPkt),
         .TXC_PKT_VALID                 (wTxcPktValid),
         .TXC_PKT_START_FLAG            (wTxcPktStartFlag),
         .TXC_PKT_START_OFFSET          (wTxcPktStartOffset),
         .TXC_PKT_END_FLAG              (wTxcPktEndFlag),
         .TXC_PKT_END_OFFSET            (wTxcPktEndOffset),
         /*AUTOINST*/
         // Outputs
         .S_AXIS_CC_TVALID              (S_AXIS_CC_TVALID),
         .S_AXIS_CC_TLAST               (S_AXIS_CC_TLAST),
         .S_AXIS_CC_TDATA               (S_AXIS_CC_TDATA[C_PCI_DATA_WIDTH-1:0]),
         .S_AXIS_CC_TKEEP               (S_AXIS_CC_TKEEP[(C_PCI_DATA_WIDTH/32)-1:0]),
         .S_AXIS_CC_TUSER               (S_AXIS_CC_TUSER[`SIG_CC_TUSER_W-1:0]),
         // Inputs
         .CLK                           (CLK),
         .RST_BUS                       (RST_BUS),
         .RST_LOGIC                     (RST_LOGIC),
         .S_AXIS_CC_TREADY              (S_AXIS_CC_TREADY));

endmodule // txc_engine_ultrascale

module txc_formatter_ultrascale
    #(
      parameter C_PCI_DATA_WIDTH = 128,
      parameter C_PIPELINE_INPUT = 1,
      parameter C_PIPELINE_OUTPUT = 1,
      parameter C_MAX_HDR_WIDTH = `UPKT_TXC_MAXHDR_W
      )
    (
     // Interface: Clocks
     input                         CLK,

     // Interface: Resets
     input                         RST_IN,

     // Interface: Configuration
     input [`SIG_CPLID_W-1:0]      CONFIG_COMPLETER_ID,

     // Interface: TXC
     input                         TXC_META_VALID,
     input [`SIG_FBE_W-1:0]        TXC_META_FDWBE,
     input [`SIG_LBE_W-1:0]        TXC_META_LDWBE,
     input [`SIG_LOWADDR_W-1:0]    TXC_META_ADDR,
     input [`SIG_LEN_W-1:0]        TXC_META_LENGTH,
     input [`SIG_TYPE_W-1:0]       TXC_META_TYPE,
     input [`SIG_BYTECNT_W-1:0]    TXC_META_BYTE_COUNT,
     input [`SIG_TAG_W-1:0]        TXC_META_TAG,
     input [`SIG_REQID_W-1:0]      TXC_META_REQUESTER_ID,
     input [`SIG_TC_W-1:0]         TXC_META_TC,
     input [`SIG_ATTR_W-1:0]       TXC_META_ATTR,
     input                         TXC_META_EP,
     output                        TXC_META_READY,

     // Interface: TX HDR
     output                        TX_HDR_VALID,
     output [C_MAX_HDR_WIDTH-1:0]  TX_HDR,
     output [`SIG_LEN_W-1:0]       TX_HDR_PAYLOAD_LEN,
     output [`SIG_NONPAY_W-1:0]    TX_HDR_NONPAY_LEN,
     output [`SIG_PACKETLEN_W-1:0] TX_HDR_PACKET_LEN,
     output                        TX_HDR_NOPAYLOAD,
     input                         TX_HDR_READY
     );

    wire [`UPKT_TXC_MAXHDR_W-1:0]  wHdr;

    wire                           wTxHdrReady;
    wire                           wTxHdrValid;
    wire [C_MAX_HDR_WIDTH-1:0]     wTxHdr;
    wire [`SIG_TYPE_W-1:0]         wTxType;
    wire [`SIG_NONPAY_W-1:0]       wTxHdrNonpayLen;
    wire [`SIG_PACKETLEN_W-1:0]    wTxHdrPacketLen;
    wire [`SIG_LEN_W-1:0]          wTxHdrPayloadLen; 
    wire                           wTxHdrNopayload;
    
    // Generic Header Fields
    // ATYPE Should be copied from the request parameters, but we only use 0
    assign wHdr[`UPKT_TXC_ADDRLOW_R] = TXC_META_ADDR;
    assign wHdr[`UPKT_TXC_RSVD0_R] = `UPKT_TXC_RSVD0_W'd0;
    assign wHdr[`UPKT_TXC_ATYPE_R] = `UPKT_TXC_ATYPE_W'd0;
    assign wHdr[`UPKT_TXC_RSVD1_R] = `UPKT_TXC_RSVD1_W'd0;
    assign wHdr[`UPKT_TXC_BYTECNT_R] = {1'b0,TXC_META_BYTE_COUNT};
    assign wHdr[`UPKT_TXC_LOCKED_R] = `UPKT_TXC_LOCKED_W'd0;
    assign wHdr[`UPKT_TXC_RSVD2_R] = `UPKT_TXC_RSVD2_W'd0;
    assign wHdr[`UPKT_TXC_LENGTH_R] = {1'b0, TXC_META_LENGTH};
    assign wHdr[`UPKT_TXC_STATUS_R] = `UPKT_TXC_STATUS_W'd0;
    assign wHdr[`UPKT_TXC_EP_R] = TXC_META_EP;
    assign wHdr[`UPKT_TXC_RSVD3_R] = `UPKT_TXC_RSVD3_W'd0;
    assign wHdr[`UPKT_TXC_REQID_R] = TXC_META_REQUESTER_ID;
    assign wHdr[`UPKT_TXC_TAG_R] = TXC_META_TAG;
    assign wHdr[`UPKT_TXC_CPLID_R] = CONFIG_COMPLETER_ID;
    assign wHdr[`UPKT_TXC_CPLIDEN_R] = 1'b0;
    assign wHdr[`UPKT_TXC_TC_R] = TXC_META_TC;
    assign wHdr[`UPKT_TXC_ATTR_R] = TXC_META_ATTR;
    assign wHdr[`UPKT_TXC_TD_R] = `UPKT_TXC_TD_W'd0;
    
    assign wTxHdrNopayload = ~wTxType[`TRLS_TYPE_PAY_I];
    assign wTxHdrNonpayLen = 3;
    assign wTxHdrPayloadLen = wTxHdrNopayload ? 0 : wTxHdr[`UPKT_TXC_LENGTH_I +: `SIG_LEN_W];
    assign wTxHdrPacketLen = wTxHdrPayloadLen + wTxHdrNonpayLen;

    pipeline
        #(
          // Parameters
          .C_DEPTH                      (C_PIPELINE_INPUT?1:0),
          .C_WIDTH                      (C_MAX_HDR_WIDTH + `SIG_TYPE_W),
          .C_USE_MEMORY                 (0)
          /*AUTOINSTPARAM*/)
    input_inst
        (
         // Outputs
         .WR_DATA_READY                 (TXC_META_READY),
         .RD_DATA                       ({wTxHdr,wTxType}),
         .RD_DATA_VALID                 (wTxHdrValid),
         // Inputs
         .WR_DATA                       ({32'b0,wHdr,TXC_META_TYPE}),
         .WR_DATA_VALID                 (TXC_META_VALID),
         .RD_DATA_READY                 (wTxHdrReady),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));

    pipeline
        #(
          // Parameters
          .C_DEPTH                      (C_PIPELINE_OUTPUT?1:0),
          .C_WIDTH                      (C_MAX_HDR_WIDTH+ 1 + `SIG_PACKETLEN_W + `SIG_LEN_W + `SIG_NONPAY_W),
          .C_USE_MEMORY                 (0)
          /*AUTOINSTPARAM*/)
    output_inst
        (
         // Outputs
         .WR_DATA_READY                 (wTxHdrReady),
         .RD_DATA                       ({TX_HDR,TX_HDR_NOPAYLOAD,TX_HDR_PACKET_LEN,TX_HDR_PAYLOAD_LEN,TX_HDR_NONPAY_LEN}),
         .RD_DATA_VALID                 (TX_HDR_VALID),
         // Inputs
         .WR_DATA                       ({wTxHdr,wTxHdrNopayload,wTxHdrPacketLen,wTxHdrPayloadLen,wTxHdrNonpayLen}),
         .WR_DATA_VALID                 (wTxHdrValid),
         .RD_DATA_READY                 (TX_HDR_READY),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));
endmodule

module txc_translation_layer
    #(parameter C_PCI_DATA_WIDTH = 10'd128,
      parameter C_PIPELINE_INPUT = 1)
    (// Interface: Clocks
     input                                   CLK,

     // Interface: Resets
     input                                   RST_BUS, // Replacement for generic RST_IN
     input                                   RST_LOGIC, // Addition for RIFFA_RST
     output                                  DONE_RST,
     output                                  RST_OUT,
     // Interface: TXC Classic
     output                                  TXC_PKT_READY,
     input [C_PCI_DATA_WIDTH-1:0]            TXC_PKT,
     input                                   TXC_PKT_VALID,
     input                                   TXC_PKT_START_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0] TXC_PKT_START_OFFSET,
     input                                   TXC_PKT_END_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0] TXC_PKT_END_OFFSET,

     // Interface: CC
     input                                   S_AXIS_CC_TREADY,
     output                                  S_AXIS_CC_TVALID,
     output                                  S_AXIS_CC_TLAST,
     output [C_PCI_DATA_WIDTH-1:0]           S_AXIS_CC_TDATA,
     output [(C_PCI_DATA_WIDTH/32)-1:0]      S_AXIS_CC_TKEEP,
     output [`SIG_CC_TUSER_W-1:0]            S_AXIS_CC_TUSER
     );

    localparam C_INPUT_STAGES = C_PIPELINE_INPUT != 0? 1:0;
    localparam C_OUTPUT_STAGES = 1;
    localparam C_RST_COUNT = 10;
    wire                                     wTxcPktReady;
    wire [C_PCI_DATA_WIDTH-1:0]              wTxcPkt;
    wire                                     wTxcPktValid;
    wire                                     wTxcPktStartFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]   wTxcPktStartOffset;
    wire                                     wTxcPktEndFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]   wTxcPktEndOffset;

    wire                                     wSAxisCcTReady;
    wire                                     wSAxisCcTValid;
    wire                                     wSAxisCcTLast;
    wire [C_PCI_DATA_WIDTH-1:0]              wSAxisCcTData;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]         wSAxisCcTKeep;
    wire [`SIG_CC_TUSER_W-1:0]               wSAxisCcTUser;

    wire                                     wRst;
    wire                                     wRstWaiting;
    /*ASSIGN TXC -> CC*/
    assign wTxcPktReady = wSAxisCcTReady;
    assign wSAxisCcTValid = wTxcPktValid;
    assign wSAxisCcTLast = wTxcPktEndFlag;
    assign wSAxisCcTData = wTxcPkt;
    // Do not enable parity bits, and no discontinues
    assign S_AXIS_CC_TUSER = `SIG_CC_TUSER_W'd0;
    assign RST_OUT = wRst;

    // This reset controller assumes there is always an output stage
    reset_controller
        #(/*AUTOINSTPARAM*/
          // Parameters
          .C_RST_COUNT                  (C_RST_COUNT))
    rc
        (// Outputs
         .RST_OUT                       (wRst),
         .WAITING_RESET                 (wRstWaiting),
         // Inputs
         .RST_IN                        (RST_BUS),
         .SIGNAL_RST                    (RST_LOGIC),
         .WAIT_RST                      (S_AXIS_CC_TVALID),
         .NEXT_CYC_RST                  (S_AXIS_CC_TREADY & S_AXIS_CC_TLAST),
         /*AUTOINST*/
         // Outputs
         .DONE_RST                      (DONE_RST),
         // Inputs
         .CLK                           (CLK));
    
    pipeline
        #(// Parameters
          .C_DEPTH                      (C_INPUT_STAGES),
          .C_WIDTH                      (C_PCI_DATA_WIDTH + 2*(1+clog2s(C_PCI_DATA_WIDTH/32))),
          .C_USE_MEMORY                 (0)
          /*AUTOINSTPARAM*/)
    input_inst
        (// Outputs
         .WR_DATA_READY                 (TXC_PKT_READY),
         .RD_DATA                       ({wTxcPkt,wTxcPktStartFlag,wTxcPktStartOffset,wTxcPktEndFlag,wTxcPktEndOffset}),
         .RD_DATA_VALID                 (wTxcPktValid),
         // Inputs
         .WR_DATA                       ({TXC_PKT,TXC_PKT_START_FLAG,TXC_PKT_START_OFFSET,
                                          TXC_PKT_END_FLAG,TXC_PKT_END_OFFSET}),
         .WR_DATA_VALID                 (TXC_PKT_VALID),
         .RD_DATA_READY                 (wTxcPktReady),
         .RST_IN                        (wRst),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    offset_to_mask
        #(// Parameters
          .C_MASK_SWAP                  (0),
          .C_MASK_WIDTH                 (C_PCI_DATA_WIDTH/32)
          /*AUTOINSTPARAM*/)
    otom_inst
        (// Outputs
         .MASK                          (wSAxisCcTKeep),
         // Inputs
         .OFFSET_ENABLE                 (wTxcPktEndFlag),
         .OFFSET                        (wTxcPktEndOffset)
         /*AUTOINST*/);
    
    pipeline
        #(// Parameters
          .C_DEPTH                      (C_OUTPUT_STAGES),
          .C_WIDTH                      (C_PCI_DATA_WIDTH + 1 + (C_PCI_DATA_WIDTH/32)),
          .C_USE_MEMORY                 (0)
          /*AUTOINSTPARAM*/)
    output_inst
        (
         // Outputs
         .WR_DATA_READY                 (wSAxisCcTReady),
         .RD_DATA                       ({S_AXIS_CC_TDATA,S_AXIS_CC_TLAST,S_AXIS_CC_TKEEP}),
         .RD_DATA_VALID                 (S_AXIS_CC_TVALID),
         // Inputs
         .WR_DATA                       ({wSAxisCcTData,wSAxisCcTLast,wSAxisCcTKeep}),
         .WR_DATA_VALID                 (wSAxisCcTValid & ~wRstWaiting),
         .RD_DATA_READY                 (S_AXIS_CC_TREADY),
         .RST_IN                        (wRst),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

endmodule
// Local Variables:
// verilog-library-directories:("." "../../../common/" "../../common/")
// End:
