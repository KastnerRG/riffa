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
// Filename:            txr_engine_ultrascale.v
// Version:             1.0
// Verilog Standard:    Verilog-2001
// Description:         The TXR Engine takes unformatted completions, formats
// these packets into AXI-style packets. These packets must meet max-request,
// max-payload, and payload termination requirements (see Read Completion
// Boundary). The TXR Engine does not check these requirements during operation,
// but may do so during simulation.
// 
// This Engine is capable of operating at "line rate".
// 
// Author: Dustin Richmond (@darichmond) 
//-----------------------------------------------------------------------------
`include "trellis.vh"
`include "ultrascale.vh"
module txr_engine_ultrascale
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
     output                                  DONE_TXR_RST,

     // Interface: Configuration
     input [`SIG_CPLID_W-1:0]                CONFIG_COMPLETER_ID,

     // Interface: RQ
     input                                   S_AXIS_RQ_TREADY,
     output                                  S_AXIS_RQ_TVALID,
     output                                  S_AXIS_RQ_TLAST,
     output [C_PCI_DATA_WIDTH-1:0]           S_AXIS_RQ_TDATA,
     output [(C_PCI_DATA_WIDTH/32)-1:0]      S_AXIS_RQ_TKEEP,
     output [`SIG_RQ_TUSER_W-1:0]            S_AXIS_RQ_TUSER,

     // Interface: TXR Engine
     input                                   TXR_DATA_VALID,
     input [C_PCI_DATA_WIDTH-1:0]            TXR_DATA,
     input                                   TXR_DATA_START_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0] TXR_DATA_START_OFFSET,
     input                                   TXR_DATA_END_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0] TXR_DATA_END_OFFSET,
     output                                  TXR_DATA_READY,

     input                                   TXR_META_VALID,
     input [`SIG_FBE_W-1:0]                  TXR_META_FDWBE, 
     input [`SIG_LBE_W-1:0]                  TXR_META_LDWBE,
     input [`SIG_ADDR_W-1:0]                 TXR_META_ADDR,
     input [`SIG_LEN_W-1:0]                  TXR_META_LENGTH,
     input [`SIG_TAG_W-1:0]                  TXR_META_TAG,
     input [`SIG_TC_W-1:0]                   TXR_META_TC,
     input [`SIG_ATTR_W-1:0]                 TXR_META_ATTR,
     input [`SIG_TYPE_W-1:0]                 TXR_META_TYPE,
     input                                   TXR_META_EP,
     output                                  TXR_META_READY     
     );

    localparam C_VENDOR = "XILINX";
    localparam C_DATA_WIDTH = C_PCI_DATA_WIDTH;
    localparam C_MAX_HDR_WIDTH = `UPKT_TXR_MAXHDR_W;
    localparam C_MAX_HDR_DWORDS = C_MAX_HDR_WIDTH/32;
    localparam C_MAX_ALIGN_DWORDS = 0;
    localparam C_MAX_NONPAY_DWORDS = C_MAX_HDR_DWORDS + C_MAX_ALIGN_DWORDS + 1;
    localparam C_MAX_PACKET_DWORDS = C_MAX_NONPAY_DWORDS + C_MAX_PAYLOAD_DWORDS;
    localparam C_PIPELINE_FORMATTER_INPUT = C_PIPELINE_INPUT;
    localparam C_PIPELINE_FORMATTER_OUTPUT = C_PIPELINE_OUTPUT;
    localparam C_FORMATTER_DELAY = C_PIPELINE_FORMATTER_OUTPUT + C_PIPELINE_FORMATTER_INPUT;
    localparam C_RST_COUNT = 10;
    /*AUTOWIRE*/
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

    wire [C_PCI_DATA_WIDTH-1:0]              wTxrPkt;
    wire                                     wTxrPktEndFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]   wTxrPktEndOffset;
    wire                                     wTxrPktStartFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]   wTxrPktStartOffset;
    wire                                     wTxrPktValid;
    wire                                     wTxrPktReady;

    wire                                     wTransDoneRst;
    wire                                     wTransRstOut;
    wire                                     wDoneEngRst;
    wire                                     wRst;
    wire [C_RST_COUNT:0]                     wShiftRegRst;

    assign DONE_TXR_RST = wTransDoneRst & wDoneEngRst;
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
         
    txr_formatter_ultrascale
        #(.C_PIPELINE_OUTPUT            (C_PIPELINE_FORMATTER_OUTPUT),
          .C_PIPELINE_INPUT             (C_PIPELINE_FORMATTER_INPUT),
          /*AUTOINSTPARAM*/
          // Parameters
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
          .C_MAX_HDR_WIDTH              (C_MAX_HDR_WIDTH),
          .C_MAX_NONPAY_DWORDS          (C_MAX_NONPAY_DWORDS),
          .C_MAX_PACKET_DWORDS          (C_MAX_PACKET_DWORDS))
    txr_formatter_inst
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
         .TXR_META_READY                (TXR_META_READY),
         // Inputs
         .CLK                           (CLK),
         .CONFIG_COMPLETER_ID           (CONFIG_COMPLETER_ID[`SIG_CPLID_W-1:0]),
         .TXR_META_VALID                (TXR_META_VALID),
         .TXR_META_FDWBE                (TXR_META_FDWBE[`SIG_FBE_W-1:0]),
         .TXR_META_LDWBE                (TXR_META_LDWBE[`SIG_LBE_W-1:0]),
         .TXR_META_ADDR                 (TXR_META_ADDR[`SIG_ADDR_W-1:0]),
         .TXR_META_LENGTH               (TXR_META_LENGTH[`SIG_LEN_W-1:0]),
         .TXR_META_TAG                  (TXR_META_TAG[`SIG_TAG_W-1:0]),
         .TXR_META_TC                   (TXR_META_TC[`SIG_TC_W-1:0]),
         .TXR_META_ATTR                 (TXR_META_ATTR[`SIG_ATTR_W-1:0]),
         .TXR_META_TYPE                 (TXR_META_TYPE[`SIG_TYPE_W-1:0]),
         .TXR_META_EP                   (TXR_META_EP));

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
    txr_engine_inst
        (// Outputs
         .TX_HDR_READY                  (wTxHdrReady),
         .TX_DATA_READY                 (TXR_DATA_READY),
         .TX_PKT                        (wTxrPkt[C_DATA_WIDTH-1:0]),
         .TX_PKT_START_FLAG             (wTxrPktStartFlag),
         .TX_PKT_START_OFFSET           (wTxrPktStartOffset[clog2s(C_DATA_WIDTH/32)-1:0]),
         .TX_PKT_END_FLAG               (wTxrPktEndFlag),
         .TX_PKT_END_OFFSET             (wTxrPktEndOffset[clog2s(C_DATA_WIDTH/32)-1:0]),
         .TX_PKT_VALID                  (wTxrPktValid),
         // Inputs
         .TX_HDR_VALID                  (wTxHdrValid),
         .TX_HDR                        (wTxHdr[C_MAX_HDR_WIDTH-1:0]),
         .TX_HDR_NOPAYLOAD              (wTxHdrNopayload),
         .TX_HDR_PAYLOAD_LEN            (wTxHdrPayloadLen[`SIG_LEN_W-1:0]),
         .TX_HDR_NONPAY_LEN             (wTxHdrNonpayLen[`SIG_NONPAY_W-1:0]),
         .TX_HDR_PACKET_LEN             (wTxHdrPacketLen[`SIG_PACKETLEN_W-1:0]),
         .TX_DATA_VALID                 (TXR_DATA_VALID),
         .TX_DATA                       (TXR_DATA[C_DATA_WIDTH-1:0]),
         .TX_DATA_START_FLAG            (TXR_DATA_START_FLAG),
         .TX_DATA_START_OFFSET          (TXR_DATA_START_OFFSET[clog2s(C_DATA_WIDTH/32)-1:0]),
         .TX_DATA_END_FLAG              (TXR_DATA_END_FLAG),
         .TX_DATA_END_OFFSET            (TXR_DATA_END_OFFSET[clog2s(C_DATA_WIDTH/32)-1:0]),
         .TX_PKT_READY                  (wTxrPktReady),
         .RST_IN                        (wRst),// TODO: 
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    txr_translation_layer
        #(/*AUTOINSTPARAM*/
          // Parameters
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
          .C_PIPELINE_INPUT             (C_PIPELINE_INPUT),
          .C_RST_COUNT                  (C_RST_COUNT))
    txr_trans_inst
        (// Outputs
         .TXR_PKT_READY                 (wTxrPktReady),
         .DONE_RST                      (wTransDoneRst),
         .RST_OUT                       (wTransRstOut),
         // Inputs
         .TXR_PKT                       (wTxrPkt),
         .TXR_PKT_VALID                 (wTxrPktValid),
         .TXR_PKT_START_FLAG            (wTxrPktStartFlag),
         .TXR_PKT_START_OFFSET          (wTxrPktStartOffset),
         .TXR_PKT_END_FLAG              (wTxrPktEndFlag),
         .TXR_PKT_END_OFFSET            (wTxrPktEndOffset),
         /*AUTOINST*/
         // Outputs
         .S_AXIS_RQ_TVALID              (S_AXIS_RQ_TVALID),
         .S_AXIS_RQ_TLAST               (S_AXIS_RQ_TLAST),
         .S_AXIS_RQ_TDATA               (S_AXIS_RQ_TDATA[C_PCI_DATA_WIDTH-1:0]),
         .S_AXIS_RQ_TKEEP               (S_AXIS_RQ_TKEEP[(C_PCI_DATA_WIDTH/32)-1:0]),
         .S_AXIS_RQ_TUSER               (S_AXIS_RQ_TUSER[`SIG_RQ_TUSER_W-1:0]),
         // Inputs
         .CLK                           (CLK),
         .RST_BUS                       (RST_BUS),
         .RST_LOGIC                     (RST_LOGIC),
         .S_AXIS_RQ_TREADY              (S_AXIS_RQ_TREADY));
endmodule // txr_engine_ultrascale


module txr_formatter_ultrascale
    #(
      parameter C_PCI_DATA_WIDTH = 128,
      parameter C_PIPELINE_INPUT = 1,
      parameter C_PIPELINE_OUTPUT = 1,
      parameter C_MAX_HDR_WIDTH = `UPKT_TXR_MAXHDR_W,
      parameter C_MAX_NONPAY_DWORDS = 5,
      parameter C_MAX_PACKET_DWORDS = 10
      )
    (
     // Interface: Clocks
     input                         CLK,

     // Interface: Resets
     input                         RST_IN,

     // Interface: Configuration
     input [`SIG_CPLID_W-1:0]      CONFIG_COMPLETER_ID,

     // Interface: TXR
     input                         TXR_META_VALID,
     input [`SIG_FBE_W-1:0]        TXR_META_FDWBE,
     input [`SIG_LBE_W-1:0]        TXR_META_LDWBE,
     input [`SIG_ADDR_W-1:0]       TXR_META_ADDR,
     input [`SIG_LEN_W-1:0]        TXR_META_LENGTH,
     input [`SIG_TAG_W-1:0]        TXR_META_TAG,
     input [`SIG_TC_W-1:0]         TXR_META_TC,
     input [`SIG_ATTR_W-1:0]       TXR_META_ATTR,
     input [`SIG_TYPE_W-1:0]       TXR_META_TYPE,
     input                         TXR_META_EP,
     output                        TXR_META_READY,

     // Interface: TX HDR
     output                        TX_HDR_VALID,
     output [C_MAX_HDR_WIDTH-1:0]  TX_HDR,
     output [`SIG_LEN_W-1:0]       TX_HDR_PAYLOAD_LEN,
     output [`SIG_NONPAY_W-1:0]    TX_HDR_NONPAY_LEN,
     output [`SIG_PACKETLEN_W-1:0] TX_HDR_PACKET_LEN,
     output                        TX_HDR_NOPAYLOAD,
     input                         TX_HDR_READY
     );

    wire                           wHdrNoPayload;
    wire [`UPKT_TXR_MAXHDR_W-1:0]  wHdr;
    
    wire                           wTxHdrReady;
    wire                           wTxHdrValid;
    wire [`UPKT_TXR_MAXHDR_W-1:0]  wTxHdr;
    wire [`SIG_NONPAY_W-1:0]       wTxHdrNonpayLen;
    wire [`SIG_PACKETLEN_W-1:0]    wTxHdrPacketLen;
    wire [`SIG_LEN_W-1:0]          wTxHdrPayloadLen; 
    wire                           wTxHdrNopayload;
    wire [`SIG_TYPE_W-1:0]         wTxHdrType;
    
    // Generic Header Fields
    assign wHdr[`UPKT_TXR_ATYPE_R] = `UPKT_TXR_ATYPE_W'd0;
    assign wHdr[`UPKT_TXR_ADDR_R] = TXR_META_ADDR[63:2];
    assign wHdr[`UPKT_TXR_LENGTH_R] = {1'b0,TXR_META_LENGTH};
    assign wHdr[`UPKT_TXR_EP_R] = TXR_META_EP;
`ifdef BE_HACK
    assign wHdr[`UPKT_TXR_FBE_R] = TXR_META_FDWBE;
    assign wHdr[`UPKT_TXR_LBE_R] = TXR_META_LDWBE;
    assign wHdr[`UPKT_TXR_RSVD0_R] = 0;

`else
    assign wHdr[`UPKT_TXR_REQID_R] = CONFIG_COMPLETER_ID;
`endif
    //assign wHdr[`UPKT_TXR_REQID_R] = `UPKT_TXR_REQID_W'd0;
    assign wHdr[`UPKT_TXR_TAG_R] = TXR_META_TAG;
    assign wHdr[`UPKT_TXR_CPLID_R] = `UPKT_TXR_CPLID_W'd0;
    assign wHdr[`UPKT_TXR_REQIDEN_R] = 0;
    assign wHdr[`UPKT_TXR_TC_R] = TXR_META_TC;
    assign wHdr[`UPKT_TXR_ATTR_R] = TXR_META_ATTR;
    assign wHdr[`UPKT_TXR_TD_R] = `UPKT_TXR_TD_W'd0;

    assign wTxHdr[`UPKT_TXR_TYPE_R] = trellis_to_upkt_type(wTxHdrType);
    assign wTxHdrNopayload = ~wTxHdrType[`TRLS_TYPE_PAY_I];
    assign wTxHdrNonpayLen = 4;
    assign wTxHdrPayloadLen = wTxHdrNopayload ? 0 : wTxHdr[`UPKT_TXR_LENGTH_R];
    assign wTxHdrPacketLen = wTxHdrNonpayLen + wTxHdrPayloadLen;

    pipeline
        #(
          // Parameters
          .C_DEPTH                         (C_PIPELINE_INPUT?1:0),
          .C_WIDTH                         (`UPKT_TXR_MAXHDR_W-1),
          .C_USE_MEMORY                    (0)
          /*AUTOINSTPARAM*/)
    input_inst
        (
         // Outputs
         .WR_DATA_READY                  (TXR_META_READY),
         .RD_DATA                        ({wTxHdr[`UPKT_TXR_MAXHDR_W-1:(`UPKT_TXR_TYPE_I + `UPKT_TXR_TYPE_W)],
                                           wTxHdr[`UPKT_TXR_TYPE_I-1:0],
                                           wTxHdrType}),
         .RD_DATA_VALID                  (wTxHdrValid),
         // Inputs
         .WR_DATA                        ({wHdr[`UPKT_TXR_MAXHDR_W-1:(`UPKT_TXR_TYPE_I + `UPKT_TXR_TYPE_W)],
                                           wHdr[`UPKT_TXR_TYPE_I-1:0],
                                           TXR_META_TYPE}),
         .WR_DATA_VALID                  (TXR_META_VALID),
         .RD_DATA_READY                  (wTxHdrReady),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));

    pipeline
        #(
          // Parameters
          .C_DEPTH                         (C_PIPELINE_OUTPUT?1:0),
          .C_WIDTH                         (`UPKT_TXR_MAXHDR_W + 1 + `SIG_PACKETLEN_W + `SIG_LEN_W + `SIG_NONPAY_W),
          .C_USE_MEMORY                    (0)
          /*AUTOINSTPARAM*/)
    output_inst
        (
         // Outputs
         .WR_DATA_READY                  (wTxHdrReady),
         .RD_DATA                        ({TX_HDR,TX_HDR_NOPAYLOAD,TX_HDR_PACKET_LEN,TX_HDR_PAYLOAD_LEN,TX_HDR_NONPAY_LEN}),
         .RD_DATA_VALID                  (TX_HDR_VALID),
         // Inputs
         .WR_DATA                        ({wTxHdr,wTxHdrNopayload,wTxHdrPacketLen,wTxHdrPayloadLen,wTxHdrNonpayLen}),
         .WR_DATA_VALID                  (wTxHdrValid),
         .RD_DATA_READY                  (TX_HDR_READY),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));
endmodule


module txr_translation_layer
    #(parameter C_PCI_DATA_WIDTH = 10'd128,
      parameter C_PIPELINE_INPUT = 1,
      parameter C_RST_COUNT = 1)
    (// Interface: Clocks
     input                                   CLK,

     // Interface: Resets
     input                                   RST_BUS, // Replacement for generic RST_IN
     input                                   RST_LOGIC, // Addition for RIFFA_RST
     output                                  RST_OUT,
     output                                  DONE_RST,

     // Interface: TXR Classic
     output                                  TXR_PKT_READY,
     input [C_PCI_DATA_WIDTH-1:0]            TXR_PKT,
     input                                   TXR_PKT_VALID,
     input                                   TXR_PKT_START_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0] TXR_PKT_START_OFFSET,
     input                                   TXR_PKT_END_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0] TXR_PKT_END_OFFSET,

     // Interface: RQ
     input                                   S_AXIS_RQ_TREADY,
     output                                  S_AXIS_RQ_TVALID,
     output                                  S_AXIS_RQ_TLAST,
     output [C_PCI_DATA_WIDTH-1:0]           S_AXIS_RQ_TDATA,
     output [(C_PCI_DATA_WIDTH/32)-1:0]      S_AXIS_RQ_TKEEP,
     output [`SIG_RQ_TUSER_W-1:0]            S_AXIS_RQ_TUSER);

    localparam C_INPUT_STAGES = C_PIPELINE_INPUT != 0? 1:0;
    localparam C_OUTPUT_STAGES = 1;
    
    wire                                     wTxrPktReady;
    wire [C_PCI_DATA_WIDTH-1:0]              wTxrPkt;
    wire                                     wTxrPktValid;
    wire                                     wTxrPktStartFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]   wTxrPktStartOffset;
    wire                                     wTxrPktEndFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]   wTxrPktEndOffset;

    wire                                     wSAxisRqTReady;
    wire                                     wSAxisRqTValid;
    wire                                     wSAxisRqTLast;
    wire [C_PCI_DATA_WIDTH-1:0]              wSAxisRqTData;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]         wSAxisRqTKeep;
    wire [`SIG_RQ_TUSER_W-1:0]               wSAxisRqTUser;

    wire                                     _wSAxisRqTReady;
    wire                                     _wSAxisRqTValid;
    wire                                     _wSAxisRqTLast;
    wire [C_PCI_DATA_WIDTH-1:0]              _wSAxisRqTData;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]         _wSAxisRqTKeep;

    wire                                     wRst;
    wire                                     wRstWaiting;

    /*ASSIGN TXR -> RQ*/
    assign wTxrPktReady = _wSAxisRqTReady;
    assign _wSAxisRqTValid = wTxrPktValid;
    assign _wSAxisRqTLast = wTxrPktEndFlag;
    assign _wSAxisRqTData = wTxrPkt;

    // BE Hack
    assign wSAxisRqTUser[3:0] = wTxrPkt[(`UPKT_TXR_FBE_I % C_PCI_DATA_WIDTH) +: `UPKT_TXR_FBE_W];
    assign wSAxisRqTUser[7:4] = wTxrPkt[(`UPKT_TXR_LBE_I % C_PCI_DATA_WIDTH) +: `UPKT_TXR_LBE_W];
    assign wSAxisRqTUser[`SIG_RQ_TUSER_W-1:8] = 0;
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
         .WAIT_RST                      (S_AXIS_RQ_TVALID),
         .NEXT_CYC_RST                  (S_AXIS_RQ_TREADY & S_AXIS_RQ_TLAST),
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
        (
         // Outputs
         .WR_DATA_READY                 (TXR_PKT_READY),
         .RD_DATA                       ({wTxrPkt,wTxrPktStartFlag,wTxrPktStartOffset,wTxrPktEndFlag,wTxrPktEndOffset}),
         .RD_DATA_VALID                 (wTxrPktValid),
         // Inputs
         .WR_DATA                       ({TXR_PKT,TXR_PKT_START_FLAG,TXR_PKT_START_OFFSET,
                                           TXR_PKT_END_FLAG,TXR_PKT_END_OFFSET}),
         .WR_DATA_VALID                 (TXR_PKT_VALID),
         .RD_DATA_READY                 (wTxrPktReady),
         .RST_IN                        (wRst),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));


    offset_to_mask
        #(
          // Parameters
          .C_MASK_SWAP                  (0),
          .C_MASK_WIDTH                 (C_PCI_DATA_WIDTH/32)
          /*AUTOINSTPARAM*/)
    otom_inst
        (
         // Outputs
         .MASK                          (_wSAxisRqTKeep),
         // Inputs
         .OFFSET_ENABLE                 (wTxrPktEndFlag),
         .OFFSET                        (wTxrPktEndOffset)
         /*AUTOINST*/);
    

    pipeline
        #(
          // Parameters
          .C_DEPTH                      (64/C_PCI_DATA_WIDTH),
          .C_WIDTH                      (C_PCI_DATA_WIDTH + 1 + (C_PCI_DATA_WIDTH/32)),
          .C_USE_MEMORY                 (0)
          /*AUTOINSTPARAM*/)
    fbe_hack_inst
        (
         // Outputs
         .WR_DATA_READY                 (_wSAxisRqTReady),
         .RD_DATA                       ({wSAxisRqTData,wSAxisRqTLast,wSAxisRqTKeep}),
         .RD_DATA_VALID                 (wSAxisRqTValid),
         // Inputs
         .WR_DATA                       ({_wSAxisRqTData,_wSAxisRqTLast,_wSAxisRqTKeep}),
         .WR_DATA_VALID                 (_wSAxisRqTValid),
         .RD_DATA_READY                 (wSAxisRqTReady),
         .RST_IN                        (wRst),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    pipeline
        #(
          // Parameters
          .C_DEPTH                      (C_OUTPUT_STAGES),
          .C_WIDTH                      (C_PCI_DATA_WIDTH + 1 + (C_PCI_DATA_WIDTH/32) + `SIG_RQ_TUSER_W),
          .C_USE_MEMORY                 (0)
          /*AUTOINSTPARAM*/)
    output_inst
        (
         // Outputs
         .WR_DATA_READY                 (wSAxisRqTReady),
         .RD_DATA                       ({S_AXIS_RQ_TDATA,S_AXIS_RQ_TLAST,S_AXIS_RQ_TKEEP,S_AXIS_RQ_TUSER}),
         .RD_DATA_VALID                 (S_AXIS_RQ_TVALID),
         // Inputs
         .WR_DATA                       ({wSAxisRqTData,wSAxisRqTLast,wSAxisRqTKeep,wSAxisRqTUser}),
         .WR_DATA_VALID                 (wSAxisRqTValid & ~wRstWaiting),
         .RD_DATA_READY                 (S_AXIS_RQ_TREADY),
         .RST_IN                        (wRst),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

endmodule
// Local Variables:
// verilog-library-directories:("." "../../../common/" "../../common/")
// End:
