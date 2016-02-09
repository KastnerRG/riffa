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
// Filename:            txc_engine_classic.v
// Version:             1.0
// Verilog Standard:    Verilog-2001
// Description:         The TXR Engine takes unformatted completions, formats
// these packets into "TLP's" or Transaction Layer Packets. These packets must
// meet max-request, max-payload, and payload termination requirements (see Read
// Completion Boundary). The TXR Engine does not check these requirements during
// operation, but may do so during simulation. This Engine is capable of
// operating at "line rate".  This file also contains the txr_formatter module,
// which formats request headers.
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
`include "trellis.vh" // Defines the user-facing signal widths.
`include "tlp.vh" // Defines the endpoint-facing field widths in a TLP
module txr_engine_classic
    #(parameter C_PCI_DATA_WIDTH = 128,
      parameter C_PIPELINE_INPUT = 1,
      parameter C_PIPELINE_OUTPUT = 0,
      parameter C_MAX_PAYLOAD_DWORDS = 64,
      parameter C_DEPTH_PACKETS = 10,
      parameter C_VENDOR = "ALTERA")
    (// Interface: Clocks
     input                                    CLK,

     // Interface: Resets
     input                                    RST_IN, // Addition for RIFFA_RST
     output                                   DONE_TXR_RST,

     // Interface: Configuration 
     input [`SIG_CPLID_W-1:0]                 CONFIG_COMPLETER_ID,

     // Interface: TXR Classic
     input                                    TXR_TLP_READY,
     output [C_PCI_DATA_WIDTH-1:0]            TXR_TLP,
     output                                   TXR_TLP_VALID,
     output                                   TXR_TLP_START_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0] TXR_TLP_START_OFFSET,
     output                                   TXR_TLP_END_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0] TXR_TLP_END_OFFSET,

     // Interface: TXR Engine
     input                                    TXR_DATA_VALID,
     input [C_PCI_DATA_WIDTH-1:0]             TXR_DATA,
     input                                    TXR_DATA_START_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]  TXR_DATA_START_OFFSET,
     input                                    TXR_DATA_END_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]  TXR_DATA_END_OFFSET,
     output                                   TXR_DATA_READY,

     input                                    TXR_META_VALID,
     input [`SIG_FBE_W-1:0]                   TXR_META_FDWBE, 
     input [`SIG_LBE_W-1:0]                   TXR_META_LDWBE,
     input [`SIG_ADDR_W-1:0]                  TXR_META_ADDR,
     input [`SIG_LEN_W-1:0]                   TXR_META_LENGTH,
     input [`SIG_TAG_W-1:0]                   TXR_META_TAG,
     input [`SIG_TC_W-1:0]                    TXR_META_TC,
     input [`SIG_ATTR_W-1:0]                  TXR_META_ATTR,
     input [`SIG_TYPE_W-1:0]                  TXR_META_TYPE,
     input                                    TXR_META_EP,
     output                                   TXR_META_READY     
     );

    localparam C_DATA_WIDTH = C_PCI_DATA_WIDTH;
    localparam C_MAX_HDR_WIDTH = `TLP_MAXHDR_W;
    localparam C_MAX_ALIGN_WIDTH = (C_VENDOR == "ALTERA") ? 32: 
                                   (C_VENDOR == "XILINX") ? 0  : 
                                   0;

    localparam C_PIPELINE_FORMATTER_INPUT = C_PIPELINE_INPUT;
    localparam C_PIPELINE_FORMATTER_OUTPUT = C_PIPELINE_OUTPUT;
    localparam C_FORMATTER_DELAY = C_PIPELINE_FORMATTER_OUTPUT + C_PIPELINE_FORMATTER_INPUT;
    
    /*AUTOWIRE*/
    /*AUTOINPUT*/
    ///*AUTOOUTPUT*/

    wire                                      wTxHdrReady;
    wire                                      wTxHdrValid;
    wire [C_MAX_HDR_WIDTH-1:0]                wTxHdr;
    wire [`SIG_TYPE_W-1:0]                    wTxType;
    wire [`SIG_NONPAY_W-1:0]                  wTxHdrNonpayLen;
    wire [`SIG_PACKETLEN_W-1:0]               wTxHdrPacketLen;
    wire [`SIG_LEN_W-1:0]                     wTxHdrPayloadLen; 
    wire                                      wTxHdrNopayload;

    assign DONE_TXR_RST = ~RST_IN;

    txr_formatter_classic
        #(
          .C_PIPELINE_OUTPUT            (C_PIPELINE_FORMATTER_OUTPUT),
          .C_PIPELINE_INPUT             (C_PIPELINE_FORMATTER_INPUT),
          /*AUTOINSTPARAM*/
          // Parameters
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
          .C_MAX_HDR_WIDTH              (C_MAX_HDR_WIDTH),
          .C_MAX_ALIGN_WIDTH            (C_MAX_ALIGN_WIDTH),
          .C_VENDOR                     (C_VENDOR))
    txr_formatter_inst
        (
         // Outputs
         .TX_HDR_VALID                  (wTxHdrValid),
         .TX_HDR                        (wTxHdr[C_MAX_HDR_WIDTH-1:0]),
         .TX_HDR_NOPAYLOAD              (wTxHdrNopayload),
         .TX_HDR_PAYLOAD_LEN            (wTxHdrPayloadLen[`SIG_LEN_W-1:0]),
         .TX_HDR_NONPAY_LEN             (wTxHdrNonpayLen[`SIG_NONPAY_W-1:0]),
         .TX_HDR_PACKET_LEN             (wTxHdrPacketLen[`SIG_PACKETLEN_W-1:0]),
         // Inputs
         .TX_HDR_READY                  (wTxHdrReady),
         /*AUTOINST*/
         // Outputs
         .TXR_META_READY                (TXR_META_READY),
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN),
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
        #(
          .C_DATA_WIDTH                 (C_PCI_DATA_WIDTH),
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
        (
         // Outputs
         .TX_HDR_READY                  (wTxHdrReady),
         .TX_DATA_READY                 (TXR_DATA_READY),
         .TX_PKT                        (TXR_TLP[C_DATA_WIDTH-1:0]),
         .TX_PKT_START_FLAG             (TXR_TLP_START_FLAG),
         .TX_PKT_START_OFFSET           (TXR_TLP_START_OFFSET[clog2s(C_DATA_WIDTH/32)-1:0]),
         .TX_PKT_END_FLAG               (TXR_TLP_END_FLAG),
         .TX_PKT_END_OFFSET             (TXR_TLP_END_OFFSET[clog2s(C_DATA_WIDTH/32)-1:0]),
         .TX_PKT_VALID                  (TXR_TLP_VALID),
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
         .TX_PKT_READY                  (TXR_TLP_READY),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));

endmodule

module txr_formatter_classic
    #(
      parameter C_PCI_DATA_WIDTH = 128,
      parameter C_MAX_HDR_WIDTH = `TLP_MAXHDR_W,
      parameter C_MAX_ALIGN_WIDTH = 32,
      parameter C_PIPELINE_INPUT = 1,
      parameter C_PIPELINE_OUTPUT = 1,
      parameter C_VENDOR = "ALTERA"
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
    
    wire                           wWrReq;
    wire [`TLP_FMT_W-1:0]          wHdrLoFmt;
    wire [63:0]                    wHdrLo;

    wire [63:0]                    _wTxHdr;
    wire                           wTxHdrReady;
    wire                           wTxHdrValid;
    wire [(`TLP_REQADDR_W/2)-1:0]  wTxHdrAddr[1:0];
    wire [(`TLP_REQADDR_W/2)-1:0]  wTxHdrAddrDW0;
    wire                           wTxHdr4DW;
    wire                           wTxHdrAlignmentNeeded;
    wire [C_MAX_HDR_WIDTH-1:0]     wTxHdr;
    wire [`SIG_TYPE_W-1:0]         wTxType;
    wire [`SIG_NONPAY_W-1:0]       wTxHdrNonpayLen;
    wire [`SIG_PACKETLEN_W-1:0]    wTxHdrPacketLen;
    wire [`SIG_LEN_W-1:0]          wTxHdrPayloadLen; 
    wire                           wTxHdrNopayload;

    assign wHdrLoFmt = {1'b0, TXR_META_TYPE[`TRLS_TYPE_PAY_I],1'bx};

    // Reserved Fields
    assign wHdrLo[`TLP_RSVD0_R] = `TLP_RSVD0_V;
    assign wHdrLo[`TLP_ADDRTYPE_R] = `TLP_ADDRTYPE_W'b0;
    assign wHdrLo[`TLP_TH_R] = `TLP_TH_W'b0;
    assign wHdrLo[`TLP_RSVD1_R] = `TLP_RSVD1_V;
    assign wHdrLo[`TLP_RSVD2_R] = `TLP_RSVD2_V;

    // Generic Header Fields
    assign wHdrLo[`TLP_LEN_R] = TXR_META_LENGTH;
    assign wHdrLo[`TLP_EP_R] = TXR_META_EP;
    assign wHdrLo[`TLP_TD_R] = `TLP_NODIGEST_V;
    assign wHdrLo[`TLP_ATTR0_R] = TXR_META_ATTR[1:0];
    assign wHdrLo[`TLP_ATTR1_R] = TXR_META_ATTR[2];
    assign wHdrLo[`TLP_TYPE_R] = TXR_META_TYPE; // WORKAROUND
    assign wHdrLo[`TLP_TC_R] = TXR_META_TC;
    assign wHdrLo[`TLP_FMT_R] = wHdrLoFmt; 

    // Request Specific Fields
    assign wHdrLo[`TLP_REQFBE_R] = TXR_META_FDWBE;
    assign wHdrLo[`TLP_REQLBE_R] = TXR_META_LDWBE;
    assign wHdrLo[`TLP_REQTAG_R] = TXR_META_TAG;
    assign wHdrLo[`TLP_REQREQID_R] = CONFIG_COMPLETER_ID;

    // Second header formatting stage
    assign wTxHdr4DW = wTxHdrAddr[1] != 32'b0;
    assign {wTxHdr[`TLP_FMT_R],wTxHdr[`TLP_TYPE_R]} = trellis_to_tlp_type(_wTxHdr[`TLP_TYPE_I +: `SIG_TYPE_W],wTxHdr4DW);
    assign wTxHdr[`TLP_TYPE_I-1:0] = _wTxHdr[`TLP_TYPE_I-1:0];
    assign wTxHdr[63:32] = _wTxHdr[63:32];
    assign wTxHdr[127:64] = {wTxHdrAddr[~wTxHdr4DW],wTxHdrAddr[wTxHdr4DW]};

    // Metadata, to the aligner
    assign wTxHdrNopayload = ~wTxHdr[`TLP_PAYBIT_I];
    assign wTxHdrAddrDW0 = wTxHdrAddr[0];
    assign wTxHdrAlignmentNeeded = (wTxHdrAddrDW0[2] == wTxHdr4DW);
    assign wTxHdrNonpayLen = {1'b0,{wTxHdr4DW,~wTxHdr4DW,~wTxHdr4DW}} + ((C_VENDOR == "ALTERA") ? {3'b0,(wTxHdrAlignmentNeeded & ~wTxHdrNopayload)}:0);    
    assign wTxHdrPayloadLen = wTxHdrNopayload ? 0 : wTxHdr[`TLP_LEN_R];
    assign wTxHdrPacketLen = wTxHdrPayloadLen + wTxHdrNonpayLen;    

    pipeline
        #(// Parameters
          .C_DEPTH                         (C_PIPELINE_INPUT?1:0),
          .C_WIDTH                         (C_MAX_HDR_WIDTH),
          .C_USE_MEMORY                    (0)
          /*AUTOINSTPARAM*/)
    input_inst
        (// Outputs
         .WR_DATA_READY                  (TXR_META_READY),
         .RD_DATA                        ({wTxHdrAddr[1],wTxHdrAddr[0],_wTxHdr[63:0]}),
         .RD_DATA_VALID                  (wTxHdrValid),
         // Inputs
         .WR_DATA                        ({TXR_META_ADDR, wHdrLo}),
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
          .C_WIDTH                         (C_MAX_HDR_WIDTH+ 1 + `SIG_PACKETLEN_W + `SIG_LEN_W + `SIG_NONPAY_W),
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
// Local Variables:
// verilog-library-directories:("." "../../../common/" "../../common/")
// End:
