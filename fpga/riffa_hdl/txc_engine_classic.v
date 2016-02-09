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
// Description: The TXC Engine takes unformatted completions, formats
// these packets into "TLP's" or Transaction Layer Packets. These
// packets must meet max-request, max-payload, and payload termination
// requirements (see Read Completion Boundary). The TXC Engine does not
// check these requirements during operation, but may do so during
// simulation.
// This file also contains the txc_formatter module, which formats
// completion headers. 
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
`include "trellis.vh" // Defines the user-facing signal widths.
`include "tlp.vh" // Defines the endpoint-facing field widths in a TLP
module txc_engine_classic
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
     output                                   DONE_TXC_RST,

     // Interface: Configuration
     input [`SIG_CPLID_W-1:0]                 CONFIG_COMPLETER_ID,

     // Interface: TXC Classic
     input                                    TXC_TLP_READY,
     output [C_PCI_DATA_WIDTH-1:0]            TXC_TLP,
     output                                   TXC_TLP_VALID,
     output                                   TXC_TLP_START_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0] TXC_TLP_START_OFFSET,
     output                                   TXC_TLP_END_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0] TXC_TLP_END_OFFSET,

     // Interface: TXC Engine
     input                                    TXC_DATA_VALID,
     input [C_PCI_DATA_WIDTH-1:0]             TXC_DATA,
     input                                    TXC_DATA_START_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]  TXC_DATA_START_OFFSET,
     input                                    TXC_DATA_END_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]  TXC_DATA_END_OFFSET,
     output                                   TXC_DATA_READY,

     input                                    TXC_META_VALID,
     input [`SIG_FBE_W-1:0]                   TXC_META_FDWBE,
     input [`SIG_LBE_W-1:0]                   TXC_META_LDWBE,
     input [`SIG_LOWADDR_W-1:0]               TXC_META_ADDR,
     input [`SIG_TYPE_W-1:0]                  TXC_META_TYPE,
     input [`SIG_LEN_W-1:0]                   TXC_META_LENGTH,
     input [`SIG_BYTECNT_W-1:0]               TXC_META_BYTE_COUNT,
     input [`SIG_TAG_W-1:0]                   TXC_META_TAG,
     input [`SIG_REQID_W-1:0]                 TXC_META_REQUESTER_ID,
     input [`SIG_TC_W-1:0]                    TXC_META_TC,
     input [`SIG_ATTR_W-1:0]                  TXC_META_ATTR,
     input                                    TXC_META_EP,
     output                                   TXC_META_READY);

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

    assign DONE_TXC_RST = ~RST_IN;

    txc_formatter_classic
        #(
          .C_PIPELINE_OUTPUT            (C_PIPELINE_FORMATTER_OUTPUT),
          .C_PIPELINE_INPUT             (C_PIPELINE_FORMATTER_INPUT),
          /*AUTOINSTPARAM*/
          // Parameters
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
          .C_MAX_HDR_WIDTH              (C_MAX_HDR_WIDTH),
          .C_MAX_ALIGN_WIDTH            (C_MAX_ALIGN_WIDTH),
          .C_VENDOR                     (C_VENDOR))
    txc_formatter_inst
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
         .TXC_META_READY                (TXC_META_READY),
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN),
         .CONFIG_COMPLETER_ID           (CONFIG_COMPLETER_ID[`SIG_CPLID_W-1:0]),
         .TXC_META_VALID                (TXC_META_VALID),
         .TXC_META_FDWBE                (TXC_META_FDWBE[`SIG_FBE_W-1:0]),
         .TXC_META_LDWBE                (TXC_META_LDWBE[`SIG_LBE_W-1:0]),
         .TXC_META_ADDR                 (TXC_META_ADDR[`SIG_LOWADDR_W-1:0]),
         .TXC_META_LENGTH               (TXC_META_LENGTH[`SIG_LEN_W-1:0]),
         .TXC_META_BYTE_COUNT           (TXC_META_BYTE_COUNT[`SIG_BYTECNT_W-1:0]),
         .TXC_META_TAG                  (TXC_META_TAG[`SIG_TAG_W-1:0]),
         .TXC_META_TYPE                 (TXC_META_TYPE[`SIG_TYPE_W-1:0]),
         .TXC_META_REQUESTER_ID         (TXC_META_REQUESTER_ID[`SIG_REQID_W-1:0]),
         .TXC_META_TC                   (TXC_META_TC[`SIG_TC_W-1:0]),
         .TXC_META_ATTR                 (TXC_META_ATTR[`SIG_ATTR_W-1:0]),
         .TXC_META_EP                   (TXC_META_EP));

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
    txc_engine_inst
        (
         // Outputs
         .TX_HDR_READY                  (wTxHdrReady),
         .TX_DATA_READY                 (TXC_DATA_READY),
         .TX_PKT                        (TXC_TLP[C_DATA_WIDTH-1:0]),
         .TX_PKT_START_FLAG             (TXC_TLP_START_FLAG),
         .TX_PKT_START_OFFSET           (TXC_TLP_START_OFFSET[clog2s(C_DATA_WIDTH/32)-1:0]),
         .TX_PKT_END_FLAG               (TXC_TLP_END_FLAG),
         .TX_PKT_END_OFFSET             (TXC_TLP_END_OFFSET[clog2s(C_DATA_WIDTH/32)-1:0]),
         .TX_PKT_VALID                  (TXC_TLP_VALID),
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
         .TX_PKT_READY                  (TXC_TLP_READY),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));
endmodule

module txc_formatter_classic
    #(
      parameter C_PCI_DATA_WIDTH = 10'd128,
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

     // Interface: TXC
     input                         TXC_META_VALID,
     input [`SIG_FBE_W-1:0]        TXC_META_FDWBE,
     input [`SIG_LBE_W-1:0]        TXC_META_LDWBE,
     input [`SIG_LOWADDR_W-1:0]    TXC_META_ADDR,
     input [`SIG_LEN_W-1:0]        TXC_META_LENGTH,
     input [`SIG_BYTECNT_W-1:0]    TXC_META_BYTE_COUNT,
     input [`SIG_TAG_W-1:0]        TXC_META_TAG,
     input [`SIG_TYPE_W-1:0]       TXC_META_TYPE,
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

    wire [C_MAX_HDR_WIDTH-1:0]     wCplHdr;


    wire                           wTxHdrReady;
    wire                           wTxHdrValid;
    wire [C_MAX_HDR_WIDTH-1:0]     wTxHdr;
    wire [`SIG_TYPE_W-1:0]         wTxType;
    wire [`SIG_NONPAY_W-1:0]       wTxHdrNonpayLen;
    wire [`SIG_PACKETLEN_W-1:0]    wTxHdrPacketLen;
    wire [`SIG_LEN_W-1:0]          wTxHdrPayloadLen; 
    wire                           wTxHdrNopayload;

    wire [`TLP_CPLADDR_W-1:0]      wTxLoAddr;


    // Reserved Fields
    assign wCplHdr[`TLP_RSVD0_R] = `TLP_RSVD0_V;
    assign wCplHdr[`TLP_ADDRTYPE_R] = `TLP_ADDRTYPE_W'b0;
    assign wCplHdr[`TLP_TH_R] = `TLP_TH_W'b0;
    assign wCplHdr[`TLP_RSVD1_R] = `TLP_RSVD1_V;
    assign wCplHdr[`TLP_RSVD2_R] = `TLP_RSVD2_V;
    assign wCplHdr[`TLP_CPLBCM_R] = `TLP_CPLBCM_W'b0;
    assign wCplHdr[`TLP_CPLRSVD0_R] = `TLP_CPLRSVD0_W'b0;
    assign wCplHdr[127:96] = 32'b0;

    // Generic Header Fields
    assign wCplHdr[`TLP_LEN_R] = TXC_META_LENGTH;
    assign wCplHdr[`TLP_EP_R] = TXC_META_EP;
    assign wCplHdr[`TLP_TD_R] = `TLP_NODIGEST_V;
    assign wCplHdr[`TLP_ATTR0_R] = TXC_META_ATTR[1:0];
    assign wCplHdr[`TLP_ATTR1_R] = TXC_META_ATTR[2];
    assign {wCplHdr[`TLP_FMT_R], wCplHdr[`TLP_TYPE_R]} = trellis_to_tlp_type(TXC_META_TYPE,0);
    assign wCplHdr[`TLP_TC_R] = TXC_META_TC;

    // Completion Specific Fields
    assign wCplHdr[`TLP_CPLBYTECNT_R] = TXC_META_BYTE_COUNT;
    assign wCplHdr[`TLP_CPLSTAT_R] = 0;
    assign wCplHdr[`TLP_CPLCPLID_R] = CONFIG_COMPLETER_ID;
    assign wCplHdr[`TLP_CPLADDR_R] = TXC_META_ADDR;
    assign wCplHdr[`TLP_CPLTAG_R] = TXC_META_TAG;
    assign wCplHdr[`TLP_CPLREQID_R] = TXC_META_REQUESTER_ID;

    // Metadata, to the aligner
    assign wTxLoAddr = wTxHdr[`TLP_CPLADDR_R];
    assign wTxHdrNopayload = ~wTxHdr[`TLP_PAYBIT_I];
    assign wTxHdrNonpayLen = 3 + ((C_VENDOR == "ALTERA")? {3'b0,(~wTxLoAddr[2] & ~wTxHdrNopayload)} : 4'b0);
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
         .WR_DATA_READY                  (TXC_META_READY),
         .RD_DATA                        (wTxHdr),
         .RD_DATA_VALID                  (wTxHdrValid),
         // Inputs
         .WR_DATA                        (wCplHdr),
         .WR_DATA_VALID                  (TXC_META_VALID),
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

