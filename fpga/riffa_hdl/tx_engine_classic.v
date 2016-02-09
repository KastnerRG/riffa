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
//
// Filename:            tx_engine_classic.v
// Version:             1.0
// Verilog Standard:    Verilog-2001
// Description:         The TX Engine takes unformatted request and completions,
// formats these packets into "TLP's" or Transaction Layer Packets and abitrates
// their sending over the PCIe bus. These packets must meet max-request,
// max-payload, and payload termination requirements (see Read Completion
// Boundary). The TX Engine does not check these requirements during operation,
// but may do so during simulation.
// 
// Valid packets are transmitted over the shared PCIe bus as determined
// by the arbiter. 
// 
// This Engine is capable of operating at "line rate".
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`include "trellis.vh"
`include "tlp.vh"
module tx_engine_classic
    #(parameter C_PCI_DATA_WIDTH = 128,
      parameter C_PIPELINE_INPUT = 1,
      parameter C_PIPELINE_OUTPUT = 1,
      parameter C_MAX_PAYLOAD_DWORDS = 256,
      parameter C_VENDOR = "ALTERA")
    (// Interface: Clocks
     input                                    CLK,

     // Interface: Resets
     input                                    RST_BUS, // Replacement for generic RST_IN
     input                                    RST_LOGIC, // Addition for RIFFA_RST
     output                                   DONE_TXC_RST,
     output                                   DONE_TXR_RST,

     // Interface: Configuration 
     input [`SIG_CPLID_W-1:0]                 CONFIG_COMPLETER_ID,

     // Interface: TX Classic
     input                                    TX_TLP_READY,
     output [C_PCI_DATA_WIDTH-1:0]            TX_TLP,
     output                                   TX_TLP_VALID,
     output                                   TX_TLP_START_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0] TX_TLP_START_OFFSET,
     output                                   TX_TLP_END_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0] TX_TLP_END_OFFSET,

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
     output                                   TXC_META_READY,
     output                                   TXC_SENT,
    
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
     output                                   TXR_META_READY,
     output                                   TXR_SENT);
    
    localparam C_MUX_TYPE = "SHIFT";
    localparam C_DEPTH_PACKETS = 10;
    localparam C_RST_COUNT = 10;
    /*AUTOWIRE*/
    /*AUTOINPUT*/

    wire [C_PCI_DATA_WIDTH-1:0]               _TXC_DATA;
    wire [C_PCI_DATA_WIDTH-1:0]               _TXR_DATA;
    wire [C_PCI_DATA_WIDTH-1:0]               wTxcTlp;
    wire                                      wTxcTlpEndFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]    wTxcTlpEndOffset;
    wire                                      wTxcTlpReady;
    wire                                      wTxcTlpStartFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]    wTxcTlpStartOffset;
    wire                                      wTxcTlpValid;

    wire [C_PCI_DATA_WIDTH-1:0]               wTxrTlp;
    wire                                      wTxrTlpEndFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]    wTxrTlpEndOffset;
    wire                                      wTxrTlpReady;
    wire                                      wTxrTlpStartFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]    wTxrTlpStartOffset;
    wire                                      wTxrTlpValid;

    wire [C_PCI_DATA_WIDTH-1:0]               wTxTlp;
    wire                                      wTxTlpEndFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]    wTxTlpEndOffset;
    wire                                      wTxTlpReady;
    wire                                      wTxTlpStartFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]    wTxTlpStartOffset;
    wire                                      wTxTlpValid;

    wire                                      wDoneTxcEngRst;
    wire                                      wDoneTxrEngRst;
    wire                                      wDoneRst;                                      
    wire                                      wRstWaiting;
    wire                                      wRstRc;
    wire                                      wRstEng;
    wire [C_RST_COUNT:0]                      wShiftRst;

    reg                                       rTxValid;
    reg [`TLP_TYPE_W-1:0]                     rTxType;
    reg                                       rTxEndFlag;
    reg                                       rTxrSent;
    reg                                       rTxcSent;

    assign DONE_TXC_RST = wDoneRst & wDoneTxcEngRst;
    assign DONE_TXR_RST = wDoneRst & wDoneTxrEngRst;
    assign TXC_SENT = rTxcSent;
    assign TXR_SENT = rTxrSent;

    assign wRstEng = wShiftRst[C_RST_COUNT-3];
    assign wDoneEngRst = ~wShiftRst[C_RST_COUNT];
    
    always @(posedge CLK) begin
        if(TX_TLP_START_FLAG) begin
            rTxType <= TX_TLP[`TLP_TYPE_R];
        end
        rTxEndFlag <= TX_TLP_END_FLAG;
        rTxValid <= TX_TLP_VALID & TX_TLP_READY;
        rTxcSent <= rTxEndFlag & rTxValid & (rTxType == `TLP_TYPE_CPL);
        rTxrSent <= rTxEndFlag & rTxValid & (rTxType == `TLP_TYPE_REQ);
    end
    
    generate
        if(C_VENDOR == "XILINX") begin : xilinx_data
            if(C_PCI_DATA_WIDTH == 128) begin : x_be_swap128
                assign _TXC_DATA = {TXC_DATA[103:96], TXC_DATA[111:104], TXC_DATA[119:112], TXC_DATA[127:120],
                                    TXC_DATA[71:64], TXC_DATA[79:72], TXC_DATA[87:80], TXC_DATA[95:88],
                                    TXC_DATA[39:32], TXC_DATA[47:40], TXC_DATA[55:48], TXC_DATA[63:56],
                                    TXC_DATA[07:00], TXC_DATA[15:08], TXC_DATA[23:16], TXC_DATA[31:24]};
                assign _TXR_DATA = {TXR_DATA[103:96], TXR_DATA[111:104], TXR_DATA[119:112], TXR_DATA[127:120],
                                    TXR_DATA[71:64], TXR_DATA[79:72], TXR_DATA[87:80], TXR_DATA[95:88],
                                    TXR_DATA[39:32], TXR_DATA[47:40], TXR_DATA[55:48], TXR_DATA[63:56],
                                    TXR_DATA[07:00], TXR_DATA[15:08], TXR_DATA[23:16], TXR_DATA[31:24]};
            end else if(C_PCI_DATA_WIDTH == 64) begin: x_be_swap64
                assign _TXC_DATA = {TXC_DATA[39:32], TXC_DATA[47:40], TXC_DATA[55:48], TXC_DATA[63:56],
                                    TXC_DATA[07:00], TXC_DATA[15:08], TXC_DATA[23:16], TXC_DATA[31:24]};
                assign _TXR_DATA = {TXR_DATA[39:32], TXR_DATA[47:40], TXR_DATA[55:48], TXR_DATA[63:56],
                                    TXR_DATA[07:00], TXR_DATA[15:08], TXR_DATA[23:16], TXR_DATA[31:24]};
            end else if(C_PCI_DATA_WIDTH == 32) begin: x_be_swap32
                assign _TXC_DATA = {TXC_DATA[07:00], TXC_DATA[15:08], TXC_DATA[23:16], TXC_DATA[31:24]};
                assign _TXR_DATA = {TXR_DATA[07:00], TXR_DATA[15:08], TXR_DATA[23:16], TXR_DATA[31:24]};
            end
        end else begin : altera_data
            assign _TXC_DATA = TXC_DATA;
            assign _TXR_DATA = TXR_DATA;
        end
    endgenerate

    txc_engine_classic
        #(.C_PIPELINE_OUTPUT            (0),
          /*AUTOINSTPARAM*/
          // Parameters
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
          .C_PIPELINE_INPUT             (C_PIPELINE_INPUT),
          .C_MAX_PAYLOAD_DWORDS         (C_MAX_PAYLOAD_DWORDS),
          .C_DEPTH_PACKETS              (C_DEPTH_PACKETS),
          .C_VENDOR                     (C_VENDOR))
    txc_engine_inst
        (// Outputs
         .TXC_TLP                       (wTxcTlp[C_PCI_DATA_WIDTH-1:0] ),
         .TXC_TLP_VALID                 (wTxcTlpValid),
         .TXC_TLP_START_FLAG            (wTxcTlpStartFlag),
         .TXC_TLP_START_OFFSET          (wTxcTlpStartOffset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TXC_TLP_END_FLAG              (wTxcTlpEndFlag),
         .TXC_TLP_END_OFFSET            (wTxcTlpEndOffset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .DONE_TXC_RST                  (wDoneTxcEngRst),
         // Inputs
         .TXC_TLP_READY                 (wTxcTlpReady),
         .TXC_DATA                      (_TXC_DATA[C_PCI_DATA_WIDTH-1:0]),
         .RST_IN                        (wRstEng),
         /*AUTOINST*/
         // Outputs
         .TXC_DATA_READY                (TXC_DATA_READY),
         .TXC_META_READY                (TXC_META_READY),
         // Inputs
         .CLK                           (CLK),
         .CONFIG_COMPLETER_ID           (CONFIG_COMPLETER_ID[`SIG_CPLID_W-1:0]),
         .TXC_DATA_VALID                (TXC_DATA_VALID),
         .TXC_DATA_START_FLAG           (TXC_DATA_START_FLAG),
         .TXC_DATA_START_OFFSET         (TXC_DATA_START_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TXC_DATA_END_FLAG             (TXC_DATA_END_FLAG),
         .TXC_DATA_END_OFFSET           (TXC_DATA_END_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TXC_META_VALID                (TXC_META_VALID),
         .TXC_META_FDWBE                (TXC_META_FDWBE[`SIG_FBE_W-1:0]),
         .TXC_META_LDWBE                (TXC_META_LDWBE[`SIG_LBE_W-1:0]),
         .TXC_META_ADDR                 (TXC_META_ADDR[`SIG_LOWADDR_W-1:0]),
         .TXC_META_TYPE                 (TXC_META_TYPE[`SIG_TYPE_W-1:0]),
         .TXC_META_LENGTH               (TXC_META_LENGTH[`SIG_LEN_W-1:0]),
         .TXC_META_BYTE_COUNT           (TXC_META_BYTE_COUNT[`SIG_BYTECNT_W-1:0]),
         .TXC_META_TAG                  (TXC_META_TAG[`SIG_TAG_W-1:0]),
         .TXC_META_REQUESTER_ID         (TXC_META_REQUESTER_ID[`SIG_REQID_W-1:0]),
         .TXC_META_TC                   (TXC_META_TC[`SIG_TC_W-1:0]),
         .TXC_META_ATTR                 (TXC_META_ATTR[`SIG_ATTR_W-1:0]),
         .TXC_META_EP                   (TXC_META_EP));

    txr_engine_classic
        #(.C_PIPELINE_OUTPUT            (0),
          /*AUTOINSTPARAM*/
          // Parameters
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
          .C_PIPELINE_INPUT             (C_PIPELINE_INPUT),
          .C_MAX_PAYLOAD_DWORDS         (C_MAX_PAYLOAD_DWORDS),
          .C_DEPTH_PACKETS              (C_DEPTH_PACKETS),
          .C_VENDOR                     (C_VENDOR))
    txr_engine_inst
        (// Outputs
         .TXR_TLP                       (wTxrTlp[C_PCI_DATA_WIDTH-1:0]),
         .TXR_TLP_VALID                 (wTxrTlpValid),
         .TXR_TLP_START_FLAG            (wTxrTlpStartFlag),
         .TXR_TLP_START_OFFSET          (wTxrTlpStartOffset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TXR_TLP_END_FLAG              (wTxrTlpEndFlag),
         .TXR_TLP_END_OFFSET            (wTxrTlpEndOffset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TXR_DATA                      (_TXR_DATA[C_PCI_DATA_WIDTH-1:0]),
         .DONE_TXR_RST                  (wDoneTxrEngRst),
         // Inputs
         .TXR_TLP_READY                 (wTxrTlpReady),
         .RST_IN                        (wRstEng),
         /*AUTOINST*/
         // Outputs
         .TXR_DATA_READY                (TXR_DATA_READY),
         .TXR_META_READY                (TXR_META_READY),
         // Inputs
         .CLK                           (CLK),
         .CONFIG_COMPLETER_ID           (CONFIG_COMPLETER_ID[`SIG_CPLID_W-1:0]),
         .TXR_DATA_VALID                (TXR_DATA_VALID),
         .TXR_DATA_START_FLAG           (TXR_DATA_START_FLAG),
         .TXR_DATA_START_OFFSET         (TXR_DATA_START_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TXR_DATA_END_FLAG             (TXR_DATA_END_FLAG),
         .TXR_DATA_END_OFFSET           (TXR_DATA_END_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
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

    tx_mux
        #(.C_PIPELINE_INPUT             (0),
          /*AUTOINSTPARAM*/
          // Parameters
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
          .C_PIPELINE_OUTPUT            (C_PIPELINE_OUTPUT),
          .C_MUX_TYPE                   (C_MUX_TYPE),
          .C_VENDOR                     (C_VENDOR))
    tx_mux_inst
        (// Inputs
         .TXC_TLP                       (wTxcTlp[C_PCI_DATA_WIDTH-1:0]),
         .TXC_TLP_VALID                 (wTxcTlpValid),
         .TXC_TLP_START_FLAG            (wTxcTlpStartFlag),
         .TXC_TLP_START_OFFSET          (wTxcTlpStartOffset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TXC_TLP_END_FLAG              (wTxcTlpEndFlag),
         .TXC_TLP_END_OFFSET            (wTxcTlpEndOffset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TXR_TLP                       (wTxrTlp[C_PCI_DATA_WIDTH-1:0]),
         .TXR_TLP_VALID                 (wTxrTlpValid),
         .TXR_TLP_START_FLAG            (wTxrTlpStartFlag),
         .TXR_TLP_START_OFFSET          (wTxrTlpStartOffset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TXR_TLP_END_FLAG              (wTxrTlpEndFlag),
         .TXR_TLP_END_OFFSET            (wTxrTlpEndOffset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .RST_IN                        (wRstEng),
         // Outputs
         .TXC_TLP_READY                 (wTxcTlpReady),
         .TXR_TLP_READY                 (wTxrTlpReady),
         .TX_TLP                        (wTxTlp[C_PCI_DATA_WIDTH-1:0]),
         .TX_TLP_VALID                  (wTxTlpValid),
         .TX_TLP_START_FLAG             (wTxTlpStartFlag),
         .TX_TLP_START_OFFSET           (wTxTlpStartOffset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TX_TLP_END_FLAG               (wTxTlpEndFlag),
         .TX_TLP_END_OFFSET             (wTxTlpEndOffset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TX_TLP_READY                  (wTxTlpReady),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    shiftreg
        #(// Parameters
          .C_DEPTH                      (C_RST_COUNT),
          .C_WIDTH                      (1),
          .C_VALUE                      (1)
          /*AUTOINSTPARAM*/)
    rst_shiftreg
        (// Outputs
         .RD_DATA                       (wShiftRst),
         // Inputs
         .RST_IN                        (RST_BUS),
         .WR_DATA                       (wRstRc),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    reset_controller
        #(// Parameters
          .C_RST_COUNT                  (C_RST_COUNT)
          /*AUTOINSTPARAM*/)
    rc_inst
        (// Outputs
         .DONE_RST                      (wDoneRst),
         .WAITING_RESET                 (wRstWaiting),
         .RST_OUT                       (wRstRc),
         // Inputs
         .RST_IN                        (RST_BUS),
         .SIGNAL_RST                    (RST_LOGIC),
         .WAIT_RST                      (TX_TLP_VALID),
         .NEXT_CYC_RST                  (TX_TLP_END_FLAG & TX_TLP_READY),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));
    
    pipeline
        #(// Parameters
          .C_DEPTH                      (1),
          .C_WIDTH                      (C_PCI_DATA_WIDTH + 
                                         2*(1 + clog2s(C_PCI_DATA_WIDTH/32))),
          .C_USE_MEMORY                 (0)
          /*AUTOINSTPARAM*/)
    output_reg_inst
        (// Outputs
         .WR_DATA_READY                 (wTxTlpReady),
         .RD_DATA                       ({TX_TLP, 
                                          TX_TLP_START_FLAG, TX_TLP_START_OFFSET,
                                          TX_TLP_END_FLAG, TX_TLP_END_OFFSET}),
         .RD_DATA_VALID                 (TX_TLP_VALID),
         // Inputs
         .RST_IN                        (wRstRc),
         .WR_DATA                       ({wTxTlp, 
                                          wTxTlpStartFlag, wTxTlpStartOffset,
                                          wTxTlpEndFlag, wTxTlpEndOffset}),
         .WR_DATA_VALID                 (wTxTlpValid & ~wRstWaiting),
         .RD_DATA_READY                 (TX_TLP_READY),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    // Put output pipeline stage here, OR, at TX Engine outputs
endmodule
// Local Variables:
// verilog-library-directories:("." "../../common/")
// End:
/*
 Filename: tx_mux.v
 Version: 1.0
 Verilog Standard: Verilog-2001

 Description: The tx_mux arbitrates access to the PCI TX interface
 between the the Request and Completion engines. The top level tx_mux
 module instantiates two sub-modules (also declared in this file):
 tx_arbiter and tx_phi. The arbiter choses the next packet that will
 be granted the TX interface, based on the priorities given to
 it. Each priority is the count of how many successive, back-to-back
 packets can be sent by either the TXC or TXR interface before
 granting time to the other engine. The mux is a simple multiplexer
 that uses the select signals from the arbiter.
 
 Notes: Any modifications to this file should meet the conditions set
 forth in the "Trellis Style Guide"

 - The pipeline stage at the start of the mux means that ready stays high for
 both TXC and TXR. This the behavior we want?
 
 Author: Dustin Richmond (@darichmond) 
 Co-Authors:
 */
`timescale 1ns/1ns
`include "trellis.vh"
module tx_mux
    #(parameter C_PCI_DATA_WIDTH = 128,
      parameter C_PIPELINE_INPUT = 1,
      parameter C_PIPELINE_OUTPUT = 1,
      parameter C_MUX_TYPE = "SHIFT",
      parameter C_VENDOR = "ALTERA")
    (// Interface: Clocks
     input                                    CLK,

     // Interface: Resets
     input                                    RST_IN,

     // Interface: TXC
     input [C_PCI_DATA_WIDTH-1:0]             TXC_TLP,
     output                                   TXC_TLP_READY,
     input                                    TXC_TLP_VALID,
     input                                    TXC_TLP_START_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]  TXC_TLP_START_OFFSET,
     input                                    TXC_TLP_END_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]  TXC_TLP_END_OFFSET,

     // Interface: TXR
     input [C_PCI_DATA_WIDTH-1:0]             TXR_TLP,
     input                                    TXR_TLP_VALID,
     input                                    TXR_TLP_START_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]  TXR_TLP_START_OFFSET,
     input                                    TXR_TLP_END_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]  TXR_TLP_END_OFFSET,
     output                                   TXR_TLP_READY,

     // Interface: TX Classic
     input                                    TX_TLP_READY,
     output [C_PCI_DATA_WIDTH-1:0]            TX_TLP,
     output                                   TX_TLP_VALID,
     output                                   TX_TLP_START_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0] TX_TLP_START_OFFSET,
     output                                   TX_TLP_END_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0] TX_TLP_END_OFFSET);
    
    localparam C_WIDTH = (C_PCI_DATA_WIDTH + 2 * (clog2s(C_PCI_DATA_WIDTH/32) + 1));
    localparam C_TXC_PRIORITY = 1;
    localparam C_TXR_PRIORITY = 2;
    /*AUTOWIRE*/
    /*AUTOINPUT*/
    /*AUTOOUTPUT*/

    // Input Pipeline Stage to Mux
    wire [C_PCI_DATA_WIDTH-1:0]               wTxcTlp;
    wire                                      wTxcTlpReady;
    wire                                      wTxcTlpValid;
    wire                                      wTxcTlpStartFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]    wTxcTlpStartOffset;
    wire                                      wTxcTlpEndFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]    wTxcTlpEndOffset;

    wire [C_PCI_DATA_WIDTH-1:0]               wTxrTlp;
    wire                                      wTxrTlpReady;
    wire                                      wTxrTlpValid;
    wire                                      wTxrTlpStartFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]    wTxrTlpStartOffset;
    wire                                      wTxrTlpEndFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]    wTxrTlpEndOffset;

    // Output of Mux to 
    wire [C_PCI_DATA_WIDTH-1:0]               wTxTlp;
    wire                                      wTxTlpReady;
    wire                                      wTxTlpValid;
    wire                                      wTxTlpStartFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]    wTxTlpStartOffset;
    wire                                      wTxTlpEndFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]    wTxTlpEndOffset;

    pipeline
        #(// Parameters
          .C_DEPTH                      (C_PIPELINE_INPUT?1:0),
          .C_USE_MEMORY                 (0),
          /*AUTOINSTPARAM*/
          // Parameters
          .C_WIDTH                      (C_WIDTH))
    txr_capture_inst
        (// Outputs
         .WR_DATA_READY                 (TXR_TLP_READY),
         .RD_DATA                       ({wTxrTlp, wTxrTlpStartFlag, wTxrTlpStartOffset,
                                          wTxrTlpEndFlag, wTxrTlpEndOffset}),
         .RD_DATA_VALID                 (wTxrTlpValid),
         // Inputs
         .WR_DATA                       ({TXR_TLP, 
                                          TXR_TLP_START_FLAG, TXR_TLP_START_OFFSET, 
                                          TXR_TLP_END_FLAG, TXR_TLP_END_OFFSET}),
         .WR_DATA_VALID                 (TXR_TLP_VALID),
         .RD_DATA_READY                 (wTxrTlpReady),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));
    
    pipeline
        #(// Parameters
          .C_DEPTH                      (C_PIPELINE_INPUT?1:0),
          .C_USE_MEMORY                 (0),
          /*AUTOINSTPARAM*/
          // Parameters
          .C_WIDTH                      (C_WIDTH))
    txc_capture_inst
        (// Outputs
         .WR_DATA_READY                 (TXC_TLP_READY),
         .RD_DATA                       ({wTxcTlp, wTxcTlpStartFlag, wTxcTlpStartOffset,
                                          wTxcTlpEndFlag, wTxcTlpEndOffset}),
         .RD_DATA_VALID                 (wTxcTlpValid),
         // Inputs
         .WR_DATA                       ({TXC_TLP, 
                                          TXC_TLP_START_FLAG, TXC_TLP_START_OFFSET, 
                                          TXC_TLP_END_FLAG, TXC_TLP_END_OFFSET}),
         .WR_DATA_VALID                 (TXC_TLP_VALID),
         .RD_DATA_READY                 (wTxcTlpReady),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));

    tx_arbiter
        #(/*AUTOINSTPARAM*/
          // Parameters
          .C_TXC_PRIORITY               (C_TXC_PRIORITY),
          .C_TXR_PRIORITY               (C_TXR_PRIORITY))
    tx_arbiter_inst
        (// Outputs
         .TXR_TLP_READY                 (wTxrTlpReady),
         .TXC_TLP_READY                 (wTxcTlpReady),
         // Inputs
         .TX_TLP_READY                  (wTxTlpReady),
         .TXC_TLP_VALID                 (wTxcTlpValid),
         .TXC_TLP_START_FLAG            (wTxcTlpStartFlag),
         .TXC_TLP_END_FLAG              (wTxcTlpEndFlag),
         .TXR_TLP_VALID                 (wTxrTlpValid),
         .TXR_TLP_START_FLAG            (wTxrTlpStartFlag),
         .TXR_TLP_END_FLAG              (wTxrTlpEndFlag),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));
    
    // MUX Selector

    tx_phi
        #(/*AUTOINSTPARAM*/
          // Parameters
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
          .C_MUX_TYPE                   (C_MUX_TYPE),
          .C_WIDTH                      (C_WIDTH))
    tx_phi_inst
        (// Outputs
         .TXC_TLP_READY                 (wTxcTlpReady),
         .TXR_TLP_READY                 (wTxrTlpReady),
        
         .TX_TLP                        (wTxTlp),
         .TX_TLP_VALID                  (wTxTlpValid),
         .TX_TLP_START_FLAG             (wTxTlpStartFlag),
         .TX_TLP_START_OFFSET           (wTxTlpStartOffset),
         .TX_TLP_END_FLAG               (wTxTlpEndFlag),
         .TX_TLP_END_OFFSET             (wTxTlpEndOffset),
         // Inputs
         .TXC_TLP                       (wTxcTlp),
         .TXC_TLP_VALID                 (wTxcTlpValid),
         .TXC_TLP_START_FLAG            (wTxcTlpStartFlag),
         .TXC_TLP_START_OFFSET          (wTxcTlpStartOffset),
         .TXC_TLP_END_FLAG              (wTxcTlpEndFlag),
         .TXC_TLP_END_OFFSET            (wTxcTlpEndOffset),
        
         .TXR_TLP                       (wTxrTlp),
         .TXR_TLP_VALID                 (wTxrTlpValid),
         .TXR_TLP_START_FLAG            (wTxrTlpStartFlag),
         .TXR_TLP_START_OFFSET          (wTxrTlpStartOffset),
         .TXR_TLP_END_FLAG              (wTxrTlpEndFlag),
         .TXR_TLP_END_OFFSET            (wTxrTlpEndOffset),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));

    pipeline
        #(// Parameters
          .C_DEPTH                      (C_PIPELINE_OUTPUT?1:0),
          .C_USE_MEMORY                 (0),
          /*AUTOINSTPARAM*/
          // Parameters
          .C_WIDTH                      (C_WIDTH))
    tx_output_inst
        (// Outputs
         .WR_DATA_READY                 (wTxTlpReady),
         .RD_DATA                       ({TX_TLP, 
                                          TX_TLP_START_FLAG, TX_TLP_START_OFFSET, 
                                          TX_TLP_END_FLAG, TX_TLP_END_OFFSET}),
         .RD_DATA_VALID                 (TX_TLP_VALID),
         // Inputs
         .WR_DATA                       ({wTxTlp, wTxTlpStartFlag, wTxTlpStartOffset,
                                          wTxTlpEndFlag, wTxTlpEndOffset}),
         .WR_DATA_VALID                 (wTxTlpValid),
         .RD_DATA_READY                 (TX_TLP_READY),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));

endmodule
// Local Variables:
// verilog-library-directories:("." "../../../common/")
// End:

/* 
 Description: The tx_arbiter arbitrates between TXC and TXR channels. The C_TX*_PRIORITY
 values are counters are the maximum number of uninterrupted TXR or TXC packets that can be
 transmitted without transmitting a packet of the other type. 
 
 Notes: Any modifications to this file should meet the conditions set
 forth in the "Trellis Style Guide"

 Author: Dustin Richmond (@darichmond) 
 Co-Authors:
 */
module tx_arbiter
    #(parameter C_TXC_PRIORITY = 1,
      parameter C_TXR_PRIORITY = 1)
    (
     // Interface: Clocks
     input  CLK,

     // Interface: Resets     
     input  RST_IN,

     // Interface: TX Classic Flow Control
     input  TX_TLP_READY,

     // Interface: TXR Flow Control
     output TXR_TLP_READY,
     input  TXR_TLP_VALID,
     input  TXR_TLP_START_FLAG,
     input  TXR_TLP_END_FLAG,

     // Interface: TXC Flow Control
     output TXC_TLP_READY,
     input  TXC_TLP_VALID,
     input  TXC_TLP_START_FLAG,
     input  TXC_TLP_END_FLAG);
    
    localparam S_TXARB_IDLE = 0; // STATE: Idle state for the arbiter (not currently used)
    localparam S_TXARB_TRANSMIT_TXR = 1; // STATE: Transmit TXR packets until the priority counter is reached
    localparam S_TXARB_TRANSMIT_TXC = 2; // STATE: Transmit TXC packets until the priority counter is reached

    // S_TXARB_PRIORITY is a special state that encodes the type (TXR/TXC) with
    // higher priority so that the state machine (below) is general.
    localparam S_TXARB_PRIORITY = (S_TXARB_TRANSMIT_TXR >= S_TXARB_TRANSMIT_TXC)? S_TXARB_TRANSMIT_TXR: S_TXARB_TRANSMIT_TXC;
    localparam C_NUM_STATES = S_TXARB_TRANSMIT_TXC;
    
    wire    wTxrGrant;
    wire    wTxrReq;
    wire    wTxrDone;

    wire    wTxcGrant;
    wire    wTxcReq;
    wire    wTxcDone;

    reg [clog2s(C_NUM_STATES):0] rArbState,_rArbState;

    reg                          rTxrLast,_rTxrLast; // Reset on RST_IN or TXC_TLP_READY
    reg                          rTxcLast,_rTxcLast; // Reset on RST_IN or TXR_TLP_READY

    reg                          rTxrActive,_rTxrActive; // Reset on RST_IN or TXC_TLP_READY
    reg                          rTxcActive,_rTxcActive; // Reset on RST_IN or TXR_TLP_READY

    reg [clog2s(C_TXC_PRIORITY)-1:0] rTxcCounter,_rTxcCounter; // Reset on RST_IN or TXC_TLP_READY
    reg [clog2s(C_TXR_PRIORITY)-1:0] rTxrCounter,_rTxrCounter; // Reset on RST_IN or TXR_TLP_READY

    assign TXR_TLP_READY = wTxrGrant & TX_TLP_READY; // TODO: Not great
    assign wTxrReq = TXR_TLP_START_FLAG & TXR_TLP_VALID;
    assign wTxrDone = TXR_TLP_END_FLAG & TXR_TLP_READY;
    assign wTxrGrant = (rArbState == S_TXARB_TRANSMIT_TXR);

    assign TXC_TLP_READY = wTxcGrant & TX_TLP_READY; // TODO: Not great
    assign wTxcReq = TXC_TLP_START_FLAG & TXC_TLP_VALID;
    assign wTxcDone = TXC_TLP_END_FLAG & TXC_TLP_READY;
    assign wTxcGrant = (rArbState == S_TXARB_TRANSMIT_TXC);
    
    always @(*) begin
        // Defaults
        _rTxcCounter  = rTxcCounter;
        _rTxcActive   = rTxcActive;
        _rTxcLast     = rTxcLast;
        
        if(wTxrGrant) begin
            _rTxcCounter = 0;
        end else if(wTxcReq & wTxcGrant & ~rTxcLast) begin
            _rTxcCounter = _rTxcCounter + 1;
        end 

        if(wTxcReq & wTxcGrant) begin
            _rTxcActive = 1;
        end else if(wTxcDone) begin
            _rTxcActive = 0;
        end

        if(wTxrGrant | RST_IN) begin
            _rTxcLast = 0;
        end else if(wTxcReq & wTxcGrant) begin
            _rTxcLast = (rTxcCounter == (C_TXC_PRIORITY - 1));
        end
    end // always @ (*)

    always @(posedge CLK) begin
        if(RST_IN) begin
            rTxcCounter <= #1 0;
            rTxcActive  <= #1 0;
            rTxcLast    <= #1 0;
        end else begin
            rTxcCounter <= #1  _rTxcCounter;
            rTxcActive  <= #1  _rTxcActive;
            rTxcLast    <= #1  _rTxcLast;
        end
    end

    always @(*) begin
        // Defaults
        _rTxrCounter  = rTxrCounter;
        _rTxrActive   = rTxrActive;
        _rTxrLast     = rTxrLast;
        
        if(wTxcGrant) begin
            _rTxrCounter = 0;
        end else if(wTxrReq & wTxrGrant & ~rTxrLast) begin
            _rTxrCounter = _rTxrCounter + 1;
        end 

        if(wTxrReq & wTxrGrant) begin
            _rTxrActive = 1;
        end else if(wTxrDone) begin
            _rTxrActive = 0;
        end

        if(wTxcGrant | RST_IN) begin
            _rTxrLast = 0;
        end else if(wTxrReq & wTxrGrant) begin
            /* verilator lint_off WIDTH */
            _rTxrLast  = (rTxrCounter == (C_TXR_PRIORITY - 1));
            /* verilator lint_on WIDTH */
        end
    end
    
    always @(posedge CLK) begin
        if(RST_IN) begin
            rTxrCounter <= #1 0;
            rTxrActive  <= #1 0;
            rTxrLast    <= #1 0;
        end else begin
            rTxrCounter <= #1 _rTxrCounter;
            rTxrActive  <= #1 _rTxrActive;
            rTxrLast    <= #1 _rTxrLast;
        end
    end

    // User encoded state machine
    always @(*) begin
        _rArbState = rArbState;

        case(rArbState)
            S_TXARB_TRANSMIT_TXR: begin
                if((rTxrLast & wTxrDone & wTxcReq) | (~rTxrActive & ~wTxrReq & wTxcReq)) begin
                    _rArbState = S_TXARB_TRANSMIT_TXC;
                end
            end
            S_TXARB_TRANSMIT_TXC: begin
                if((rTxcLast & wTxcDone & wTxrReq) | (~rTxcActive & ~wTxcReq & wTxrReq)) begin
                    _rArbState = S_TXARB_TRANSMIT_TXR;
                end
            end
            default: begin
                // Error! This should never happen...
            end
        endcase
    end // always @ begin
    
    always @(posedge CLK) begin
        if(RST_IN) begin
            rArbState <= #1 S_TXARB_PRIORITY;
        end else begin
            rArbState <= #1 _rArbState;
        end
    end
    
endmodule

/* 
 Description: The tx_phi wraps a mux instantiation for the tx_mux. It is
 controlled by the tx_arbiter
 
 Notes: Any modifications to this file should meet the conditions set
 forth in the "Trellis Style Guide"

 Author: Dustin Richmond (@darichmond) 
 Co-Authors:
 */
module tx_phi
    #(parameter C_PCI_DATA_WIDTH = 10'd128,
      parameter C_MUX_TYPE = "SHIFT",
      parameter C_WIDTH = (C_PCI_DATA_WIDTH + 2 * (clog2s(C_PCI_DATA_WIDTH/32) + 1)))
    (// Interface: Clocks
     input                                    CLK,

     // Interface: Resets
     input                                    RST_IN,

     // Interface: TXC Flow Control
     input                                    TXC_TLP_READY,

     // Interface: TXR Flow Control
     input                                    TXR_TLP_READY,

     // Interface: TXC
     input [C_PCI_DATA_WIDTH-1:0]             TXC_TLP,
     input                                    TXC_TLP_VALID,
     input                                    TXC_TLP_START_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]  TXC_TLP_START_OFFSET,
     input                                    TXC_TLP_END_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]  TXC_TLP_END_OFFSET,

     // Interface: TXR
     input [C_PCI_DATA_WIDTH-1:0]             TXR_TLP,
     input                                    TXR_TLP_VALID,
     input                                    TXR_TLP_START_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]  TXR_TLP_START_OFFSET,
     input                                    TXR_TLP_END_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]  TXR_TLP_END_OFFSET,

     // Interface: TX Classic
     output [C_PCI_DATA_WIDTH-1:0]            TX_TLP,
     output                                   TX_TLP_VALID,
     output                                   TX_TLP_START_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0] TX_TLP_START_OFFSET,
     output                                   TX_TLP_END_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0] TX_TLP_END_OFFSET);

    // Width = 2 * (DATA WIDTH + VALID + START FLAG + START OFFSET + END FLAG + END OFFSET)
    localparam C_MUX_WIDTH = C_PCI_DATA_WIDTH + 3 + 2*clog2s(C_PCI_DATA_WIDTH/32);
    
    wire [2*C_MUX_WIDTH-1:0]                  wAggregate;

    assign wAggregate = {{TXR_TLP,TXR_TLP_VALID,TXR_TLP_START_FLAG,
                          TXR_TLP_START_OFFSET,TXR_TLP_END_FLAG,TXR_TLP_END_OFFSET},
                         {TXC_TLP,TXC_TLP_VALID,TXC_TLP_START_FLAG,
                          TXC_TLP_START_OFFSET,TXC_TLP_END_FLAG,TXC_TLP_END_OFFSET}};

    mux
        #(// Parameters
          .C_NUM_INPUTS                 (2),
          .C_CLOG_NUM_INPUTS            (1),
          .C_WIDTH                      (C_MUX_WIDTH),
          .C_MUX_TYPE                   ("SELECT")
          /*AUTOINSTPARAM*/)
    mux_inst
        (// Outputs
         .MUX_OUTPUT                    ({TX_TLP,TX_TLP_VALID,TX_TLP_START_FLAG,
                                          TX_TLP_START_OFFSET,TX_TLP_END_FLAG,
                                          TX_TLP_END_OFFSET}),
         // Inputs
         .MUX_INPUTS                    (wAggregate),
         .MUX_SELECT                    (TXR_TLP_READY)
         /*AUTOINST*/);

endmodule
