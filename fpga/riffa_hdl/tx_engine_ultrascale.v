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
// Filename:            tx_engine_ultrascale.v
// Version:             1.0
// Verilog Standard:    Verilog-2001
// Description:         The TX Engine takes unformatted request and completions,
// formats these packets into AXI packets for the Xilinx Core. These packets
// must meet max-request, max-payload, and payload termination requirements (see
// Read Completion Boundary). The TX Engine does not check these requirements
// during operation, but may do so during simulation.
// 
// This Engine is capable of operating at "line rate".
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`include "trellis.vh"
`include "ultrascale.vh"
module tx_engine_ultrascale
    #(parameter C_PCI_DATA_WIDTH = 128,
      parameter C_PIPELINE_INPUT = 1,
      parameter C_PIPELINE_OUTPUT = 0,
      parameter C_MAX_PAYLOAD_DWORDS = 64)
    (// Interface: Clocks
     input                                   CLK,

     // Interface: Resets
     input                                   RST_BUS, // Replacement for generic RST_IN
     input                                   RST_LOGIC, // Addition for RIFFA_RST
     output                                  DONE_TXC_RST,
     output                                  DONE_TXR_RST,

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
     output                                  TXC_META_READY,
     output                                  TXC_SENT,

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
     output                                  TXR_META_READY,
     output                                  TXR_SENT
     );

    localparam C_DEPTH_PACKETS = 10;
    /*AUTOWIRE*/
    /*AUTOINPUT*/
    /*AUTOOUTPUT*/
    // Beginning of automatic outputs (from unused autoinst outputs)
    // End of automatics

    reg                                      rTxcSent;
    reg                                      rTxrSent;

    assign TXC_SENT = rTxcSent;
    assign TXR_SENT = rTxrSent;
    
    always @(posedge CLK) begin
        rTxcSent <= S_AXIS_CC_TLAST & S_AXIS_CC_TVALID & S_AXIS_CC_TREADY;
        rTxrSent <= S_AXIS_RQ_TLAST & S_AXIS_RQ_TVALID & S_AXIS_RQ_TREADY;
    end                                  
    
    txr_engine_ultrascale
        #(/*AUTOINSTPARAM*/
          // Parameters
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
          .C_PIPELINE_INPUT             (C_PIPELINE_INPUT),
          .C_PIPELINE_OUTPUT            (C_PIPELINE_OUTPUT),
          .C_DEPTH_PACKETS              (C_DEPTH_PACKETS),
          .C_MAX_PAYLOAD_DWORDS         (C_MAX_PAYLOAD_DWORDS))
    txr_engine_inst
        (/*AUTOINST*/
         // Outputs
         .DONE_TXR_RST                  (DONE_TXR_RST),
         .S_AXIS_RQ_TVALID              (S_AXIS_RQ_TVALID),
         .S_AXIS_RQ_TLAST               (S_AXIS_RQ_TLAST),
         .S_AXIS_RQ_TDATA               (S_AXIS_RQ_TDATA[C_PCI_DATA_WIDTH-1:0]),
         .S_AXIS_RQ_TKEEP               (S_AXIS_RQ_TKEEP[(C_PCI_DATA_WIDTH/32)-1:0]),
         .S_AXIS_RQ_TUSER               (S_AXIS_RQ_TUSER[`SIG_RQ_TUSER_W-1:0]),
         .TXR_DATA_READY                (TXR_DATA_READY),
         .TXR_META_READY                (TXR_META_READY),
         // Inputs
         .CLK                           (CLK),
         .RST_BUS                       (RST_BUS),
         .RST_LOGIC                     (RST_LOGIC),
         .CONFIG_COMPLETER_ID           (CONFIG_COMPLETER_ID[`SIG_CPLID_W-1:0]),
         .S_AXIS_RQ_TREADY              (S_AXIS_RQ_TREADY),
         .TXR_DATA_VALID                (TXR_DATA_VALID),
         .TXR_DATA                      (TXR_DATA[C_PCI_DATA_WIDTH-1:0]),
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
    

    txc_engine_ultrascale
        #(
          /*AUTOINSTPARAM*/
          // Parameters
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
          .C_PIPELINE_INPUT             (C_PIPELINE_INPUT),
          .C_PIPELINE_OUTPUT            (C_PIPELINE_OUTPUT),
          .C_DEPTH_PACKETS              (C_DEPTH_PACKETS),
          .C_MAX_PAYLOAD_DWORDS         (C_MAX_PAYLOAD_DWORDS))
    txc_engine_inst
        (/*AUTOINST*/
         // Outputs
         .DONE_TXC_RST                  (DONE_TXC_RST),
         .S_AXIS_CC_TVALID              (S_AXIS_CC_TVALID),
         .S_AXIS_CC_TLAST               (S_AXIS_CC_TLAST),
         .S_AXIS_CC_TDATA               (S_AXIS_CC_TDATA[C_PCI_DATA_WIDTH-1:0]),
         .S_AXIS_CC_TKEEP               (S_AXIS_CC_TKEEP[(C_PCI_DATA_WIDTH/32)-1:0]),
         .S_AXIS_CC_TUSER               (S_AXIS_CC_TUSER[`SIG_CC_TUSER_W-1:0]),
         .TXC_DATA_READY                (TXC_DATA_READY),
         .TXC_META_READY                (TXC_META_READY),
         // Inputs
         .CLK                           (CLK),
         .RST_BUS                       (RST_BUS),
         .RST_LOGIC                     (RST_LOGIC),
         .CONFIG_COMPLETER_ID           (CONFIG_COMPLETER_ID[`SIG_CPLID_W-1:0]),
         .S_AXIS_CC_TREADY              (S_AXIS_CC_TREADY),
         .TXC_DATA_VALID                (TXC_DATA_VALID),
         .TXC_DATA                      (TXC_DATA[C_PCI_DATA_WIDTH-1:0]),
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

endmodule
