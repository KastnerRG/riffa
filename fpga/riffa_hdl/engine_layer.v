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
// Filename:            engine_layer.v
// Version:             1.0
// Verilog Standard:    Verilog-2001
// Description:         The engine layer encapsulates the RX and TX engines.
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
`include "trellis.vh"
`include "ultrascale.vh"
module engine_layer
    #(parameter C_PCI_DATA_WIDTH = 128,
      parameter C_LOG_NUM_TAGS=6,
      parameter C_PIPELINE_INPUT = 1,
      parameter C_PIPELINE_OUTPUT = 0,
      parameter C_MAX_PAYLOAD_DWORDS = 64,
      parameter C_VENDOR="ULTRASCALE")
    (// Interface: Clocks
     input                                    CLK_BUS, // Replacement for generic CLK

     // Interface: Resets
     input                                    RST_BUS, // Replacement for generic RST_IN
     input                                    RST_LOGIC, // Addition for RIFFA_RST
     output                                   DONE_TXC_RST,
     output                                   DONE_TXR_RST,
     output                                   DONE_RXR_RST,
     output                                   DONE_RXC_RST,

     // Interface: Configuration 
     input [`SIG_CPLID_W-1:0]                 CONFIG_COMPLETER_ID,
    
     // Interface: RX Classic
     input [C_PCI_DATA_WIDTH-1:0]             RX_TLP,
     input                                    RX_TLP_VALID,
     input                                    RX_TLP_START_FLAG,
     input [`SIG_OFFSET_W-1:0]                RX_TLP_START_OFFSET,
     input                                    RX_TLP_END_FLAG,
     input [`SIG_OFFSET_W-1:0]                RX_TLP_END_OFFSET,
     input [`SIG_BARDECODE_W-1:0]             RX_TLP_BAR_DECODE,
     output                                   RX_TLP_READY,

     // Interface: TX Classic
     input                                    TX_TLP_READY,
     output [C_PCI_DATA_WIDTH-1:0]            TX_TLP,
     output                                   TX_TLP_VALID,
     output                                   TX_TLP_START_FLAG,
     output [`SIG_OFFSET_W-1:0]               TX_TLP_START_OFFSET,
     output                                   TX_TLP_END_FLAG,
     output [`SIG_OFFSET_W-1:0]               TX_TLP_END_OFFSET,

     //Interface: CQ Ultrascale (RXR)
     input                                    M_AXIS_CQ_TVALID,
     input                                    M_AXIS_CQ_TLAST,
     input [C_PCI_DATA_WIDTH-1:0]             M_AXIS_CQ_TDATA,
     input [(C_PCI_DATA_WIDTH/32)-1:0]        M_AXIS_CQ_TKEEP,
     input [`SIG_CQ_TUSER_W-1:0]              M_AXIS_CQ_TUSER,
     output                                   M_AXIS_CQ_TREADY,

     //Interface: RC Ultrascale (RXC)
     input                                    M_AXIS_RC_TVALID,
     input                                    M_AXIS_RC_TLAST,
     input [C_PCI_DATA_WIDTH-1:0]             M_AXIS_RC_TDATA,
     input [(C_PCI_DATA_WIDTH/32)-1:0]        M_AXIS_RC_TKEEP,
     input [`SIG_RC_TUSER_W-1:0]              M_AXIS_RC_TUSER,
     output                                   M_AXIS_RC_TREADY,

     //Interface: CC Ultrascale (TXC)
     input                                    S_AXIS_CC_TREADY,
     output                                   S_AXIS_CC_TVALID,
     output                                   S_AXIS_CC_TLAST,
     output [C_PCI_DATA_WIDTH-1:0]            S_AXIS_CC_TDATA,
     output [(C_PCI_DATA_WIDTH/32)-1:0]       S_AXIS_CC_TKEEP,
     output [`SIG_CC_TUSER_W-1:0]             S_AXIS_CC_TUSER,
    
     //Interface: RQ Ultrascale (TXR) 
     input                                    S_AXIS_RQ_TREADY,
     output                                   S_AXIS_RQ_TVALID,
     output                                   S_AXIS_RQ_TLAST,
     output [C_PCI_DATA_WIDTH-1:0]            S_AXIS_RQ_TDATA,
     output [(C_PCI_DATA_WIDTH/32)-1:0]       S_AXIS_RQ_TKEEP,
     output [`SIG_RQ_TUSER_W-1:0]             S_AXIS_RQ_TUSER,

     // Interface: RXC Engine
     output [C_PCI_DATA_WIDTH-1:0]            RXC_DATA,
     output                                   RXC_DATA_VALID,
     output [(C_PCI_DATA_WIDTH/32)-1:0]       RXC_DATA_WORD_ENABLE,
     output                                   RXC_DATA_START_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0] RXC_DATA_START_OFFSET,
     output                                   RXC_DATA_END_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0] RXC_DATA_END_OFFSET,

     output [`SIG_LBE_W-1:0]                  RXC_META_LDWBE,
     output [`SIG_FBE_W-1:0]                  RXC_META_FDWBE,
     output [`SIG_TAG_W-1:0]                  RXC_META_TAG,
     output [`SIG_LOWADDR_W-1:0]              RXC_META_ADDR,
     output [`SIG_TYPE_W-1:0]                 RXC_META_TYPE,
     output [`SIG_LEN_W-1:0]                  RXC_META_LENGTH,
     output [`SIG_BYTECNT_W-1:0]              RXC_META_BYTES_REMAINING,
     output [`SIG_CPLID_W-1:0]                RXC_META_COMPLETER_ID,
     output                                   RXC_META_EP,

     // Interface: RXR Engine
     output [C_PCI_DATA_WIDTH-1:0]            RXR_DATA,
     output                                   RXR_DATA_VALID,
     output [(C_PCI_DATA_WIDTH/32)-1:0]       RXR_DATA_WORD_ENABLE,
     output                                   RXR_DATA_START_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0] RXR_DATA_START_OFFSET,
     output                                   RXR_DATA_END_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0] RXR_DATA_END_OFFSET,
    
     output [`SIG_FBE_W-1:0]                  RXR_META_FDWBE,
     output [`SIG_LBE_W-1:0]                  RXR_META_LDWBE,
     output [`SIG_TC_W-1:0]                   RXR_META_TC,
     output [`SIG_ATTR_W-1:0]                 RXR_META_ATTR,
     output [`SIG_TAG_W-1:0]                  RXR_META_TAG,
     output [`SIG_TYPE_W-1:0]                 RXR_META_TYPE,
     output [`SIG_ADDR_W-1:0]                 RXR_META_ADDR,
     output [`SIG_BARDECODE_W-1:0]            RXR_META_BAR_DECODED,
     output [`SIG_REQID_W-1:0]                RXR_META_REQUESTER_ID,
     output [`SIG_LEN_W-1:0]                  RXR_META_LENGTH,
     output                                   RXR_META_EP,
    
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

    wire                                      CLK;


    assign CLK = CLK_BUS;    
    generate
        /* verilator lint_off WIDTH */
        if(C_VENDOR != "ULTRASCALE") begin
            assign M_AXIS_CQ_TREADY = 0;
            assign M_AXIS_RC_TREADY = 0;
            assign S_AXIS_CC_TVALID = 0;
            assign S_AXIS_CC_TLAST = 0;
            assign S_AXIS_CC_TDATA = 0;
            assign S_AXIS_CC_TKEEP = 0;
            assign S_AXIS_CC_TUSER = 0;
            
            assign S_AXIS_RQ_TVALID = 0;
            assign S_AXIS_RQ_TLAST = 0;
            assign S_AXIS_RQ_TDATA = 0;
            assign S_AXIS_RQ_TKEEP = 0;
            assign S_AXIS_RQ_TUSER = 0;

            /* verilator lint_on WIDTH */
	        rx_engine_classic
	            #(/*AUTOINSTPARAM*/
                  // Parameters
                  .C_VENDOR             (C_VENDOR),
                  .C_PCI_DATA_WIDTH     (C_PCI_DATA_WIDTH),
                  .C_LOG_NUM_TAGS       (C_LOG_NUM_TAGS))
	        rx_engine_classic_inst
	            (/*AUTOINST*/
                 // Outputs
                 .DONE_RXR_RST          (DONE_RXR_RST),
                 .DONE_RXC_RST          (DONE_RXC_RST),
                 .RX_TLP_READY          (RX_TLP_READY),
                 .RXC_DATA              (RXC_DATA[C_PCI_DATA_WIDTH-1:0]),
                 .RXC_DATA_VALID        (RXC_DATA_VALID),
                 .RXC_DATA_WORD_ENABLE  (RXC_DATA_WORD_ENABLE[(C_PCI_DATA_WIDTH/32)-1:0]),
                 .RXC_DATA_START_FLAG   (RXC_DATA_START_FLAG),
                 .RXC_DATA_START_OFFSET (RXC_DATA_START_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .RXC_DATA_END_FLAG     (RXC_DATA_END_FLAG),
                 .RXC_DATA_END_OFFSET   (RXC_DATA_END_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .RXC_META_LDWBE        (RXC_META_LDWBE[`SIG_LBE_W-1:0]),
                 .RXC_META_FDWBE        (RXC_META_FDWBE[`SIG_FBE_W-1:0]),
                 .RXC_META_TAG          (RXC_META_TAG[`SIG_TAG_W-1:0]),
                 .RXC_META_ADDR         (RXC_META_ADDR[`SIG_LOWADDR_W-1:0]),
                 .RXC_META_TYPE         (RXC_META_TYPE[`SIG_TYPE_W-1:0]),
                 .RXC_META_LENGTH       (RXC_META_LENGTH[`SIG_LEN_W-1:0]),
                 .RXC_META_BYTES_REMAINING(RXC_META_BYTES_REMAINING[`SIG_BYTECNT_W-1:0]),
                 .RXC_META_COMPLETER_ID (RXC_META_COMPLETER_ID[`SIG_CPLID_W-1:0]),
                 .RXC_META_EP           (RXC_META_EP),
                 .RXR_DATA              (RXR_DATA[C_PCI_DATA_WIDTH-1:0]),
                 .RXR_DATA_VALID        (RXR_DATA_VALID),
                 .RXR_DATA_WORD_ENABLE  (RXR_DATA_WORD_ENABLE[(C_PCI_DATA_WIDTH/32)-1:0]),
                 .RXR_DATA_START_FLAG   (RXR_DATA_START_FLAG),
                 .RXR_DATA_START_OFFSET (RXR_DATA_START_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .RXR_DATA_END_FLAG     (RXR_DATA_END_FLAG),
                 .RXR_DATA_END_OFFSET   (RXR_DATA_END_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .RXR_META_FDWBE        (RXR_META_FDWBE[`SIG_FBE_W-1:0]),
                 .RXR_META_LDWBE        (RXR_META_LDWBE[`SIG_LBE_W-1:0]),
                 .RXR_META_TC           (RXR_META_TC[`SIG_TC_W-1:0]),
                 .RXR_META_ATTR         (RXR_META_ATTR[`SIG_ATTR_W-1:0]),
                 .RXR_META_TAG          (RXR_META_TAG[`SIG_TAG_W-1:0]),
                 .RXR_META_TYPE         (RXR_META_TYPE[`SIG_TYPE_W-1:0]),
                 .RXR_META_ADDR         (RXR_META_ADDR[`SIG_ADDR_W-1:0]),
                 .RXR_META_BAR_DECODED  (RXR_META_BAR_DECODED[`SIG_BARDECODE_W-1:0]),
                 .RXR_META_REQUESTER_ID (RXR_META_REQUESTER_ID[`SIG_REQID_W-1:0]),
                 .RXR_META_LENGTH       (RXR_META_LENGTH[`SIG_LEN_W-1:0]),
                 .RXR_META_EP           (RXR_META_EP),
                 // Inputs
                 .CLK                   (CLK),
                 .RST_BUS               (RST_BUS),
                 .RST_LOGIC             (RST_LOGIC),
                 .RX_TLP                (RX_TLP[C_PCI_DATA_WIDTH-1:0]),
                 .RX_TLP_VALID          (RX_TLP_VALID),
                 .RX_TLP_START_FLAG     (RX_TLP_START_FLAG),
                 .RX_TLP_START_OFFSET   (RX_TLP_START_OFFSET[`SIG_OFFSET_W-1:0]),
                 .RX_TLP_END_FLAG       (RX_TLP_END_FLAG),
                 .RX_TLP_END_OFFSET     (RX_TLP_END_OFFSET[`SIG_OFFSET_W-1:0]),
                 .RX_TLP_BAR_DECODE     (RX_TLP_BAR_DECODE[`SIG_BARDECODE_W-1:0]));

	        tx_engine_classic
	            #(/*AUTOINSTPARAM*/
                  // Parameters
                  .C_PCI_DATA_WIDTH     (C_PCI_DATA_WIDTH),
                  .C_PIPELINE_INPUT     (C_PIPELINE_INPUT),
                  .C_PIPELINE_OUTPUT    (C_PIPELINE_OUTPUT),
                  .C_MAX_PAYLOAD_DWORDS (C_MAX_PAYLOAD_DWORDS),
                  .C_VENDOR             (C_VENDOR))
	        tx_engine_classic_inst
	            (/*AUTOINST*/
                 // Outputs
                 .DONE_TXC_RST          (DONE_TXC_RST),
                 .DONE_TXR_RST          (DONE_TXR_RST),
                 .TX_TLP                (TX_TLP[C_PCI_DATA_WIDTH-1:0]),
                 .TX_TLP_VALID          (TX_TLP_VALID),
                 .TX_TLP_START_FLAG     (TX_TLP_START_FLAG),
                 .TX_TLP_START_OFFSET   (TX_TLP_START_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .TX_TLP_END_FLAG       (TX_TLP_END_FLAG),
                 .TX_TLP_END_OFFSET     (TX_TLP_END_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .TXC_DATA_READY        (TXC_DATA_READY),
                 .TXC_META_READY        (TXC_META_READY),
                 .TXC_SENT              (TXC_SENT),
                 .TXR_DATA_READY        (TXR_DATA_READY),
                 .TXR_META_READY        (TXR_META_READY),
                 .TXR_SENT              (TXR_SENT),
                 // Inputs
                 .CLK                   (CLK),
                 .RST_BUS               (RST_BUS),
                 .RST_LOGIC             (RST_LOGIC),
                 .CONFIG_COMPLETER_ID   (CONFIG_COMPLETER_ID[`SIG_CPLID_W-1:0]),
                 .TX_TLP_READY          (TX_TLP_READY),
                 .TXC_DATA_VALID        (TXC_DATA_VALID),
                 .TXC_DATA              (TXC_DATA[C_PCI_DATA_WIDTH-1:0]),
                 .TXC_DATA_START_FLAG   (TXC_DATA_START_FLAG),
                 .TXC_DATA_START_OFFSET (TXC_DATA_START_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .TXC_DATA_END_FLAG     (TXC_DATA_END_FLAG),
                 .TXC_DATA_END_OFFSET   (TXC_DATA_END_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .TXC_META_VALID        (TXC_META_VALID),
                 .TXC_META_FDWBE        (TXC_META_FDWBE[`SIG_FBE_W-1:0]),
                 .TXC_META_LDWBE        (TXC_META_LDWBE[`SIG_LBE_W-1:0]),
                 .TXC_META_ADDR         (TXC_META_ADDR[`SIG_LOWADDR_W-1:0]),
                 .TXC_META_TYPE         (TXC_META_TYPE[`SIG_TYPE_W-1:0]),
                 .TXC_META_LENGTH       (TXC_META_LENGTH[`SIG_LEN_W-1:0]),
                 .TXC_META_BYTE_COUNT   (TXC_META_BYTE_COUNT[`SIG_BYTECNT_W-1:0]),
                 .TXC_META_TAG          (TXC_META_TAG[`SIG_TAG_W-1:0]),
                 .TXC_META_REQUESTER_ID (TXC_META_REQUESTER_ID[`SIG_REQID_W-1:0]),
                 .TXC_META_TC           (TXC_META_TC[`SIG_TC_W-1:0]),
                 .TXC_META_ATTR         (TXC_META_ATTR[`SIG_ATTR_W-1:0]),
                 .TXC_META_EP           (TXC_META_EP),
                 .TXR_DATA_VALID        (TXR_DATA_VALID),
                 .TXR_DATA              (TXR_DATA[C_PCI_DATA_WIDTH-1:0]),
                 .TXR_DATA_START_FLAG   (TXR_DATA_START_FLAG),
                 .TXR_DATA_START_OFFSET (TXR_DATA_START_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .TXR_DATA_END_FLAG     (TXR_DATA_END_FLAG),
                 .TXR_DATA_END_OFFSET   (TXR_DATA_END_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .TXR_META_VALID        (TXR_META_VALID),
                 .TXR_META_FDWBE        (TXR_META_FDWBE[`SIG_FBE_W-1:0]),
                 .TXR_META_LDWBE        (TXR_META_LDWBE[`SIG_LBE_W-1:0]),
                 .TXR_META_ADDR         (TXR_META_ADDR[`SIG_ADDR_W-1:0]),
                 .TXR_META_LENGTH       (TXR_META_LENGTH[`SIG_LEN_W-1:0]),
                 .TXR_META_TAG          (TXR_META_TAG[`SIG_TAG_W-1:0]),
                 .TXR_META_TC           (TXR_META_TC[`SIG_TC_W-1:0]),
                 .TXR_META_ATTR         (TXR_META_ATTR[`SIG_ATTR_W-1:0]),
                 .TXR_META_TYPE         (TXR_META_TYPE[`SIG_TYPE_W-1:0]),
                 .TXR_META_EP           (TXR_META_EP));

        end else begin 

            assign TX_TLP = 0;
            assign TX_TLP_VALID = 0;
            assign TX_TLP_START_FLAG = 0;
            assign TX_TLP_START_OFFSET = 0;
            assign TX_TLP_END_FLAG = 0;
            assign TX_TLP_END_OFFSET = 0;
            assign RX_TLP_READY = 0;
	        rx_engine_ultrascale
	            #(/*AUTOINSTPARAM*/
                  // Parameters
                  .C_PCI_DATA_WIDTH     (C_PCI_DATA_WIDTH))
	        rx_engine_ultrascale_inst
	            (/*AUTOINST*/
                 // Outputs
                 .DONE_RXR_RST          (DONE_RXR_RST),
                 .DONE_RXC_RST          (DONE_RXC_RST),
                 .M_AXIS_CQ_TREADY      (M_AXIS_CQ_TREADY),
                 .M_AXIS_RC_TREADY      (M_AXIS_RC_TREADY),
                 .RXC_DATA              (RXC_DATA[C_PCI_DATA_WIDTH-1:0]),
                 .RXC_DATA_VALID        (RXC_DATA_VALID),
                 .RXC_DATA_WORD_ENABLE  (RXC_DATA_WORD_ENABLE[(C_PCI_DATA_WIDTH/32)-1:0]),
                 .RXC_DATA_START_FLAG   (RXC_DATA_START_FLAG),
                 .RXC_DATA_START_OFFSET (RXC_DATA_START_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .RXC_DATA_END_FLAG     (RXC_DATA_END_FLAG),
                 .RXC_DATA_END_OFFSET   (RXC_DATA_END_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .RXC_META_LDWBE        (RXC_META_LDWBE[`SIG_LBE_W-1:0]),
                 .RXC_META_FDWBE        (RXC_META_FDWBE[`SIG_FBE_W-1:0]),
                 .RXC_META_TAG          (RXC_META_TAG[`SIG_TAG_W-1:0]),
                 .RXC_META_ADDR         (RXC_META_ADDR[`SIG_LOWADDR_W-1:0]),
                 .RXC_META_TYPE         (RXC_META_TYPE[`SIG_TYPE_W-1:0]),
                 .RXC_META_LENGTH       (RXC_META_LENGTH[`SIG_LEN_W-1:0]),
                 .RXC_META_BYTES_REMAINING(RXC_META_BYTES_REMAINING[`SIG_BYTECNT_W-1:0]),
                 .RXC_META_COMPLETER_ID (RXC_META_COMPLETER_ID[`SIG_CPLID_W-1:0]),
                 .RXC_META_EP           (RXC_META_EP),
                 .RXR_DATA              (RXR_DATA[C_PCI_DATA_WIDTH-1:0]),
                 .RXR_DATA_VALID        (RXR_DATA_VALID),
                 .RXR_DATA_WORD_ENABLE  (RXR_DATA_WORD_ENABLE[(C_PCI_DATA_WIDTH/32)-1:0]),
                 .RXR_DATA_START_FLAG   (RXR_DATA_START_FLAG),
                 .RXR_DATA_START_OFFSET (RXR_DATA_START_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .RXR_DATA_END_FLAG     (RXR_DATA_END_FLAG),
                 .RXR_DATA_END_OFFSET   (RXR_DATA_END_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .RXR_META_FDWBE        (RXR_META_FDWBE[`SIG_FBE_W-1:0]),
                 .RXR_META_LDWBE        (RXR_META_LDWBE[`SIG_LBE_W-1:0]),
                 .RXR_META_TC           (RXR_META_TC[`SIG_TC_W-1:0]),
                 .RXR_META_ATTR         (RXR_META_ATTR[`SIG_ATTR_W-1:0]),
                 .RXR_META_TAG          (RXR_META_TAG[`SIG_TAG_W-1:0]),
                 .RXR_META_TYPE         (RXR_META_TYPE[`SIG_TYPE_W-1:0]),
                 .RXR_META_ADDR         (RXR_META_ADDR[`SIG_ADDR_W-1:0]),
                 .RXR_META_BAR_DECODED  (RXR_META_BAR_DECODED[`SIG_BARDECODE_W-1:0]),
                 .RXR_META_REQUESTER_ID (RXR_META_REQUESTER_ID[`SIG_REQID_W-1:0]),
                 .RXR_META_LENGTH       (RXR_META_LENGTH[`SIG_LEN_W-1:0]),
                 .RXR_META_EP           (RXR_META_EP),
                 // Inputs
                 .CLK                   (CLK),
                 .RST_BUS               (RST_BUS),
                 .RST_LOGIC             (RST_LOGIC),
                 .M_AXIS_CQ_TVALID      (M_AXIS_CQ_TVALID),
                 .M_AXIS_CQ_TLAST       (M_AXIS_CQ_TLAST),
                 .M_AXIS_CQ_TDATA       (M_AXIS_CQ_TDATA[C_PCI_DATA_WIDTH-1:0]),
                 .M_AXIS_CQ_TKEEP       (M_AXIS_CQ_TKEEP[(C_PCI_DATA_WIDTH/32)-1:0]),
                 .M_AXIS_CQ_TUSER       (M_AXIS_CQ_TUSER[`SIG_CQ_TUSER_W-1:0]),
                 .M_AXIS_RC_TVALID      (M_AXIS_RC_TVALID),
                 .M_AXIS_RC_TLAST       (M_AXIS_RC_TLAST),
                 .M_AXIS_RC_TDATA       (M_AXIS_RC_TDATA[C_PCI_DATA_WIDTH-1:0]),
                 .M_AXIS_RC_TKEEP       (M_AXIS_RC_TKEEP[(C_PCI_DATA_WIDTH/32)-1:0]),
                 .M_AXIS_RC_TUSER       (M_AXIS_RC_TUSER[`SIG_RC_TUSER_W-1:0]));

	        tx_engine_ultrascale
	            #(/*AUTOINSTPARAM*/
                  // Parameters
                  .C_PCI_DATA_WIDTH     (C_PCI_DATA_WIDTH),
                  .C_PIPELINE_INPUT     (C_PIPELINE_INPUT),
                  .C_PIPELINE_OUTPUT    (C_PIPELINE_OUTPUT),
                  .C_MAX_PAYLOAD_DWORDS (C_MAX_PAYLOAD_DWORDS))
	        tx_engine_ultrascale_inst
	            (/*AUTOINST*/
                 // Outputs
                 .DONE_TXC_RST          (DONE_TXC_RST),
                 .DONE_TXR_RST          (DONE_TXR_RST),
                 .S_AXIS_CC_TVALID      (S_AXIS_CC_TVALID),
                 .S_AXIS_CC_TLAST       (S_AXIS_CC_TLAST),
                 .S_AXIS_CC_TDATA       (S_AXIS_CC_TDATA[C_PCI_DATA_WIDTH-1:0]),
                 .S_AXIS_CC_TKEEP       (S_AXIS_CC_TKEEP[(C_PCI_DATA_WIDTH/32)-1:0]),
                 .S_AXIS_CC_TUSER       (S_AXIS_CC_TUSER[`SIG_CC_TUSER_W-1:0]),
                 .TXC_DATA_READY        (TXC_DATA_READY),
                 .TXC_META_READY        (TXC_META_READY),
                 .TXC_SENT              (TXC_SENT),
                 .S_AXIS_RQ_TVALID      (S_AXIS_RQ_TVALID),
                 .S_AXIS_RQ_TLAST       (S_AXIS_RQ_TLAST),
                 .S_AXIS_RQ_TDATA       (S_AXIS_RQ_TDATA[C_PCI_DATA_WIDTH-1:0]),
                 .S_AXIS_RQ_TKEEP       (S_AXIS_RQ_TKEEP[(C_PCI_DATA_WIDTH/32)-1:0]),
                 .S_AXIS_RQ_TUSER       (S_AXIS_RQ_TUSER[`SIG_RQ_TUSER_W-1:0]),
                 .TXR_DATA_READY        (TXR_DATA_READY),
                 .TXR_META_READY        (TXR_META_READY),
                 .TXR_SENT              (TXR_SENT),
                 // Inputs
                 .CLK                   (CLK),
                 .RST_BUS               (RST_BUS),
                 .RST_LOGIC             (RST_LOGIC),
                 .CONFIG_COMPLETER_ID   (CONFIG_COMPLETER_ID[`SIG_CPLID_W-1:0]),
                 .S_AXIS_CC_TREADY      (S_AXIS_CC_TREADY),
                 .TXC_DATA_VALID        (TXC_DATA_VALID),
                 .TXC_DATA              (TXC_DATA[C_PCI_DATA_WIDTH-1:0]),
                 .TXC_DATA_START_FLAG   (TXC_DATA_START_FLAG),
                 .TXC_DATA_START_OFFSET (TXC_DATA_START_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .TXC_DATA_END_FLAG     (TXC_DATA_END_FLAG),
                 .TXC_DATA_END_OFFSET   (TXC_DATA_END_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .TXC_META_VALID        (TXC_META_VALID),
                 .TXC_META_FDWBE        (TXC_META_FDWBE[`SIG_FBE_W-1:0]),
                 .TXC_META_LDWBE        (TXC_META_LDWBE[`SIG_LBE_W-1:0]),
                 .TXC_META_ADDR         (TXC_META_ADDR[`SIG_LOWADDR_W-1:0]),
                 .TXC_META_TYPE         (TXC_META_TYPE[`SIG_TYPE_W-1:0]),
                 .TXC_META_LENGTH       (TXC_META_LENGTH[`SIG_LEN_W-1:0]),
                 .TXC_META_BYTE_COUNT   (TXC_META_BYTE_COUNT[`SIG_BYTECNT_W-1:0]),
                 .TXC_META_TAG          (TXC_META_TAG[`SIG_TAG_W-1:0]),
                 .TXC_META_REQUESTER_ID (TXC_META_REQUESTER_ID[`SIG_REQID_W-1:0]),
                 .TXC_META_TC           (TXC_META_TC[`SIG_TC_W-1:0]),
                 .TXC_META_ATTR         (TXC_META_ATTR[`SIG_ATTR_W-1:0]),
                 .TXC_META_EP           (TXC_META_EP),
                 .S_AXIS_RQ_TREADY      (S_AXIS_RQ_TREADY),
                 .TXR_DATA_VALID        (TXR_DATA_VALID),
                 .TXR_DATA              (TXR_DATA[C_PCI_DATA_WIDTH-1:0]),
                 .TXR_DATA_START_FLAG   (TXR_DATA_START_FLAG),
                 .TXR_DATA_START_OFFSET (TXR_DATA_START_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .TXR_DATA_END_FLAG     (TXR_DATA_END_FLAG),
                 .TXR_DATA_END_OFFSET   (TXR_DATA_END_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .TXR_META_VALID        (TXR_META_VALID),
                 .TXR_META_FDWBE        (TXR_META_FDWBE[`SIG_FBE_W-1:0]),
                 .TXR_META_LDWBE        (TXR_META_LDWBE[`SIG_LBE_W-1:0]),
                 .TXR_META_ADDR         (TXR_META_ADDR[`SIG_ADDR_W-1:0]),
                 .TXR_META_LENGTH       (TXR_META_LENGTH[`SIG_LEN_W-1:0]),
                 .TXR_META_TAG          (TXR_META_TAG[`SIG_TAG_W-1:0]),
                 .TXR_META_TC           (TXR_META_TC[`SIG_TC_W-1:0]),
                 .TXR_META_ATTR         (TXR_META_ATTR[`SIG_ATTR_W-1:0]),
                 .TXR_META_TYPE         (TXR_META_TYPE[`SIG_TYPE_W-1:0]),
                 .TXR_META_EP           (TXR_META_EP));
        end
    endgenerate
endmodule // engine_layer
// Local Variables:
// verilog-library-directories:("." "ultrascale/rx/" "ultrascale/tx/" "classic/rx/" "classic/tx/")
// End:
