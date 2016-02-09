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
// Filename:            rx_engine_ultrascale.v
// Version:             1.0
// Verilog Standard:    Verilog-2001
// Description:         The RX Engine (Ultrascale) takes a the two streams of
// AXI from the Xilinx endpoint packets and provides the request packets on the
// RXR Interface, and the completion packets on the RXC Interface.
// This Engine is capable of operating at "line rate".
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
`include "ultrascale.vh"
`include "trellis.vh"
module rx_engine_ultrascale
    #(parameter C_PCI_DATA_WIDTH = 128)
    (// Interface: Clocks
     input                                    CLK, // Replacement for generic CLK

     // Interface: Resets
     input                                    RST_BUS, // Replacement for generic RST_IN
     input                                    RST_LOGIC, // Addition for RIFFA_RST
     output                                   DONE_RXR_RST,
     output                                   DONE_RXC_RST,

     // Interface: CQ
     input                                    M_AXIS_CQ_TVALID,
     input                                    M_AXIS_CQ_TLAST,
     input [C_PCI_DATA_WIDTH-1:0]             M_AXIS_CQ_TDATA,
     input [(C_PCI_DATA_WIDTH/32)-1:0]        M_AXIS_CQ_TKEEP,
     input [`SIG_CQ_TUSER_W-1:0]              M_AXIS_CQ_TUSER,
     output                                   M_AXIS_CQ_TREADY,
    
     // Interface: RC
     input                                    M_AXIS_RC_TVALID,
     input                                    M_AXIS_RC_TLAST,
     input [C_PCI_DATA_WIDTH-1:0]             M_AXIS_RC_TDATA,
     input [(C_PCI_DATA_WIDTH/32)-1:0]        M_AXIS_RC_TKEEP,
     input [`SIG_RC_TUSER_W-1:0]              M_AXIS_RC_TUSER,
     output                                   M_AXIS_RC_TREADY,

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
     output                                   RXR_META_EP
     );

    localparam C_RX_PIPELINE_DEPTH = 3;

    rxc_engine_ultrascale
        #(///*AUTOINSTPARAM*/
          // Parameters
          .C_PCI_DATA_WIDTH		(C_PCI_DATA_WIDTH),
          .C_RX_PIPELINE_DEPTH		(C_RX_PIPELINE_DEPTH))
    rxc_engine_inst
        (/*AUTOINST*/
         // Outputs
         .DONE_RXC_RST                  (DONE_RXC_RST),
         .M_AXIS_RC_TREADY              (M_AXIS_RC_TREADY),
         .RXC_DATA                      (RXC_DATA[C_PCI_DATA_WIDTH-1:0]),
         .RXC_DATA_VALID                (RXC_DATA_VALID),
         .RXC_DATA_WORD_ENABLE          (RXC_DATA_WORD_ENABLE[(C_PCI_DATA_WIDTH/32)-1:0]),
         .RXC_DATA_START_FLAG           (RXC_DATA_START_FLAG),
         .RXC_DATA_START_OFFSET         (RXC_DATA_START_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .RXC_DATA_END_FLAG             (RXC_DATA_END_FLAG),
         .RXC_DATA_END_OFFSET           (RXC_DATA_END_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .RXC_META_LDWBE                (RXC_META_LDWBE[`SIG_LBE_W-1:0]),
         .RXC_META_FDWBE                (RXC_META_FDWBE[`SIG_FBE_W-1:0]),
         .RXC_META_TAG                  (RXC_META_TAG[`SIG_TAG_W-1:0]),
         .RXC_META_ADDR                 (RXC_META_ADDR[`SIG_LOWADDR_W-1:0]),
         .RXC_META_TYPE                 (RXC_META_TYPE[`SIG_TYPE_W-1:0]),
         .RXC_META_LENGTH               (RXC_META_LENGTH[`SIG_LEN_W-1:0]),
         .RXC_META_BYTES_REMAINING      (RXC_META_BYTES_REMAINING[`SIG_BYTECNT_W-1:0]),
         .RXC_META_COMPLETER_ID         (RXC_META_COMPLETER_ID[`SIG_CPLID_W-1:0]),
         .RXC_META_EP                   (RXC_META_EP),
         // Inputs
         .CLK                           (CLK),
         .RST_BUS                       (RST_BUS),
         .RST_LOGIC                     (RST_LOGIC),
         .M_AXIS_RC_TVALID              (M_AXIS_RC_TVALID),
         .M_AXIS_RC_TLAST               (M_AXIS_RC_TLAST),
         .M_AXIS_RC_TDATA               (M_AXIS_RC_TDATA[C_PCI_DATA_WIDTH-1:0]),
         .M_AXIS_RC_TKEEP               (M_AXIS_RC_TKEEP[(C_PCI_DATA_WIDTH/32)-1:0]),
         .M_AXIS_RC_TUSER               (M_AXIS_RC_TUSER[`SIG_RC_TUSER_W-1:0]));

    rxr_engine_ultrascale
        #(/*AUTOINSTPARAM*/
          // Parameters
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
          .C_RX_PIPELINE_DEPTH          (C_RX_PIPELINE_DEPTH))
    rxr_engine_inst
        (/*AUTOINST*/
         // Outputs
         .DONE_RXR_RST                  (DONE_RXR_RST),
         .M_AXIS_CQ_TREADY              (M_AXIS_CQ_TREADY),
         .RXR_DATA                      (RXR_DATA[C_PCI_DATA_WIDTH-1:0]),
         .RXR_DATA_VALID                (RXR_DATA_VALID),
         .RXR_DATA_WORD_ENABLE          (RXR_DATA_WORD_ENABLE[(C_PCI_DATA_WIDTH/32)-1:0]),
         .RXR_DATA_START_FLAG           (RXR_DATA_START_FLAG),
         .RXR_DATA_START_OFFSET         (RXR_DATA_START_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .RXR_DATA_END_FLAG             (RXR_DATA_END_FLAG),
         .RXR_DATA_END_OFFSET           (RXR_DATA_END_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .RXR_META_FDWBE                (RXR_META_FDWBE[`SIG_FBE_W-1:0]),
         .RXR_META_LDWBE                (RXR_META_LDWBE[`SIG_LBE_W-1:0]),
         .RXR_META_TC                   (RXR_META_TC[`SIG_TC_W-1:0]),
         .RXR_META_ATTR                 (RXR_META_ATTR[`SIG_ATTR_W-1:0]),
         .RXR_META_TAG                  (RXR_META_TAG[`SIG_TAG_W-1:0]),
         .RXR_META_TYPE                 (RXR_META_TYPE[`SIG_TYPE_W-1:0]),
         .RXR_META_ADDR                 (RXR_META_ADDR[`SIG_ADDR_W-1:0]),
         .RXR_META_BAR_DECODED          (RXR_META_BAR_DECODED[`SIG_BARDECODE_W-1:0]),
         .RXR_META_REQUESTER_ID         (RXR_META_REQUESTER_ID[`SIG_REQID_W-1:0]),
         .RXR_META_LENGTH               (RXR_META_LENGTH[`SIG_LEN_W-1:0]),
         .RXR_META_EP                   (RXR_META_EP),
         // Inputs
         .CLK                           (CLK),
         .RST_BUS                       (RST_BUS),
         .RST_LOGIC                     (RST_LOGIC),
         .M_AXIS_CQ_TVALID              (M_AXIS_CQ_TVALID),
         .M_AXIS_CQ_TLAST               (M_AXIS_CQ_TLAST),
         .M_AXIS_CQ_TDATA               (M_AXIS_CQ_TDATA[C_PCI_DATA_WIDTH-1:0]),
         .M_AXIS_CQ_TKEEP               (M_AXIS_CQ_TKEEP[(C_PCI_DATA_WIDTH/32)-1:0]),
         .M_AXIS_CQ_TUSER               (M_AXIS_CQ_TUSER[`SIG_CQ_TUSER_W-1:0]));

endmodule // rx_engine_ultrascale
// Local Variables:
// verilog-library-directories:("." "./rx/")
// End:
