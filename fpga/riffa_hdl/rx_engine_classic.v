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
// Filename:            rx_engine_classic.v
// Version:             1.0
// Verilog Standard:    Verilog-2001
// Description:         The RX Engine (Classic) takes a single stream of TLP
// packets and provides the request packets on the RXR Interface, and the 
// completion packets on the RXC Interface.
// This Engine is capable of operating at "line rate".
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
`include "trellis.vh"
`include "tlp.vh"
module rx_engine_classic
    #(parameter C_VENDOR = "ALTERA",
      parameter C_PCI_DATA_WIDTH = 128,
      parameter C_LOG_NUM_TAGS=6)
    (// Interface: Clocks
     input                                    CLK,

     // Interface: Resets
     input                                    RST_BUS, // Replacement for generic RST_IN
     input                                    RST_LOGIC, // Addition for RIFFA_RST
     output                                   DONE_RXR_RST,
     output                                   DONE_RXC_RST,

     // Interface: RX Classic
     input [C_PCI_DATA_WIDTH-1:0]             RX_TLP,
     input                                    RX_TLP_VALID,
     output                                   RX_TLP_READY,
     input                                    RX_TLP_START_FLAG,
     input [`SIG_OFFSET_W-1:0]                RX_TLP_START_OFFSET,
     input                                    RX_TLP_END_FLAG,
     input [`SIG_OFFSET_W-1:0]                RX_TLP_END_OFFSET,
     input [`SIG_BARDECODE_W-1:0]             RX_TLP_BAR_DECODE,
    
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
    localparam C_RX_PIPELINE_DEPTH = 4;

    wire [C_PCI_DATA_WIDTH-1:0]               _RXC_DATA;
    wire [C_PCI_DATA_WIDTH-1:0]               _RXR_DATA;
    wire [(C_RX_PIPELINE_DEPTH+1)*C_PCI_DATA_WIDTH-1:0] wRxSrData;
    wire [C_RX_PIPELINE_DEPTH:0]                        wRxSrSop;
    wire [C_RX_PIPELINE_DEPTH:0]                        wRxSrEop;
    wire [(C_RX_PIPELINE_DEPTH+1)*`SIG_OFFSET_W-1:0]    wRxSrEoff;
    wire [(C_RX_PIPELINE_DEPTH+1)*`SIG_OFFSET_W-1:0]    wRxSrSoff;
    wire [C_RX_PIPELINE_DEPTH:0]                        wRxSrDataValid;

    generate
        if(C_VENDOR == "XILINX") begin : xilinx_data
            if(C_PCI_DATA_WIDTH == 128) begin : x_be_swap128
                assign RXC_DATA = {_RXC_DATA[103:96], _RXC_DATA[111:104], _RXC_DATA[119:112], _RXC_DATA[127:120],
                                   _RXC_DATA[71:64], _RXC_DATA[79:72], _RXC_DATA[87:80], _RXC_DATA[95:88],
                                   _RXC_DATA[39:32], _RXC_DATA[47:40], _RXC_DATA[55:48], _RXC_DATA[63:56],
                                   _RXC_DATA[07:00], _RXC_DATA[15:08], _RXC_DATA[23:16], _RXC_DATA[31:24]};
                assign RXR_DATA = {_RXR_DATA[103:96], _RXR_DATA[111:104], _RXR_DATA[119:112], _RXR_DATA[127:120],
                                   _RXR_DATA[71:64], _RXR_DATA[79:72], _RXR_DATA[87:80], _RXR_DATA[95:88],
                                   _RXR_DATA[39:32], _RXR_DATA[47:40], _RXR_DATA[55:48], _RXR_DATA[63:56],
                                   _RXR_DATA[07:00], _RXR_DATA[15:08], _RXR_DATA[23:16], _RXR_DATA[31:24]};
            end else if(C_PCI_DATA_WIDTH == 64) begin: x_be_swap64
                assign RXC_DATA = {_RXC_DATA[39:32], _RXC_DATA[47:40], _RXC_DATA[55:48], _RXC_DATA[63:56],
                                   _RXC_DATA[07:00], _RXC_DATA[15:08], _RXC_DATA[23:16], _RXC_DATA[31:24]};
                assign RXR_DATA = {_RXR_DATA[39:32], _RXR_DATA[47:40], _RXR_DATA[55:48], _RXR_DATA[63:56],
                                   _RXR_DATA[07:00], _RXR_DATA[15:08], _RXR_DATA[23:16], _RXR_DATA[31:24]};
            end else if(C_PCI_DATA_WIDTH == 32) begin: x_be_swap32
                assign RXC_DATA = {_RXC_DATA[07:00], _RXC_DATA[15:08], _RXC_DATA[23:16], _RXC_DATA[31:24]};
                assign RXR_DATA = {_RXR_DATA[07:00], _RXR_DATA[15:08], _RXR_DATA[23:16], _RXR_DATA[31:24]};
            end
        end else begin : altera_data
            assign RXC_DATA = _RXC_DATA;
            assign RXR_DATA = _RXR_DATA;
        end
    endgenerate

    assign RX_TLP_READY = 1'b1;
    // Shift register for input data with output taps for each delayed
    // cycle.  Shared by RXC and RXR engines.
    shiftreg
        #(// Parameters
          .C_DEPTH                      (C_RX_PIPELINE_DEPTH),
          .C_WIDTH                      (C_PCI_DATA_WIDTH),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    data_shiftreg_inst
        (// Outputs
         .RD_DATA                       (wRxSrData),
         // Inputs
         .WR_DATA                       (RX_TLP),
         .RST_IN                        (0),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    // Start Flag Shift Register. Data enables are derived from the
    // taps on this shift register.
    shiftreg 
        #(// Parameters
          .C_DEPTH                      (C_RX_PIPELINE_DEPTH),
          .C_WIDTH                      (1'b1),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    sop_shiftreg_inst
        (// Outputs
         .RD_DATA                       (wRxSrSop),
         // Inputs
         .WR_DATA                       (RX_TLP_START_FLAG & RX_TLP_VALID),
         .RST_IN                        (0),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    // Start Flag Shift Register. Data enables are derived from the
    // taps on this shift register.
    shiftreg 
        #(// Parameters
          .C_DEPTH                      (C_RX_PIPELINE_DEPTH),
          .C_WIDTH                      (1'b1),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    valid_shiftreg_inst
        (// Outputs
         .RD_DATA                       (wRxSrDataValid),
         // Inputs
         .WR_DATA                       (RX_TLP_VALID),
         .RST_IN                        (0),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));


    // End Flag Shift Register. Data valid is deasserted based on the
    // taps in this register
    shiftreg 
        #(// Parameters
          .C_DEPTH                      (C_RX_PIPELINE_DEPTH),
          .C_WIDTH                      (1'b1),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    eop_shiftreg_inst
        (// Outputs
         .RD_DATA                       (wRxSrEop),
         // Inputs
         .WR_DATA                       (RX_TLP_END_FLAG & RX_TLP_VALID),
         .RST_IN                        (0),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    // End Flag Shift Register. Data valid is deasserted based on the
    // taps in this register
    shiftreg 
        #(// Parameters
          .C_DEPTH                      (C_RX_PIPELINE_DEPTH),
          .C_WIDTH                      (`SIG_OFFSET_W),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    eoff_shiftreg_inst
        (// Outputs
         .RD_DATA                       (wRxSrEoff),
         // Inputs
         .WR_DATA                       (RX_TLP_END_OFFSET),
         .RST_IN                        (0),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    // End Flag Shift Register. Data valid is deasserted based on the
    // taps in this register
    shiftreg 
        #(// Parameters
          .C_DEPTH                      (C_RX_PIPELINE_DEPTH),
          .C_WIDTH                      (`SIG_OFFSET_W),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    soff_shiftreg_inst
        (
         // Outputs
         .RD_DATA                       (wRxSrSoff),
         // Inputs
         .WR_DATA                       (RX_TLP_START_OFFSET),
         .RST_IN                        (0),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    generate
        if(C_VENDOR == "XILINX" && C_PCI_DATA_WIDTH == 128) begin
            
            rxr_engine_128
                #(/*AUTOINSTPARAM*/
                  // Parameters
                  .C_PCI_DATA_WIDTH     (C_PCI_DATA_WIDTH),
                  .C_RX_PIPELINE_DEPTH  (C_RX_PIPELINE_DEPTH))
            rxr_engine_inst
                (
                 // Inputs
                 .RX_SR_DATA            (wRxSrData),
                 .RX_SR_EOP             (wRxSrEop),
                 .RX_SR_END_OFFSET      (wRxSrEoff),
                 .RX_SR_SOP             (wRxSrSop),
                 .RX_SR_START_OFFSET    (wRxSrSoff),
                 .RX_SR_VALID           (wRxSrDataValid),
                 // Outputs
                 .RXR_DATA              (_RXR_DATA[C_PCI_DATA_WIDTH-1:0]),
                 /*AUTOINST*/
                 // Outputs
                 .DONE_RXR_RST          (DONE_RXR_RST),
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

            rxc_engine_128
                #(/*AUTOINSTPARAM*/
                  // Parameters
                  .C_PCI_DATA_WIDTH     (C_PCI_DATA_WIDTH),
                  .C_RX_PIPELINE_DEPTH  (C_RX_PIPELINE_DEPTH))
            rxc_engine_inst
                (
                 // Inputs
                 .RX_SR_DATA            (wRxSrData),
                 .RX_SR_EOP             (wRxSrEop),
                 .RX_SR_END_OFFSET      (wRxSrEoff),
                 .RX_SR_SOP             (wRxSrSop),
                 .RX_SR_START_OFFSET    (wRxSrSoff),
                 .RX_SR_VALID           (wRxSrDataValid),
                 // Outputs
                 .RXC_DATA              (_RXC_DATA[C_PCI_DATA_WIDTH-1:0]),
                 /*AUTOINST*/
                 // Outputs
                 .DONE_RXC_RST          (DONE_RXC_RST),
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
        end else begin // if (C_VENDOR != "XILINX" & C_PCI_DATA_WIDTH !=128)
            rxr_engine_classic
                #(
                  // Parameters
                  .C_VENDOR                        (C_VENDOR),
                  .C_PCI_DATA_WIDTH                (C_PCI_DATA_WIDTH),
                  .C_RX_PIPELINE_DEPTH             (C_RX_PIPELINE_DEPTH))
            rxr_engine_inst
                (
                 // Inputs
                 .RX_SR_DATA                       (wRxSrData),
                 .RX_SR_EOP                        (wRxSrEop),
                 .RX_SR_END_OFFSET                 (wRxSrEoff),
                 .RX_SR_SOP                        (wRxSrSop),
                 .RX_SR_VALID                      (wRxSrDataValid),
                 // Outputs
                 .RXR_DATA                      (_RXR_DATA[C_PCI_DATA_WIDTH-1:0]),
                 /*AUTOINST*/
                 // Outputs
                 .DONE_RXR_RST          (DONE_RXR_RST),
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

            rxc_engine_classic
                #(
                  // Parameters
                  .C_VENDOR                        (C_VENDOR),
                  .C_PCI_DATA_WIDTH                (C_PCI_DATA_WIDTH),
                  .C_RX_PIPELINE_DEPTH             (C_RX_PIPELINE_DEPTH))
            rxc_engine_inst
                (
                 // Inputs
                 .RX_SR_DATA                       (wRxSrData),
                 .RX_SR_EOP                        (wRxSrEop),
                 .RX_SR_END_OFFSET                 (wRxSrEoff),
                 .RX_SR_SOP                        (wRxSrSop),
                 .RX_SR_VALID                      (wRxSrDataValid),
                 // Outputs
                 .RXC_DATA                      (_RXC_DATA[C_PCI_DATA_WIDTH-1:0]),
                 /*AUTOINST*/
                 // Outputs
                 .DONE_RXC_RST          (DONE_RXC_RST),
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
        end // else: !if(C_VENDOR != "XILINX" & C_PCI_DATA_WIDTH !=128)
    endgenerate
endmodule
// Local Variables:
// verilog-library-directories:("." "../../../common/")
// End:
