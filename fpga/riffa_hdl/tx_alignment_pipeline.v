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
// Filename: tx_alignment_pipeline
// Version: 1.0
// Verilog Standard: Verilog-2001
//
// Description: The TX alignment pipeline takes a formatted header and data and
// "aligns" them to create a formatted PKT. The aligner is used in both the TXC
// and TXR engines.
// 
// The data interface (TX_DATA) is an interface for N 32-bit FIFOs, where N =
// (C_DATA_WIDTH/32). The START_FLAG signal indicates that the first dword of
// a packet is in FIFO 0 (TX_DATA[31:0]). Each FIFO interface also contains an
// END_FLAG signal in the END_FLAGS bus. When a bit in END_FLAGS bus is asserted,
// its corresponding fifo contains the last dword of data for the current
// packet. START_FLAG, END_FLAG and DATA are all qualified by the VALID signal,
// and read by the READY signal.
// 
// The header interface (TX_HDR) presents the entire header in a single cycle on a
// fifo-like read-interface. The interface also contains the metadata signals
// NOPAYLOAD, ABLANKS, and LEN. NOPAYLOAD indicates that the header is not
// associated with a payload. ABLANKS indicates how many blanks are inserted
// between header and payload for address alignment. LEN indicates the length of
// the header (in DWORDS). The previous two signals determine the multiplexer
// schedule.
// 
// The aligner is built around N alignment muxes that chose between header and
// data. The aligner uses a multiplexer ROM-based schedule to determine the
// outputs of the alignment muxes. This schedule is listed in the schedules.vh
// include file. It initializes the wSchedule ROM. The ROM is indexed by the
// concatenation of ABLANKS, (Header) LEN, and a saturating counter. The
// saturating counter stops when the selection bits for all multiplexers reach a
// steady state. 
// 
// See schedules.vh for more information regarding the wSchedule and wTxMuxInputs
// arrays.
// 
// Plans:
// - At some point in the future, wSchedule and wTxMuxInputs should be set by
// initialization functions to improve extensibility and reusability, but it may
// decrease readability.
// 
// - (with above) Right now the alignment pipeline works for devices that have 3
// or 4 header dwords and insert a maximum of one alignment blank. To extend
// this, wScheduleSelect, and C_MAX_SCHEDULE_LENGTH need to be
// changed. wSchedule select needs to incorporate additional information bits
// (including the minimum header length). and C_MAX_SCHEDULE_LENGTH must be
// calculated using a function (see previous note)
//
// Author: Dustin Richmond (@darichmond)
//----------------------------------------------------------------------------
`timescale 1ns/1ns
`include "trellis.vh" // Defines the user-facing signal widths.
module tx_alignment_pipeline
    #(parameter C_PIPELINE_OUTPUT = 1,
      parameter C_PIPELINE_DATA_INPUT = 1,
      parameter C_PIPELINE_HDR_INPUT = 1,
      parameter C_USE_COMPUTE_REG = 1,
      parameter C_USE_READY_REG = 1,
      parameter C_DATA_WIDTH = 128,
      parameter C_MAX_HDR_WIDTH = 128,
      parameter C_VENDOR = "ALTERA")
    (// Interface: Clocks
     input                                CLK,

     // Interface: Reset
     input                                RST_IN,

     // Interface: TX DATA FIFOS
     input [(C_DATA_WIDTH/32)-1:0]        TX_DATA_WORD_VALID,
     input [C_DATA_WIDTH-1:0]             TX_DATA,
     input                                TX_DATA_START_FLAG,
     input                                TX_DATA_PACKET_VALID,
     input [(C_DATA_WIDTH/32)-1:0]        TX_DATA_END_FLAGS,
     output [(C_DATA_WIDTH/32)-1:0]       TX_DATA_WORD_READY,
    
     // Interface: TX HDR
     input                                TX_HDR_VALID,
     input [C_MAX_HDR_WIDTH-1:0]          TX_HDR,
     input [`SIG_LEN_W-1:0]               TX_HDR_PAYLOAD_LEN,
     input [`SIG_NONPAY_W-1:0]            TX_HDR_NONPAY_LEN,
     input [`SIG_PACKETLEN_W-1:0]         TX_HDR_PACKET_LEN,
     input                                TX_HDR_NOPAYLOAD,
     output                               TX_HDR_READY,

     // TX Interface (Unified)
     input                                TX_PKT_READY,
     output [C_DATA_WIDTH-1:0]            TX_PKT,
     output                               TX_PKT_START_FLAG,
     output [clog2s(C_DATA_WIDTH/32)-1:0] TX_PKT_START_OFFSET,
     output                               TX_PKT_END_FLAG,
     output [clog2s(C_DATA_WIDTH/32)-1:0] TX_PKT_END_OFFSET,
     output                               TX_PKT_VALID);

    localparam C_OFFSET_WIDTH = clog2s(C_DATA_WIDTH/32);
    localparam C_AGGREGATE_WIDTH = (C_DATA_WIDTH+C_MAX_HDR_WIDTH);
    localparam C_MASK_WIDTH = (C_DATA_WIDTH/32);
    localparam C_NUM_MUXES = (C_DATA_WIDTH/32);
    localparam C_MUX_INPUTS = (C_DATA_WIDTH == 32)?5:4;
    localparam C_CLOG_MUX_INPUTS = clog2s(C_MUX_INPUTS);
    localparam C_MAX_SCHEDULE = (C_DATA_WIDTH == 256)? 2 : (C_DATA_WIDTH == 128)? 3: (C_DATA_WIDTH == 64)? 4: (C_DATA_WIDTH == 32)? 6 : 0;
    localparam C_CLOG_MAX_SCHEDULE = clog2s(C_MAX_SCHEDULE);

    genvar                                i;

    // Wires from the data interface input registers
    wire [(C_DATA_WIDTH/32)-1:0]          wTxDataWordValid;
    wire                                  wTxDataPacketValid;
    wire [(C_DATA_WIDTH/32)-1:0]          wTxDataWordReady;
    wire [C_DATA_WIDTH-1:0]               wTxData;
    wire                                  wTxDataStartFlag;
    wire [(C_DATA_WIDTH/32)-1:0]          wTxDataEndFlags;
    wire [(C_DATA_WIDTH/32)-1:0]          wTxDataPacketWordValid;
    wire [clog2s(C_DATA_WIDTH/32)-1:0]    wTxDataEndOffset;

    // Wires from the header interface input register
    wire                                  wTxHdrReady,_wTxHdrReady,__wTxHdrReady;
    wire [C_MAX_HDR_WIDTH -1:0]           wTxHdr,_wTxHdr,__wTxHdr;
    wire                                  wTxHdrValid,_wTxHdrValid,__wTxHdrValid;
    wire                                  wTxHdrNoPayload,_wTxHdrNoPayload,__wTxHdrNoPayload;
    wire [`SIG_LEN_W-1:0]                 wTxHdrPayloadLen,_wTxHdrPayloadLen,__wTxHdrPayloadLen; 
    wire [`SIG_NONPAY_W-1:0]              wTxHdrNonpayLen,_wTxHdrNonpayLen,__wTxHdrNonpayLen; 
    wire [`SIG_PACKETLEN_W-1:0]           wTxHdrPacketLen,_wTxHdrPacketLen,__wTxHdrPacketLen; 

    wire [`SIG_PACKETLEN_W:0]             __wTxHdrPacketLenMinus1; 
    wire [C_MUX_INPUTS-1:0]               __wTxHdrPacketMask;
    wire [C_MUX_INPUTS-1:0]               __wTxHdrLenMask;    
    // wSchedule is the array containing all of the schedules for each mux and ready signal
    // wSchedule is indexed by the concatenation {Insert Blanks, Header Length, Saturating Counter}
    wire [C_CLOG_MUX_INPUTS-1:0]          wSchedule[C_NUM_MUXES-1:0][(1<<(3+C_CLOG_MAX_SCHEDULE))-1:0];
    // Create an array of mux selects, and a bus of ready signals. The ready
    // signals are indicate when a dword is being read from the input fifo and
    // are statically determined in the schedules.vh file
    wire [(3+C_CLOG_MAX_SCHEDULE)-1:0]    wScheduleSelect;

    wire [C_NUM_MUXES-1:0]                __wTxHdrStartEndReady,_wTxHdrStartEndReady;
    wire [C_NUM_MUXES-1:0]                __wTxHdrStartReady,_wTxHdrStartReady;
    wire [C_NUM_MUXES-1:0]                __wTxHdrEndReady,_wTxHdrEndReady;
    wire [C_NUM_MUXES-1:0]                __wTxHdrSteadyStateReady,_wTxHdrSteadyStateReady;

    wire [1:0]                            wReadyMuxSelect;
    wire [C_NUM_MUXES-1:0]                wReadyMux[3:0];

    // Aggreate the header and the current data inputs into an array.
    wire [31:0]                           wAggregate[C_AGGREGATE_WIDTH/32-1:0];
    wire [32*C_MUX_INPUTS-1:0]            wTxMuxInputs[C_NUM_MUXES-1:0];
    wire [(C_CLOG_MUX_INPUTS*C_NUM_MUXES)-1:0] wTxMuxSelect,_wTxMuxSelect;
    wire [C_NUM_MUXES-1:0]                     wTxMuxSelectDataReady,_wTxMuxSelectDataReady;
    wire [C_NUM_MUXES-1:0]                     wTxMuxSelectDataReadyAndPayload,_wTxMuxSelectDataReadyAndPayload;
    wire                                       wTxMuxSelectDataEndFlag,_wTxMuxSelectDataEndFlag;
    wire                                       wTxMuxSelectDataStartFlag,_wTxMuxSelectDataStartFlag;
    wire                                       wTxMuxSelectPktStartFlag,_wTxMuxSelectPktStartFlag;
    wire                                       wTxMuxSelectReady,_wTxMuxSelectReady; 
    wire                                       wTxMuxSelectValid,_wTxMuxSelectValid; 

    // Wires from the output of the muxes to the input of the output register stage
    // wTxPktReady is asserted when a packet is complete
    wire                                       wTxPktReady;
    wire                                       wTxPktValid;
    wire [C_DATA_WIDTH-1:0]                    wTxPkt;
    wire                                       wTxPktStartFlag;
    wire                                       wTxPktEndFlag;
    wire [C_OFFSET_WIDTH:0]                    wTxPktEndOffset; // An additional bit for addition overflow
    
    // Saturating Counter Wires
    wire                                       wSatCtrEnable;
    wire                                       wSatCtrReset;
    wire [C_CLOG_MAX_SCHEDULE-1:0]             wSatCtr;

    // Packet Cycle Counter Wires
    wire                                       wPktCtrEnable;
    wire                                       wPktCtrReset;
    wire [`SIG_PACKETLEN_W-1:0]                wPktCtr;

    wire                                       wCounterReset;
`include "schedules.vh"

    // Assignments for the Input Register Stage
    assign __wTxHdrPacketLenMinus1 = __wTxHdrPacketLen - 1;
    assign __wTxHdrSteadyStateReady = {C_NUM_MUXES{1'b1}};
    assign __wTxHdrStartReady = {C_NUM_MUXES{1'b1}} >> __wTxHdrNonpayLen[C_OFFSET_WIDTH-1:0];
    //assign __wTxHdrEndReady = __wTxHdrPacketMask ROTATE-RIGHT __wTxHdrNonpayLen[C_OFFSET_WIDTH-1:0];
    assign __wTxHdrStartEndReady = __wTxHdrLenMask;
    
    // Assignments for the computation stage
    // Counter logic
    assign wCounterReset = _wTxMuxSelectDataEndFlag & _wTxMuxSelectReady;
    assign wSatCtrReset = RST_IN | wCounterReset;
    assign wSatCtrEnable = _wTxMuxSelectReady & _wTxMuxSelectValid;
    assign wPktCtrReset = RST_IN | wCounterReset;
    assign wPktCtrEnable = _wTxMuxSelectReady & _wTxMuxSelectValid;
    assign wScheduleSelect = {_wTxHdrNonpayLen[2:0],wSatCtr[C_CLOG_MAX_SCHEDULE-1:0]};
    // Ready Mux Logic
    assign wReadyMuxSelect[0] = _wTxMuxSelectDataStartFlag;
    assign wReadyMuxSelect[1] = _wTxMuxSelectDataEndFlag;
    assign wReadyMux[0] = _wTxHdrSteadyStateReady;
    assign wReadyMux[1] = _wTxHdrStartReady;
    assign wReadyMux[2] = _wTxHdrEndReady;
    assign wReadyMux[3] = _wTxHdrStartEndReady;
    assign _wTxMuxSelectValid = _wTxHdrValid;
    assign _wTxMuxSelectDataReady = wReadyMux[wReadyMuxSelect] & {C_NUM_MUXES{(wPktCtr >= _wTxHdrNonpayLen[`SIG_NONPAY_W-1:clog2s(C_NUM_MUXES)])}};
    assign _wTxMuxSelectDataReadyAndPayload = wReadyMux[wReadyMuxSelect] & 
                                              {C_NUM_MUXES{(wPktCtr >= _wTxHdrNonpayLen[`SIG_NONPAY_W-1:clog2s(C_NUM_MUXES)])}} &
                                              {C_NUM_MUXES{~_wTxHdrNoPayload}} & 
                                              {C_NUM_MUXES{_wTxHdrValid}};
    assign _wTxMuxSelectPktStartFlag = wPktCtr == 0;
    assign _wTxMuxSelectDataStartFlag = wPktCtr == _wTxHdrNonpayLen[`SIG_NONPAY_W-1:clog2s(C_NUM_MUXES)];
    assign _wTxMuxSelectDataEndFlag   = ({wPktCtr,{clog2s(C_NUM_MUXES){1'b0}}} + C_NUM_MUXES) >= _wTxHdrPacketLen;// TODO: Simplify

    // Assignments for the ready stage
    assign wTxHdrReady = (wTxMuxSelectDataEndFlag & wTxMuxSelectValid & wTxMuxSelectReady) | ~wTxMuxSelectValid;
    assign wTxMuxSelectReady = (wTxPktReady & wTxHdrNoPayload) | 
                               (wTxPktReady & wTxDataPacketValid) | 
                               (~wTxMuxSelectValid);
    assign wTxPktStartFlag = wTxMuxSelectPktStartFlag;
    assign wTxPktEndFlag = wTxMuxSelectDataEndFlag;
    assign wTxPktEndOffset = wTxHdrPacketLen[C_OFFSET_WIDTH-1:0]-1; // TODO: Retime -1?
    assign wTxPktValid = wTxMuxSelectValid & (wTxHdrNoPayload | (~wTxHdrNoPayload & wTxDataPacketValid));
    // assign wTxDataWordReady = wTxMuxSelectDataReady & {C_NUM_MUXES{wTxPktReady & wTxMuxSelectValid & wTxDataPacketValid}};
    assign wTxDataWordReady = wTxMuxSelectDataReadyAndPayload & {C_NUM_MUXES{wTxPktReady & wTxDataPacketValid}}; // TODO: Change this to bit-wise AND of wTxDataPacketValid

    // Assignments for the output stage
    assign TX_PKT_START_OFFSET = {C_OFFSET_WIDTH{1'b0}};

    assign wTxDataPacketValid = wTxDataPacketWordValid != 0;
    /*See comment block at start of module*/
    generate
        for(i = 0 ; i < C_NUM_MUXES ; i = i + 1) begin : muxes
            assign _wTxMuxSelect[i*C_CLOG_MUX_INPUTS +: C_CLOG_MUX_INPUTS] = wSchedule[i][wScheduleSelect];
        end
    endgenerate

    offset_to_mask
        #(// Parameters
          .C_MASK_SWAP                  (0),
          .C_MASK_WIDTH                 (C_NUM_MUXES)
          /*AUTOINSTPARAM*/)
    packet_mask
        (
         // Outputs
         .MASK                          (__wTxHdrPacketMask),
         // Inputs
         .OFFSET_ENABLE                 (1),
         .OFFSET                        (__wTxHdrPacketLenMinus1[clog2s(C_NUM_MUXES)-1:0])
         /*AUTOINST*/);

    offset_to_mask
        #(// Parameters
          .C_MASK_SWAP                  (0),
          .C_MASK_WIDTH                 (C_NUM_MUXES)
          /*AUTOINSTPARAM*/)
    len_mask
        (// Outputs
         .MASK                          (__wTxHdrLenMask),
         // Inputs
         .OFFSET_ENABLE                 (1),
         .OFFSET                        (__wTxHdrPayloadLen[clog2s(C_NUM_MUXES)-1:0]-1)
         /*AUTOINST*/);

    rotate
        #(// Parameters
          .C_DIRECTION                  ("RIGHT"),
          .C_WIDTH                      (C_NUM_MUXES)
          /*AUTOINSTPARAM*/)
    rot_inst
        (// Outputs
         .RD_DATA                   (__wTxHdrEndReady),
         // Inputs
         .WR_DATA                   (__wTxHdrPacketMask),
         .WR_SHIFTAMT               (__wTxHdrNonpayLen[C_OFFSET_WIDTH-1:0])
         /*AUTOINST*/);

    pipeline
        #(// Parameters
          .C_DEPTH                      (C_PIPELINE_HDR_INPUT?1:0),
          .C_WIDTH                      (C_MAX_HDR_WIDTH + `SIG_NONPAY_W + `SIG_PACKETLEN_W + `SIG_LEN_W + 1),
          .C_USE_MEMORY                 (0)
          /*AUTOINSTPARAM*/)
    hdr_input_reg
        (// Outputs
         .WR_DATA_READY             (TX_HDR_READY),
         .RD_DATA                   ({__wTxHdr,__wTxHdrNonpayLen,__wTxHdrPacketLen,__wTxHdrPayloadLen,__wTxHdrNoPayload}),
         .RD_DATA_VALID             (__wTxHdrValid),
         // Inputs
         .WR_DATA                   ({TX_HDR,TX_HDR_NONPAY_LEN,TX_HDR_PACKET_LEN,TX_HDR_PAYLOAD_LEN,TX_HDR_NOPAYLOAD}),
         .WR_DATA_VALID             (TX_HDR_VALID),
         .RD_DATA_READY             (__wTxHdrReady),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));

    pipeline
        #(// Parameters
          .C_DEPTH                      (C_USE_COMPUTE_REG?1:0),
          .C_WIDTH                      (C_MAX_HDR_WIDTH + `SIG_NONPAY_W + `SIG_PACKETLEN_W + `SIG_LEN_W + 1 + 4*C_MASK_WIDTH),
          .C_USE_MEMORY                 (0)
          /*AUTOINSTPARAM*/)
    compute_reg
        (// Outputs
         .WR_DATA_READY             (__wTxHdrReady),
         .RD_DATA                   ({_wTxHdr,_wTxHdrNonpayLen,_wTxHdrPacketLen,_wTxHdrPayloadLen,_wTxHdrNoPayload,
                                      _wTxHdrSteadyStateReady,_wTxHdrStartReady,_wTxHdrEndReady,_wTxHdrStartEndReady}),
         .RD_DATA_VALID             (_wTxHdrValid),
         // Inputs
         .WR_DATA                   ({__wTxHdr,__wTxHdrNonpayLen,__wTxHdrPacketLen,__wTxHdrPayloadLen,__wTxHdrNoPayload,
                                      __wTxHdrSteadyStateReady,__wTxHdrStartReady,__wTxHdrEndReady,__wTxHdrStartEndReady}),
         .WR_DATA_VALID             (__wTxHdrValid),
         .RD_DATA_READY             (_wTxMuxSelectDataEndFlag & _wTxMuxSelectReady),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));

    pipeline
        #(// Parameters
          .C_DEPTH                      (C_USE_READY_REG?1:0),
          .C_WIDTH                      (C_MAX_HDR_WIDTH + `SIG_NONPAY_W + `SIG_PACKETLEN_W + `SIG_LEN_W + 1),
          .C_USE_MEMORY                 (0)
          /*AUTOINSTPARAM*/)
    ready_reg
        (// Outputs
         .WR_DATA_READY             (_wTxHdrReady),
         .RD_DATA                   ({wTxHdr,wTxHdrNonpayLen,wTxHdrPacketLen,wTxHdrPayloadLen,wTxHdrNoPayload}),
         .RD_DATA_VALID             (wTxHdrValid),
         // Inputs
         .WR_DATA                   ({_wTxHdr,_wTxHdrNonpayLen,_wTxHdrPacketLen,_wTxHdrPayloadLen,_wTxHdrNoPayload}),
         .WR_DATA_VALID             (_wTxHdrValid),
         .RD_DATA_READY             (wTxHdrReady),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));
    
    pipeline
        #(// Parameters
          .C_DEPTH                      (C_USE_READY_REG?1:0),
          .C_WIDTH                      (2*C_NUM_MUXES + C_CLOG_MUX_INPUTS * C_NUM_MUXES + 3),
          .C_USE_MEMORY                 (0)
          /*AUTOINSTPARAM*/)
    select_reg
        (// Outputs
         .WR_DATA_READY             (_wTxMuxSelectReady),
         .RD_DATA                   ({wTxMuxSelectDataReady,wTxMuxSelect,
                                      wTxMuxSelectDataEndFlag,wTxMuxSelectDataStartFlag,
                                      wTxMuxSelectPktStartFlag, 
                                      wTxMuxSelectDataReadyAndPayload}),
         .RD_DATA_VALID             (wTxMuxSelectValid),
         // Inputs
         .WR_DATA                   ({_wTxMuxSelectDataReady,_wTxMuxSelect,
                                      _wTxMuxSelectDataEndFlag,_wTxMuxSelectDataStartFlag,
                                      _wTxMuxSelectPktStartFlag, 
                                      _wTxMuxSelectDataReadyAndPayload}),
         .WR_DATA_VALID             (_wTxMuxSelectValid),
         .RD_DATA_READY             (wTxMuxSelectReady),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));

    counter
        #(// Parameters
          .C_MAX_VALUE                  (C_MAX_SCHEDULE-1),
          .C_SAT_VALUE                  (C_MAX_SCHEDULE-1),
          .C_RST_VALUE                  (0)
          /*AUTOINSTPARAM*/)
    satctr_inst
        (// Outputs
         .VALUE                         (wSatCtr),
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (wSatCtrReset),
         .ENABLE                        (wSatCtrEnable)
         /*AUTOINST*/);

    counter
        #(// Parameters
          .C_MAX_VALUE                  (1<<`SIG_PACKETLEN_W),
          .C_SAT_VALUE                  (1<<`SIG_PACKETLEN_W + 1), // Never saturate
          .C_RST_VALUE                  (0)
          /*AUTOINSTPARAM*/)
    pktctr_inst
        (// Outputs
         .VALUE                         (wPktCtr),
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (wPktCtrReset),
         .ENABLE                        (wPktCtrEnable)
         /*AUTOINST*/);

    generate
        for( i = 0  ; i < C_MAX_HDR_WIDTH/32 ; i = i + 1) begin : gen_aggregate
            assign wAggregate[i] = wTxHdr[i*32 +: 32];
        end

//            pipeline
//                #(// Parameters
//                  .C_DEPTH                      (C_PIPELINE_DATA_INPUT?1:0),
//                  .C_WIDTH                      (1),
//                  .C_USE_MEMORY                 (0)
//                  /*AUTOINSTPARAM*/)
//            packet_valid_register
//                (// Outputs
//                 .WR_DATA_READY             (),
//                 .RD_DATA                   (),
//                 .RD_DATA_VALID             (wTxDataPacketValid),
//                 // Inputs
//                 .WR_DATA                   (),
//                 .WR_DATA_VALID             (TX_DATA_PACKET_VALID | ((wTxDataWordValid & wTxDataEndFlags[i])),
//                 .RD_DATA_READY             (~wTxDataPacketValid | 
//                                             ((wTxDataEndFlags & wTxDataWordReady) != 0)),
//                 // TODO: End flag read? This is odd, you want to read when there is not a valid packet 
//                 /*AUTOINST*/
//                 // Inputs
//                 .CLK                   (CLK),
//                 .RST_IN                (RST_IN));
        for( i = 0; i < C_NUM_MUXES ; i = i + 1) begin : gen_data_input_regs
            assign wAggregate[i + C_MAX_HDR_WIDTH/32] = wTxData[32*i +: 32];
            pipeline
                #(// Parameters
                  .C_DEPTH                      (C_PIPELINE_DATA_INPUT?1:0),
                  .C_WIDTH                      (32),
                  .C_USE_MEMORY                 (0)
                  /*AUTOINSTPARAM*/)
            data_register_
                (// Outputs
                 .WR_DATA_READY             (TX_DATA_WORD_READY[i]),
                 .RD_DATA                   (wTxData[32*i +: 32]),
                 .RD_DATA_VALID             (wTxDataWordValid[i]),
                 // Inputs
                 .WR_DATA                   (TX_DATA[32*i +: 32]),
                 .WR_DATA_VALID             (TX_DATA_WORD_VALID[i]),
                 .RD_DATA_READY             (wTxDataWordReady[i]),
                 /*AUTOINST*/
                 // Inputs
                 .CLK                   (CLK),
                 .RST_IN                (RST_IN));
            pipeline
                #(// Parameters
                  .C_DEPTH                      (C_PIPELINE_DATA_INPUT?1:0),
                  .C_WIDTH                      (1),
                  .C_USE_MEMORY                 (0)
                  /*AUTOINSTPARAM*/)
            packet_valid_register
                (// Outputs
                 .WR_DATA_READY             (),
                 .RD_DATA                   (),
                 .RD_DATA_VALID             (wTxDataPacketWordValid[i]),
                 // Inputs
                 .WR_DATA                   (),
                 .WR_DATA_VALID             ((TX_DATA_END_FLAGS[i] & TX_DATA_WORD_VALID[i]) | 
                                             (TX_DATA_PACKET_VALID & TX_DATA_WORD_VALID[i] & (TX_DATA_END_FLAGS == 0))),
                 .RD_DATA_READY             (wTxDataWordReady[i] | ~wTxDataPacketWordValid[i]),
                 // TODO: End flag read? This is odd, you want to read when there is not a valid packet 
                 /*AUTOINST*/
                 // Inputs
                 .CLK                   (CLK),
                 .RST_IN                (RST_IN));
        end

        for( i = 0 ; i < C_NUM_MUXES ; i = i + 1) begin : gen_packet_format_multiplexers
            mux
                 #(
                   // Parameters
                   .C_NUM_INPUTS                 (C_MUX_INPUTS),
                   .C_CLOG_NUM_INPUTS            (C_CLOG_MUX_INPUTS),
                   .C_WIDTH                      (32),
                   .C_MUX_TYPE                   ("SELECT")
                   /*AUTOINSTPARAM*/)
            dw_mux_
                 (
                  // Outputs
                  .MUX_OUTPUT                (wTxPkt[32*i +: 32]),
                  // Inputs
                  .MUX_INPUTS                (wTxMuxInputs[i]),
                  .MUX_SELECT                (wTxMuxSelect[i*C_CLOG_MUX_INPUTS +: C_CLOG_MUX_INPUTS])
                  /*AUTOINST*/);
        end
    endgenerate

    pipeline
        #(
          // Parameters
          .C_DEPTH                      (C_PIPELINE_OUTPUT?1:0),
          .C_WIDTH                      (C_DATA_WIDTH + 2 + C_OFFSET_WIDTH),
          .C_USE_MEMORY                 (0)
          /*AUTOINSTPARAM*/)
    output_register_inst
        (
         // Outputs
         .WR_DATA_READY             (wTxPktReady),
         .RD_DATA                   ({TX_PKT,TX_PKT_START_FLAG,TX_PKT_END_FLAG,TX_PKT_END_OFFSET}),
         .RD_DATA_VALID             (TX_PKT_VALID),
         // Inputs
         .WR_DATA                   ({wTxPkt,wTxPktStartFlag,wTxPktEndFlag,wTxPktEndOffset[C_OFFSET_WIDTH-1:0]}),
         .WR_DATA_VALID             (wTxPktValid),
         .RD_DATA_READY             (TX_PKT_READY),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));

endmodule
// Local Variables:
// verilog-library-directories:("." "../../common/")
// End:
