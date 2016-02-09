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
// Filename:            rxc_engine_classic.v
// Version:             1.0
// Verilog Standard:    Verilog-2001
// Description:         The RXC Engine (Ultrascale) takes a single stream of 
// AXI packets and provides the completion packets on the RXC Interface.
// This Engine is capable of operating at "line rate".
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
`include "trellis.vh"
`include "ultrascale.vh"
module rxc_engine_ultrascale
    #(parameter C_PCI_DATA_WIDTH = 128,
      parameter C_RX_PIPELINE_DEPTH=10,
      // Number of data pipeline registers for metadata and data stages
      parameter C_RX_META_STAGES = 0,
      parameter C_RX_DATA_STAGES = 1)
    (// Interface: Clocks
     input                                    CLK,

     // Interface: Resets
     input                                    RST_BUS, // Replacement for generic RST_IN
     input                                    RST_LOGIC, // Addition for RIFFA_RST
     output                                   DONE_RXC_RST,

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
     output                                   RXC_META_EP
     );

    // Width of the Byte Enable Shift register
    localparam C_RX_BE_W = (`SIG_FBE_W + `SIG_LBE_W);

    localparam C_RX_INPUT_STAGES = 0;
    localparam C_RX_OUTPUT_STAGES = 2; // Should always be at least one
    localparam C_RX_COMPUTATION_STAGES = 1;
    localparam C_TOTAL_STAGES = C_RX_COMPUTATION_STAGES + C_RX_OUTPUT_STAGES + C_RX_INPUT_STAGES;

    // CYCLE = LOW ORDER BIT (INDEX) / C_PCI_DATA_WIDTH
    localparam C_RX_METADW0_CYCLE = (`UPKT_RXC_METADW0_I/C_PCI_DATA_WIDTH) + C_RX_INPUT_STAGES;
    localparam C_RX_METADW1_CYCLE = (`UPKT_RXC_METADW1_I/C_PCI_DATA_WIDTH) + C_RX_INPUT_STAGES;
    localparam C_RX_METADW2_CYCLE = (`UPKT_RXC_METADW2_I/C_PCI_DATA_WIDTH) + C_RX_INPUT_STAGES;
    localparam C_RX_PAYLOAD_CYCLE = (`UPKT_RXC_PAYLOAD_I/C_PCI_DATA_WIDTH) + C_RX_INPUT_STAGES;
    localparam C_RX_BE_CYCLE = C_RX_INPUT_STAGES; // Available on the first cycle (as per the spec)
    
    localparam C_RX_METADW0_INDEX = C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES + (`UPKT_RXC_METADW0_I%C_PCI_DATA_WIDTH);
    localparam C_RX_METADW1_INDEX = C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES + (`UPKT_RXC_METADW1_I%C_PCI_DATA_WIDTH);
    localparam C_RX_METADW2_INDEX = C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES + (`UPKT_RXC_METADW2_I%C_PCI_DATA_WIDTH);
    localparam C_RX_BE_INDEX = C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES;

    // Mask width of the calculated SOF/EOF fields
    localparam C_OFFSET_WIDTH = clog2(C_PCI_DATA_WIDTH/32);

    wire                                      wMAxisRcSop;
    wire                                      wMAxisRcTlast;
    wire [C_RX_PIPELINE_DEPTH:0]              wRxSrSop;
    wire [C_RX_PIPELINE_DEPTH:0]              wRxSrEop;
    wire [C_RX_PIPELINE_DEPTH:0]              wRxSrDataValid;
    wire [(C_RX_PIPELINE_DEPTH+1)*C_RX_BE_W-1:0] wRxSrBe;
    wire [(C_RX_PIPELINE_DEPTH+1)*C_PCI_DATA_WIDTH-1:0] wRxSrData;

    wire                                                wRxcDataValid;
    wire                                                wRxcDataReady; // Pinned High
    wire [(C_PCI_DATA_WIDTH/32)-1:0]                    wRxcDataWordEnable;
    wire                                                wRxcDataEndFlag;
    wire [clog2(C_PCI_DATA_WIDTH/32)-1:0]               wRxcDataEndOffset;
    wire                                                wRxcDataStartFlag;
    wire [clog2(C_PCI_DATA_WIDTH/32)-1:0]               wRxcDataStartOffset;
    wire [`SIG_BYTECNT_W-1:0]                           wRxcMetaBytesRemaining;
    wire [`SIG_CPLID_W-1:0]                             wRxcMetaCompleterId;
    wire [`UPKT_RXC_MAXHDR_W-1:0]                       wRxcHdr;
    wire [`SIG_TYPE_W-1:0]                              wRxcType;
    wire [`SIG_BARDECODE_W-1:0]                         wRxcBarDecoded;
    
    wire [`UPKT_RXC_MAXHDR_W-1:0]                       wHdr;
    wire [`SIG_TYPE_W-1:0]                              wType;
    
    wire                                                wHasPayload;
    wire                                                _wEndFlag;
    wire                                                wEndFlag;
    wire                                                wEndFlagLastCycle;
    wire [clog2(C_PCI_DATA_WIDTH/32)-1:0]               wEndOffset;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]                    wEndMask;
    wire                                                _wStartFlag;
    wire                                                wStartFlag;
    wire [1:0]                                          wStartFlags;
    wire [clog2(C_PCI_DATA_WIDTH/32)-1:0]               wStartOffset;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]                    wStartMask;
    wire [C_OFFSET_WIDTH-1:0]                           wOffsetMask;

    reg                                                 rValid,_rValid;
    reg                                                 rRST;
        
    assign DONE_RXC_RST = ~rRST;

    assign wMAxisRcSop = M_AXIS_RC_TUSER[`UPKT_RC_TUSER_SOP_I];
    assign wMAxisRcTlast = M_AXIS_RC_TLAST;

    // We assert the end flag on the last cycle of a packet, however on single
    // cycle packets we need to check that there wasn't an end flag last cycle
    // (because wStartFlag will take priority when setting rValid) so we can
    // deassert rValid if necessary.
    assign wEndFlag = wRxSrEop[C_RX_INPUT_STAGES + C_RX_COMPUTATION_STAGES];
    assign wEndFlagLastCycle = wRxSrEop[C_RX_INPUT_STAGES + C_RX_COMPUTATION_STAGES + 1];

    /* verilator lint_off WIDTH */
    assign wStartOffset = 3;
    assign wEndOffset = wHdr[`UPKT_RXC_LENGTH_I +: C_OFFSET_WIDTH] + ((`UPKT_RXC_MAXHDR_W-32)/32);
    /* verilator lint_on WIDTH */

    // Output assignments. See the header file derived from the user
    // guide for indices.
    assign RXC_META_LENGTH = wRxcHdr[`UPKT_RXC_LENGTH_I+:`SIG_LEN_W];
    //assign RXC_META_ATTR = wRxcHdr[`UPKT_RXC_ATTR_R];
    //assign RXC_META_TC = wRxcHdr[`UPKT_RXC_TC_R];
    assign RXC_META_TAG = wRxcHdr[`UPKT_RXC_TAG_R];
    assign RXC_META_FDWBE = 0;// TODO: Remove (use addr)
    assign RXC_META_LDWBE = 0;// TODO: Remove (use addr)
    assign RXC_META_ADDR = wRxcHdr[(`UPKT_RXC_ADDRLOW_I) +: `SIG_LOWADDR_W];
    assign RXC_DATA_START_FLAG = wRxcDataStartFlag;                      
    assign RXC_DATA_START_OFFSET = {C_PCI_DATA_WIDTH > 64, 1'b1};
    assign RXC_DATA_END_FLAG = wRxcDataEndFlag;                      
    assign RXC_DATA_END_OFFSET = wRxcDataEndOffset;
    assign RXC_DATA_VALID = wRxcDataValid;
    assign RXC_DATA = wRxSrData[(C_TOTAL_STAGES)*C_PCI_DATA_WIDTH +: C_PCI_DATA_WIDTH];
    assign RXC_META_TYPE = wRxcType;
    assign RXC_META_BYTES_REMAINING = wRxcHdr[`UPKT_RXC_BYTECNT_I +: `SIG_BYTECNT_W];
    assign RXC_META_COMPLETER_ID = wRxcHdr[`UPKT_RXC_CPLID_R];
    assign RXC_META_EP = wRxcHdr[`UPKT_RXC_EP_R];
    
    assign M_AXIS_RC_TREADY = 1'b1;

    assign _wEndFlag = wRxSrEop[C_RX_INPUT_STAGES];
    assign wEndFlag = wRxSrEop[C_RX_INPUT_STAGES+1];
    assign _wStartFlag = wStartFlags != 0;
    assign wType = (wHasPayload)? `TRLS_CPL_WD: `TRLS_CPL_ND;
    
    generate
        if(C_PCI_DATA_WIDTH == 64) begin
            assign wStartFlags[0] = 0;
            assign wStartFlags[1] = wRxSrSop[C_RX_INPUT_STAGES + 1];
            //assign wStartFlags[0] = wRxSrSop[C_RX_INPUT_STAGES + 1] & wRxSrEop[C_RX_INPUT_STAGES]; // No Payload
        end else if (C_PCI_DATA_WIDTH == 128) begin    
            assign wStartFlags[1] = 0;
            assign wStartFlags[0] = wRxSrSop[C_RX_INPUT_STAGES];
        end else begin // 256
            assign wStartFlags[1] = 0;
            assign wStartFlags[0] = wRxSrSop[C_RX_INPUT_STAGES];
        end // else: !if(C_PCI_DATA_WIDTH == 128)
    endgenerate

    always @(*) begin
        _rValid = rValid;
        if(_wStartFlag) begin
	        _rValid = 1'b1;
        end else if (wEndFlag) begin
	        _rValid = 1'b0;
        end
    end
    
    always @(posedge CLK) begin
        if(rRST) begin
	        rValid <= 1'b0;
        end else begin
	        rValid <= _rValid;
        end
    end

    always @(posedge CLK) begin
        rRST <= RST_BUS | RST_LOGIC;
    end

    register
        #(// Parameters
          .C_WIDTH                      (1),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    start_flag_register
        (// Outputs
         .RD_DATA                       (wStartFlag),
         // Inputs
         .WR_DATA                       (_wStartFlag),
         .WR_EN                         (1),
         .RST_IN                        (0),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    register
        #(// Parameters
          .C_WIDTH                      (32),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    meta_DW2_register
        (// Outputs
         .RD_DATA                       (wHdr[95:64]),
         // Inputs
         .WR_DATA                       (wRxSrData[C_RX_METADW2_INDEX +: 32]),
         .WR_EN                         (wRxSrSop[C_RX_METADW2_CYCLE]),
         .RST_IN                        (0),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    register
        #(// Parameters
          .C_WIDTH                      (32 + 1),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    meta_DW1_register
        (// Outputs
         .RD_DATA                       ({wHdr[63:32],wHasPayload}),
         // Inputs
         .WR_DATA                       ({wRxSrData[C_RX_METADW1_INDEX +: 32],
                                          wRxSrData[C_RX_METADW1_INDEX +: `UPKT_LEN_W] != 0}),
         .WR_EN                         (wRxSrSop[C_RX_METADW1_CYCLE]),
         .RST_IN                        (0),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    register
        #(// Parameters
          .C_WIDTH                      (32),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    metadata_DW0_register
        (// Outputs
         .RD_DATA                       (wHdr[31:0]),
         // Inputs
         .WR_DATA                       (wRxSrData[C_RX_METADW0_INDEX +: 32]),
         .WR_EN                         (wRxSrSop[C_RX_METADW0_CYCLE]),
         .RST_IN                        (0),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));


    // Shift register for input data with output taps for each delayed
    // cycle. 
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
         .WR_DATA                       (M_AXIS_RC_TDATA),
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
         .WR_DATA                       (wMAxisRcSop & M_AXIS_RC_TVALID),
         .RST_IN                        (0),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    // End Flag Shift Register. 
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
         .WR_DATA                       (wMAxisRcTlast),
         .RST_IN                        (0),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    // Data Valid Shift Register. Data enables are derived from the
    // taps on this shift register.
    shiftreg 
        #(// Parameters
          .C_DEPTH                      (C_RX_PIPELINE_DEPTH),
          .C_WIDTH                      (1),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    valid_shiftreg_inst
        (// Outputs
         .RD_DATA                       (wRxSrDataValid),
         // Inputs
         .WR_DATA                       (M_AXIS_RC_TVALID),
         .RST_IN                        (rRST),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    assign wStartMask = {C_PCI_DATA_WIDTH/32{1'b1}} << ({C_OFFSET_WIDTH{wStartFlag}}& wStartOffset[C_OFFSET_WIDTH-1:0]);

    offset_to_mask
        #(// Parameters
          .C_MASK_SWAP                  (0),
          .C_MASK_WIDTH                 (C_PCI_DATA_WIDTH/32)
          /*AUTOINSTPARAM*/)
    o2m_ef
        (// Outputs
         .MASK                          (wEndMask),
         // Inputs
         .OFFSET_ENABLE                 (wEndFlag),
         .OFFSET                        (wEndOffset)
         /*AUTOINST*/);

    generate
        if(C_RX_OUTPUT_STAGES == 0) begin
            assign RXC_DATA_WORD_ENABLE = {wEndMask & wStartMask} & {C_PCI_DATA_WIDTH/32{~rValid | ~wHasPayload}};
        end else begin
            register
                #(// Parameters
                  .C_WIDTH              (C_PCI_DATA_WIDTH/32),
                  .C_VALUE              (0)
                  /*AUTOINSTPARAM*/)
            dw_enable
                (// Outputs
                 .RD_DATA               (wRxcDataWordEnable),
                 // Inputs
                 .RST_IN                (~rValid | ~wHasPayload),
                 .WR_DATA               (wEndMask & wStartMask),
                 .WR_EN                 (1),
                 /*AUTOINST*/
                 // Inputs
                 .CLK                   (CLK));

            pipeline
                #(
                  // Parameters
                  .C_DEPTH              (C_RX_OUTPUT_STAGES-1),
                  .C_WIDTH              (C_PCI_DATA_WIDTH/32),
                  .C_USE_MEMORY         (0)
                  /*AUTOINSTPARAM*/)
            dw_pipeline
                (// Outputs
                 .WR_DATA_READY         (), // Pinned to 1
                 .RD_DATA               (RXC_DATA_WORD_ENABLE),
                 .RD_DATA_VALID         (),
                 // Inputs
                 .WR_DATA               (wRxcDataWordEnable),
                 .WR_DATA_VALID         (1),
                 .RD_DATA_READY         (1'b1),
                 .RST_IN                (0),
                 /*AUTOINST*/
                 // Inputs
                 .CLK                   (CLK));
        end
    endgenerate

    // Shift register for input data with output taps for each delayed
    // cycle. 
    pipeline
        #(
          // Parameters
          .C_DEPTH                      (C_RX_OUTPUT_STAGES),
          .C_WIDTH                      (`UPKT_RXC_MAXHDR_W + 
                                         2*(1 + clog2(C_PCI_DATA_WIDTH/32))+`SIG_TYPE_W),
          .C_USE_MEMORY                 (0)
          /*AUTOINSTPARAM*/)
    output_pipeline
        (
         // Outputs
         .WR_DATA_READY                 (), // Pinned to 1
         .RD_DATA                       ({wRxcHdr, wRxcDataStartFlag,
                                          wRxcDataStartOffset,wRxcDataEndFlag, 
                                          wRxcDataEndOffset,wRxcType}),
         .RD_DATA_VALID                 (wRxcDataValid),
         // Inputs
         .WR_DATA                       ({wHdr,wStartFlag,
                                          wStartOffset[C_OFFSET_WIDTH-1:0],
                                          wEndFlag,wEndOffset[C_OFFSET_WIDTH-1:0],wType}),
         .WR_DATA_VALID                 (rValid),
         .RD_DATA_READY                 (1'b1),
         .RST_IN                        (rRST),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));
endmodule
// Local Variables:
// verilog-library-directories:("." "../../../common/")
// End:
