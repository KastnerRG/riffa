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
// Filename:            rxr_engine_classic.v
// Version:             1.0
// Verilog Standard:    Verilog-2001
// Description:         The RXR Engine (Ultrascale) takes a single stream of 
// AXI packets and provides the completion packets on the RXR Interface.
// This Engine is capable of operating at "line rate".
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
`include "trellis.vh"
`include "ultrascale.vh"
module rxr_engine_ultrascale
    #(parameter C_PCI_DATA_WIDTH = 128,
      parameter C_RX_PIPELINE_DEPTH=10)
    (// Interface: Clocks
     input                                    CLK,

     // Interface: Resets
     input                                    RST_BUS, // Replacement for generic RST_IN
     input                                    RST_LOGIC, // Addition for RIFFA_RST
     output                                   DONE_RXR_RST,

     // Interface: CQ
     input                                    M_AXIS_CQ_TVALID,
     input                                    M_AXIS_CQ_TLAST,
     input [C_PCI_DATA_WIDTH-1:0]             M_AXIS_CQ_TDATA,
     input [(C_PCI_DATA_WIDTH/32)-1:0]        M_AXIS_CQ_TKEEP,
     input [`SIG_CQ_TUSER_W-1:0]              M_AXIS_CQ_TUSER,
     output                                   M_AXIS_CQ_TREADY,

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

    // Width of the Byte Enable Shift register
    localparam C_RX_BE_W = (`SIG_FBE_W + `SIG_LBE_W);

    localparam C_RX_INPUT_STAGES = 0;
    localparam C_RX_OUTPUT_STAGES = 2; // Should always be at least one
    localparam C_RX_COMPUTATION_STAGES = 1;
    localparam C_TOTAL_STAGES = C_RX_COMPUTATION_STAGES + C_RX_OUTPUT_STAGES + C_RX_INPUT_STAGES;

    // CYCLE = LOW ORDER BIT (INDEX) / C_PCI_DATA_WIDTH
    localparam C_RX_ADDRDW0_CYCLE = (`UPKT_RXR_ADDRDW0_I/C_PCI_DATA_WIDTH) + C_RX_INPUT_STAGES;
    localparam C_RX_ADDRDW1_CYCLE = (`UPKT_RXR_ADDRDW1_I/C_PCI_DATA_WIDTH) + C_RX_INPUT_STAGES;
    localparam C_RX_METADW0_CYCLE = (`UPKT_RXR_METADW0_I/C_PCI_DATA_WIDTH) + C_RX_INPUT_STAGES;
    localparam C_RX_METADW1_CYCLE = (`UPKT_RXR_METADW1_I/C_PCI_DATA_WIDTH) + C_RX_INPUT_STAGES;
    localparam C_RX_PAYLOAD_CYCLE = (`UPKT_RXR_PAYLOAD_I/C_PCI_DATA_WIDTH) + C_RX_INPUT_STAGES;
    localparam C_RX_BE_CYCLE = C_RX_INPUT_STAGES; // Available on the first cycle (as per the spec)
    
    localparam C_RX_ADDRDW0_INDEX = C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES + (`UPKT_RXR_ADDRDW0_I%C_PCI_DATA_WIDTH);
    localparam C_RX_ADDRDW1_INDEX = C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES + (`UPKT_RXR_ADDRDW1_I%C_PCI_DATA_WIDTH);
    localparam C_RX_METADW0_INDEX = C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES + (`UPKT_RXR_METADW0_I%C_PCI_DATA_WIDTH);
    localparam C_RX_METADW1_INDEX = C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES + (`UPKT_RXR_METADW1_I%C_PCI_DATA_WIDTH);
    localparam C_RX_BE_INDEX = C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES;

    // Mask width of the calculated SOF/EOF fields
    localparam C_OFFSET_WIDTH = clog2s(C_PCI_DATA_WIDTH/32);

    wire                                      wMAxisCqSop;
    wire                                      wMAxisCqTlast;
    wire [C_RX_PIPELINE_DEPTH:0]              wRxSrSop;
    wire [C_RX_PIPELINE_DEPTH:0]              wRxSrEop;
    wire [C_RX_PIPELINE_DEPTH:0]              wRxSrDataValid;
    wire [(C_RX_PIPELINE_DEPTH+1)*C_RX_BE_W-1:0] wRxSrBe;
    wire [(C_RX_PIPELINE_DEPTH+1)*C_PCI_DATA_WIDTH-1:0] wRxSrData;

    wire                                                wRxrDataValid;
    wire                                                wRxrDataReady; // Pinned High
    wire                                                wRxrDataEndFlag;
    wire [C_OFFSET_WIDTH-1:0]                           wRxrDataEndOffset;
    wire                                                wRxrDataStartFlag;
    wire [C_OFFSET_WIDTH-1:0]                           wRxrDataStartOffset;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]                    wRxrDataWordEnable;
    wire [127:0]                                        wRxrHdr;
    wire [`SIG_TYPE_W-1:0]                              wRxrType;
    wire [`SIG_FBE_W-1:0]                               wRxrMetaFdwbe;
    wire [`SIG_LBE_W-1:0]                               wRxrMetaLdwbe;
    wire [C_RX_BE_W-1:0]                                wRxrBe;
    wire [`SIG_BARDECODE_W-1:0]                         wRxrBarDecoded;
    
    wire [127:0]                                        wHdr;
    wire                                                wEndFlag;
    wire                                                wEndFlagLastCycle;
    wire                                                _wEndFlag;    
    wire [C_OFFSET_WIDTH-1:0]                           wEndOffset;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]                    wEndMask;
    wire                                                _wStartFlag;
    wire                                                wStartFlag;
    wire [1:0]                                          wStartFlags;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]                    wStartMask;
    wire [C_OFFSET_WIDTH-1:0]                           wStartOffset;
    wire [C_RX_BE_W-1:0]                                wByteEnables;                    
    wire [`SIG_BARDECODE_W-1:0]                         wBarDecoded;
    wire                                                wHasPayload;
    
    wire [`SIG_TYPE_W-1:0]                              wType;
    reg                                                 rValid,_rValid;
    reg                                                 rRST;

    assign DONE_RXR_RST = ~rRST;

    assign wMAxisCqSop = M_AXIS_CQ_TUSER[`UPKT_CQ_TUSER_SOP_R];
    assign wMAxisCqTlast = M_AXIS_CQ_TLAST;

    assign wBarDecoded = (8'b0000_0001 << wHdr[`UPKT_RXR_BARID_R]);
    

    // We assert the end flag on the last cycle of a packet, however on single
    // cycle packets we need to check that there wasn't an end flag last cycle
    // (because wStartFlag will take priority when setting rValid) so we can
    // deassert rValid if necessary.
    assign wEndFlag = wRxSrEop[C_RX_INPUT_STAGES + C_RX_COMPUTATION_STAGES];
    assign wEndFlagLastCycle = wRxSrEop[C_RX_INPUT_STAGES + C_RX_COMPUTATION_STAGES + 1];

    /* verilator lint_off WIDTH */
    assign wStartOffset = 4;
    assign wEndOffset = wHdr[`UPKT_RXR_LENGTH_I +: C_OFFSET_WIDTH] + ((`UPKT_RXR_MAXHDR_W-32)/32);
    /* verilator lint_on WIDTH */

    // Output assignments. See the header file derived from the user
    // guide for indices.
    assign RXR_META_EP = wRxrHdr[`UPKT_RXR_EP_R];
    assign RXR_META_LENGTH = wRxrHdr[`UPKT_RXR_LENGTH_I+:`SIG_LEN_W];// The top three bits are ignored (fine)
    assign RXR_META_ATTR = wRxrHdr[`UPKT_RXR_ATTR_R];
    assign RXR_META_TC = wRxrHdr[`UPKT_RXR_TC_R];
    assign RXR_META_TYPE = wRxrType;
    assign RXR_META_REQUESTER_ID = wRxrHdr[`UPKT_RXR_REQID_R];
    assign RXR_META_TAG = wRxrHdr[`UPKT_RXR_TAG_R];
    assign RXR_META_FDWBE = wRxrMetaFdwbe;
    assign RXR_META_LDWBE = wRxrMetaLdwbe;
    assign RXR_META_ADDR = {wRxrHdr[`UPKT_RXR_ADDR_R],2'b0};
    assign RXR_DATA_START_FLAG = wRxrDataStartFlag;
    assign RXR_DATA_START_OFFSET = 0;
    assign RXR_DATA_END_FLAG = wRxrDataEndFlag;
    assign RXR_DATA_END_OFFSET = wEndOffset;
    assign RXR_META_BAR_DECODED = wRxrBarDecoded;
    assign RXR_DATA_VALID = wRxrDataValid;
    assign RXR_DATA = wRxSrData[(C_TOTAL_STAGES)*C_PCI_DATA_WIDTH +: C_PCI_DATA_WIDTH];

    assign M_AXIS_CQ_TREADY = 1'b1;

    assign wType = upkt_to_trellis_type({wHdr[`UPKT_RXR_TYPE_R], wHdr[`UPKT_RXR_LENGTH_R] != 0});
    
    assign _wEndFlag = wRxSrEop[C_RX_INPUT_STAGES];
    assign wEndFlag = wRxSrEop[C_RX_INPUT_STAGES+1];
    assign _wStartFlag = wStartFlags != 0;
    assign wHasPayload = ~wType[`TRLS_TYPE_PAY_I];
    assign wStartMask = {C_PCI_DATA_WIDTH/32{1'b1}} << ({C_OFFSET_WIDTH{wStartFlag}}& wStartOffset[C_OFFSET_WIDTH-1:0]);

    generate
        if(C_PCI_DATA_WIDTH == 64) begin
            assign wStartFlags[1] = wRxSrSop[C_RX_INPUT_STAGES + 2] & ~rValid;
            assign wStartFlags[0] = wRxSrSop[C_RX_INPUT_STAGES + 1] & wRxSrEop[C_RX_INPUT_STAGES]; // No Payload
        end else if (C_PCI_DATA_WIDTH == 128) begin    
            assign wStartFlags[1] = wRxSrSop[C_RX_INPUT_STAGES + 1] & ~rValid;
            assign wStartFlags[0] = wRxSrSop[C_RX_INPUT_STAGES] & wRxSrEop[C_RX_INPUT_STAGES]; // No Payload
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
          .C_WIDTH                      (32),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    meta_DW1_register
        (// Outputs
         .RD_DATA                       (wHdr[127:96]),
         // Inputs
         .RST_IN                        (0),
         .WR_DATA                       (wRxSrData[C_RX_METADW1_INDEX +: 32]),
         .WR_EN                         (wRxSrSop[C_RX_METADW1_CYCLE]),
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
         .RD_DATA                       (wHdr[95:64]),
         // Inputs
         .RST_IN                        (0),
         .WR_DATA                       (wRxSrData[C_RX_METADW0_INDEX +: 32]),
         .WR_EN                         (wRxSrSop[C_RX_METADW0_CYCLE]),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    register
        #(// Parameters
          .C_WIDTH                      (32),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    addr_DW1_register
        (// Outputs
         .RD_DATA                       (wHdr[63:32]),
         // Inputs
         .RST_IN                        (0),
         .WR_DATA                       (wRxSrData[C_RX_ADDRDW1_INDEX +: 32]),
         .WR_EN                         (wRxSrSop[C_RX_ADDRDW1_CYCLE]),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    register
        #(// Parameters
          .C_WIDTH                      (32),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    addr_DW0_register
        (// Outputs
         .RD_DATA                       (wHdr[31:0]),
         // Inputs
         .RST_IN                        (0),
         .WR_DATA                       (wRxSrData[C_RX_ADDRDW0_INDEX +: 32]),
         .WR_EN                         (wRxSrSop[C_RX_ADDRDW0_CYCLE]),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    register
        #(// Parameters
          .C_WIDTH                      (C_RX_BE_W),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    be_register
        (// Outputs
         .RD_DATA                       (wByteEnables),
         // Inputs
         .RST_IN                        (0),
         .WR_DATA                       (wRxSrBe[C_RX_BE_INDEX +: C_RX_BE_W]),
         .WR_EN                         (wRxSrSop[C_RX_BE_CYCLE]),
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
         .WR_DATA                       (M_AXIS_CQ_TDATA),
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
         .WR_DATA                       (wMAxisCqSop & M_AXIS_CQ_TVALID),
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
         .WR_DATA                       (wMAxisCqTlast),
         .RST_IN                        (0),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    // Data Valid Shift Register. Data enables are derived from the
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
         .WR_DATA                       (M_AXIS_CQ_TVALID),
         .RST_IN                        (rRst),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));


    // Shift register for input data with output taps for each delayed
    // cycle. 
    shiftreg
        #(// Parameters
          .C_DEPTH                      (C_RX_PIPELINE_DEPTH),
          .C_WIDTH                      (C_RX_BE_W),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    be_shiftreg_inst
        (// Outputs
         .RD_DATA                       (wRxSrBe),
         // Inputs
         .WR_DATA                       (M_AXIS_CQ_TUSER[`UPKT_CQ_TUSER_BE_R]),
         .RST_IN                        (0),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    register
        #(// Parameters
          .C_WIDTH                      (1),
          .C_VALUE                      (1'b0)
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
            assign RXR_DATA_WORD_ENABLE = {wEndMask & wStartMask} & {C_PCI_DATA_WIDTH/32{~rValid | wHasPayload}};
        end else begin
            register
                #(// Parameters
                  .C_WIDTH              (C_PCI_DATA_WIDTH/32),
                  .C_VALUE              (0)
                  /*AUTOINSTPARAM*/)
            dw_enable
                (// Outputs
                 .RD_DATA               (wRxrDataWordEnable),
                 // Inputs
                 .RST_IN                (~rValid | wHasPayload),
                 .WR_DATA               (wEndMask & wStartMask),
                 .WR_EN                 (1),
                 /*AUTOINST*/
                 // Inputs
                 .CLK                   (CLK));

            pipeline
                #(// Parameters
                  .C_DEPTH                      (C_RX_OUTPUT_STAGES-1),
                  .C_WIDTH                      (C_PCI_DATA_WIDTH/32),
                  .C_USE_MEMORY                 (0)
                  /*AUTOINSTPARAM*/)
            dw_pipeline
                (// Outputs
                 .WR_DATA_READY         (), // Pinned to 1
                 .RD_DATA               (RXR_DATA_WORD_ENABLE),
                 .RD_DATA_VALID         (),
                 // Inputs
                 .WR_DATA               (wRxrDataWordEnable),
                 .WR_DATA_VALID         (1),
                 .RD_DATA_READY         (1'b1),
                 .RST_IN                (0),
                 /*AUTOINST*/
                 // Inputs
                 .CLK                   (CLK));
        end
    endgenerate

    pipeline
        #(// Parameters
          .C_DEPTH                      (C_RX_OUTPUT_STAGES),
          .C_WIDTH                      (`UPKT_RXR_MAXHDR_W + 2*(1 + C_OFFSET_WIDTH) +
                                         `SIG_LBE_W + `SIG_FBE_W + `SIG_BARDECODE_W +
                                         `SIG_TYPE_W),
          .C_USE_MEMORY                 (0)
          /*AUTOINSTPARAM*/)
    output_pipeline
        (// Outputs
         .WR_DATA_READY                 (), // Pinned to 1
         .RD_DATA                       ({wRxrHdr,wRxrBarDecoded,wRxrType,wRxrDataStartFlag,wRxrDataStartOffset,wRxrDataEndFlag,wRxrDataEndOffset,wRxrMetaLdwbe,wRxrMetaFdwbe}),
         .RD_DATA_VALID                 (wRxrDataValid),
         // Inputs
         .WR_DATA                       ({wHdr,wBarDecoded,wType,wStartFlag,wStartOffset[C_OFFSET_WIDTH-1:0],wEndFlag,wEndOffset[C_OFFSET_WIDTH-1:0],wByteEnables}),
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
