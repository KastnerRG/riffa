// ----------------------------------------------------------------------
// Copyright (c) 2015, The Regents of the University of California All
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
// Description:         The RXR Engine (Classic) takes a single stream of TLP
// packets and provides the request packets on the RXR Interface.
// This Engine is capable of operating at "line rate".
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
`include "trellis.vh"
`include "tlp.vh"
module rxr_engine_classic
    #(parameter C_VENDOR = "ALTERA",
      parameter C_PCI_DATA_WIDTH = 128,
      parameter C_RX_PIPELINE_DEPTH=10
      )
    (
     // Interface: Clocks
     input                                                CLK,

     // Interface: Resets
     input                                                RST_IN,

     // Interface: RX Classic
     input [C_PCI_DATA_WIDTH-1:0]                         RX_TLP,
     input                                                RX_TLP_VALID,
     input                                                RX_TLP_START_FLAG,
     input [`SIG_OFFSET_W-1:0]                            RX_TLP_START_OFFSET,
     input                                                RX_TLP_END_FLAG,
     input [`SIG_OFFSET_W-1:0]                            RX_TLP_END_OFFSET,
     input [`SIG_BARDECODE_W-1:0]                         RX_TLP_BAR_DECODE,

     // Interface: RXR
     output [C_PCI_DATA_WIDTH-1:0]                        RXR_DATA,
     output                                               RXR_DATA_VALID,
     output [(C_PCI_DATA_WIDTH/32)-1:0]                   RXR_DATA_WORD_ENABLE,
     output                                               RXR_DATA_START_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0]             RXR_DATA_START_OFFSET,
     output                                               RXR_DATA_END_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0]             RXR_DATA_END_OFFSET,
    
     output [`SIG_FBE_W-1:0]                              RXR_META_FDWBE,
     output [`SIG_LBE_W-1:0]                              RXR_META_LDWBE,
     output [`SIG_TC_W-1:0]                               RXR_META_TC,
     output [`SIG_ATTR_W-1:0]                             RXR_META_ATTR,
     output [`SIG_TAG_W-1:0]                              RXR_META_TAG,
     output [`SIG_TYPE_W-1:0]                             RXR_META_TYPE,
     output [`SIG_ADDR_W-1:0]                             RXR_META_ADDR,
     output [`SIG_BARDECODE_W-1:0]                        RXR_META_BAR_DECODED,
     output [`SIG_REQID_W-1:0]                            RXR_META_REQUESTER_ID,
     output [`SIG_LEN_W-1:0]                              RXR_META_LENGTH,
     output                                               RXR_META_EP,

     // Interface: RX Shift Register
     input [(C_RX_PIPELINE_DEPTH+1)*C_PCI_DATA_WIDTH-1:0] RX_SR_DATA,
     input [C_RX_PIPELINE_DEPTH:0]                        RX_SR_EOP,
     input [(C_RX_PIPELINE_DEPTH+1)*`SIG_OFFSET_W-1:0]    RX_SR_END_OFFSET,
     input [C_RX_PIPELINE_DEPTH:0]                        RX_SR_SOP,
     input [C_RX_PIPELINE_DEPTH:0]                        RX_SR_VALID
     );

    /*AUTOWIRE*/
    ///*AUTOOUTPUT*/
    // End of automatics
    localparam C_RX_BE_W = (`SIG_FBE_W+`SIG_LBE_W);

    localparam C_RX_INPUT_STAGES = 1;
    localparam C_RX_OUTPUT_STAGES = 1; // Must always be at least one
    localparam C_RX_COMPUTATION_STAGES = 1;
    localparam C_TOTAL_STAGES = C_RX_COMPUTATION_STAGES + C_RX_OUTPUT_STAGES + C_RX_INPUT_STAGES;
    
    // Cycle index in the SOP register when enable is raised
    // Computation can begin when the last DW of the header is recieved. 
    localparam C_RX_COMPUTATION_CYCLE = C_RX_COMPUTATION_STAGES + (`TLP_REQADDRDW1_I/C_PCI_DATA_WIDTH) + C_RX_INPUT_STAGES;
    // The computation cycle must be at least one cycle before the address is enabled
    localparam C_RX_DATA_CYCLE = C_RX_COMPUTATION_CYCLE;

    localparam C_RX_ADDRDW0_CYCLE = (`TLP_REQADDRDW0_I/C_PCI_DATA_WIDTH) + C_RX_INPUT_STAGES;
    localparam C_RX_ADDRDW1_CYCLE = (`TLP_REQADDRDW1_I/C_PCI_DATA_WIDTH) + C_RX_INPUT_STAGES;
    localparam C_RX_METADW0_CYCLE = (`TLP_REQMETADW0_I/C_PCI_DATA_WIDTH) + C_RX_INPUT_STAGES;
    localparam C_RX_METADW1_CYCLE = (`TLP_REQMETADW1_I/C_PCI_DATA_WIDTH) + C_RX_INPUT_STAGES;

    localparam C_RX_ADDRDW0_INDEX = C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES + (`TLP_REQADDRDW0_I%C_PCI_DATA_WIDTH);
    localparam C_RX_ADDRDW1_INDEX = C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES + (`TLP_REQADDRDW1_I%C_PCI_DATA_WIDTH);
    localparam C_RX_ADDRDW1_RESET_INDEX = C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES + 
                                          C_PCI_DATA_WIDTH*(C_RX_ADDRDW1_CYCLE - C_RX_METADW0_CYCLE) +
                                          `TLP_4DWHBIT_I;
    
    localparam C_RX_METADW0_INDEX = C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES + (`TLP_REQMETADW0_I%C_PCI_DATA_WIDTH);
    localparam C_RX_METADW1_INDEX = C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES + (`TLP_REQMETADW1_I%C_PCI_DATA_WIDTH);

    localparam C_OFFSET_WIDTH = clog2s(C_PCI_DATA_WIDTH/32);
    localparam C_MAX_ABLANK_WIDTH = 32;
    localparam C_MAX_START_OFFSET = (`TLP_MAXHDR_W + C_MAX_ABLANK_WIDTH)/32;
    localparam C_STD_START_DELAY = (64/C_PCI_DATA_WIDTH);

    wire [63:0]                                           wAddrFmt;
    wire [63:0]                                           wMetadata;
    wire [`TLP_TYPE_W-1:0]                                wType;
    wire [`TLP_LEN_W-1:0]                                 wLength;
    wire                                                  wAddrDW0Bit2;
    wire                                                  wAddrDW1Bit2;
    wire                                                  wAddrHiReset;
    wire [31:0]                                           wAddrMux[(`TLP_REQADDR_W / 32)-1:0];
    wire [63:0]                                           wAddr;
    wire                                                  w4DWH;
    wire                                                  wHasPayload;
    wire [2:0]                                            wHdrLength;
    wire [2:0]                                            wHdrLengthM1;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]                      wEndMask;
    wire                                                  _wEndFlag;
    wire                                                  wEndFlag;
    wire [C_OFFSET_WIDTH-1:0]                             wEndOffset;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]                      wStartMask;
    wire [3:0]                                            wStartFlags;
    wire                                                  wStartFlag;
    wire                                                  _wStartFlag;
    wire [clog2s(C_MAX_START_OFFSET)-1:0]                 wStartOffset;
    wire                                                  wInsertBlank;
    wire                                                  wRotateAddressField;


    wire [C_PCI_DATA_WIDTH-1:0]                           wRxrData;

    wire [`SIG_ADDR_W-1:0]                                wRxrMetaAddr;
    wire [63:0]                                           wRxrMetadata;
    wire                                                  wRxrDataValid;
    wire                                                  wRxrDataReady; // Pinned High
    wire                                                  wRxrDataEndFlag;
    wire [C_OFFSET_WIDTH-1:0]                             wRxrDataEndOffset;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]                      wRxrDataWordEnable;
    wire                                                  wRxrDataStartFlag;
    wire [C_OFFSET_WIDTH-1:0]                             wRxrDataStartOffset;

    wire [C_RX_PIPELINE_DEPTH:0]                          wRxSrSop;

    reg                                                   rValid,_rValid;

    
    assign wAddrHiReset = ~RX_SR_DATA[C_RX_ADDRDW1_RESET_INDEX];

    // Select Addr[31:0] from one of the two possible locations in the TLP based
    // on header length (1 bit)
    assign wRotateAddressField = w4DWH;
    assign wAddrFmt = {wAddrMux[~wRotateAddressField],wAddrMux[wRotateAddressField]};
    assign wAddrMux[0] = wAddr[31:0];    
    assign wAddrMux[1] = wAddr[63:32];    
    // Calculate the header length (start offset), and header length minus 1 (end offset)
    assign wHdrLength = {w4DWH,~w4DWH,~w4DWH};
    assign wHdrLengthM1 = {1'b0,1'b1,w4DWH};
    // Determine if the TLP has an inserted blank before the payload
    assign wInsertBlank = ((w4DWH & wAddrDW1Bit2) | (~w4DWH & ~wAddrDW0Bit2)) & (C_VENDOR == "ALTERA");
    

    assign wStartOffset = (wHdrLength + {2'd0,wInsertBlank}); // Start offset in dwords
    assign wEndOffset = wHdrLengthM1 + wInsertBlank + wLength;//RX_SR_END_OFFSET[(C_TOTAL_STAGES-1)*`SIG_OFFSET_W +: C_OFFSET_WIDTH];

    // Inputs
    // Technically an input, but the trellis protocol specifies it must be held high at all times
    assign wRxrDataReady = 1;
    // Outputs
    assign RXR_DATA = RX_SR_DATA[(C_TOTAL_STAGES)*C_PCI_DATA_WIDTH +: C_PCI_DATA_WIDTH];
    assign RXR_DATA_VALID = wRxrDataValid;
    assign RXR_DATA_END_FLAG = wRxrDataEndFlag;
    assign RXR_DATA_END_OFFSET = wRxrDataEndOffset;
    assign RXR_DATA_START_FLAG = wRxrDataStartFlag;
    assign RXR_DATA_START_OFFSET = wRxrDataStartOffset;

    assign RXR_META_BAR_DECODED = 0;
    assign RXR_META_LENGTH = wRxrMetadata[`TLP_LEN_R];
    assign RXR_META_TC = wRxrMetadata[`TLP_TC_R];
    assign RXR_META_ATTR = {wRxrMetadata[`TLP_ATTR1_R], wRxrMetadata[`TLP_ATTR0_R]};
    assign RXR_META_TYPE = tlp_to_trellis_type({wRxrMetadata[`TLP_FMT_R],wRxrMetadata[`TLP_TYPE_R]});
    assign RXR_META_ADDR = wRxrMetaAddr;
    assign RXR_META_REQUESTER_ID = wRxrMetadata[`TLP_REQREQID_R];
    assign RXR_META_TAG = wRxrMetadata[`TLP_REQTAG_R];
    assign RXR_META_FDWBE = wRxrMetadata[`TLP_REQFBE_R];
    assign RXR_META_LDWBE = wRxrMetadata[`TLP_REQLBE_R];
    assign RXR_META_EP = wRxrMetadata[`TLP_EP_R];
    
    assign _wEndFlag = RX_SR_EOP[C_RX_INPUT_STAGES];
    assign wEndFlag = RX_SR_EOP[C_RX_INPUT_STAGES+1];
    assign _wStartFlag = wStartFlags != 0;
    generate
        if(C_PCI_DATA_WIDTH == 32) begin
            assign wStartFlags[3] = 0;
            assign wStartFlags[2] = wRxSrSop[C_RX_INPUT_STAGES + 3] & wMetadata[`TLP_PAYBIT_I] & ~rValid; // Any remaining cases
            assign wStartFlags[1] = wRxSrSop[C_RX_INPUT_STAGES + 2] & wMetadata[`TLP_PAYBIT_I] & ~wMetadata[`TLP_4DWHBIT_I]; // 3DWH, No Blank
            assign wStartFlags[0] = wRxSrSop[C_RX_INPUT_STAGES + 2] & ~wMetadata[`TLP_PAYBIT_I]; // No Payload
        end else if(C_PCI_DATA_WIDTH == 64) begin
            assign wStartFlags[3] = 0;
            assign wStartFlags[2] = wRxSrSop[C_RX_INPUT_STAGES + 2] & wMetadata[`TLP_PAYBIT_I] & ~rValid; // Any remaining cases
            if(C_VENDOR == "ALTERA") begin
                assign wStartFlags[1] = wRxSrSop[C_RX_INPUT_STAGES + 1] & wMetadata[`TLP_PAYBIT_I] & ~wMetadata[`TLP_4DWHBIT_I] & RX_SR_DATA[C_RX_ADDRDW0_INDEX + 2]; // 3DWH, No Blank
            end else begin
                assign wStartFlags[1] = wRxSrSop[C_RX_INPUT_STAGES + 1] & wMetadata[`TLP_PAYBIT_I] & ~wMetadata[`TLP_4DWHBIT_I]; // 3DWH, No Blank
            end
            assign wStartFlags[0] = wRxSrSop[C_RX_INPUT_STAGES + 1] & ~wMetadata[`TLP_PAYBIT_I]; // No Payload
        end else if (C_PCI_DATA_WIDTH == 128) begin    
            assign wStartFlags[3] = 0;
            assign wStartFlags[2] = wRxSrSop[C_RX_INPUT_STAGES + 1] & wMetadata[`TLP_PAYBIT_I] & ~rValid; // Is this correct?
            if(C_VENDOR == "ALTERA") begin
                assign wStartFlags[1] = wRxSrSop[C_RX_INPUT_STAGES] & RX_SR_DATA[C_RX_METADW0_INDEX + `TLP_PAYBIT_I] & ~RX_SR_DATA[C_RX_METADW0_INDEX + `TLP_4DWHBIT_I] & RX_SR_DATA[C_RX_ADDRDW0_INDEX + 2]; // 3DWH, No Blank
            end else begin
                assign wStartFlags[1] = wRxSrSop[C_RX_INPUT_STAGES] & RX_SR_DATA[C_RX_METADW0_INDEX + `TLP_PAYBIT_I] & ~RX_SR_DATA[C_RX_METADW0_INDEX + `TLP_4DWHBIT_I];
            end
            assign wStartFlags[0] = wRxSrSop[C_RX_INPUT_STAGES] & ~RX_SR_DATA[C_RX_METADW0_INDEX + `TLP_PAYBIT_I]; // No Payload
        end else begin // 256
            assign wStartFlags[3] = 0;
            assign wStartFlags[2] = 0;
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
        if(RST_IN) begin
	        rValid <= 1'b0;
        end else begin
	        rValid <= _rValid;
        end
    end
    assign wStartMask = {C_PCI_DATA_WIDTH/32{1'b1}} << ({C_OFFSET_WIDTH{wStartFlag}}& wStartOffset[C_OFFSET_WIDTH-1:0]);

    offset_to_mask
        #(// Parameters
          .C_MASK_SWAP                  (0),
          .C_MASK_WIDTH                 (C_PCI_DATA_WIDTH/32)
          /*AUTOINSTPARAM*/)
    o2m_ef
        (
         // Outputs
         .MASK                          (wEndMask),
         // Inputs
         .OFFSET_ENABLE                 (wEndFlag),
         .OFFSET                        (wEndOffset[C_OFFSET_WIDTH-1:0])
         /*AUTOINST*/);
    generate
        if(C_RX_OUTPUT_STAGES == 0) begin
            assign RXR_DATA_WORD_ENABLE = {wEndMask & wStartMask} & {C_PCI_DATA_WIDTH/32{~rValid | ~wMetadata[`TLP_PAYBIT_I]}};
        end else begin
            register
                #(
                  // Parameters
                  .C_WIDTH              (C_PCI_DATA_WIDTH/32),
                  .C_VALUE              (0)
                  /*AUTOINSTPARAM*/)
            dw_enable
                (// Outputs
                 .RD_DATA               (wRxrDataWordEnable),
                 // Inputs
                 .RST_IN                (~rValid | ~wMetadata[`TLP_PAYBIT_I]),
                 .WR_DATA               (wEndMask & wStartMask),
                 .WR_EN                 (1),
                 /*AUTOINST*/
                 .CLK                   (CLK));

            pipeline
                #(
                  // Parameters
                  .C_DEPTH                      (C_RX_OUTPUT_STAGES-1),
                  .C_WIDTH                      (C_PCI_DATA_WIDTH/32),
                  .C_USE_MEMORY                 (0)
                  /*AUTOINSTPARAM*/)
            dw_pipeline
                (
                 // Outputs
                 .WR_DATA_READY                 (), // Pinned to 1
                 .RD_DATA                       (RXR_DATA_WORD_ENABLE),
                 .RD_DATA_VALID                 (),
                 // Inputs
                 .WR_DATA                       (wRxrDataWordEnable),
                 .WR_DATA_VALID                 (1),
                 .RD_DATA_READY                 (1'b1),
                 /*AUTOINST*/
                 // Inputs
                 .CLK                           (CLK),
                 .RST_IN                        (RST_IN));
        end
    endgenerate
    
    register
        #(
          // Parameters
          .C_WIDTH                      (32),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    metadata_DW0_register
        (
         // Outputs
         .RD_DATA                       (wMetadata[31:0]),
         // Inputs
         .WR_DATA                       (RX_SR_DATA[C_RX_METADW0_INDEX +: 32]),
         .WR_EN                         (wRxSrSop[C_RX_METADW0_CYCLE]),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));

    register
        #(
          // Parameters
          .C_WIDTH                      (32),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    meta_DW1_register
        (
         // Outputs
         .RD_DATA                       (wMetadata[63:32]),
         // Inputs
         .WR_DATA                       (RX_SR_DATA[C_RX_METADW1_INDEX +: 32]),
         .WR_EN                         (wRxSrSop[C_RX_METADW1_CYCLE]),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));

    register
        #(
          // Parameters
          .C_WIDTH                      (32),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    addr_DW0_register
        (
         // Outputs
         .RD_DATA                       (wAddr[31:0]),
         // Inputs
         .WR_DATA                       (RX_SR_DATA[C_RX_ADDRDW0_INDEX +: 32]),
         .WR_EN                         (wRxSrSop[C_RX_ADDRDW0_CYCLE]),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));

    register
        #(
          // Parameters
          .C_WIDTH                      (32),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    addr_DW1_register
        (
         // Outputs
         .RD_DATA                       (wAddr[63:32]),
         // Inputs
         .WR_DATA                       (RX_SR_DATA[C_RX_ADDRDW1_INDEX +: 32]),
         .WR_EN                         (wRxSrSop[C_RX_ADDRDW1_CYCLE]),
         .RST_IN                        (RST_IN | (wAddrHiReset & wRxSrSop[C_RX_ADDRDW1_CYCLE])),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    register
        #(
          // Parameters
          .C_WIDTH                      (2),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    metadata_4DWH_register
        (
         // Outputs
         .RD_DATA                       ({wHasPayload,w4DWH}),
         // Inputs
         .WR_DATA                       (RX_SR_DATA[`TLP_FMT_I + C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES +: 2]),
         .WR_EN                         (wRxSrSop[`TLP_4DWHBIT_I/C_PCI_DATA_WIDTH + C_RX_INPUT_STAGES]),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));

    register
        #(
          // Parameters
          .C_WIDTH                      (`TLP_TYPE_W),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    metadata_type_register
        (
         // Outputs
         .RD_DATA                       (wType),
         // Inputs
         .WR_DATA                       (RX_SR_DATA[(`TLP_TYPE_I/* + C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES*/) +: `TLP_TYPE_W]),
         .WR_EN                         (wRxSrSop[`TLP_TYPE_I/C_PCI_DATA_WIDTH/* + C_RX_INPUT_STAGES*/]),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));

    register
        #(
          // Parameters
          .C_WIDTH                      (`TLP_LEN_W),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    metadata_length_register
        (
         // Outputs
         .RD_DATA                       (wLength),
         // Inputs
         .WR_DATA                       (RX_SR_DATA[(`TLP_LEN_I + C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES) +: `TLP_LEN_W]),
         .WR_EN                         (wRxSrSop[`TLP_LEN_I/C_PCI_DATA_WIDTH + C_RX_INPUT_STAGES]),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));


    register
        #(
          // Parameters
          .C_WIDTH                      (1),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    addr_DW0_bit_2_register
        (
         // Outputs
         .RD_DATA                       (wAddrDW0Bit2),
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN),
         .WR_DATA                       (RX_SR_DATA[(`TLP_REQADDRDW0_I%C_PCI_DATA_WIDTH) + 2 + C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES]),
         .WR_EN                         (wRxSrSop[(`TLP_REQADDRDW0_I/C_PCI_DATA_WIDTH) + C_RX_INPUT_STAGES]));

    register
        #(
          // Parameters
          .C_WIDTH                      (1),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    addr_DW1_bit_2_register
        (
         // Outputs
         .RD_DATA                       (wAddrDW1Bit2),
         // Inputs
         .WR_DATA                       (RX_SR_DATA[(`TLP_REQADDRDW1_I%C_PCI_DATA_WIDTH) + 2 + C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES]),
         .WR_EN                         (wRxSrSop[(`TLP_REQADDRDW1_I/C_PCI_DATA_WIDTH) + C_RX_INPUT_STAGES]),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));

    register
        #(
          // Parameters
          .C_WIDTH                      (1),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    start_flag_register
        (
         // Outputs
         .RD_DATA                       (wStartFlag),
         // Inputs
         .WR_DATA                       (_wStartFlag),
         .WR_EN                         (1),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));
    
    pipeline
        #(
          // Parameters
          .C_DEPTH                      (C_RX_OUTPUT_STAGES),
          .C_WIDTH                      (`TLP_MAXHDR_W + 2*(1 + C_OFFSET_WIDTH)),
          .C_USE_MEMORY                 (0)
          /*AUTOINSTPARAM*/)
    output_pipeline
        (
         // Outputs
         .WR_DATA_READY                 (), // Pinned to 1
         .RD_DATA                       ({wRxrMetadata,wRxrMetaAddr,wRxrDataStartFlag,wRxrDataStartOffset,wRxrDataEndFlag,wRxrDataEndOffset}),
         .RD_DATA_VALID                 (wRxrDataValid),
         // Inputs
         .WR_DATA                       ({wMetadata, wAddrFmt, wStartFlag,wStartOffset[C_OFFSET_WIDTH-1:0],wEndFlag,wEndOffset[C_OFFSET_WIDTH-1:0]}),
         .WR_DATA_VALID                 (rValid),
         .RD_DATA_READY                 (1'b1),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));

    // Start Flag Shift Register. Data enables are derived from the
    // taps on this shift register.
    shiftreg 
        #(
          // Parameters
          .C_DEPTH                      (C_RX_PIPELINE_DEPTH),
          .C_WIDTH                      (1'b1)
          /*AUTOINSTPARAM*/)
    sop_shiftreg_inst
        (
         // Outputs
         .RD_DATA                       (wRxSrSop),
         // Inputs
         .WR_DATA                       (RX_TLP_START_FLAG & RX_TLP_VALID & (RX_SR_DATA[`TLP_TYPE_R] == `TLP_TYPE_REQ)),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));
    
endmodule
module rxr_engine_128
    #(parameter C_PCI_DATA_WIDTH = 128,
      parameter C_RX_PIPELINE_DEPTH=10
      )
    (
     // Interface: Clocks
     input                                                CLK,

     // Interface: Resets
     input                                                RST_IN,

     // Interface: RX Classic
     input [C_PCI_DATA_WIDTH-1:0]                         RX_TLP,
     input                                                RX_TLP_VALID,
     input                                                RX_TLP_START_FLAG,
     input [`SIG_OFFSET_W-1:0]                            RX_TLP_START_OFFSET,
     input                                                RX_TLP_END_FLAG,
     input [`SIG_OFFSET_W-1:0]                            RX_TLP_END_OFFSET,
     input [`SIG_BARDECODE_W-1:0]                         RX_TLP_BAR_DECODE,

     // Interface: RXR
     output [C_PCI_DATA_WIDTH-1:0]                        RXR_DATA,
     output                                               RXR_DATA_VALID,
     output [(C_PCI_DATA_WIDTH/32)-1:0]                   RXR_DATA_WORD_ENABLE,
     output                                               RXR_DATA_START_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0]             RXR_DATA_START_OFFSET,
     output                                               RXR_DATA_END_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0]             RXR_DATA_END_OFFSET,
    
     output [`SIG_FBE_W-1:0]                              RXR_META_FDWBE,
     output [`SIG_LBE_W-1:0]                              RXR_META_LDWBE,
     output [`SIG_TC_W-1:0]                               RXR_META_TC,
     output [`SIG_ATTR_W-1:0]                             RXR_META_ATTR,
     output [`SIG_TAG_W-1:0]                              RXR_META_TAG,
     output [`SIG_TYPE_W-1:0]                             RXR_META_TYPE,
     output [`SIG_ADDR_W-1:0]                             RXR_META_ADDR,
     output [`SIG_BARDECODE_W-1:0]                        RXR_META_BAR_DECODED,
     output [`SIG_REQID_W-1:0]                            RXR_META_REQUESTER_ID,
     output [`SIG_LEN_W-1:0]                              RXR_META_LENGTH,
     output                                               RXR_META_EP,

     // Interface: RX Shift Register
     input [(C_RX_PIPELINE_DEPTH+1)*C_PCI_DATA_WIDTH-1:0] RX_SR_DATA,
     input [C_RX_PIPELINE_DEPTH:0]                        RX_SR_EOP,
     input [(C_RX_PIPELINE_DEPTH+1)*`SIG_OFFSET_W-1:0]    RX_SR_END_OFFSET,
     input [(C_RX_PIPELINE_DEPTH+1)*`SIG_OFFSET_W-1:0]    RX_SR_START_OFFSET,
     input [C_RX_PIPELINE_DEPTH:0]                        RX_SR_SOP,
     input [C_RX_PIPELINE_DEPTH:0]                        RX_SR_VALID
     );

    /*AUTOWIRE*/
    // Beginning of automatic wires (for undeclared instantiated-module outputs)
    // End of automatics
    ///*AUTOOUTPUT*/
    // End of automatics
    localparam C_RX_BE_W = (`SIG_FBE_W+`SIG_LBE_W);

    localparam C_RX_INPUT_STAGES = 1;
    localparam C_RX_OUTPUT_STAGES = 1;
    localparam C_RX_COMPUTATION_STAGES = 1;
    localparam C_RX_HDR_STAGES = 1; // Specific to the Xilinx 128-bit RXR Engine
    localparam C_TOTAL_STAGES = C_RX_COMPUTATION_STAGES + C_RX_OUTPUT_STAGES + C_RX_INPUT_STAGES + C_RX_HDR_STAGES;
    
    localparam C_OFFSET_WIDTH = clog2s(C_PCI_DATA_WIDTH/32);
    localparam C_STRADDLE_W = 64;
    localparam C_HDR_NOSTRADDLE_I = C_RX_INPUT_STAGES * C_PCI_DATA_WIDTH;
    localparam C_OUTPUT_STAGE_WIDTH = (C_PCI_DATA_WIDTH/32) + 2 + clog2s(C_PCI_DATA_WIDTH/32) + 1 + `SIG_FBE_W + `SIG_LBE_W + `SIG_TC_W + `SIG_ATTR_W + `SIG_TAG_W + `SIG_TYPE_W + `SIG_ADDR_W + `SIG_BARDECODE_W + `SIG_REQID_W + `SIG_LEN_W;

    // Header Reg Inputs
    wire [`SIG_OFFSET_W-1:0]                              __wRxrStartOffset; 
    wire [`SIG_OFFSET_W-1:0]                              __wRxrStraddledStartOffset; 
    wire [`TLP_MAXHDR_W-1:0]                              __wRxrHdr;
    wire [`TLP_MAXHDR_W-1:0]                              __wRxrHdrStraddled;
    wire [`TLP_MAXHDR_W-1:0]                              __wRxrHdrNotStraddled;
    wire                                                  __wRxrHdrValid;
    wire [`TLP_TYPE_W-1:0]                                __wRxrHdrType;
    wire [`TLP_TYPE_W-1:0]                                __wRxrHdrTypeStraddled;

    wire                                                  __wRxrHdrSOP; // Asserted on non-straddle SOP
    wire                                                  __wRxrHdrSOPStraddle;
    wire                                                  __wRxrHdr4DWHWDataSF;

    // Header Reg Outputs
    wire                                                  _wRxrHdrValid;
    wire [`TLP_MAXHDR_W-1:0]                              _wRxrHdr;
    wire [`SIG_ADDR_W-1:0]                                _wRxrAddrUnformatted;
    wire [`SIG_ADDR_W-1:0]                                _wRxrAddr;
    wire [63:0]                                           _wRxrTlpMetadata;
    wire [`TLP_TYPE_W-1:0]                                _wRxrType;
    wire [`TLP_LEN_W-1:0]                                 _wRxrLength;
    wire [2:0]                                            _wRxrHdrHdrLen;// TODO: 
    wire [`SIG_OFFSET_W-1:0]                              _wRxrHdrStartOffset;// TODO: 

    wire                                                  _wRxrHdrDelayedSOP;
    wire                                                  _wRxrHdrSOPStraddle;
    wire                                                  _wRxrHdrSOP;

    wire                                                  _wRxrHdrSF;
    wire                                                  _wRxrHdrEF;
    wire                                                  _wRxrHdrSCP; // Single Cycle Packet
    wire                                                  _wRxrHdrMCP; // Multi Cycle Packet
    wire                                                  _wRxrHdrRegSF;
    wire                                                  _wRxrHdrRegValid;
    wire                                                  _wRxrHdr4DWHSF;
    wire                                                  _wRxrHdr4DWHNoDataSF;
    wire                                                  _wRxrHdr4DWHWDataSF;
    wire                                                  _wRxrHdr3DWHSF;
    wire [2:0]                                            _wRxrHdrDataSoff;
    wire [1:0]                                            _wRxrHdrDataEoff;
    wire [3:0]                                            _wRxrHdrStartMask;
    wire [3:0]                                            _wRxrHdrEndMask;
    
    // Header Reg Outputs
    wire                                                  wRxrHdrSF;
    wire                                                  wRxrHdrEF;
    wire                                                  wRxrHdrValid;
    wire [`TLP_MAXHDR_W-1:0]                              wRxrHdr;
    wire [63:0]                                           wRxrMetadata;
    wire [`TLP_TYPE_W-1:0]                                wRxrType;
    wire [`TLP_LEN_W-1:0]                                 wRxrLength;
    wire [2:0]                                            wRxrHdrLength; // TODO: 
    wire [`SIG_OFFSET_W-1:0]                              wRxrHdrStartOffset; // TODO: 
    wire                                                  wRxrHdrSCP; // Single Cycle Packet
    wire                                                  wRxrHdrMCP; // Multi Cycle Packet
    wire [1:0]                                            wRxrHdrDataSoff;
    wire [3:0]                                            wRxrHdrStartMask;
    wire [3:0]                                            wRxrHdrEndMask;

    // Output Register Inputs
    wire [C_PCI_DATA_WIDTH-1:0]                           wRxrData;
    wire                                                  wRxrDataValid;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]                      wRxrDataWordEnable;
    wire                                                  wRxrDataStartFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]                wRxrDataStartOffset;
    wire                                                  wRxrDataEndFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]                wRxrDataEndOffset;
    wire [`SIG_FBE_W-1:0]                                 wRxrMetaFdwbe;
    wire [`SIG_LBE_W-1:0]                                 wRxrMetaLdwbe;
    wire [`SIG_TC_W-1:0]                                  wRxrMetaTC;
    wire [`SIG_ATTR_W-1:0]                                wRxrMetaAttr;
    wire [`SIG_TAG_W-1:0]                                 wRxrMetaTag;
    wire [`SIG_TYPE_W-1:0]                                wRxrMetaType;
    wire [`SIG_ADDR_W-1:0]                                wRxrMetaAddr;
    wire [`SIG_BARDECODE_W-1:0]                           wRxrMetaBarDecoded;
    wire [`SIG_REQID_W-1:0]                               wRxrMetaRequesterId;
    wire [`SIG_LEN_W-1:0]                                 wRxrMetaLength;
    wire                                                  wRxrMetaEP;

    reg                                                   rStraddledSOP;
    reg                                                   rStraddledSOPSplit;
    
    // ----- Header Register -----
    assign __wRxrHdrSOP = RX_SR_SOP[C_RX_INPUT_STAGES] & ~__wRxrStartOffset[1];
    assign __wRxrHdrSOPStraddle = RX_SR_SOP[C_RX_INPUT_STAGES] & __wRxrStraddledStartOffset[1];
    
    assign __wRxrHdrNotStraddled = RX_SR_DATA[C_HDR_NOSTRADDLE_I +: C_PCI_DATA_WIDTH];
    assign __wRxrHdrStraddled = {RX_SR_DATA[C_RX_INPUT_STAGES*C_PCI_DATA_WIDTH +: C_STRADDLE_W],
                                 RX_SR_DATA[(C_RX_INPUT_STAGES+1)*C_PCI_DATA_WIDTH + C_STRADDLE_W +: C_STRADDLE_W ]};
    assign __wRxrStartOffset = RX_SR_START_OFFSET[`SIG_OFFSET_W*C_RX_INPUT_STAGES +: `SIG_OFFSET_W];
    assign __wRxrStraddledStartOffset = RX_SR_START_OFFSET[`SIG_OFFSET_W*(C_RX_INPUT_STAGES) +: `SIG_OFFSET_W];
    assign __wRxrHdrValid = __wRxrHdrSOP | ((rStraddledSOP | rStraddledSOPSplit) & RX_SR_VALID[C_RX_INPUT_STAGES]);
    assign __wRxrHdr4DWHWDataSF = (_wRxrHdr[`TLP_4DWHBIT_I] & _wRxrHdr[`TLP_PAYBIT_I] & RX_SR_VALID[C_RX_INPUT_STAGES] & _wRxrHdrDelayedSOP);
                     

    assign _wRxrHdrHdrLen = {_wRxrHdr[`TLP_4DWHBIT_I],~_wRxrHdr[`TLP_4DWHBIT_I],~_wRxrHdr[`TLP_4DWHBIT_I]};
    assign _wRxrHdrDataSoff = {1'b0,_wRxrHdrSOPStraddle,1'b0} + _wRxrHdrHdrLen;
    assign _wRxrHdrRegSF = RX_SR_SOP[C_RX_INPUT_STAGES + C_RX_HDR_STAGES];
    assign _wRxrHdrRegValid = RX_SR_VALID[C_RX_INPUT_STAGES + C_RX_HDR_STAGES];

    assign _wRxrHdr4DWHNoDataSF = _wRxrHdr[`TLP_4DWHBIT_I] & ~_wRxrHdr[`TLP_PAYBIT_I] & _wRxrHdrSOP;
    assign _wRxrHdr4DWHSF = _wRxrHdr4DWHNoDataSF | (_wRxrHdr4DWHWDataSF & _wRxrHdrRegValid);
    assign _wRxrHdr3DWHSF = ~_wRxrHdr[`TLP_4DWHBIT_I] & _wRxrHdrSOP;
    
    assign _wRxrHdrSF = (_wRxrHdr3DWHSF | _wRxrHdr4DWHSF | _wRxrHdrSOPStraddle);
    assign _wRxrHdrEF = RX_SR_EOP[C_RX_INPUT_STAGES + C_RX_HDR_STAGES];

    assign _wRxrHdrDataEoff = RX_SR_END_OFFSET[(C_RX_INPUT_STAGES+C_RX_HDR_STAGES)*`SIG_OFFSET_W +: C_OFFSET_WIDTH];
    assign _wRxrHdrSCP = _wRxrHdrSF & _wRxrHdrEF & (_wRxrHdr[`TLP_TYPE_R] == `TLP_TYPE_REQ);
    assign _wRxrHdrMCP = (_wRxrHdrSF & ~_wRxrHdrEF & (_wRxrHdr[`TLP_TYPE_R] == `TLP_TYPE_REQ)) | 
                         (wRxrHdrMCP & ~wRxrHdrEF);
    
    assign _wRxrHdrStartMask = 4'hf << (_wRxrHdrSF ? _wRxrHdrDataSoff[1:0] : 0);

    assign wRxrDataWordEnable = wRxrHdrEndMask & wRxrHdrStartMask & {4{wRxrDataValid}};
    assign wRxrDataValid = wRxrHdrSCP | wRxrHdrMCP;
    assign wRxrDataStartFlag = wRxrHdrSF;
    assign wRxrDataEndFlag = wRxrHdrEF;
    assign wRxrDataStartOffset = wRxrHdrDataSoff;
    assign wRxrMetaFdwbe = wRxrHdr[`TLP_REQFBE_R];
    assign wRxrMetaLdwbe = wRxrHdr[`TLP_REQLBE_R];
    assign wRxrMetaTC = wRxrHdr[`TLP_TC_R];
    assign wRxrMetaAttr = {wRxrHdr[`TLP_ATTR1_R], wRxrHdr[`TLP_ATTR0_R]};
    assign wRxrMetaTag = wRxrHdr[`TLP_REQTAG_R];
    assign wRxrMetaAddr = wRxrHdr[`TLP_REQADDRDW0_I +: `TLP_REQADDR_W];/* TODO: REQADDR_R*/
    assign wRxrMetaRequesterId = wRxrHdr[`TLP_REQREQID_R];
    assign wRxrMetaLength = wRxrHdr[`TLP_LEN_R];
    assign wRxrMetaEP = wRxrHdr[`TLP_EP_R];
    assign wRxrMetaType = tlp_to_trellis_type({wRxrHdr[`TLP_FMT_R],wRxrHdr[`TLP_TYPE_R]});

    assign RXR_DATA = RX_SR_DATA[C_PCI_DATA_WIDTH*C_TOTAL_STAGES +: C_PCI_DATA_WIDTH];
    assign RXR_DATA_END_OFFSET = RX_SR_END_OFFSET[`SIG_OFFSET_W*(C_TOTAL_STAGES) +: C_OFFSET_WIDTH];

    always @(posedge CLK) begin
        rStraddledSOP <= __wRxrHdrSOPStraddle;
        // Set Straddled SOP Split when there is a straddled packet where the
        // header is not contiguous. (Not sure if this is ever possible, but
        // better safe than sorry assert Straddled SOP Split. See Virtex 6 PCIe
        // errata.
        if(__wRxrHdrSOP | RST_IN) begin
            rStraddledSOPSplit <=0;
        end else begin
            rStraddledSOPSplit <= (__wRxrHdrSOPStraddle | rStraddledSOPSplit) & ~RX_SR_VALID[C_RX_INPUT_STAGES];
        end
    end
    
    mux
        #(
          // Parameters
          .C_NUM_INPUTS                 (2),
          .C_CLOG_NUM_INPUTS            (1),
          .C_WIDTH                      (`TLP_MAXHDR_W),
          .C_MUX_TYPE                   ("SELECT")
          /*AUTOINSTPARAM*/)
    hdr_mux
        (
         // Outputs
         .MUX_OUTPUT                    (__wRxrHdr[`TLP_MAXHDR_W-1:0]),
         // Inputs
         .MUX_INPUTS                    ({__wRxrHdrStraddled[`TLP_MAXHDR_W-1:0],
                                          __wRxrHdrNotStraddled[`TLP_MAXHDR_W-1:0]}),
         .MUX_SELECT                    (rStraddledSOP | rStraddledSOPSplit)
         /*AUTOINST*/);
    
    register
        #(
          // Parameters
          .C_WIDTH                      (64 + 1),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    hdr_register_63_0
        (
         // Outputs
         .RD_DATA                       ({_wRxrHdr[C_STRADDLE_W-1:0], _wRxrHdrValid}),
                                          
         // Inputs
         .WR_DATA                       ({__wRxrHdr[C_STRADDLE_W-1:0], __wRxrHdrValid}),
         .WR_EN                         (__wRxrHdrSOP | rStraddledSOP),
         .RST_IN                        (RST_IN), // TODO: Remove
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));


    register
        #(
          // Parameters
          .C_WIDTH                      (3),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    sf4dwh
        (
         // Outputs
         .RD_DATA                       ({_wRxrHdr4DWHWDataSF, _wRxrHdrSOPStraddle,_wRxrHdrSOP}),
         // Inputs
         .WR_DATA                       ({__wRxrHdr4DWHWDataSF,rStraddledSOP,__wRxrHdrSOP}),
         .WR_EN                         (1),
         .RST_IN                        (RST_IN), // TODO: Remove
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    register
        #(
          // Parameters
          .C_WIDTH                      (1),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    delayed_sop
        (
         // Outputs
         .RD_DATA                       ({_wRxrHdrDelayedSOP}),
         // Inputs
         .WR_DATA                       ({__wRxrHdrSOP}),
         .WR_EN                         (RX_SR_VALID[C_RX_INPUT_STAGES]),
         .RST_IN                        (RST_IN), // TODO: Remove
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    register
        #(
          // Parameters
          .C_WIDTH                      (64),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    hdr_register_127_64
        (
         // Outputs
         .RD_DATA                       (_wRxrHdr[`TLP_MAXHDR_W-1:C_STRADDLE_W]),
         // Inputs
         .WR_DATA                       (__wRxrHdr[`TLP_MAXHDR_W-1:C_STRADDLE_W]),
         .WR_EN                         (__wRxrHdrSOP | rStraddledSOP | rStraddledSOPSplit), // Non straddled start, Straddled, or straddled split
         .RST_IN                        (RST_IN), // TODO: Remove
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    // ----- Computation Register -----
    register
        #(
          // Parameters
          .C_WIDTH              (64 + 4),/* TODO: TLP_METADATA_W*/
          .C_VALUE              (0)
          /*AUTOINSTPARAM*/)
    metadata 
        (// Outputs
         .RD_DATA               ({wRxrHdr[`TLP_REQMETADW0_I +: 64],
                                  wRxrHdrSF,wRxrHdrDataSoff,
                                  wRxrHdrEF}),/* TODO: TLP_METADATA_R and other signals*/
         // Inputs
         .RST_IN                (0),/* TODO: Never need to reset?*/
         .WR_DATA               ({_wRxrHdr[`TLP_REQMETADW0_I +: 64],
                                  _wRxrHdrSF,_wRxrHdrDataSoff[1:0],
                                  _wRxrHdrEF}),/* TODO: TLP_METADATA_R*/
         .WR_EN                 (1),
         /*AUTOINST*/
         // Inputs
         .CLK                   (CLK));

    register
        #(
          // Parameters
          .C_WIDTH              (3+8),
          .C_VALUE              (0)
          /*AUTOINSTPARAM*/)
    metadata_valid
        (// Output
         .RD_DATA               ({wRxrHdrValid, 
                                  wRxrHdrSCP, wRxrHdrMCP,
                                  wRxrHdrEndMask, wRxrHdrStartMask}),
         // Inputs
         .RST_IN                (RST_IN),
         .WR_DATA               ({_wRxrHdrValid, 
                                  _wRxrHdrSCP, _wRxrHdrMCP,
                                  _wRxrHdrEndMask, _wRxrHdrStartMask}),
         .WR_EN                 (1),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    register
        #(
          // Parameters
          .C_WIDTH              (`SIG_ADDR_W/2),
          .C_VALUE              (0)
          /*AUTOINSTPARAM*/)
    addr_63_32
        (// Outputs
         .RD_DATA               (wRxrHdr[`TLP_REQADDRHI_R]),
         // Inputs
         .RST_IN                (~_wRxrHdr[`TLP_4DWHBIT_I]),
         .WR_DATA               (_wRxrHdr[`TLP_REQADDRLO_R]), // Instead of a mux, we'll use the reset
         .WR_EN                 (1),
         /*AUTOINST*/
         // Inputs
         .CLK                   (CLK));

    register
        #(
          // Parameters
          .C_WIDTH              (`SIG_ADDR_W/2),
          .C_VALUE              (0)
          /*AUTOINSTPARAM*/)
    addr_31_0
        (// Outputs
         .RD_DATA               (wRxrHdr[`TLP_REQADDRLO_R]),
         // Inputs
         .RST_IN                (0),// Never need to reset
         .WR_DATA               (_wRxrHdr[`TLP_4DWHBIT_I] ? _wRxrHdr[`TLP_REQADDRHI_R] : _wRxrHdr[`TLP_REQADDRLO_R]),
         .WR_EN                 (1),
         /*AUTOINST*/
         // Inputs
         .CLK                   (CLK));

    offset_to_mask
        #(// Parameters
          .C_MASK_SWAP                  (0),
          .C_MASK_WIDTH                 (4)
          /*AUTOINSTPARAM*/)
    o2m_ef
        (
         // Outputs
         .MASK                          (_wRxrHdrEndMask),
         // Inputs
         .OFFSET_ENABLE                 (_wRxrHdrEF),
         .OFFSET                        (_wRxrHdrDataEoff)
         /*AUTOINST*/);

    pipeline
        #(
          // Parameters
          .C_DEPTH                      (C_RX_OUTPUT_STAGES),
          .C_WIDTH                      (C_OUTPUT_STAGE_WIDTH),// TODO: 
          .C_USE_MEMORY                 (0)
          /*AUTOINSTPARAM*/)
    output_pipeline
        (
         // Outputs
         .WR_DATA_READY                 (), // Pinned to 1
         .RD_DATA                       ({RXR_DATA_WORD_ENABLE, RXR_DATA_START_FLAG, RXR_DATA_START_OFFSET,
                                          RXR_DATA_END_FLAG,
                                          RXR_META_FDWBE, RXR_META_LDWBE, RXR_META_TC, 
                                          RXR_META_ATTR, RXR_META_TAG, RXR_META_TYPE, 
                                          RXR_META_ADDR, RXR_META_BAR_DECODED, RXR_META_REQUESTER_ID, 
                                          RXR_META_LENGTH, RXR_META_EP}),
         .RD_DATA_VALID                 (RXR_DATA_VALID),
         // Inputs
         .WR_DATA                       ({wRxrDataWordEnable, wRxrDataStartFlag, wRxrDataStartOffset,
                                          wRxrDataEndFlag, 
                                          wRxrMetaFdwbe, wRxrMetaLdwbe, wRxrMetaTC, 
                                          wRxrMetaAttr, wRxrMetaTag, wRxrMetaType, 
                                          wRxrMetaAddr, wRxrMetaBarDecoded, wRxrMetaRequesterId, 
                                          wRxrMetaLength, wRxrMetaEP}),
         .WR_DATA_VALID                 (wRxrDataValid),
         .RD_DATA_READY                 (1'b1),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));
endmodule
// Local Variables:
// verilog-library-directories:("." "../../../common")
// End:
