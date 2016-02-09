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
      parameter C_RX_PIPELINE_DEPTH=10)
    (// Interface: Clocks
     input                                                CLK,

     // Interface: Resets
     input                                                RST_BUS, // Replacement for generic RST_IN
     input                                                RST_LOGIC, // Addition for RIFFA_RST
     output                                               DONE_RXR_RST,

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
    reg                                                   rRST;

    assign DONE_RXR_RST = ~rRST;

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
        if(rRST) begin
	        rValid <= 1'b0;
        end else begin
	        rValid <= _rValid;
        end
    end

    always @(posedge CLK) begin
        rRST <= RST_BUS | RST_LOGIC;
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
                 // Inputs
                 .CLK                   (CLK));

            pipeline
                #(
                  // Parameters
                  .C_DEPTH                      (C_RX_OUTPUT_STAGES-1),
                  .C_WIDTH                      (C_PCI_DATA_WIDTH/32),
                  .C_USE_MEMORY                 (0)
                  /*AUTOINSTPARAM*/)
            dw_pipeline
                (// Outputs
                 .WR_DATA_READY                 (), // Pinned to 1
                 .RD_DATA                       (RXR_DATA_WORD_ENABLE),
                 .RD_DATA_VALID                 (),
                 // Inputs
                 .WR_DATA                       (wRxrDataWordEnable),
                 .WR_DATA_VALID                 (1),
                 .RD_DATA_READY                 (1'b1),
                 .RST_IN                        (rRST),
                 /*AUTOINST*/
                 // Inputs
                 .CLK                   (CLK));
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
         .RST_IN                        (rRST),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    register
        #(// Parameters
          .C_WIDTH                      (32),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    meta_DW1_register
        (// Outputs
         .RD_DATA                       (wMetadata[63:32]),
         // Inputs
         .WR_DATA                       (RX_SR_DATA[C_RX_METADW1_INDEX +: 32]),
         .WR_EN                         (wRxSrSop[C_RX_METADW1_CYCLE]),
         .RST_IN                        (rRST),
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
         .RD_DATA                       (wAddr[31:0]),
         // Inputs
         .WR_DATA                       (RX_SR_DATA[C_RX_ADDRDW0_INDEX +: 32]),
         .WR_EN                         (wRxSrSop[C_RX_ADDRDW0_CYCLE]),
         .RST_IN                        (0),
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
         .RD_DATA                       (wAddr[63:32]),
         // Inputs
         .WR_DATA                       (RX_SR_DATA[C_RX_ADDRDW1_INDEX +: 32]),
         .WR_EN                         (wRxSrSop[C_RX_ADDRDW1_CYCLE]),
         .RST_IN                        (wAddrHiReset & wRxSrSop[C_RX_ADDRDW1_CYCLE]),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    register
        #(// Parameters
          .C_WIDTH                      (2),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    metadata_4DWH_register
        (// Outputs
         .RD_DATA                       ({wHasPayload,w4DWH}),
         // Inputs
         .WR_DATA                       (RX_SR_DATA[`TLP_FMT_I + C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES +: 2]),
         .WR_EN                         (wRxSrSop[`TLP_4DWHBIT_I/C_PCI_DATA_WIDTH + C_RX_INPUT_STAGES]),
         .RST_IN                        (0),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    register
        #(// Parameters
          .C_WIDTH                      (`TLP_TYPE_W),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    metadata_type_register
        (// Outputs
         .RD_DATA                       (wType),
         // Inputs
         .WR_DATA                       (RX_SR_DATA[(`TLP_TYPE_I/* + C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES*/) +: `TLP_TYPE_W]),
         .WR_EN                         (wRxSrSop[`TLP_TYPE_I/C_PCI_DATA_WIDTH/* + C_RX_INPUT_STAGES*/]),
         .RST_IN                        (0),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    register
        #(// Parameters
          .C_WIDTH                      (`TLP_LEN_W),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    metadata_length_register
        (// Outputs
         .RD_DATA                       (wLength),
         // Inputs
         .WR_DATA                       (RX_SR_DATA[(`TLP_LEN_I + C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES) +: `TLP_LEN_W]),
         .WR_EN                         (wRxSrSop[`TLP_LEN_I/C_PCI_DATA_WIDTH + C_RX_INPUT_STAGES]),
         .RST_IN                        (0),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));


    register
        #(// Parameters
          .C_WIDTH                      (1),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    addr_DW0_bit_2_register
        (// Outputs
         .RD_DATA                       (wAddrDW0Bit2),
         // Inputs
         .RST_IN                        (0),
         .WR_DATA                       (RX_SR_DATA[(`TLP_REQADDRDW0_I%C_PCI_DATA_WIDTH) + 2 + C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES]),
         .WR_EN                         (wRxSrSop[(`TLP_REQADDRDW0_I/C_PCI_DATA_WIDTH) + C_RX_INPUT_STAGES]),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    register
        #(// Parameters
          .C_WIDTH                      (1),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    addr_DW1_bit_2_register
        (// Outputs
         .RD_DATA                       (wAddrDW1Bit2),
         // Inputs
         .WR_DATA                       (RX_SR_DATA[(`TLP_REQADDRDW1_I%C_PCI_DATA_WIDTH) + 2 + C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES]),
         .WR_EN                         (wRxSrSop[(`TLP_REQADDRDW1_I/C_PCI_DATA_WIDTH) + C_RX_INPUT_STAGES]),
         .RST_IN                        (0),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

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
    
    pipeline
        #(// Parameters
          .C_DEPTH                      (C_RX_OUTPUT_STAGES),
          .C_WIDTH                      (`TLP_MAXHDR_W + 2*(1 + C_OFFSET_WIDTH)),
          .C_USE_MEMORY                 (0)
          /*AUTOINSTPARAM*/)
    output_pipeline
        (// Outputs
         .WR_DATA_READY                 (), // Pinned to 1
         .RD_DATA                       ({wRxrMetadata,wRxrMetaAddr,wRxrDataStartFlag,wRxrDataStartOffset,wRxrDataEndFlag,wRxrDataEndOffset}),
         .RD_DATA_VALID                 (wRxrDataValid),
         // Inputs
         .WR_DATA                       ({wMetadata, wAddrFmt, wStartFlag,wStartOffset[C_OFFSET_WIDTH-1:0],wEndFlag,wEndOffset[C_OFFSET_WIDTH-1:0]}),
         .WR_DATA_VALID                 (rValid & RX_SR_VALID[C_TOTAL_STAGES-C_RX_OUTPUT_STAGES]),
         .RD_DATA_READY                 (1'b1),
         .RST_IN                        (rRST),
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
         .WR_DATA                       (RX_TLP_START_FLAG & RX_TLP_VALID & (RX_SR_DATA[`TLP_TYPE_R] == `TLP_TYPE_REQ)),
         .RST_IN                        (0),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));
    
endmodule
// Local Variables:
// verilog-library-directories:("." "../../../common")
// End:
