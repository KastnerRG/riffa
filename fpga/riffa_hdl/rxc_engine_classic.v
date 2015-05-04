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
// Filename:            rxc_engine_classic.v
// Version:             1.0
// Verilog Standard:    Verilog-2001
// Description:         The RXC Engine (Classic) takes a single stream of TLP
// packets and provides the completion packets on the RXC Interface.
// This Engine is capable of operating at "line rate".
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
`include "trellis.vh"
`include "tlp.vh"
module rxc_engine_classic
    #(parameter C_VENDOR = "ALTERA",
      parameter C_PCI_DATA_WIDTH = 128,
      parameter C_RX_PIPELINE_DEPTH = 10
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

     // Interface: RXC Engine
     output [C_PCI_DATA_WIDTH-1:0]                        RXC_DATA,
     output                                               RXC_DATA_VALID,
     output [(C_PCI_DATA_WIDTH/32)-1:0]                   RXC_DATA_WORD_ENABLE,
     output                                               RXC_DATA_START_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0]             RXC_DATA_START_OFFSET,
     output                                               RXC_DATA_END_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0]             RXC_DATA_END_OFFSET,

     output [`SIG_LBE_W-1:0]                              RXC_META_LDWBE,
     output [`SIG_FBE_W-1:0]                              RXC_META_FDWBE,
     output [`SIG_TAG_W-1:0]                              RXC_META_TAG,
     output [`SIG_LOWADDR_W-1:0]                          RXC_META_ADDR,
     output [`SIG_TYPE_W-1:0]                             RXC_META_TYPE,
     output [`SIG_LEN_W-1:0]                              RXC_META_LENGTH,
     output [`SIG_BYTECNT_W-1:0]                          RXC_META_BYTES_REMAINING,
     output [`SIG_CPLID_W-1:0]                            RXC_META_COMPLETER_ID,
     output                                               RXC_META_EP,
    
     // Interface: RX Shift Register
     input [(C_RX_PIPELINE_DEPTH+1)*C_PCI_DATA_WIDTH-1:0] RX_SR_DATA,
     input [C_RX_PIPELINE_DEPTH:0]                        RX_SR_EOP,
     input [(C_RX_PIPELINE_DEPTH+1)*`SIG_OFFSET_W-1:0]    RX_SR_END_OFFSET,
     input [C_RX_PIPELINE_DEPTH:0]                        RX_SR_SOP,
     input [C_RX_PIPELINE_DEPTH:0]                        RX_SR_VALID
     );


    /*AUTOWIRE*/
    /*AUTOINPUT*/
    ///*AUTOOUTPUT*/
    // End of automatics
    localparam C_RX_BE_W = (`SIG_FBE_W+`SIG_LBE_W);

    localparam C_RX_INPUT_STAGES = 1;
    localparam C_RX_OUTPUT_STAGES = 1; // Must always be at least one
    localparam C_RX_COMPUTATION_STAGES = 1;
    localparam C_RX_DATA_STAGES = C_RX_COMPUTATION_STAGES;
    localparam C_RX_META_STAGES = C_RX_DATA_STAGES - 1;
    localparam C_TOTAL_STAGES = C_RX_COMPUTATION_STAGES + C_RX_OUTPUT_STAGES + C_RX_INPUT_STAGES;
    
    // Cycle index in the SOP register when enable is raised
    // Computation can begin when the last DW of the header is recieved. 
    localparam C_RX_COMPUTATION_CYCLE = C_RX_COMPUTATION_STAGES + (`TLP_CPLMETADW2_I/C_PCI_DATA_WIDTH);
    // The computation cycle must be at least one cycle before the address is enabled
    localparam C_RX_DATA_CYCLE = C_RX_COMPUTATION_CYCLE;

    localparam C_RX_METADW0_CYCLE = (`TLP_CPLMETADW0_I/C_PCI_DATA_WIDTH) + C_RX_INPUT_STAGES;
    localparam C_RX_METADW1_CYCLE = (`TLP_CPLMETADW1_I/C_PCI_DATA_WIDTH) + C_RX_INPUT_STAGES;
    localparam C_RX_METADW2_CYCLE = (`TLP_CPLMETADW2_I/C_PCI_DATA_WIDTH) + C_RX_INPUT_STAGES;

    localparam C_RX_METADW0_INDEX = C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES + (`TLP_CPLMETADW0_I%C_PCI_DATA_WIDTH);
    localparam C_RX_METADW1_INDEX = C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES + (`TLP_CPLMETADW1_I%C_PCI_DATA_WIDTH);
    localparam C_RX_METADW2_INDEX = C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES + (`TLP_CPLMETADW2_I%C_PCI_DATA_WIDTH);

    localparam C_OFFSET_WIDTH = clog2s(C_PCI_DATA_WIDTH/32);
    localparam C_MAX_ABLANK_WIDTH = 32;
    localparam C_MAX_START_OFFSET = (`TLP_MAXHDR_W + C_MAX_ABLANK_WIDTH)/32;
    localparam C_STD_START_DELAY = (64/C_PCI_DATA_WIDTH);

    wire [`TLP_CPLADDR_W-1:0]                             wAddr;
    wire [`TLP_CPLHDR_W-1:0]                              wMetadata;
    wire [`TLP_TYPE_W-1:0]                                wType;
    wire [`TLP_LEN_W-1:0]                                 wLength;
    wire [2:0]                                            wHdrLength;
    wire [2:0]                                            wHdrLengthM1;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]                      wEndMask;
    wire                                                  wEndFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]                wEndOffset;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]                      wStartMask;
    wire                                                  wStartFlag;
    wire                                                  _wStartFlag;
    wire [2:0]                                            wStartOffset;
    wire [3:0]                                            wStartFlags;
    
    wire                                                  wInsertBlank;

    wire [C_PCI_DATA_WIDTH-1:0]                           wRxcData;

    wire [95:0]                                           wRxcMetadata;
    wire                                                  wRxcDataValid;
    wire                                                  wRxcDataEndFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]                wRxcDataEndOffset;
    wire                                                  wRxcDataStartFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]                wRxcDataStartOffset;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]                      wRxcDataWordEnable;
    wire [C_RX_PIPELINE_DEPTH:0]                          wRxSrSop;

    reg                                                   rValid,_rValid;
    
    
    // Calculate the header length (start offset), and header length minus 1 (end offset)
    assign wHdrLength = 3'b011;
    assign wHdrLengthM1 = 3'b010;
    // Determine if the TLP has an inserted blank before the payload
    assign wInsertBlank = ~wAddr[2] & (C_VENDOR == "ALTERA");

    assign wStartOffset = (wHdrLength + {2'd0,wInsertBlank}); // Start offset in dwords
    assign wEndOffset = wHdrLengthM1 + wInsertBlank + wLength; //RX_SR_END_OFFSET[(C_TOTAL_STAGES-1)*`SIG_OFFSET_W +: C_OFFSET_WIDTH];
    
    // Outputs
    assign RXC_DATA = RX_SR_DATA[(C_TOTAL_STAGES)*C_PCI_DATA_WIDTH +: C_PCI_DATA_WIDTH];
    assign RXC_DATA_VALID = wRxcDataValid;
    assign RXC_DATA_END_FLAG = wRxcDataEndFlag;
    assign RXC_DATA_END_OFFSET = wRxcDataEndOffset;
    assign RXC_DATA_START_FLAG = wRxcDataStartFlag;
    assign RXC_DATA_START_OFFSET = wRxcDataStartOffset;

    assign RXC_META_LENGTH = wRxcMetadata[`TLP_LEN_R];
    //assign RXC_META_TC = wRxcMetadata[`TLP_TC_R];
    //assign RXC_META_ATTR = {wRxcMetadata[`TLP_ATTR1_R], wRxcMetadata[`TLP_ATTR0_R]};
    assign RXC_META_TYPE = tlp_to_trellis_type({wRxcMetadata[`TLP_FMT_R],wRxcMetadata[`TLP_TYPE_R]});
    assign RXC_META_ADDR = wRxcMetadata[`TLP_CPLADDR_R];
    assign RXC_META_COMPLETER_ID = wRxcMetadata[`TLP_CPLCPLID_R];
    assign RXC_META_BYTES_REMAINING = wRxcMetadata[`TLP_CPLBYTECNT_R];
    assign RXC_META_TAG = wRxcMetadata[`TLP_CPLTAG_R];
    assign RXC_META_EP = wRxcMetadata[`TLP_EP_R];
    assign RXC_META_FDWBE = 0;// TODO: Remove (use addr)
    assign RXC_META_LDWBE = 0;// TODO: Remove (use addr)
    
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
                assign wStartFlags[1] = wRxSrSop[C_RX_INPUT_STAGES + 1] & wMetadata[`TLP_PAYBIT_I] & ~wMetadata[`TLP_4DWHBIT_I] & RX_SR_DATA[C_RX_METADW2_INDEX + 2]; // 3DWH, No Blank
            end else begin
                assign wStartFlags[1] = wRxSrSop[C_RX_INPUT_STAGES + 1] & wMetadata[`TLP_PAYBIT_I] & ~wMetadata[`TLP_4DWHBIT_I]; // 3DWH, No Blank
            end
            assign wStartFlags[0] = wRxSrSop[C_RX_INPUT_STAGES + 1] & ~wMetadata[`TLP_PAYBIT_I] & rValid; // No Payload
        end else if (C_PCI_DATA_WIDTH == 128) begin    
            assign wStartFlags[3] = 0;
            assign wStartFlags[2] = wRxSrSop[C_RX_INPUT_STAGES + 1] & wMetadata[`TLP_PAYBIT_I] & ~rValid; // Is this correct?
            if(C_VENDOR == "ALTERA") begin
                assign wStartFlags[1] = wRxSrSop[C_RX_INPUT_STAGES] & RX_SR_DATA[C_RX_METADW0_INDEX + `TLP_PAYBIT_I] & ~RX_SR_DATA[C_RX_METADW0_INDEX + `TLP_4DWHBIT_I] & RX_SR_DATA[C_RX_METADW2_INDEX + 2]; // 3DWH, No Blank
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
        end else if (RX_SR_EOP[C_RX_INPUT_STAGES+1]) begin
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


    register
        #(
          // Parameters
          .C_WIDTH                      (32))
    metadata_DW0_register
        (
         // Outputs
         .RD_DATA                       (wMetadata[31:0]),
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN),
         .WR_DATA                       (RX_SR_DATA[C_RX_METADW0_INDEX +: 32]),
         .WR_EN                         (wRxSrSop[C_RX_METADW0_CYCLE]));

    register
        #(
          // Parameters
          .C_WIDTH                      (32))
    meta_DW1_register
        (
         // Outputs
         .RD_DATA                       (wMetadata[63:32]),
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN),
         .WR_DATA                       (RX_SR_DATA[C_RX_METADW1_INDEX +: 32]),
         .WR_EN                         (wRxSrSop[C_RX_METADW1_CYCLE]));

    register
        #(
          // Parameters
          .C_WIDTH                      (32))
    meta_DW2_register
        (
         // Outputs
         .RD_DATA                       (wMetadata[95:64]),
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN),
         .WR_DATA                       (RX_SR_DATA[C_RX_METADW2_INDEX +: 32]),
         .WR_EN                         (wRxSrSop[C_RX_METADW2_CYCLE]));

    register
        #(
          // Parameters
          .C_WIDTH                      (`TLP_TYPE_W))
    metadata_type_register
        (
         // Outputs
         .RD_DATA                       (wType),
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN),
         .WR_DATA                       (RX_SR_DATA[(`TLP_TYPE_I + C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES) +: `TLP_TYPE_W]),
         .WR_EN                         (wRxSrSop[`TLP_TYPE_I/C_PCI_DATA_WIDTH  + C_RX_INPUT_STAGES]));

    register
        #(
          // Parameters
          .C_WIDTH                      (`TLP_LEN_W))
    metadata_length_register
        (
         // Outputs
         .RD_DATA                       (wLength),
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN),
         .WR_DATA                       (RX_SR_DATA[((`TLP_LEN_I%C_PCI_DATA_WIDTH) + C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES) +: `TLP_LEN_W]),
         .WR_EN                         (wRxSrSop[`TLP_LEN_I/C_PCI_DATA_WIDTH + C_RX_INPUT_STAGES]));

    register
        #(
          // Parameters
          .C_WIDTH                      (`TLP_CPLADDR_W))
    metadata_address_register
        (
         // Outputs
         .RD_DATA                       (wAddr),
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN),
         .WR_DATA                       (RX_SR_DATA[((`TLP_CPLADDR_I%C_PCI_DATA_WIDTH) + C_PCI_DATA_WIDTH*C_RX_INPUT_STAGES) +: `TLP_CPLADDR_W]),
         .WR_EN                         (wRxSrSop[`TLP_CPLADDR_I/C_PCI_DATA_WIDTH + C_RX_INPUT_STAGES]));

    
    register
        #(
          // Parameters
          .C_WIDTH                      (1),
          .C_VALUE                      (1'b0)
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
         .OFFSET                        (wEndOffset)
         /*AUTOINST*/);
    generate
        if(C_RX_OUTPUT_STAGES == 0) begin
            assign RXC_DATA_WORD_ENABLE = {wEndMask & wStartMask} & {C_PCI_DATA_WIDTH/32{~rValid | ~wMetadata[`TLP_PAYBIT_I]}};
        end else begin
            register
                #(
                  // Parameters
                  .C_WIDTH              (C_PCI_DATA_WIDTH/32),
                  .C_VALUE              (0)
                  /*AUTOINSTPARAM*/)
            dw_enable
                (// Outputs
                 .RD_DATA               (wRxcDataWordEnable),
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
                 .RD_DATA                       (RXC_DATA_WORD_ENABLE),
                 .RD_DATA_VALID                 (),
                 // Inputs
                 .WR_DATA                       (wRxcDataWordEnable),
                 .WR_DATA_VALID                 (1),
                 .RD_DATA_READY                 (1'b1),
                 /*AUTOINST*/
                 // Inputs
                 .CLK                           (CLK),
                 .RST_IN                        (RST_IN));

        end
    endgenerate

    pipeline
        #(
          // Parameters
          .C_DEPTH                      (C_RX_OUTPUT_STAGES),
          .C_WIDTH                      (`TLP_CPLHDR_W + 2*(clog2s(C_PCI_DATA_WIDTH/32) + 1)),
          .C_USE_MEMORY                 (0)
          /*AUTOINSTPARAM*/)
    output_pipeline
        (
         // Outputs
         .WR_DATA_READY                 (), // Pinned to 1
         .RD_DATA                       ({wRxcMetadata,wRxcDataStartFlag,wRxcDataStartOffset,wRxcDataEndFlag,wRxcDataEndOffset}),
         .RD_DATA_VALID                 (wRxcDataValid),
         // Inputs
         .WR_DATA                       ({wMetadata, wStartFlag,wStartOffset[C_OFFSET_WIDTH-1:0],wEndFlag,wEndOffset[C_OFFSET_WIDTH-1:0]}),
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
         .WR_DATA                       (RX_TLP_START_FLAG & RX_TLP_VALID & (RX_SR_DATA[`TLP_TYPE_R] == `TLP_TYPE_CPL)),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));

endmodule
module rxc_engine_128
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

     // Interface: RXC Engine
     output [C_PCI_DATA_WIDTH-1:0]                        RXC_DATA,
     output                                               RXC_DATA_VALID,
     output [(C_PCI_DATA_WIDTH/32)-1:0]                   RXC_DATA_WORD_ENABLE,
     output                                               RXC_DATA_START_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0]             RXC_DATA_START_OFFSET,
     output                                               RXC_DATA_END_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0]             RXC_DATA_END_OFFSET,

     output [`SIG_LBE_W-1:0]                              RXC_META_LDWBE,
     output [`SIG_FBE_W-1:0]                              RXC_META_FDWBE,
     output [`SIG_TAG_W-1:0]                              RXC_META_TAG,
     output [`SIG_LOWADDR_W-1:0]                          RXC_META_ADDR,
     output [`SIG_TYPE_W-1:0]                             RXC_META_TYPE,
     output [`SIG_LEN_W-1:0]                              RXC_META_LENGTH,
     output [`SIG_BYTECNT_W-1:0]                          RXC_META_BYTES_REMAINING,
     output [`SIG_CPLID_W-1:0]                            RXC_META_COMPLETER_ID,
     output                                               RXC_META_EP,

     // Interface: RX Shift Register
     input [(C_RX_PIPELINE_DEPTH+1)*C_PCI_DATA_WIDTH-1:0] RX_SR_DATA,
     input [C_RX_PIPELINE_DEPTH:0]                        RX_SR_EOP,
     input [(C_RX_PIPELINE_DEPTH+1)*`SIG_OFFSET_W-1:0]    RX_SR_END_OFFSET,
     input [(C_RX_PIPELINE_DEPTH+1)*`SIG_OFFSET_W-1:0]    RX_SR_START_OFFSET,
     input [C_RX_PIPELINE_DEPTH:0]                        RX_SR_SOP,
     input [C_RX_PIPELINE_DEPTH:0]                        RX_SR_VALID
     );

    /*AUTOWIRE*/
    ///*AUTOOUTPUT*/
    localparam C_RX_BE_W = (`SIG_FBE_W+`SIG_LBE_W);
    localparam C_RX_INPUT_STAGES = 1;
    localparam C_RX_OUTPUT_STAGES = 1;
    localparam C_RX_COMPUTATION_STAGES = 1;
    localparam C_RX_HDR_STAGES = 1; // Specific to the Xilinx 128-bit RXC Engine
    localparam C_TOTAL_STAGES = C_RX_COMPUTATION_STAGES + C_RX_OUTPUT_STAGES + C_RX_INPUT_STAGES + C_RX_HDR_STAGES;

    localparam C_OFFSET_WIDTH = clog2s(C_PCI_DATA_WIDTH/32);
    localparam C_STRADDLE_W = 64;
    localparam C_HDR_NOSTRADDLE_I = C_RX_INPUT_STAGES * C_PCI_DATA_WIDTH;
    localparam C_OUTPUT_STAGE_WIDTH = (C_PCI_DATA_WIDTH/32) + 2 + clog2s(C_PCI_DATA_WIDTH/32) + 1 + `SIG_TAG_W + `SIG_TYPE_W + `SIG_LOWADDR_W + `SIG_REQID_W + `SIG_LEN_W + `SIG_BYTECNT_W;

    // Header Reg Inputs
    wire [`SIG_OFFSET_W-1:0]                              __wRxcStartOffset; 
    wire [`SIG_OFFSET_W-1:0]                              __wRxcStraddledStartOffset; 
    wire [`TLP_MAXHDR_W-1:0]                              __wRxcHdr;
    wire [`TLP_MAXHDR_W-1:0]                              __wRxcHdrStraddled;
    wire [`TLP_MAXHDR_W-1:0]                              __wRxcHdrNotStraddled;
    wire                                                  __wRxcHdrStraddle;
    wire                                                  __wRxcHdrValid;
    wire                                                  __wRxcHdrSOP;
    wire                                                  __wRxcHdrSOPStraddle;

    // Header Reg Outputs
    wire                                                  _wRxcHdrValid;
    wire                                                  _wRxcHdrStraddle;
    wire                                                  _wRxcHdrSOPStraddle;
    wire                                                  _wRxcHdrSOP;
    wire [`TLP_MAXHDR_W-1:0]                              _wRxcHdr;

    wire                                                  _wRxcHdrSF;
    wire [2:0]                                            _wRxcHdrDataSoff;
    wire                                                  _wRxcHdrEF;
    wire [1:0]                                            _wRxcHdrDataEoff;
    wire                                                  _wRxcHdrSCP; // Single Cycle Packet
    wire                                                  _wRxcHdrMCP; // Multi Cycle Packet
    wire                                                  _wRxcHdrRegSF;
    wire                                                  _wRxcHdrRegValid;
    wire                                                  _wRxcHdrStartFlag;
    wire                                                  _wRxcHdr3DWHSF;
    wire [3:0]                                            _wRxcHdrStartMask;
    wire [3:0]                                            _wRxcHdrEndMask;
    
    // Header Reg Outputs
    wire [`TLP_MAXHDR_W-1:0]                              wRxcHdr;
    wire                                                  wRxcHdrSF;
    wire                                                  wRxcHdrEF;    
    wire                                                  wRxcHdrValid;
    wire [63:0]                                           wRxcMetadata;
    wire [`TLP_TYPE_W-1:0]                                wRxcType;
    wire [`TLP_LEN_W-1:0]                                 wRxcLength;
    wire [2:0]                                            wRxcHdrLength;// TODO: 
    wire [`SIG_OFFSET_W-1:0]                              wRxcHdrStartOffset;// TODO: 
    wire                                                  wRxcHdrSCP; // Single Cycle Packet
    wire                                                  wRxcHdrMCP; // Multi Cycle Packet
    wire [1:0]                                            wRxcHdrDataSoff;
    wire [3:0]                                            wRxcHdrStartMask;
    wire [3:0]                                            wRxcHdrEndMask;

    // Output Register Inputs
    wire [C_PCI_DATA_WIDTH-1:0]                           wRxcData;
    wire                                                  wRxcDataValid;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]                      wRxcDataWordEnable;
    wire                                                  wRxcDataStartFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]                wRxcDataStartOffset;
    wire                                                  wRxcDataEndFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]                wRxcDataEndOffset;
    wire [`SIG_TAG_W-1:0]                                 wRxcMetaTag;
    wire [`SIG_TYPE_W-1:0]                                wRxcMetaType;
    wire [`SIG_LOWADDR_W-1:0]                             wRxcMetaAddr;
    wire [`SIG_REQID_W-1:0]                               wRxcMetaCompleterId;
    wire [`SIG_LEN_W-1:0]                                 wRxcMetaLength;
    wire                                                  wRxcMetaEP;
    wire [`SIG_BYTECNT_W-1:0]                             wRxcMetaBytesRemaining;

    reg                                                   rStraddledSOP;
    reg                                                   rStraddledSOPSplit;    

    // ----- Header Register -----
    assign __wRxcHdrSOP = RX_SR_SOP[C_RX_INPUT_STAGES] & ~__wRxcStartOffset[1];
    assign __wRxcHdrSOPStraddle = RX_SR_SOP[C_RX_INPUT_STAGES] & __wRxcStraddledStartOffset[1];

    assign __wRxcHdrNotStraddled = RX_SR_DATA[C_HDR_NOSTRADDLE_I +: C_PCI_DATA_WIDTH];
    assign __wRxcHdrStraddled = {RX_SR_DATA[C_RX_INPUT_STAGES*C_PCI_DATA_WIDTH +: C_STRADDLE_W],
                                 RX_SR_DATA[(C_RX_INPUT_STAGES+1)*C_PCI_DATA_WIDTH + C_STRADDLE_W +: C_STRADDLE_W ]};
    assign __wRxcStartOffset = RX_SR_START_OFFSET[`SIG_OFFSET_W*C_RX_INPUT_STAGES +: `SIG_OFFSET_W];
    assign __wRxcStraddledStartOffset = RX_SR_START_OFFSET[`SIG_OFFSET_W*(C_RX_INPUT_STAGES) +: `SIG_OFFSET_W];
    assign __wRxcHdrValid = __wRxcHdrSOP | ((rStraddledSOP | rStraddledSOPSplit) & RX_SR_VALID[C_RX_INPUT_STAGES]);

    assign _wRxcHdrRegSF = RX_SR_SOP[C_RX_INPUT_STAGES + C_RX_HDR_STAGES] & _wRxcHdrValid;
    assign _wRxcHdrDataSoff = {1'b0,_wRxcHdrSOPStraddle,1'b0} + 3'd3;
    assign _wRxcHdrRegValid = RX_SR_VALID[C_RX_INPUT_STAGES + C_RX_HDR_STAGES];
    assign _wRxcHdr3DWHSF = ~_wRxcHdr[`TLP_4DWHBIT_I] & _wRxcHdrSOP;

    assign _wRxcHdrSF = (_wRxcHdr3DWHSF | _wRxcHdrSOPStraddle);
    assign _wRxcHdrEF = RX_SR_EOP[C_RX_INPUT_STAGES + C_RX_HDR_STAGES];
    assign _wRxcHdrDataEoff = RX_SR_END_OFFSET[(C_RX_INPUT_STAGES+C_RX_HDR_STAGES)*`SIG_OFFSET_W +: C_OFFSET_WIDTH];

    assign _wRxcHdrSCP = _wRxcHdrSF & _wRxcHdrEF & (_wRxcHdr[`TLP_TYPE_R] == `TLP_TYPE_CPL);
    assign _wRxcHdrMCP = (_wRxcHdrSF & ~_wRxcHdrEF & (_wRxcHdr[`TLP_TYPE_R] == `TLP_TYPE_CPL)) | 
                         (wRxcHdrMCP & ~wRxcHdrEF);

    assign _wRxcHdrStartMask = 4'hf << (_wRxcHdrSF ? _wRxcHdrDataSoff[1:0] : 0);

    assign wRxcDataWordEnable = wRxcHdrEndMask & wRxcHdrStartMask & {4{wRxcDataValid}};
    assign wRxcDataValid = wRxcHdrSCP | wRxcHdrMCP;
    assign wRxcDataStartFlag = wRxcHdrSF;
    assign wRxcDataEndFlag = wRxcHdrEF;
    assign wRxcDataStartOffset = wRxcHdrDataSoff;
    assign wRxcMetaBytesRemaining = wRxcHdr[`TLP_CPLBYTECNT_R];
    assign wRxcMetaTag = wRxcHdr[`TLP_CPLTAG_R];
    assign wRxcMetaAddr = wRxcHdr[`TLP_CPLADDR_R];
    assign wRxcMetaCompleterId = wRxcHdr[`TLP_REQREQID_R];
    assign wRxcMetaLength = wRxcHdr[`TLP_LEN_R];
    assign wRxcMetaEP = wRxcHdr[`TLP_EP_R];
    assign wRxcMetaType = tlp_to_trellis_type({wRxcHdr[`TLP_FMT_R],wRxcHdr[`TLP_TYPE_R]});

    assign RXC_DATA = RX_SR_DATA[C_PCI_DATA_WIDTH*C_TOTAL_STAGES +: C_PCI_DATA_WIDTH];
    assign RXC_DATA_END_OFFSET = RX_SR_END_OFFSET[`SIG_OFFSET_W*(C_TOTAL_STAGES) +: C_OFFSET_WIDTH];
    
    always @(posedge CLK) begin
        rStraddledSOP <= RX_SR_SOP[C_RX_INPUT_STAGES] & __wRxcStraddledStartOffset[1];
        // Set Straddled SOP Split when there is a straddled packet where the
        // header is not contiguous. (Not sure if this is ever possible, but
        // better safe than sorry assert Straddled SOP Split. See Virtex 6 PCIe
        // errata.)
        if(__wRxcHdrSOP | RST_IN) begin
            rStraddledSOPSplit <=0;
        end else begin
            rStraddledSOPSplit <= (rStraddledSOP | rStraddledSOPSplit) & ~RX_SR_VALID[C_RX_INPUT_STAGES];
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
         .MUX_OUTPUT                    (__wRxcHdr[`TLP_MAXHDR_W-1:0]),
         // Inputs
         .MUX_INPUTS                    ({__wRxcHdrStraddled[`TLP_MAXHDR_W-1:0],
                                          __wRxcHdrNotStraddled[`TLP_MAXHDR_W-1:0]}),
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
         .RD_DATA                       ({_wRxcHdr[C_STRADDLE_W-1:0], _wRxcHdrValid}),
         // Inputs
         .WR_DATA                       ({__wRxcHdr[C_STRADDLE_W-1:0], __wRxcHdrValid}),
         .WR_EN                         (__wRxcHdrSOP | rStraddledSOP),
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
         .RD_DATA                       (_wRxcHdr[`TLP_MAXHDR_W-1:C_STRADDLE_W]),
         // Inputs
         .WR_DATA                       (__wRxcHdr[`TLP_MAXHDR_W-1:C_STRADDLE_W]),
         .WR_EN                         (__wRxcHdrSOP | rStraddledSOP | rStraddledSOPSplit), // Non straddled start, Straddled, or straddled split
         .RST_IN                        (RST_IN), // TODO: Remove
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    register
        #(
          // Parameters
          .C_WIDTH                      (2),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    sf4dwh// TODO: Rename
        (
         // Outputs
         .RD_DATA                       ({_wRxcHdrSOPStraddle,_wRxcHdrSOP}),
         // Inputs
         .WR_DATA                       ({rStraddledSOP,__wRxcHdrSOP}),
         .WR_EN                         (1),
         .RST_IN                        (RST_IN), // TODO: Remove
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    // ----- Computation Register -----
    register
        #(
          // Parameters
          .C_WIDTH              (128 + 4),/* TODO: TLP_METADATA_W*/
          .C_VALUE              (0)
          /*AUTOINSTPARAM*/)
    metadata 
        (// Output
         .RD_DATA               ({wRxcHdr,
                                  wRxcHdrSF, wRxcHdrDataSoff,
                                  wRxcHdrEF}),
         // Inputs
         .RST_IN                (0),
         .WR_DATA               ({_wRxcHdr,
                                  _wRxcHdrSF, _wRxcHdrDataSoff[1:0],
                                  _wRxcHdrEF}),
         .WR_EN                 (1),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    register
        #(
          // Parameters
          .C_WIDTH              (3+8),
          .C_VALUE              (0)
          /*AUTOINSTPARAM*/)
    metadata_valid
        (// Output
         .RD_DATA               ({wRxcHdrValid, 
                                  wRxcHdrSCP, wRxcHdrMCP,
                                  wRxcHdrEndMask, wRxcHdrStartMask}),
         // Inputs
         .RST_IN                (RST_IN),
         .WR_DATA               ({_wRxcHdrValid, 
                                  _wRxcHdrSCP, _wRxcHdrMCP,
                                  _wRxcHdrEndMask, _wRxcHdrStartMask}), // Need to invert the start mask
         .WR_EN                 (1),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    offset_to_mask
        #(// Parameters
          .C_MASK_SWAP                  (0),
          .C_MASK_WIDTH                 (4)
          /*AUTOINSTPARAM*/)
    o2m_ef
        (
         // Outputs
         .MASK                          (_wRxcHdrEndMask),
         // Inputs
         .OFFSET_ENABLE                 (_wRxcHdrEF),
         .OFFSET                        (_wRxcHdrDataEoff)
         /*AUTOINST*/);

    pipeline
        #(
          // Parameters
          .C_DEPTH                      (C_RX_OUTPUT_STAGES),
          .C_WIDTH                      (C_OUTPUT_STAGE_WIDTH),
          .C_USE_MEMORY                 (0)
          /*AUTOINSTPARAM*/)
    output_pipeline
        (
         // Outputs
         .WR_DATA_READY                 (), // Pinned to 1
         .RD_DATA                       ({RXC_DATA_WORD_ENABLE, RXC_DATA_START_FLAG, RXC_DATA_START_OFFSET,
                                          RXC_DATA_END_FLAG, RXC_META_TAG, RXC_META_TYPE, 
                                          RXC_META_ADDR, RXC_META_COMPLETER_ID, RXC_META_BYTES_REMAINING, 
                                          RXC_META_LENGTH, RXC_META_EP}),
         .RD_DATA_VALID                 (RXC_DATA_VALID),
         // Inputs
         .WR_DATA                       ({wRxcDataWordEnable, wRxcDataStartFlag, wRxcDataStartOffset,
                                          wRxcDataEndFlag, wRxcMetaTag, wRxcMetaType, 
                                          wRxcMetaAddr, wRxcMetaCompleterId, wRxcMetaBytesRemaining,
                                          wRxcMetaLength, wRxcMetaEP}),
         .WR_DATA_VALID                 (wRxcDataValid),
         .RD_DATA_READY                 (1'b1),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));
endmodule
