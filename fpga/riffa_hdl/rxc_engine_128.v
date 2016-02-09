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
// Filename:            rxc_engine_128.v
// Version:             1.0
// Verilog Standard:    Verilog-2001
// Description:         The RX Engine (Classic) takes a single stream of TLP
// packets and provides the request packets on the RXR Interface, and the 
// completion packets on the RXC Interface.
// This Engine is capable of operating at "line rate".
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`include "trellis.vh"
`include "tlp.vh"
module rxc_engine_128
    #(parameter C_PCI_DATA_WIDTH = 128,
      parameter C_RX_PIPELINE_DEPTH=10)
    (// Interface: Clocks
     input                                                CLK,

     // Interface: Resets
     input                                                RST_BUS, // Replacement for generic RST_IN
     input                                                RST_LOGIC, // Addition for RIFFA_RST
     output                                               DONE_RXC_RST,

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
    reg                                                   rRST;

    assign DONE_RXC_RST = ~rRST;

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
        if(__wRxcHdrSOP) begin
            rStraddledSOPSplit <=0;
        end else begin
            rStraddledSOPSplit <= (rStraddledSOP | rStraddledSOPSplit) & ~RX_SR_VALID[C_RX_INPUT_STAGES];
        end
        
    end

    always @(posedge CLK) begin
        rRST <= RST_BUS | RST_LOGIC;
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
         .RST_IN                        (0), // TODO: Remove
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
         .RST_IN                        (0),
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
         .RST_IN                        (0),
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
         .RST_IN                (rRST),
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
         .RST_IN                        (rRST));
endmodule
// Local Variables:
// verilog-library-directories:("." "../../../common")
// End:
