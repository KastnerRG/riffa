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
`include "trellis.vh"
`include "riffa.vh"
module registers
    #(parameter C_PCI_DATA_WIDTH = 128,
      parameter C_NUM_CHNL = 12,
      parameter C_MAX_READ_REQ_BYTES = 512, // Max size of read requests (in bytes)
      parameter C_VENDOR = "ALTERA",
      parameter C_NUM_VECTORS = 2,
      parameter C_VECTOR_WIDTH = 32,
      parameter C_FPGA_NAME = "FPGA",
      parameter C_PIPELINE_OUTPUT= 1,
      parameter C_PIPELINE_INPUT= 1)
    (
     // Interface: Clocks
     input                                     CLK,

     // Interface: Resets
     input                                     RST_IN,

     // Interface: RXR Engine
     input [C_PCI_DATA_WIDTH-1:0]              RXR_DATA,
     input                                     RXR_DATA_VALID,
     input                                     RXR_DATA_START_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]   RXR_DATA_START_OFFSET,
     input [`SIG_FBE_W-1:0]                    RXR_META_FDWBE,
     input                                     RXR_DATA_END_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]   RXR_DATA_END_OFFSET,
     input [`SIG_LBE_W-1:0]                    RXR_META_LDWBE,
     input [`SIG_TC_W-1:0]                     RXR_META_TC,
     input [`SIG_ATTR_W-1:0]                   RXR_META_ATTR,
     input [`SIG_TAG_W-1:0]                    RXR_META_TAG,
     input [`SIG_TYPE_W-1:0]                   RXR_META_TYPE,
     input [`SIG_ADDR_W-1:0]                   RXR_META_ADDR,
     input [`SIG_BARDECODE_W-1:0]              RXR_META_BAR_DECODED,
     input [`SIG_REQID_W-1:0]                  RXR_META_REQUESTER_ID,
     input [`SIG_LEN_W-1:0]                    RXR_META_LENGTH,

     // Interface: TXC Engine
     output                                    TXC_DATA_VALID,
     output [C_PCI_DATA_WIDTH-1:0]             TXC_DATA,
     output                                    TXC_DATA_START_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0]  TXC_DATA_START_OFFSET,
     output                                    TXC_DATA_END_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0]  TXC_DATA_END_OFFSET,
     input                                     TXC_DATA_READY,
    
     output                                    TXC_META_VALID,
     output [`SIG_FBE_W-1:0]                   TXC_META_FDWBE,
     output [`SIG_LBE_W-1:0]                   TXC_META_LDWBE,
     output [`SIG_LOWADDR_W-1:0]               TXC_META_ADDR,
     output [`SIG_TYPE_W-1:0]                  TXC_META_TYPE,
     output [`SIG_LEN_W-1:0]                   TXC_META_LENGTH,
     output [`SIG_BYTECNT_W-1:0]               TXC_META_BYTE_COUNT,
     output [`SIG_TAG_W-1:0]                   TXC_META_TAG,
     output [`SIG_REQID_W-1:0]                 TXC_META_REQUESTER_ID,
     output [`SIG_TC_W-1:0]                    TXC_META_TC,
     output [`SIG_ATTR_W-1:0]                  TXC_META_ATTR,
     output                                    TXC_META_EP,
     input                                     TXC_META_READY,

     // Interface: Channel - WR
     output [31:0]                             CHNL_REQ_DATA, 

     output [C_NUM_CHNL-1:0]                   CHNL_SGRX_LEN_VALID,
     output [C_NUM_CHNL-1:0]                   CHNL_SGRX_ADDRLO_VALID,
     output [C_NUM_CHNL-1:0]                   CHNL_SGRX_ADDRHI_VALID,
     output [C_NUM_CHNL-1:0]                   CHNL_SGTX_LEN_VALID,
     output [C_NUM_CHNL-1:0]                   CHNL_SGTX_ADDRLO_VALID,
     output [C_NUM_CHNL-1:0]                   CHNL_SGTX_ADDRHI_VALID,
     output [C_NUM_CHNL-1:0]                   CHNL_RX_LEN_VALID,
     output [C_NUM_CHNL-1:0]                   CHNL_RX_OFFLAST_VALID,
    
     // Interface: Channel - RD
     input [(`SIG_TXRLEN_W*C_NUM_CHNL)-1:0]    CHNL_TX_REQLEN,
     input [(`SIG_OFFLAST_W*C_NUM_CHNL)-1:0]   CHNL_TX_OFFLAST,
     input [(`SIG_TXDONELEN_W*C_NUM_CHNL)-1:0] CHNL_TX_DONELEN,
     input [(`SIG_RXDONELEN_W*C_NUM_CHNL)-1:0] CHNL_RX_DONELEN,
     input [`SIG_CORESETTINGS_W-1:0]           CORE_SETTINGS, 
     output [C_NUM_CHNL-1:0]                   CHNL_TX_LEN_READY,
     output [C_NUM_CHNL-1:0]                   CHNL_TX_OFFLAST_READY,
     output                                    CORE_SETTINGS_READY,
     output [C_NUM_VECTORS-1:0]                INTR_VECTOR_READY,
     output [C_NUM_CHNL-1:0]                   CHNL_TX_DONE_READY,
     output [C_NUM_CHNL-1:0]                   CHNL_RX_DONE_READY,
     output                                    CHNL_NAME_READY,

     // Interface: Interrupt Vectors
     input [C_NUM_VECTORS*C_VECTOR_WIDTH-1:0]  INTR_VECTOR
     );

    localparam C_ADDR_RANGE = 256;
    localparam C_ARRAY_LENGTH = (32*C_ADDR_RANGE)/C_PCI_DATA_WIDTH;
    localparam C_NAME_WIDTH = 32;
    localparam C_FIELDS_WIDTH = 4;
    localparam C_OUTPUT_STAGES = C_PIPELINE_OUTPUT > 0 ? 1:0;
    localparam C_INPUT_STAGES = C_PIPELINE_INPUT > 0 ? 1:0;
    localparam C_TXC_REGISTER_WIDTH = C_PCI_DATA_WIDTH + 2*(1 + clog2(C_PCI_DATA_WIDTH/32) + `SIG_FBE_W) + `SIG_LOWADDR_W + `SIG_TYPE_W + `SIG_LEN_W + `SIG_BYTECNT_W + `SIG_TAG_W + `SIG_REQID_W + `SIG_TC_W + `SIG_ATTR_W + 1;
    localparam C_RXR_REGISTER_WIDTH = C_PCI_DATA_WIDTH + 2*(1 + clog2(C_PCI_DATA_WIDTH/32) + `SIG_FBE_W) + `SIG_ADDR_W + `SIG_TYPE_W + `SIG_LEN_W + `SIG_TAG_W + `SIG_REQID_W + `SIG_TC_W + `SIG_ATTR_W;
    
    // The Mem/IO read/write address space should be at least 8 bits wide. This 
    // means we'll need at least 10 bits of BAR 0, at least 1024 bytes. The bottom
    // two bits must always be zero (i.e. all addresses are 4 byte word aligned).
    // The Mem/IO read/write address space is partitioned as illustrated below.
    // {CHANNEL_NUM} {DATA_OFFSETS} {ZERO}
    // ------4-------------4-----------2--
    // The lower 2 bits are always zero. The middle 4 bits are used according to
    // the listing below. The top 4 bits differentiate between channels for values
    // defined in the table below.
    // 0000 = Length of SG buffer for RX transaction                        (Write only)
    // 0001 = PC low address of SG buffer for RX transaction                (Write only)
    // 0010 = PC high address of SG buffer for RX transaction               (Write only)
    // 0011 = Transfer length for RX transaction                            (Write only)
    // 0100 = Offset/Last for RX transaction                                (Write only)
    // 0101 = Length of SG buffer for TX transaction                        (Write only)
    // 0110 = PC low address of SG buffer for TX transaction                (Write only)
    // 0111 = PC high address of SG buffer for TX transaction               (Write only)
    // 1000 = Transfer length for TX transaction                            (Read only) (ACK'd on read)
    // 1001 = Offset/Last for TX transaction                                (Read only)
    // 1010 = Link rate, link width, bus master enabled, number of channels (Read only)
    // 1011 = Interrupt vector 1                                            (Read only) (Reset on read)
    // 1100 = Interrupt vector 2                                            (Read only) (Reset on read)
    // 1101 = Transferred length for RX transaction                         (Read only) (ACK'd on read)
    // 1110 = Transferred length for TX transaction                         (Read only) (ACK'd on read)
    // 1111 = Name of FPGA                                                  (Read only)

    wire [31:0]                                __wRdMemory[C_ADDR_RANGE-1:0];
    wire [32*C_ADDR_RANGE-1:0]                 _wRdMemory;
    wire [C_PCI_DATA_WIDTH-1:0]                wRdMemory[C_ARRAY_LENGTH-1:0];

    wire [C_PCI_DATA_WIDTH-1:0]                wRxrData;
    wire                                       wRxrDataValid;
    wire                                       wRxrDataStartFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]     wRxrDataStartOffset;
    wire [`SIG_FBE_W-1:0]                      wRxrMetaFdwbe;
    wire                                       wRxrDataEndFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]     wRxrDataEndOffset;
    wire [`SIG_LBE_W-1:0]                      wRxrMetaLdwbe;
    wire [`SIG_TC_W-1:0]                       wRxrMetaTc;
    wire [`SIG_ATTR_W-1:0]                     wRxrMetaAttr;
    wire [`SIG_TAG_W-1:0]                      wRxrMetaTag;
    wire [`SIG_TYPE_W-1:0]                     wRxrMetaType;
    wire [`SIG_ADDR_W-1:0]                     wRxrMetaAddr;
    wire [`SIG_REQID_W-1:0]                    wRxrMetaRequesterId;
    wire [`SIG_LEN_W-1:0]                      wRxrMetaLength;
    
    wire [C_PCI_DATA_WIDTH-1:0]                wTxcData;
    wire                                       wTxcDataValid;
    wire                                       wTxcDataStartFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]     wTxcDataStartOffset;
    wire [`SIG_FBE_W-1:0]                      wTxcMetaFdwbe;
    wire                                       wTxcDataEndFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]     wTxcDataEndOffset;
    wire [`SIG_LBE_W-1:0]                      wTxcMetaLdwbe;
    wire [`SIG_LOWADDR_W-1:0]                  wTxcMetaAddr;
    wire [`SIG_TYPE_W-1:0]                     wTxcMetaType;
    wire [`SIG_LEN_W-1:0]                      wTxcMetaLength;
    wire [`SIG_BYTECNT_W-1:0]                  wTxcMetaByteCount;
    wire [`SIG_TAG_W-1:0]                      wTxcMetaTag;
    wire [`SIG_REQID_W-1:0]                    wTxcMetaRequesterId;
    wire [`SIG_TC_W-1:0]                       wTxcMetaTc;
    wire [`SIG_ATTR_W-1:0]                     wTxcMetaAttr;
    wire                                       wTxcMetaEp;
    wire                                       wTxcDataReady;

    wire [clog2s(C_NUM_CHNL)-1:0]              wReqChnl;
    wire [C_FIELDS_WIDTH-1:0]                  wReqField;
    wire [(1<<C_FIELDS_WIDTH)-1:0]             wReqFieldDemux;

    wire [C_NUM_CHNL-1:0]                      wChnlSgrxLenValid;
    wire [C_NUM_CHNL-1:0]                      wChnlSgrxAddrLowValid;
    wire [C_NUM_CHNL-1:0]                      wChnlSgrxAddrHiValid;
    wire [C_NUM_CHNL-1:0]                      wChnlSgtxLenValid;
    wire [C_NUM_CHNL-1:0]                      wChnlSgtxAddrLowValid;
    wire [C_NUM_CHNL-1:0]                      wChnlSgtxAddrHiValid;
    wire [C_NUM_CHNL-1:0]                      wChnlRxLenValid;
    wire [C_NUM_CHNL-1:0]                      wChnlRxOfflastValid;

    wire [C_NUM_CHNL-1:0]                      wChnlTxLenReady;
    wire [C_NUM_CHNL-1:0]                      wChnlTxOfflastReady;
    wire [`SIG_CORESETTINGS_W-1:0]             wCoreSettings;
    wire                                       wCoreSettingsReady;
    wire [C_NUM_VECTORS - 1 : 0]               wInterVectorReady;
    wire [C_NUM_CHNL-1:0]                      wChnlTxDoneReady;
    wire [C_NUM_CHNL-1:0]                      wChnlRxDoneReady;
    wire                                       wChnlNameReady;

    wire [31:0]                                wChnlReqData;

    genvar                                     addr;
    genvar                                     channel;
    genvar                                     vector;

    assign wReqChnl = wRxrMetaAddr[(C_FIELDS_WIDTH + 2) +:clog2s(C_NUM_CHNL)];
    assign wReqField = wRxrMetaAddr[2 +: C_FIELDS_WIDTH];
    assign wChnlReqData[31:0] = wRxrData[32*wRxrDataStartOffset +: 32];
    
    /* verilator lint_off WIDTH */
    assign __wRdMemory[`ADDR_CORESETTINGS] = CORE_SETTINGS;
    assign __wRdMemory[`ADDR_INTR_VECTOR_0] = INTR_VECTOR[C_VECTOR_WIDTH*0 +: C_VECTOR_WIDTH];
    assign __wRdMemory[`ADDR_INTR_VECTOR_1] = INTR_VECTOR[C_VECTOR_WIDTH*1 +: C_VECTOR_WIDTH];
    assign __wRdMemory[`ADDR_FPGA_NAME] = {"    ",C_FPGA_NAME};
    /* verilator lint_on WIDTH */
    assign wTxcData = {{(C_PCI_DATA_WIDTH-32){1'b0}},__wRdMemory[{wReqChnl,wReqField}]};
    assign wTxcDataValid = wRxrDataValid & wRxrMetaType == `TRLS_REQ_RD;
    assign wTxcDataStartFlag = 1;
    assign wTxcDataStartOffset = 0;
    assign wTxcMetaFdwbe = 4'b1111;
    assign wTxcDataEndFlag = 1;
    assign wTxcDataEndOffset = 0;
    assign wTxcMetaLdwbe = 4'b0000;
    assign wTxcMetaAddr = wRxrMetaAddr[`SIG_LOWADDR_W-1:0];
    assign wTxcMetaType = `TRLS_CPL_WD;
    assign wTxcMetaLength = 1;
    assign wTxcMetaByteCount = 4;
    assign wTxcMetaTag = wRxrMetaTag;
    assign wTxcMetaRequesterId = wRxrMetaRequesterId;
    assign wTxcMetaTc = wRxrMetaTc;
    assign wTxcMetaAttr = wRxrMetaAttr;
    assign wTxcMetaEp = 0;

    generate            
        for(channel = 0; channel < C_NUM_CHNL ; channel = channel + 1) begin : gen__wRdMemory
            
            assign __wRdMemory[{channel[27:0] , `ADDR_TX_LEN}] = CHNL_TX_REQLEN[32*channel +: 32];
            assign __wRdMemory[{channel[27:0] , `ADDR_TX_OFFLAST}] = CHNL_TX_OFFLAST[32*channel +: 32];
            assign __wRdMemory[{channel[27:0] , `ADDR_RX_LEN_XFERD}] = CHNL_RX_DONELEN[32*channel +: 32];            
            assign __wRdMemory[{channel[27:0] , `ADDR_TX_LEN_XFERD}] = CHNL_TX_DONELEN[32*channel +: 32];
        end
        for(addr = 0 ; addr < C_ADDR_RANGE ; addr = addr + 1) begin : gen_wRdMemory
            assign _wRdMemory[(addr*32) +: 32] = __wRdMemory[addr];
        end
        for(addr = 0 ; addr < C_ARRAY_LENGTH ; addr = addr + 1) begin : genwRdMemory
            assign wRdMemory[addr]  = _wRdMemory[(addr*C_PCI_DATA_WIDTH) +: C_PCI_DATA_WIDTH];
        end
    endgenerate

    assign wChnlNameReady = wReqFieldDemux[`ADDR_FPGA_NAME];
    assign wCoreSettingsReady = wReqFieldDemux[`ADDR_CORESETTINGS];
    assign wInterVectorReady[0] = wReqFieldDemux[`ADDR_INTR_VECTOR_0];
    assign wInterVectorReady[1] = wReqFieldDemux[`ADDR_INTR_VECTOR_1];
    assign TXC_META_VALID = TXC_DATA_VALID;
    pipeline
        #(
          // Parameters
          .C_DEPTH                      (C_INPUT_STAGES),
          .C_WIDTH                      (C_RXR_REGISTER_WIDTH),
          .C_USE_MEMORY                 (0)
          /*AUTOINSTPARAM*/)
    rxr_input_register
        (// Outputs
         .WR_DATA_READY                 (),// Unconnected
         .RD_DATA                       ({wRxrData,
                                          wRxrDataStartFlag, wRxrDataStartOffset, wRxrMetaFdwbe,
                                          wRxrDataEndFlag, wRxrDataEndOffset, wRxrMetaLdwbe,
                                          wRxrMetaAddr, wRxrMetaType, wRxrMetaLength, 
                                          wRxrMetaTag, wRxrMetaRequesterId, wRxrMetaTc, wRxrMetaAttr}),
         .RD_DATA_VALID                 (wRxrDataValid),
         // Inputs
         .WR_DATA                       ({RXR_DATA, 
                                          RXR_DATA_START_FLAG, RXR_DATA_START_OFFSET, RXR_META_FDWBE,
                                          RXR_DATA_END_FLAG, RXR_DATA_END_OFFSET, RXR_META_LDWBE,
                                          RXR_META_ADDR, RXR_META_TYPE, RXR_META_LENGTH,
                                          RXR_META_TAG, RXR_META_REQUESTER_ID, RXR_META_TC, RXR_META_ATTR}),
         .WR_DATA_VALID                 (RXR_DATA_VALID),
         .RD_DATA_READY                 (1),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));

    demux
        #(
          // Parameters
          .C_OUTPUTS                    (1<<C_FIELDS_WIDTH),
          .C_WIDTH                      (1)
          /*AUTOINSTPARAM*/)
    field_demux
        (
         // Outputs
         .RD_DATA                      (wReqFieldDemux),
         // Inputs
         .WR_DATA                      (wRxrDataValid),
         .WR_SEL                       (wReqField)
         /*AUTOINST*/);

    demux
        #(
          // Parameters
          .C_OUTPUTS                    (C_NUM_CHNL),
          .C_WIDTH                      (1)
          /*AUTOINSTPARAM*/)
    tx_len_ready_demux
        (
         // Outputs
         .RD_DATA                      (wChnlTxLenReady),
         // Inputs
         .WR_DATA                      (wReqFieldDemux[`ADDR_TX_LEN]),
         .WR_SEL                       (wReqChnl)
         /*AUTOINST*/);

    demux
        #(
          // Parameters
          .C_OUTPUTS                    (C_NUM_CHNL),
          .C_WIDTH                      (1)
          /*AUTOINSTPARAM*/)
    tx_offlast_ready_demux
        (
         // Outputs
         .RD_DATA                      (wChnlTxOfflastReady),
         // Inputs
         .WR_DATA                      (wReqFieldDemux[`ADDR_TX_OFFLAST]),
         .WR_SEL                       (wReqChnl)
         /*AUTOINST*/);
    
    demux
        #(
          // Parameters
          .C_OUTPUTS                    (C_NUM_CHNL),
          .C_WIDTH                      (1)
          /*AUTOINSTPARAM*/)
    rxdone_demux
        (
         // Outputs
         .RD_DATA                      (wChnlRxDoneReady),
         // Inputs
         .WR_DATA                      (wReqFieldDemux[`ADDR_RX_LEN_XFERD]),
         .WR_SEL                       (wReqChnl)
         /*AUTOINST*/);

    demux
        #(
          // Parameters
          .C_OUTPUTS                    (C_NUM_CHNL),
          .C_WIDTH                      (1)
          /*AUTOINSTPARAM*/)
    txdone_demux
        (
         // Outputs
         .RD_DATA                      (wChnlTxDoneReady),
         // Inputs
         .WR_DATA                      (wReqFieldDemux[`ADDR_TX_LEN_XFERD]),
         .WR_SEL                       (wReqChnl)
         /*AUTOINST*/);
    demux
        #(
          // Parameters
          .C_OUTPUTS                    (C_NUM_CHNL),
          .C_WIDTH                      (1)
          /*AUTOINSTPARAM*/)
    rx_len_demux
        (
         // Outputs
         .RD_DATA                      (wChnlRxLenValid),
         // Inputs
         .WR_DATA                      (wReqFieldDemux[`ADDR_RX_LEN]),
         .WR_SEL                       (wReqChnl)
         /*AUTOINST*/);

    demux
        #(
          // Parameters
          .C_OUTPUTS                    (C_NUM_CHNL),
          .C_WIDTH                      (1)
          /*AUTOINSTPARAM*/)
    rx_offlast_demux
        (
         // Outputs
         .RD_DATA                      (wChnlRxOfflastValid),
         // Inputs
         .WR_DATA                      (wReqFieldDemux[`ADDR_RX_OFFLAST]),
         .WR_SEL                       (wReqChnl)
         /*AUTOINST*/);

    demux
        #(
          // Parameters
          .C_OUTPUTS                    (C_NUM_CHNL),
          .C_WIDTH                      (1)
          /*AUTOINSTPARAM*/)
    sgtx_addrhi_demux
        (
         // Outputs
         .RD_DATA                      (wChnlSgtxAddrHiValid),
         // Inputs
         .WR_DATA                      (wReqFieldDemux[`ADDR_SGTX_ADDRHI]),
         .WR_SEL                       (wReqChnl)
         /*AUTOINST*/);

    demux
        #(
          // Parameters
          .C_OUTPUTS                    (C_NUM_CHNL),
          .C_WIDTH                      (1)
          /*AUTOINSTPARAM*/)
    sgtx_addrlo_demux
        (
         // Outputs
         .RD_DATA                      (wChnlSgtxAddrLowValid),
         // Inputs
         .WR_DATA                      (wReqFieldDemux[`ADDR_SGTX_ADDRLO]),
         .WR_SEL                       (wReqChnl)
         /*AUTOINST*/);

    demux
        #(
          // Parameters
          .C_OUTPUTS                    (C_NUM_CHNL),
          .C_WIDTH                      (1)
          /*AUTOINSTPARAM*/)
    sgtxlen_demux
        (
         // Outputs
         .RD_DATA                      (wChnlSgtxLenValid),
         // Inputs
         .WR_DATA                      (wReqFieldDemux[`ADDR_SGTX_LEN]),
         .WR_SEL                       (wReqChnl)
         /*AUTOINST*/);

    demux
        #(
          // Parameters
          .C_OUTPUTS                    (C_NUM_CHNL),
          .C_WIDTH                      (1)
          /*AUTOINSTPARAM*/)
    sgrx_addrhi_demux
        (
         // Outputs
         .RD_DATA                      (wChnlSgrxAddrHiValid),
         // Inputs
         .WR_DATA                      (wReqFieldDemux[`ADDR_SGRX_ADDRHI]),
         .WR_SEL                       (wReqChnl)
         /*AUTOINST*/);

    demux
        #(
          // Parameters
          .C_OUTPUTS                    (C_NUM_CHNL),
          .C_WIDTH                      (1)
          /*AUTOINSTPARAM*/)
    sgrx_addrlo_demux
        (
         // Outputs
         .RD_DATA                      (wChnlSgrxAddrLowValid),
         // Inputs
         .WR_DATA                      (wReqFieldDemux[`ADDR_SGRX_ADDRLO]),
         .WR_SEL                       (wReqChnl)
         /*AUTOINST*/);

    demux
        #(
          // Parameters
          .C_OUTPUTS                    (C_NUM_CHNL),
          .C_WIDTH                      (1)
          /*AUTOINSTPARAM*/)
    sgrxlen_demux
        (
         // Outputs
         .RD_DATA                      (wChnlSgrxLenValid),
         // Inputs
         .WR_DATA                      (wReqFieldDemux[`ADDR_SGRX_LEN]),
         .WR_SEL                       (wReqChnl)
         /*AUTOINST*/);

    pipeline
        #(
          // Parameters
          .C_DEPTH                      (C_OUTPUT_STAGES),
          .C_WIDTH                      (12*C_NUM_CHNL + C_NUM_VECTORS + 2 + 32),
          .C_USE_MEMORY                 (0)
          /*AUTOINSTPARAM*/)
    chnl_output_register
        (
         // Outputs
         .WR_DATA_READY                 (),// Unconnected
         .RD_DATA                       ({CHNL_TX_LEN_READY, CHNL_TX_OFFLAST_READY, CORE_SETTINGS_READY,
                                          INTR_VECTOR_READY, CHNL_TX_DONE_READY, CHNL_RX_DONE_READY, 
                                          CHNL_NAME_READY,CHNL_SGRX_LEN_VALID, CHNL_SGRX_ADDRLO_VALID, CHNL_SGRX_ADDRHI_VALID,
                                          CHNL_SGTX_LEN_VALID, CHNL_SGTX_ADDRLO_VALID, CHNL_SGTX_ADDRHI_VALID,
                                          CHNL_RX_LEN_VALID, CHNL_RX_OFFLAST_VALID,
                                          CHNL_REQ_DATA}),
         .RD_DATA_VALID                 (),
         // Inputs
         .WR_DATA                       ({wChnlTxLenReady, wChnlTxOfflastReady, wCoreSettingsReady,
                                          wInterVectorReady, wChnlTxDoneReady, wChnlRxDoneReady, 
                                          wChnlNameReady,wChnlSgrxLenValid,wChnlSgrxAddrLowValid,wChnlSgrxAddrHiValid,
                                          wChnlSgtxLenValid,wChnlSgtxAddrLowValid,wChnlSgtxAddrHiValid,
                                          wChnlRxLenValid,wChnlRxOfflastValid,
                                          wChnlReqData}),
         .WR_DATA_VALID                 (1),
         .RD_DATA_READY                 (1),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));

    pipeline
        #(
          // Parameters
          .C_DEPTH                      (C_OUTPUT_STAGES),
          .C_WIDTH                      (C_TXC_REGISTER_WIDTH),
          .C_USE_MEMORY                 (0)
          /*AUTOINSTPARAM*/)
    txc_output_register
        (
         // Outputs
         .WR_DATA_READY                 (),// Unconnected
         .RD_DATA                       ({TXC_DATA, 
                                          TXC_DATA_START_FLAG, TXC_DATA_START_OFFSET, TXC_META_FDWBE,
                                          TXC_DATA_END_FLAG, TXC_DATA_END_OFFSET, TXC_META_LDWBE,
                                          TXC_META_ADDR, TXC_META_TYPE, TXC_META_LENGTH, TXC_META_BYTE_COUNT,
                                          TXC_META_TAG, TXC_META_REQUESTER_ID, TXC_META_TC, TXC_META_ATTR,
                                          TXC_META_EP}),
         .RD_DATA_VALID                 (TXC_DATA_VALID),
         // Inputs
         .WR_DATA                       ({wTxcData, 
                                          wTxcDataStartFlag, wTxcDataStartOffset, wTxcMetaFdwbe,
                                          wTxcDataEndFlag, wTxcDataEndOffset, wTxcMetaLdwbe,
                                          wTxcMetaAddr, wTxcMetaType, wTxcMetaLength, wTxcMetaByteCount,
                                          wTxcMetaTag, wTxcMetaRequesterId, wTxcMetaTc, wTxcMetaAttr,
                                          wTxcMetaEp}),
         .WR_DATA_VALID                 (wTxcDataValid),
         .RD_DATA_READY                 (TXC_DATA_READY),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));
endmodule
// Local Variables:
// verilog-library-directories:("." "../common/")
// End:
