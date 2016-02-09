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
`timescale 1ns/1ns
module riffa
    #(parameter C_PCI_DATA_WIDTH = 128,
      parameter C_NUM_CHNL = 12,
      parameter C_MAX_READ_REQ_BYTES = 512, // Max size of read requests (in bytes)
      parameter C_TAG_WIDTH = 5, // Number of outstanding requests 
      parameter C_VENDOR = "ALTERA",
      parameter C_FPGA_NAME = "FPGA", // TODO: Give each channel a unique name
      parameter C_FPGA_ID = 0,// A value from 0 to 255 uniquely identifying this RIFFA design
      parameter C_DEPTH_PACKETS = 10)
    (input                                      CLK,
     input                                      RST_BUS,
     output                                     RST_OUT,
     input                                      DONE_TXC_RST,
     input                                      DONE_TXR_RST,
    
     // Interface: RXC Engine
     input [C_PCI_DATA_WIDTH-1:0]               RXC_DATA,
     input                                      RXC_DATA_VALID,
     input [(C_PCI_DATA_WIDTH/32)-1:0]          RXC_DATA_WORD_ENABLE,
     input                                      RXC_DATA_START_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]    RXC_DATA_START_OFFSET,
     input                                      RXC_DATA_END_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]    RXC_DATA_END_OFFSET,

     input [`SIG_LBE_W-1:0]                     RXC_META_LDWBE,
     input [`SIG_FBE_W-1:0]                     RXC_META_FDWBE,
     input [`SIG_TAG_W-1:0]                     RXC_META_TAG,
     input [`SIG_LOWADDR_W-1:0]                 RXC_META_ADDR,
     input [`SIG_TYPE_W-1:0]                    RXC_META_TYPE,
     input [`SIG_LEN_W-1:0]                     RXC_META_LENGTH,
     input [`SIG_BYTECNT_W-1:0]                 RXC_META_BYTES_REMAINING,
     input [`SIG_CPLID_W-1:0]                   RXC_META_COMPLETER_ID,
     input                                      RXC_META_EP,

     // Interface: RXR Engine
     input [C_PCI_DATA_WIDTH-1:0]               RXR_DATA,
     input                                      RXR_DATA_VALID,
     input [(C_PCI_DATA_WIDTH/32)-1:0]          RXR_DATA_WORD_ENABLE,
     input                                      RXR_DATA_START_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]    RXR_DATA_START_OFFSET,
     input                                      RXR_DATA_END_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]    RXR_DATA_END_OFFSET,
    
     input [`SIG_FBE_W-1:0]                     RXR_META_FDWBE,
     input [`SIG_LBE_W-1:0]                     RXR_META_LDWBE,
     input [`SIG_TC_W-1:0]                      RXR_META_TC,
     input [`SIG_ATTR_W-1:0]                    RXR_META_ATTR,
     input [`SIG_TAG_W-1:0]                     RXR_META_TAG,
     input [`SIG_TYPE_W-1:0]                    RXR_META_TYPE,
     input [`SIG_ADDR_W-1:0]                    RXR_META_ADDR,
     input [`SIG_BARDECODE_W-1:0]               RXR_META_BAR_DECODED,
     input [`SIG_REQID_W-1:0]                   RXR_META_REQUESTER_ID,
     input [`SIG_LEN_W-1:0]                     RXR_META_LENGTH,
     input                                      RXR_META_EP,
    
     // Interface: TXC Engine
     output [C_PCI_DATA_WIDTH-1:0]              TXC_DATA,
     output                                     TXC_DATA_VALID,
     output                                     TXC_DATA_START_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0]   TXC_DATA_START_OFFSET,
     output                                     TXC_DATA_END_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0]   TXC_DATA_END_OFFSET,
     input                                      TXC_DATA_READY,

     output                                     TXC_META_VALID,
     output [`SIG_FBE_W-1:0]                    TXC_META_FDWBE,
     output [`SIG_LBE_W-1:0]                    TXC_META_LDWBE,
     output [`SIG_LOWADDR_W-1:0]                TXC_META_ADDR,
     output [`SIG_TYPE_W-1:0]                   TXC_META_TYPE,
     output [`SIG_LEN_W-1:0]                    TXC_META_LENGTH,
     output [`SIG_BYTECNT_W-1:0]                TXC_META_BYTE_COUNT,
     output [`SIG_TAG_W-1:0]                    TXC_META_TAG,
     output [`SIG_REQID_W-1:0]                  TXC_META_REQUESTER_ID,
     output [`SIG_TC_W-1:0]                     TXC_META_TC,
     output [`SIG_ATTR_W-1:0]                   TXC_META_ATTR,
     output                                     TXC_META_EP,
     input                                      TXC_META_READY,
     input                                      TXC_SENT,

     // Interface: TXR Engine
     output                                     TXR_DATA_VALID,
     output [C_PCI_DATA_WIDTH-1:0]              TXR_DATA,
     output                                     TXR_DATA_START_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0]   TXR_DATA_START_OFFSET,
     output                                     TXR_DATA_END_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0]   TXR_DATA_END_OFFSET,
     input                                      TXR_DATA_READY,

     output                                     TXR_META_VALID,
     output [`SIG_FBE_W-1:0]                    TXR_META_FDWBE, 
     output [`SIG_LBE_W-1:0]                    TXR_META_LDWBE,
     output [`SIG_ADDR_W-1:0]                   TXR_META_ADDR,
     output [`SIG_LEN_W-1:0]                    TXR_META_LENGTH,
     output [`SIG_TAG_W-1:0]                    TXR_META_TAG,
     output [`SIG_TC_W-1:0]                     TXR_META_TC,
     output [`SIG_ATTR_W-1:0]                   TXR_META_ATTR,
     output [`SIG_TYPE_W-1:0]                   TXR_META_TYPE,
     output                                     TXR_META_EP,
     input                                      TXR_META_READY,
     input                                      TXR_SENT,

     // Interface: Configuration 
     input [`SIG_CPLID_W-1:0]                   CONFIG_COMPLETER_ID,
     input                                      CONFIG_BUS_MASTER_ENABLE, 
     input [`SIG_LINKWIDTH_W-1:0]               CONFIG_LINK_WIDTH,
     input [`SIG_LINKRATE_W-1:0]                CONFIG_LINK_RATE,
     input [`SIG_MAXREAD_W-1:0]                 CONFIG_MAX_READ_REQUEST_SIZE, 
     input [`SIG_MAXPAYLOAD_W-1:0]              CONFIG_MAX_PAYLOAD_SIZE,
     input [`SIG_FC_CPLD_W-1:0]                 CONFIG_MAX_CPL_DATA, // Receive credit limit for data
     input [`SIG_FC_CPLH_W-1:0]                 CONFIG_MAX_CPL_HDR, // Receive credit limit for headers
     input                                      CONFIG_INTERRUPT_MSIENABLE,
     input                                      CONFIG_CPL_BOUNDARY_SEL,
     // Interrupt Request
     input                                      INTR_MSI_RDY, // High when interrupt is able to be sent
     output                                     INTR_MSI_REQUEST, // High to request interrupt, when both INTR_MSI_RDY and INTR_MSI_RE

     input [C_NUM_CHNL-1:0]                     CHNL_RX_CLK, 
     output [C_NUM_CHNL-1:0]                    CHNL_RX, 
     input [C_NUM_CHNL-1:0]                     CHNL_RX_ACK, 
     output [C_NUM_CHNL-1:0]                    CHNL_RX_LAST, 
     output [(C_NUM_CHNL*32)-1:0]               CHNL_RX_LEN, 
     output [(C_NUM_CHNL*31)-1:0]               CHNL_RX_OFF, 
     output [(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0] CHNL_RX_DATA, 
     output [C_NUM_CHNL-1:0]                    CHNL_RX_DATA_VALID, 
     input [C_NUM_CHNL-1:0]                     CHNL_RX_DATA_REN,
    
     input [C_NUM_CHNL-1:0]                     CHNL_TX_CLK, 
     input [C_NUM_CHNL-1:0]                     CHNL_TX, 
     output [C_NUM_CHNL-1:0]                    CHNL_TX_ACK,
     input [C_NUM_CHNL-1:0]                     CHNL_TX_LAST, 
     input [(C_NUM_CHNL*32)-1:0]                CHNL_TX_LEN, 
     input [(C_NUM_CHNL*31)-1:0]                CHNL_TX_OFF, 
     input [(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0]  CHNL_TX_DATA, 
     input [C_NUM_CHNL-1:0]                     CHNL_TX_DATA_VALID, 
     output [C_NUM_CHNL-1:0]                    CHNL_TX_DATA_REN

     );
    localparam C_MAX_READ_REQ = clog2s(C_MAX_READ_REQ_BYTES)-7;    // Max read: 000=128B; 001=256B; 010=512B; 011=1024B; 100=2048B; 101=4096B
    localparam C_NUM_CHNL_WIDTH = clog2s(C_NUM_CHNL);
    localparam C_PCI_DATA_WORD_WIDTH = clog2s((C_PCI_DATA_WIDTH/32)+1);
    localparam C_NUM_VECTORS = 2;
    localparam C_VECTOR_WIDTH = 32;

    // Interface: Reorder Buffer Output
    wire [(C_NUM_CHNL*C_PCI_DATA_WORD_WIDTH)-1:0] wRxEngMainDataEn; // Start offset and end offset
    wire [C_PCI_DATA_WIDTH-1:0]                   wRxEngData;
    wire [C_NUM_CHNL-1:0]                         wRxEngMainDone;
    wire [C_NUM_CHNL-1:0]                         wRxEngMainErr;

    // Interface: Reorder Buffer to SG RX engines    
    wire [(C_NUM_CHNL*C_PCI_DATA_WORD_WIDTH)-1:0] wRxEngSgRxDataEn;
    wire [C_NUM_CHNL-1:0]                         wRxEngSgRxDone;
    wire [C_NUM_CHNL-1:0]                         wRxEngSgRxErr;
    
    // Interface: Reorder Buffer to SG TX engines    
    wire [(C_NUM_CHNL*C_PCI_DATA_WORD_WIDTH)-1:0] wRxEngSgTxDataEn;
    wire [C_NUM_CHNL-1:0]                         wRxEngSgTxDone;
    wire [C_NUM_CHNL-1:0]                         wRxEngSgTxErr;

    // Interface: Channel TX Write
    wire [C_NUM_CHNL-1:0]                         wTxEngWrReq;
    wire [(C_NUM_CHNL*`SIG_ADDR_W)-1:0]           wTxEngWrAddr;
    wire [(C_NUM_CHNL*`SIG_LEN_W)-1:0]            wTxEngWrLen;
    wire [(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0]      wTxEngWrData;
    wire [C_NUM_CHNL-1:0]                         wTxEngWrDataRen;
    wire [C_NUM_CHNL-1:0]                         wTxEngWrAck; 
    wire [C_NUM_CHNL-1:0]                         wTxEngWrSent;

    // Interface: Channel TX Read
    wire [C_NUM_CHNL-1:0]                         wTxEngRdReq;
    wire [(C_NUM_CHNL*2)-1:0]                     wTxEngRdSgChnl;
    wire [(C_NUM_CHNL*`SIG_ADDR_W)-1:0]           wTxEngRdAddr;
    wire [(C_NUM_CHNL*`SIG_LEN_W)-1:0]            wTxEngRdLen;
    wire [C_NUM_CHNL-1:0]                         wTxEngRdAck;

    // Interface: Channel Interrupts
    wire [C_NUM_CHNL-1:0]                         wChnlSgRxBufRecvd;
    wire [C_NUM_CHNL-1:0]                         wChnlRxDone;
    wire [C_NUM_CHNL-1:0]                         wChnlTxRequest;
    wire [C_NUM_CHNL-1:0]                         wChnlTxDone;
    wire [C_NUM_CHNL-1:0]                         wChnlSgTxBufRecvd;

    wire                                          wInternalTagValid;
    wire [5:0]                                    wInternalTag;
    wire                                          wExternalTagValid;
    wire [C_TAG_WIDTH-1:0]                        wExternalTag;

    // Interface: Channel - PIO Read
    wire [C_NUM_CHNL-1:0]                         wChnlTxLenReady;
    wire [(`SIG_TXRLEN_W*C_NUM_CHNL)-1:0]         wChnlTxReqLen;
    wire [C_NUM_CHNL-1:0]                         wChnlTxOfflastReady;
    wire [(`SIG_OFFLAST_W*C_NUM_CHNL)-1:0]        wChnlTxOfflast;
    wire                                          wCoreSettingsReady;
    wire [`SIG_CORESETTINGS_W-1:0]                wCoreSettings;
    wire [C_NUM_VECTORS-1:0]                      wIntrVectorReady;
    wire [C_NUM_VECTORS*C_VECTOR_WIDTH-1:0]       wIntrVector;
    wire [C_NUM_CHNL-1:0]                         wChnlTxDoneReady;
    wire [(`SIG_TXDONELEN_W*C_NUM_CHNL)-1:0]      wChnlTxDoneLen;
    wire [C_NUM_CHNL-1:0]                         wChnlRxDoneReady;
    wire [(`SIG_RXDONELEN_W*C_NUM_CHNL)-1:0]      wChnlRxDoneLen;
    wire                                          wChnlNameReady;

    // Interface: Channel - PIO Write
    wire [31:0]                                   wChnlReqData; 
    wire [C_NUM_CHNL-1:0]                         wChnlSgRxLenValid;
    wire [C_NUM_CHNL-1:0]                         wChnlSgRxAddrLoValid;
    wire [C_NUM_CHNL-1:0]                         wChnlSgRxAddrHiValid;
    wire [C_NUM_CHNL-1:0]                         wChnlSgTxLenValid;
    wire [C_NUM_CHNL-1:0]                         wChnlSgTxAddrLoValid;
    wire [C_NUM_CHNL-1:0]                         wChnlSgTxAddrHiValid;
    wire [C_NUM_CHNL-1:0]                         wChnlRxLenValid;
    wire [C_NUM_CHNL-1:0]                         wChnlRxOfflastValid;

    // Interface: TXC Engine
    wire [C_PCI_DATA_WIDTH-1:0]                   _wTxcData, wTxcData;
    wire                                          _wTxcDataValid, wTxcDataValid;
    wire                                          _wTxcDataStartFlag, wTxcDataStartFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]        _wTxcDataStartOffset, wTxcDataStartOffset;
    wire                                          _wTxcDataEndFlag, wTxcDataEndFlag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]        _wTxcDataEndOffset, wTxcDataEndOffset;
    wire                                          _wTxcDataReady, wTxcDataReady;

    wire                                          _wTxcMetaValid, wTxcMetaValid;
    wire [`SIG_FBE_W-1:0]                         _wTxcMetaFdwbe, wTxcMetaFdwbe;
    wire [`SIG_LBE_W-1:0]                         _wTxcMetaLdwbe, wTxcMetaLdwbe;
    wire [`SIG_LOWADDR_W-1:0]                     _wTxcMetaAddr, wTxcMetaAddr;
    wire [`SIG_TYPE_W-1:0]                        _wTxcMetaType, wTxcMetaType;
    wire [`SIG_LEN_W-1:0]                         _wTxcMetaLength, wTxcMetaLength;
    wire [`SIG_BYTECNT_W-1:0]                     _wTxcMetaByteCount, wTxcMetaByteCount;
    wire [`SIG_TAG_W-1:0]                         _wTxcMetaTag, wTxcMetaTag;
    wire [`SIG_REQID_W-1:0]                       _wTxcMetaRequesterId, wTxcMetaRequesterId;
    wire [`SIG_TC_W-1:0]                          _wTxcMetaTc, wTxcMetaTc;
    wire [`SIG_ATTR_W-1:0]                        _wTxcMetaAttr, wTxcMetaAttr;
    wire                                          _wTxcMetaEp, wTxcMetaEp;
    wire                                          _wTxcMetaReady, wTxcMetaReady;

    wire                                          wRxBufSpaceAvail;
    wire                                          wTxEngRdReqSent;
    wire                                          wRxEngRdComplete;

    wire [31:0]                                   wCPciDataWidth;
    reg [31:0]                                    wCFpgaId;

    reg [4:0]                                     rWideRst;
    reg                                           rRst;

    genvar                                        i;

    
    assign wRxEngRdComplete = RXC_DATA_END_FLAG & RXC_DATA_VALID & 
                              (RXC_META_LENGTH >= RXC_META_BYTES_REMAINING[`SIG_BYTECNT_W-1:2]);// TODO: Retime (if possible)

    assign wCoreSettings = {1'd0, wCFpgaId, wCPciDataWidth[8:5], 
                            CONFIG_MAX_PAYLOAD_SIZE, CONFIG_MAX_READ_REQUEST_SIZE, 
                            CONFIG_LINK_RATE[1:0], CONFIG_LINK_WIDTH, CONFIG_BUS_MASTER_ENABLE, 
                            C_NUM_CHNL[3:0]};


    // Interface: TXC Engine
    assign TXC_DATA = wTxcData;
    assign TXC_DATA_START_FLAG = wTxcDataStartFlag;
    assign TXC_DATA_START_OFFSET = wTxcDataStartOffset;
    assign TXC_DATA_END_FLAG = wTxcDataEndFlag;
    assign TXC_DATA_END_OFFSET = wTxcDataEndOffset;
    assign TXC_DATA_VALID = wTxcDataValid & ~wPendingRst & DONE_TXC_RST;
    assign wTxcDataReady = TXC_DATA_READY & ~wPendingRst & DONE_TXC_RST;

    assign TXC_META_FDWBE = wTxcMetaFdwbe;
    assign TXC_META_LDWBE = wTxcMetaLdwbe;
    assign TXC_META_ADDR = wTxcMetaAddr;
    assign TXC_META_TYPE = wTxcMetaType;
    assign TXC_META_LENGTH = wTxcMetaLength;
    assign TXC_META_BYTE_COUNT = wTxcMetaByteCount;
    assign TXC_META_TAG = wTxcMetaTag; 
    assign TXC_META_REQUESTER_ID = wTxcMetaRequesterId;
    assign TXC_META_TC = wTxcMetaTc;
    assign TXC_META_ATTR = wTxcMetaAttr;
    assign TXC_META_EP = wTxcMetaEp;
    assign TXC_META_VALID = wTxcMetaValid & ~wPendingRst & DONE_TXC_RST;
    assign wTxcMetaReady = TXC_META_READY & ~wPendingRst & DONE_TXC_RST;

    /* Workaround for a bug reported by the NetFPGA group, where the parameter
       C_PCI_DATA_WIDTH cannot be directly assigned to a wire. */

    generate
        if(C_PCI_DATA_WIDTH == 32) begin
            assign wCPciDataWidth = 32;
        end else if (C_PCI_DATA_WIDTH == 64) begin
            assign wCPciDataWidth = 64;
        end else if (C_PCI_DATA_WIDTH == 128) begin
            assign wCPciDataWidth = 128;
        end else if (C_PCI_DATA_WIDTH == 256) begin
            assign wCPciDataWidth = 256;
        end
        
        always @(*) begin
            wCFpgaId = 0;
            if((C_FPGA_ID & 128) != 0) begin
                wCFpgaId[7] = 1;
            end else if ((C_FPGA_ID & 64) != 1) begin
                wCFpgaId[6] = 1;
            end else if ((C_FPGA_ID & 32) != 1) begin
                wCFpgaId[5] = 1;
            end else if ((C_FPGA_ID & 16) != 1) begin
                wCFpgaId[4] = 1;
            end else if ((C_FPGA_ID &  8) != 1) begin
                wCFpgaId[3] = 1;
            end else if ((C_FPGA_ID &  4) != 1) begin
                wCFpgaId[2] = 1;
            end else if ((C_FPGA_ID &  2) != 1) begin
                wCFpgaId[1] = 1;
            end else if ((C_FPGA_ID &  1) != 1) begin
                wCFpgaId[0] = 1;
            end
         end
    endgenerate

    /* The purpose of these two hold modules is to safely reset the TX path and
     still respond to the core status request (which causes a RIFFA reset). We
     could wait until after the completion has been transmitted, but we have no
     guarantee that the TX path is operating correctly until after we reset */

    pipeline
        #(// Parameters
          .C_DEPTH                       (1),
        
          .C_WIDTH                       (2 * `SIG_FBE_W + `SIG_LOWADDR_W + 
                                          `SIG_TYPE_W + `SIG_LEN_W + 
                                          `SIG_BYTECNT_W + `SIG_TAG_W + 
                                          `SIG_REQID_W + `SIG_TC_W + 
                                          `SIG_ATTR_W + 1),
          .C_USE_MEMORY                  (0)
          /*AUTOINSTPARAM*/)
    txc_meta_hold
        (// Outputs
         .WR_DATA_READY                  (_wTxcMetaReady), // NC
         .RD_DATA                        ({wTxcMetaFdwbe, wTxcMetaLdwbe,
                                           wTxcMetaAddr, wTxcMetaType,
                                           wTxcMetaLength,
                                           wTxcMetaByteCount, wTxcMetaTag,
                                           wTxcMetaRequesterId, wTxcMetaTc,
                                           wTxcMetaAttr, wTxcMetaEp}),
         .RD_DATA_VALID                  (wTxcMetaValid),
         // Inputs
         .WR_DATA                        ({_wTxcMetaFdwbe, _wTxcMetaLdwbe,
                                           _wTxcMetaAddr, _wTxcMetaType,
                                           _wTxcMetaLength,
                                           _wTxcMetaByteCount, _wTxcMetaTag,
                                           _wTxcMetaRequesterId, _wTxcMetaTc,
                                           _wTxcMetaAttr, _wTxcMetaEp}),
         .WR_DATA_VALID                  (_wTxcMetaValid),
         .RD_DATA_READY                  (wTxcMetaReady),
         .RST_IN                        (RST_BUS),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    pipeline
        #(// Parameters
          .C_DEPTH                       (1),
          .C_WIDTH                       (C_PCI_DATA_WIDTH + 
                                          2 * (clog2s(C_PCI_DATA_WIDTH/32) + 1)),
          .C_USE_MEMORY                  (0)
          /*AUTOINSTPARAM*/)
    txc_data_hold
        (// Outputs
         .WR_DATA_READY                  (_wTxcDataReady), // NC
         .RD_DATA                        ({wTxcData, wTxcDataStartFlag, 
                                           wTxcDataStartOffset, wTxcDataEndFlag,
                                           wTxcDataEndOffset}),
         .RD_DATA_VALID                  (wTxcDataValid),
         // Inputs
         .WR_DATA                        ({_wTxcData, _wTxcDataStartFlag, 
                                           _wTxcDataStartOffset, _wTxcDataEndFlag,
                                           _wTxcDataEndOffset}),
         .WR_DATA_VALID                  (_wTxcDataValid),
         .RD_DATA_READY                  (wTxcDataReady),
         .RST_IN                        (RST_BUS),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    reset_extender
        #(.C_RST_COUNT                  (8)
          /*AUTOINSTPARAM*/)
    reset_extender_inst
        (// Outputs
         .PENDING_RST                   (wPendingRst),
         // Inputs
         .RST_LOGIC                     (wCoreSettingsReady),
         /*AUTOINST*/
         // Outputs
         .RST_OUT                       (RST_OUT),
         // Inputs
         .CLK                           (CLK),
         .RST_BUS                       (RST_BUS));
    
    reorder_queue 
        #(.C_PCI_DATA_WIDTH(C_PCI_DATA_WIDTH),
          .C_NUM_CHNL(C_NUM_CHNL),
          .C_MAX_READ_REQ_BYTES(C_MAX_READ_REQ_BYTES),
          .C_TAG_WIDTH(C_TAG_WIDTH)) 
    reorderQueue 
        (.RST                           (RST_OUT),
         .VALID                         (RXC_DATA_VALID),
         .DATA_START_FLAG               (RXC_DATA_START_FLAG),
         .DATA_START_OFFSET             (RXC_DATA_START_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .DATA_END_FLAG                 (RXC_DATA_END_FLAG),
         .DATA_END_OFFSET               (RXC_DATA_END_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .DATA                          (RXC_DATA),
         .DATA_EN                       (RXC_DATA_WORD_ENABLE),
         .DONE                          (wRxEngRdComplete),
         .ERR                           (RXC_META_EP),
         .TAG                           (RXC_META_TAG[C_TAG_WIDTH-1:0]), 
         .INT_TAG                       (wInternalTag),
         .INT_TAG_VALID                 (wInternalTagValid),
         .EXT_TAG                       (wExternalTag),
         .EXT_TAG_VALID                 (wExternalTagValid),
         .ENG_DATA                      (wRxEngData), 
         .MAIN_DATA_EN                  (wRxEngMainDataEn),
         .MAIN_DONE                     (wRxEngMainDone),
         .MAIN_ERR                      (wRxEngMainErr),
         .SG_RX_DATA_EN                 (wRxEngSgRxDataEn),
         .SG_RX_DONE                    (wRxEngSgRxDone),
         .SG_RX_ERR                     (wRxEngSgRxErr),
         .SG_TX_DATA_EN                 (wRxEngSgTxDataEn), 
         .SG_TX_DONE                    (wRxEngSgTxDone), 
         .SG_TX_ERR                     (wRxEngSgTxErr),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    registers
        #(// Parameters
          .C_PIPELINE_OUTPUT            (1),
          .C_PIPELINE_INPUT             (1),
          /*AUTOINSTPARAM*/
          // Parameters
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
          .C_NUM_CHNL                   (C_NUM_CHNL),
          .C_MAX_READ_REQ_BYTES         (C_MAX_READ_REQ_BYTES),
          .C_VENDOR                     (C_VENDOR),
          .C_NUM_VECTORS                (C_NUM_VECTORS),
          .C_VECTOR_WIDTH               (C_VECTOR_WIDTH),
          .C_FPGA_NAME                  (C_FPGA_NAME))
    reg_inst
        (// Outputs
         // Write Interfaces
         .CHNL_REQ_DATA                 (wChnlReqData[31:0]),
         .CHNL_SGRX_LEN_VALID           (wChnlSgRxLenValid),
         .CHNL_SGRX_ADDRLO_VALID        (wChnlSgRxAddrLoValid),
         .CHNL_SGRX_ADDRHI_VALID        (wChnlSgRxAddrHiValid),
         .CHNL_SGTX_LEN_VALID           (wChnlSgTxLenValid),
         .CHNL_SGTX_ADDRLO_VALID        (wChnlSgTxAddrLoValid),
         .CHNL_SGTX_ADDRHI_VALID        (wChnlSgTxAddrHiValid),
         .CHNL_RX_LEN_VALID             (wChnlRxLenValid),
         .CHNL_RX_OFFLAST_VALID         (wChnlRxOfflastValid),
         // Read Interfaces
         .CHNL_TX_LEN_READY             (wChnlTxLenReady),
         .CHNL_TX_OFFLAST_READY         (wChnlTxOfflastReady),
         .CORE_SETTINGS_READY           (wCoreSettingsReady),
         .INTR_VECTOR_READY             (wIntrVectorReady),
         .CHNL_TX_DONE_READY            (wChnlTxDoneReady),
         .CHNL_RX_DONE_READY            (wChnlRxDoneReady),
         .CHNL_NAME_READY               (wChnlNameReady), // TODO: Could do this on a per-channel basis
         // TXC Engine Interface
         .TXC_DATA_VALID                (_wTxcDataValid),
         .TXC_DATA                      (_wTxcData[C_PCI_DATA_WIDTH-1:0]),
         .TXC_DATA_START_FLAG           (_wTxcDataStartFlag),
         .TXC_DATA_START_OFFSET         (_wTxcDataStartOffset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TXC_DATA_END_FLAG             (_wTxcDataEndFlag),
         .TXC_DATA_END_OFFSET           (_wTxcDataEndOffset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TXC_META_VALID                (_wTxcMetaValid),
         .TXC_META_FDWBE                (_wTxcMetaFdwbe[`SIG_FBE_W-1:0]),
         .TXC_META_LDWBE                (_wTxcMetaLdwbe[`SIG_LBE_W-1:0]),
         .TXC_META_ADDR                 (_wTxcMetaAddr[`SIG_LOWADDR_W-1:0]),
         .TXC_META_TYPE                 (_wTxcMetaType[`SIG_TYPE_W-1:0]),
         .TXC_META_LENGTH               (_wTxcMetaLength[`SIG_LEN_W-1:0]),
         .TXC_META_BYTE_COUNT           (_wTxcMetaByteCount[`SIG_BYTECNT_W-1:0]),
         .TXC_META_TAG                  (_wTxcMetaTag[`SIG_TAG_W-1:0]),
         .TXC_META_REQUESTER_ID         (_wTxcMetaRequesterId[`SIG_REQID_W-1:0]),
         .TXC_META_TC                   (_wTxcMetaTc[`SIG_TC_W-1:0]),
         .TXC_META_ATTR                 (_wTxcMetaAttr[`SIG_ATTR_W-1:0]),
         .TXC_META_EP                   (_wTxcMetaEp),
         // Inputs
         // Read Data
         .CORE_SETTINGS                 (wCoreSettings),
         .CHNL_TX_REQLEN                (wChnlTxReqLen),
         .CHNL_TX_OFFLAST               (wChnlTxOfflast),
         .CHNL_TX_DONELEN               (wChnlTxDoneLen),
         .CHNL_RX_DONELEN               (wChnlRxDoneLen),
         .INTR_VECTOR                   (wIntrVector),
         .RST_IN                        (RST_OUT),
         .TXC_DATA_READY                (_wTxcDataReady),
         .TXC_META_READY                (_wTxcMetaReady),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RXR_DATA                      (RXR_DATA[C_PCI_DATA_WIDTH-1:0]),
         .RXR_DATA_VALID                (RXR_DATA_VALID),
         .RXR_DATA_START_FLAG           (RXR_DATA_START_FLAG),
         .RXR_DATA_START_OFFSET         (RXR_DATA_START_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .RXR_META_FDWBE                (RXR_META_FDWBE[`SIG_FBE_W-1:0]),
         .RXR_DATA_END_FLAG             (RXR_DATA_END_FLAG),
         .RXR_DATA_END_OFFSET           (RXR_DATA_END_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .RXR_META_LDWBE                (RXR_META_LDWBE[`SIG_LBE_W-1:0]),
         .RXR_META_TC                   (RXR_META_TC[`SIG_TC_W-1:0]),
         .RXR_META_ATTR                 (RXR_META_ATTR[`SIG_ATTR_W-1:0]),
         .RXR_META_TAG                  (RXR_META_TAG[`SIG_TAG_W-1:0]),
         .RXR_META_TYPE                 (RXR_META_TYPE[`SIG_TYPE_W-1:0]),
         .RXR_META_ADDR                 (RXR_META_ADDR[`SIG_ADDR_W-1:0]),
         .RXR_META_BAR_DECODED          (RXR_META_BAR_DECODED[`SIG_BARDECODE_W-1:0]),
         .RXR_META_REQUESTER_ID         (RXR_META_REQUESTER_ID[`SIG_REQID_W-1:0]),
         .RXR_META_LENGTH               (RXR_META_LENGTH[`SIG_LEN_W-1:0]));
    
    // Track receive buffer flow control credits (header & Data)
    recv_credit_flow_ctrl rc_fc 
        (// Outputs
         .RXBUF_SPACE_AVAIL             (wRxBufSpaceAvail),
         // Inputs
         .RX_ENG_RD_DONE                (wRxEngRdComplete),
         .TX_ENG_RD_REQ_SENT            (wTxEngRdReqSent),
         .RST                           (RST_OUT),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .CONFIG_MAX_READ_REQUEST_SIZE  (CONFIG_MAX_READ_REQUEST_SIZE[2:0]),
         .CONFIG_MAX_CPL_DATA           (CONFIG_MAX_CPL_DATA[11:0]),
         .CONFIG_MAX_CPL_HDR            (CONFIG_MAX_CPL_HDR[7:0]),
         .CONFIG_CPL_BOUNDARY_SEL       (CONFIG_CPL_BOUNDARY_SEL));
    // Connect the interrupt vector and controller.
    interrupt 
        #(.C_NUM_CHNL                   (C_NUM_CHNL)) 
    intr 
        (// Inputs
         .RST                           (RST_OUT),
         .RX_SG_BUF_RECVD               (wChnlSgRxBufRecvd),
         .RX_TXN_DONE                   (wChnlRxDone),
         .TX_TXN                        (wChnlTxRequest),
         .TX_SG_BUF_RECVD               (wChnlSgTxBufRecvd),
         .TX_TXN_DONE                   (wChnlTxDone),
         .VECT_0_RST                    (wIntrVectorReady[0]),
         .VECT_1_RST                    (wIntrVectorReady[1]),
         .VECT_RST                      (_wTxcData[31:0]),
         .VECT_0                        (wIntrVector[31:0]),
         .VECT_1                        (wIntrVector[63:32]),
         .INTR_LEGACY_CLR               (1'd0),
         /*AUTOINST*/
         // Outputs
         .INTR_MSI_REQUEST              (INTR_MSI_REQUEST),
         // Inputs
         .CLK                           (CLK),
         .CONFIG_INTERRUPT_MSIENABLE    (CONFIG_INTERRUPT_MSIENABLE),
         .INTR_MSI_RDY                  (INTR_MSI_RDY));


    tx_multiplexer
        #(/*AUTOINSTPARAM*/
          // Parameters
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
          .C_NUM_CHNL                   (C_NUM_CHNL),
          .C_TAG_WIDTH                  (C_TAG_WIDTH),
          .C_VENDOR                     (C_VENDOR),
          .C_DEPTH_PACKETS              (C_DEPTH_PACKETS))
    tx_mux_inst
        (
         // Outputs
         .WR_DATA_REN                   (wTxEngWrDataRen[C_NUM_CHNL-1:0]),
         .WR_ACK                        (wTxEngWrAck[C_NUM_CHNL-1:0]),
         .RD_ACK                        (wTxEngRdAck[C_NUM_CHNL-1:0]),
         .INT_TAG                       (wInternalTag[5:0]),
         .INT_TAG_VALID                 (wInternalTagValid),
         .TX_ENG_RD_REQ_SENT            (wTxEngRdReqSent),
         // Inputs
         .RST_IN                        (RST_OUT),
         .WR_REQ                        (wTxEngWrReq[C_NUM_CHNL-1:0]),
         .WR_ADDR                       (wTxEngWrAddr[(C_NUM_CHNL*`SIG_ADDR_W)-1:0]),
         .WR_LEN                        (wTxEngWrLen[(C_NUM_CHNL*`SIG_LEN_W)-1:0]),
         .WR_DATA                       (wTxEngWrData[(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0]),
         .WR_SENT                       (wTxEngWrSent[C_NUM_CHNL-1:0]),
         .RD_REQ                        (wTxEngRdReq[C_NUM_CHNL-1:0]),
         .RD_SG_CHNL                    (wTxEngRdSgChnl[(C_NUM_CHNL*2)-1:0]),
         .RD_ADDR                       (wTxEngRdAddr[(C_NUM_CHNL*`SIG_ADDR_W)-1:0]),
         .RD_LEN                        (wTxEngRdLen[(C_NUM_CHNL*`SIG_LEN_W)-1:0]),
         .EXT_TAG                       (wExternalTag[C_TAG_WIDTH-1:0]),
         .EXT_TAG_VALID                 (wExternalTagValid),
         .RXBUF_SPACE_AVAIL             (wRxBufSpaceAvail),
         /*AUTOINST*/
         // Outputs
         .TXR_DATA_VALID                (TXR_DATA_VALID),
         .TXR_DATA                      (TXR_DATA[C_PCI_DATA_WIDTH-1:0]),
         .TXR_DATA_START_FLAG           (TXR_DATA_START_FLAG),
         .TXR_DATA_START_OFFSET         (TXR_DATA_START_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TXR_DATA_END_FLAG             (TXR_DATA_END_FLAG),
         .TXR_DATA_END_OFFSET           (TXR_DATA_END_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TXR_META_VALID                (TXR_META_VALID),
         .TXR_META_FDWBE                (TXR_META_FDWBE[`SIG_FBE_W-1:0]),
         .TXR_META_LDWBE                (TXR_META_LDWBE[`SIG_LBE_W-1:0]),
         .TXR_META_ADDR                 (TXR_META_ADDR[`SIG_ADDR_W-1:0]),
         .TXR_META_LENGTH               (TXR_META_LENGTH[`SIG_LEN_W-1:0]),
         .TXR_META_TAG                  (TXR_META_TAG[`SIG_TAG_W-1:0]),
         .TXR_META_TC                   (TXR_META_TC[`SIG_TC_W-1:0]),
         .TXR_META_ATTR                 (TXR_META_ATTR[`SIG_ATTR_W-1:0]),
         .TXR_META_TYPE                 (TXR_META_TYPE[`SIG_TYPE_W-1:0]),
         .TXR_META_EP                   (TXR_META_EP),
         // Inputs
         .CLK                           (CLK),
         .TXR_DATA_READY                (TXR_DATA_READY),
         .TXR_META_READY                (TXR_META_READY),
         .TXR_SENT                      (TXR_SENT));
    
    // Generate and link up the channels.
    generate
        for (i = 0; i < C_NUM_CHNL; i = i + 1) begin : channels
            channel 
                 #(
                   .C_DATA_WIDTH(C_PCI_DATA_WIDTH), 
                   .C_MAX_READ_REQ(C_MAX_READ_REQ)
                   )
            channel 
                 (
                  .RST(RST_OUT),
                  .CLK(CLK), 
                  .CONFIG_MAX_READ_REQUEST_SIZE(CONFIG_MAX_READ_REQUEST_SIZE), 
                  .CONFIG_MAX_PAYLOAD_SIZE(CONFIG_MAX_PAYLOAD_SIZE), 

                  .PIO_DATA(wChnlReqData), 
                  .ENG_DATA(wRxEngData), 
                
                  .SG_RX_BUF_RECVD(wChnlSgRxBufRecvd[i]),
                  .SG_TX_BUF_RECVD(wChnlSgTxBufRecvd[i]),
                  .TXN_TX(wChnlTxRequest[i]),
                  .TXN_TX_DONE(wChnlTxDone[i]),
                  .TXN_RX_DONE(wChnlRxDone[i]),
                
                  .SG_RX_BUF_LEN_VALID(wChnlSgRxLenValid[i]),
                  .SG_RX_BUF_ADDR_HI_VALID(wChnlSgRxAddrHiValid[i]),
                  .SG_RX_BUF_ADDR_LO_VALID(wChnlSgRxAddrLoValid[i]),
                
                  .SG_TX_BUF_LEN_VALID(wChnlSgTxLenValid[i]),
                  .SG_TX_BUF_ADDR_HI_VALID(wChnlSgTxAddrHiValid[i]),
                  .SG_TX_BUF_ADDR_LO_VALID(wChnlSgTxAddrLoValid[i]),
                
                  .TXN_RX_LEN_VALID(wChnlRxLenValid[i]), 
                  .TXN_RX_OFF_LAST_VALID(wChnlRxOfflastValid[i]), 
                  .TXN_RX_DONE_LEN(wChnlRxDoneLen[(`SIG_RXDONELEN_W*i) +: `SIG_RXDONELEN_W]),
                  .TXN_RX_DONE_ACK(wChnlRxDoneReady[i]),
                
                  .TXN_TX_ACK(wChnlTxLenReady[i]), // ACK'd on length read
                  .TXN_TX_LEN(wChnlTxReqLen[(`SIG_TXRLEN_W*i) +: `SIG_TXRLEN_W]),
                  .TXN_TX_OFF_LAST(wChnlTxOfflast[(`SIG_OFFLAST_W*i) +: `SIG_OFFLAST_W]),
                  .TXN_TX_DONE_LEN(wChnlTxDoneLen[(`SIG_TXDONELEN_W*i) +:`SIG_TXDONELEN_W]),
                  .TXN_TX_DONE_ACK(wChnlTxDoneReady[i]),
                
                  .RX_REQ(wTxEngRdReq[i]),
                  .RX_REQ_ACK(wTxEngRdAck[i]),
                  .RX_REQ_TAG(wTxEngRdSgChnl[(2*i) +:2]),// TODO: `SIG_INTERNALTAG_W
                  .RX_REQ_ADDR(wTxEngRdAddr[(`SIG_ADDR_W*i) +:`SIG_ADDR_W]),
                  .RX_REQ_LEN(wTxEngRdLen[(`SIG_LEN_W*i) +:`SIG_LEN_W]),

                  .TX_REQ(wTxEngWrReq[i]), 
                  .TX_REQ_ACK(wTxEngWrAck[i]),
                  .TX_ADDR(wTxEngWrAddr[(`SIG_ADDR_W*i) +: `SIG_ADDR_W]), 
                  .TX_LEN(wTxEngWrLen[(`SIG_LEN_W*i) +: `SIG_LEN_W]), 
                  .TX_DATA(wTxEngWrData[(C_PCI_DATA_WIDTH*i) +:C_PCI_DATA_WIDTH]),
                  .TX_DATA_REN(wTxEngWrDataRen[i]), 
                  .TX_SENT(wTxEngWrSent[i]),
                
                  .MAIN_DATA_EN(wRxEngMainDataEn[(C_PCI_DATA_WORD_WIDTH*i) +:C_PCI_DATA_WORD_WIDTH]), 
                  .MAIN_DONE(wRxEngMainDone[i]), 
                  .MAIN_ERR(wRxEngMainErr[i]),
                
                  .SG_RX_DATA_EN(wRxEngSgRxDataEn[(C_PCI_DATA_WORD_WIDTH*i) +:C_PCI_DATA_WORD_WIDTH]),  
                  .SG_RX_DONE(wRxEngSgRxDone[i]), 
                  .SG_RX_ERR(wRxEngSgRxErr[i]),

                  .SG_TX_DATA_EN(wRxEngSgTxDataEn[(C_PCI_DATA_WORD_WIDTH*i) +:C_PCI_DATA_WORD_WIDTH]), 
                  .SG_TX_DONE(wRxEngSgTxDone[i]), 
                  .SG_TX_ERR(wRxEngSgTxErr[i]),

                  .CHNL_RX_CLK(CHNL_RX_CLK[i]), 
                  .CHNL_RX(CHNL_RX[i]), 
                  .CHNL_RX_ACK(CHNL_RX_ACK[i]), 
                  .CHNL_RX_LAST(CHNL_RX_LAST[i]), 
                  .CHNL_RX_LEN(CHNL_RX_LEN[(32*i) +:32]), 
                  .CHNL_RX_OFF(CHNL_RX_OFF[(31*i) +:31]), 
                  .CHNL_RX_DATA(CHNL_RX_DATA[(C_PCI_DATA_WIDTH*i) +:C_PCI_DATA_WIDTH]), 
                  .CHNL_RX_DATA_VALID(CHNL_RX_DATA_VALID[i]), 
                  .CHNL_RX_DATA_REN(CHNL_RX_DATA_REN[i]),

                  .CHNL_TX_CLK(CHNL_TX_CLK[i]), 
                  .CHNL_TX(CHNL_TX[i]), 
                  .CHNL_TX_ACK(CHNL_TX_ACK[i]),
                  .CHNL_TX_LAST(CHNL_TX_LAST[i]), 
                  .CHNL_TX_LEN(CHNL_TX_LEN[(32*i) +:32]), 
                  .CHNL_TX_OFF(CHNL_TX_OFF[(31*i) +:31]), 
                  .CHNL_TX_DATA(CHNL_TX_DATA[(C_PCI_DATA_WIDTH*i) +:C_PCI_DATA_WIDTH]), 
                  .CHNL_TX_DATA_VALID(CHNL_TX_DATA_VALID[i]), 
                  .CHNL_TX_DATA_REN(CHNL_TX_DATA_REN[i])
                  );

        end
    endgenerate
endmodule
// Local Variables:
// verilog-library-directories:("." "registers/" "import")
// End:

