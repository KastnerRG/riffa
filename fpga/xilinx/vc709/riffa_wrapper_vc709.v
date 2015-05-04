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
// Filename:            riffa_wrapper_vc709.v
// Version:             1.00.a
// Verilog Standard:    Verilog-2001
// Description:         RIFFA wrapper for the VC709 Development board.
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`include "trellis.vh"
`include "riffa.vh"
`include "ultrascale.vh"
`include "functions.vh"
`timescale 1ps / 1ps
module riffa_wrapper_vc709
    #(// Number of RIFFA Channels
      parameter C_NUM_CHNL = 1,
      // Bit-Width from Vivado IP Generator
      parameter C_PCI_DATA_WIDTH = 128,
      // 4-Byte Name for this FPGA
      parameter C_MAX_PAYLOAD_BYTES = 256,
      parameter C_LOG_NUM_TAGS = 5
      ) 
    (
     //Interface: CQ Ultrascale (RXR)
     input                                        M_AXIS_CQ_TVALID,
     input                                        M_AXIS_CQ_TLAST,
     input [C_PCI_DATA_WIDTH-1:0]                 M_AXIS_CQ_TDATA,
     input [(C_PCI_DATA_WIDTH/32)-1:0]            M_AXIS_CQ_TKEEP,
     input [`SIG_CQ_TUSER_W-1:0]                  M_AXIS_CQ_TUSER,
     output                                       M_AXIS_CQ_TREADY,

     //Interface: RC Ultrascale (RXC)
     input                                        M_AXIS_RC_TVALID,
     input                                        M_AXIS_RC_TLAST,
     input [C_PCI_DATA_WIDTH-1:0]                 M_AXIS_RC_TDATA,
     input [(C_PCI_DATA_WIDTH/32)-1:0]            M_AXIS_RC_TKEEP,
     input [`SIG_RC_TUSER_W-1:0]                  M_AXIS_RC_TUSER,
     output                                       M_AXIS_RC_TREADY,

     //Interface: CC Ultrascale (TXC)
     input                                        S_AXIS_CC_TREADY,
     output                                       S_AXIS_CC_TVALID,
     output                                       S_AXIS_CC_TLAST,
     output [C_PCI_DATA_WIDTH-1:0]                S_AXIS_CC_TDATA,
     output [(C_PCI_DATA_WIDTH/32)-1:0]           S_AXIS_CC_TKEEP,
     output [`SIG_CC_TUSER_W-1:0]                 S_AXIS_CC_TUSER,
    
     //Interface: RQ Ultrascale (TXR) 
     input                                        S_AXIS_RQ_TREADY,
     output                                       S_AXIS_RQ_TVALID,
     output                                       S_AXIS_RQ_TLAST,
     output [C_PCI_DATA_WIDTH-1:0]                S_AXIS_RQ_TDATA,
     output [(C_PCI_DATA_WIDTH/32)-1:0]           S_AXIS_RQ_TKEEP,
     output [`SIG_RQ_TUSER_W-1:0]                 S_AXIS_RQ_TUSER,

     input                                        USER_CLK,
     input                                        USER_RESET,

     output [3:0]                                 CFG_INTERRUPT_INT,
     output [1:0]                                 CFG_INTERRUPT_PENDING,
     input [1:0]                                  CFG_INTERRUPT_MSI_ENABLE,
     input                                        CFG_INTERRUPT_MSI_MASK_UPDATE,
     input [31:0]                                 CFG_INTERRUPT_MSI_DATA,
     output [3:0]                                 CFG_INTERRUPT_MSI_SELECT,
     output [31:0]                                CFG_INTERRUPT_MSI_INT,
     output [63:0]                                CFG_INTERRUPT_MSI_PENDING_STATUS,
     input                                        CFG_INTERRUPT_MSI_SENT,
     input                                        CFG_INTERRUPT_MSI_FAIL,
     output [2:0]                                 CFG_INTERRUPT_MSI_ATTR,
     output                                       CFG_INTERRUPT_MSI_TPH_PRESENT,
     output [1:0]                                 CFG_INTERRUPT_MSI_TPH_TYPE,
     output [8:0]                                 CFG_INTERRUPT_MSI_TPH_ST_TAG,
     output [2:0]                                 CFG_INTERRUPT_MSI_FUNCTION_NUMBER,

     input [7:0]                                  CFG_FC_CPLH,
     input [11:0]                                 CFG_FC_CPLD,
     output [2:0]                                 CFG_FC_SEL,

     input [3:0]                                  CFG_NEGOTIATED_WIDTH, // CONFIG_LINK_WIDTH
     input [2:0]                                  CFG_CURRENT_SPEED, // CONFIG_LINK_RATE
     input [2:0]                                  CFG_MAX_PAYLOAD, // CONFIG_MAX_PAYLOAD
     input [2:0]                                  CFG_MAX_READ_REQ, // CONFIG_MAX_READ_REQUEST
     input [7:0]                                  CFG_FUNCTION_STATUS, // [2] = CONFIG_BUS_MASTER_ENABLE
     input [1:0]                                  CFG_RCB_STATUS,
    
     output                                       PCIE_CQ_NP_REQ,

     // RIFFA Interface Signals
     output                                       RST_OUT,
     input [C_NUM_CHNL-1:0]                       CHNL_RX_CLK, // Channel read clock
     output [C_NUM_CHNL-1:0]                      CHNL_RX, // Channel read receive signal
     input [C_NUM_CHNL-1:0]                       CHNL_RX_ACK, // Channel read received signal
     output [C_NUM_CHNL-1:0]                      CHNL_RX_LAST, // Channel last read
     output [(C_NUM_CHNL*`SIG_CHNL_LENGTH_W)-1:0] CHNL_RX_LEN, // Channel read length
     output [(C_NUM_CHNL*`SIG_CHNL_OFFSET_W)-1:0] CHNL_RX_OFF, // Channel read offset
     output [(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0]   CHNL_RX_DATA, // Channel read data
     output [C_NUM_CHNL-1:0]                      CHNL_RX_DATA_VALID, // Channel read data valid
     input [C_NUM_CHNL-1:0]                       CHNL_RX_DATA_REN, // Channel read data has been recieved

     input [C_NUM_CHNL-1:0]                       CHNL_TX_CLK, // Channel write clock
     input [C_NUM_CHNL-1:0]                       CHNL_TX, // Channel write receive signal
     output [C_NUM_CHNL-1:0]                      CHNL_TX_ACK, // Channel write acknowledgement signal
     input [C_NUM_CHNL-1:0]                       CHNL_TX_LAST, // Channel last write
     input [(C_NUM_CHNL*`SIG_CHNL_LENGTH_W)-1:0]  CHNL_TX_LEN, // Channel write length (in 32 bit words)
     input [(C_NUM_CHNL*`SIG_CHNL_OFFSET_W)-1:0]  CHNL_TX_OFF, // Channel write offset
     input [(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0]    CHNL_TX_DATA, // Channel write data
     input [C_NUM_CHNL-1:0]                       CHNL_TX_DATA_VALID, // Channel write data valid
     output [C_NUM_CHNL-1:0]                      CHNL_TX_DATA_REN // Channel write data has been recieved

     );
    localparam C_FPGA_NAME = "REGT"; // This is not yet exposed in the driver
    localparam C_MAX_READ_REQ_BYTES = C_MAX_PAYLOAD_BYTES * 2;
    // ALTERA, XILINX or ULTRASCALE
    localparam C_VENDOR = "ULTRASCALE";

    localparam C_KEEP_WIDTH = C_PCI_DATA_WIDTH / 32;
    localparam C_PIPELINE_OUTPUT = 1;
    localparam C_PIPELINE_INPUT = 1;

    wire                                          clk;
    wire                                          rst_in;

    // Interface: RXC Engine
    wire [C_PCI_DATA_WIDTH-1:0]                   rxc_data;
    wire                                          rxc_data_valid;
    wire                                          rxc_data_start_flag;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]              rxc_data_word_enable;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]        rxc_data_start_offset;
    wire [`SIG_FBE_W-1:0]                         rxc_meta_fdwbe;
    wire                                          rxc_data_end_flag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]        rxc_data_end_offset;
    wire [`SIG_LBE_W-1:0]                         rxc_meta_ldwbe;
    wire [`SIG_TAG_W-1:0]                         rxc_meta_tag;
    wire [`SIG_LOWADDR_W-1:0]                     rxc_meta_addr;
    wire [`SIG_TYPE_W-1:0]                        rxc_meta_type;
    wire [`SIG_LEN_W-1:0]                         rxc_meta_length;
    wire [`SIG_BYTECNT_W-1:0]                     rxc_meta_bytes_remaining;
    wire [`SIG_CPLID_W-1:0]                       rxc_meta_completer_id;
    wire                                          rxc_meta_ep;

    // Interface: RXR Engine
    wire [C_PCI_DATA_WIDTH-1:0]                   rxr_data;
    wire                                          rxr_data_valid;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]              rxr_data_word_enable;
    wire                                          rxr_data_start_flag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]        rxr_data_start_offset;
    wire [`SIG_FBE_W-1:0]                         rxr_meta_fdwbe;
    wire                                          rxr_data_end_flag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]        rxr_data_end_offset;
    wire [`SIG_LBE_W-1:0]                         rxr_meta_ldwbe;
    wire [`SIG_TC_W-1:0]                          rxr_meta_tc;
    wire [`SIG_ATTR_W-1:0]                        rxr_meta_attr;
    wire [`SIG_TAG_W-1:0]                         rxr_meta_tag;
    wire [`SIG_TYPE_W-1:0]                        rxr_meta_type;
    wire [`SIG_ADDR_W-1:0]                        rxr_meta_addr;
    wire [`SIG_BARDECODE_W-1:0]                   rxr_meta_bar_decoded;
    wire [`SIG_REQID_W-1:0]                       rxr_meta_requester_id;
    wire [`SIG_LEN_W-1:0]                         rxr_meta_length;
    wire                                          rxr_meta_ep;
    
    // interface: TXC Engine
    wire                                          txc_data_valid;
    wire [C_PCI_DATA_WIDTH-1:0]                   txc_data;
    wire                                          txc_data_start_flag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]        txc_data_start_offset;
    wire                                          txc_data_end_flag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]        txc_data_end_offset;
    wire                                          txc_data_ready;

    wire                                          txc_meta_valid;
    wire [`SIG_FBE_W-1:0]                         txc_meta_fdwbe;
    wire [`SIG_LBE_W-1:0]                         txc_meta_ldwbe;
    wire [`SIG_LOWADDR_W-1:0]                     txc_meta_addr;
    wire [`SIG_TYPE_W-1:0]                        txc_meta_type;
    wire [`SIG_LEN_W-1:0]                         txc_meta_length;
    wire [`SIG_BYTECNT_W-1:0]                     txc_meta_byte_count;
    wire [`SIG_TAG_W-1:0]                         txc_meta_tag;
    wire [`SIG_REQID_W-1:0]                       txc_meta_requester_id;
    wire [`SIG_TC_W-1:0]                          txc_meta_tc;
    wire [`SIG_ATTR_W-1:0]                        txc_meta_attr;
    wire                                          txc_meta_ep;
    wire                                          txc_meta_ready;
    wire                                          txc_sent;

    // Interface: TXR Engine
    wire                                          txr_data_valid;
    wire [C_PCI_DATA_WIDTH-1:0]                   txr_data;
    wire                                          txr_data_start_flag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]        txr_data_start_offset;
    wire                                          txr_data_end_flag;
    wire [clog2s(C_PCI_DATA_WIDTH/32)-1:0]        txr_data_end_offset;
    wire                                          txr_data_ready;

    wire                                          txr_meta_valid;
    wire [`SIG_FBE_W-1:0]                         txr_meta_fdwbe; 
    wire [`SIG_LBE_W-1:0]                         txr_meta_ldwbe;
    wire [`SIG_ADDR_W-1:0]                        txr_meta_addr;
    wire [`SIG_LEN_W-1:0]                         txr_meta_length;
    wire [`SIG_TAG_W-1:0]                         txr_meta_tag;
    wire [`SIG_TC_W-1:0]                          txr_meta_tc;
    wire [`SIG_ATTR_W-1:0]                        txr_meta_attr;
    wire [`SIG_TYPE_W-1:0]                        txr_meta_type;
    wire                                          txr_meta_ep;
    wire                                          txr_meta_ready;
    wire                                          txr_sent;

    // Unconnected Wires (Used in classic interface)
    wire                                          wRxTlpReady_nc;
    wire [C_PCI_DATA_WIDTH-1:0]                   wRxTlp_nc = 0;
    wire                                          wRxTlpEndFlag_nc = 0;
    wire [`SIG_OFFSET_W-1:0]                      wRxTlpEndOffset_nc = 0;
    wire                                          wRxTlpStartFlag_nc = 0;
    wire [`SIG_OFFSET_W-1:0]                      wRxTlpStartOffset_nc = 0;
    wire                                          wRxTlpValid_nc = 0;
    wire [`SIG_BARDECODE_W-1:0]                   wRxTlpBarDecode_nc = 0;
    
    wire                                          wTxTlpReady_nc = 0;
    wire [C_PCI_DATA_WIDTH-1:0]                   wTxTlp_nc;
    wire                                          wTxTlpEndFlag_nc;
    wire [`SIG_OFFSET_W-1:0]                      wTxTlpEndOffset_nc;
    wire                                          wTxTlpStartFlag_nc;
    wire [`SIG_OFFSET_W-1:0]                      wTxTlpStartOffset_nc;
    wire                                          wTxTlpValid_nc;

    //--------------------------------------------------------------------------

    // Interface: Configuration
    wire                                          config_bus_master_enable;
    wire [`SIG_CPLID_W-1:0]                       config_completer_id;
    wire                                          config_cpl_boundary_sel;
    wire                                          config_interrupt_msienable;
    wire [`SIG_LINKRATE_W-1:0]                    config_link_rate;
    wire [`SIG_LINKWIDTH_W-1:0]                   config_link_width;
    wire [`SIG_MAXPAYLOAD_W-1:0]                  config_max_payload_size;
    wire [`SIG_MAXREAD_W-1:0]                     config_max_read_request_size;
    wire [`SIG_FC_CPLD_W-1:0]                     config_max_cpl_data;
    wire [`SIG_FC_CPLH_W-1:0]                     config_max_cpl_hdr;

    wire                                          intr_msi_request;
    wire                                          intr_msi_rdy;

    genvar                                        chnl;

    assign clk = USER_CLK;
    assign rst_in = USER_RESET;

    assign config_completer_id = 0; // Not used in ULTRASCALE implementation
    assign config_bus_master_enable = CFG_FUNCTION_STATUS[2];
    assign config_link_width = {2'b00,CFG_NEGOTIATED_WIDTH}; // CONFIG_LINK_WIDTH
    assign config_link_rate = CFG_CURRENT_SPEED[2]? 2'b11 : CFG_CURRENT_SPEED[2] ? 2'b10 : 2'b01;
    assign config_max_payload_size = CFG_MAX_PAYLOAD; // CONFIG_MAX_PAYLOAD
    assign config_max_read_request_size = CFG_MAX_READ_REQ; // CONFIG_MAX_READ_REQUEST
    assign config_cpl_boundary_sel =  CFG_RCB_STATUS[0];
    assign config_interrupt_msienable = CFG_INTERRUPT_MSI_ENABLE[0];
    assign config_max_cpl_data = CFG_FC_CPLD;    
    assign config_max_cpl_hdr = CFG_FC_CPLH;    

    assign CFG_FC_SEL = 3'b001; // Always display credit maximum for the signals below
    assign CFG_INTERRUPT_MSI_INT = {31'b0,intr_msi_request};
    assign CFG_INTERRUPT_MSI_SELECT = 0;
    assign CFG_INTERRUPT_INT = 0;
    assign CFG_INTERRUPT_PENDING = 0;
    assign CFG_INTERRUPT_MSI_SELECT = 0;
    assign CFG_INTERRUPT_MSI_PENDING_STATUS = {63'b0,intr_msi_request};
    assign CFG_INTERRUPT_MSI_ATTR = 0;
    assign CFG_INTERRUPT_MSI_TPH_PRESENT = 0;
    assign CFG_INTERRUPT_MSI_TPH_ST_TAG = 0;
    assign CFG_INTERRUPT_MSI_TPH_TYPE = 0;
    assign CFG_INTERRUPT_MSI_FUNCTION_NUMBER = 0;
    
    assign intr_msi_rdy = CFG_INTERRUPT_MSI_SENT & ~CFG_INTERRUPT_MSI_FAIL;

    assign PCIE_CQ_NP_REQ = 1;
    
    engine_layer
        #(// Parameters
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
          .C_LOG_NUM_TAGS               (C_LOG_NUM_TAGS),
          .C_PIPELINE_INPUT             (C_PIPELINE_INPUT),
          .C_PIPELINE_OUTPUT            (C_PIPELINE_OUTPUT),
          .C_MAX_PAYLOAD_DWORDS         (C_MAX_PAYLOAD_BYTES/4),
          .C_VENDOR                     (C_VENDOR))
    engine_layer_inst
        (// Outputs
         .RXC_DATA                      (rxc_data[C_PCI_DATA_WIDTH-1:0]),
         .RXC_DATA_WORD_ENABLE          (rxc_data_word_enable[(C_PCI_DATA_WIDTH/32)-1:0]),
         .RXC_DATA_VALID                (rxc_data_valid),
         .RXC_DATA_START_FLAG           (rxc_data_start_flag),
         .RXC_DATA_START_OFFSET         (rxc_data_start_offset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .RXC_META_FDWBE                (rxc_meta_fdwbe[`SIG_FBE_W-1:0]),
         .RXC_DATA_END_FLAG             (rxc_data_end_flag),
         .RXC_DATA_END_OFFSET           (rxc_data_end_offset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .RXC_META_LDWBE                (rxc_meta_ldwbe[`SIG_LBE_W-1:0]),
         .RXC_META_TAG                  (rxc_meta_tag[`SIG_TAG_W-1:0]),
         .RXC_META_ADDR                 (rxc_meta_addr[`SIG_LOWADDR_W-1:0]),
         .RXC_META_TYPE                 (rxc_meta_type[`SIG_TYPE_W-1:0]),
         .RXC_META_LENGTH               (rxc_meta_length[`SIG_LEN_W-1:0]),
         .RXC_META_BYTES_REMAINING      (rxc_meta_bytes_remaining[`SIG_BYTECNT_W-1:0]),
         .RXC_META_COMPLETER_ID         (rxc_meta_completer_id[`SIG_CPLID_W-1:0]),
         .RXC_META_EP                   (rxc_meta_ep),

         .RXR_DATA                      (rxr_data[C_PCI_DATA_WIDTH-1:0]),
         .RXR_DATA_WORD_ENABLE          (rxr_data_word_enable[(C_PCI_DATA_WIDTH/32)-1:0]),
         .RXR_DATA_VALID                (rxr_data_valid),
         .RXR_DATA_START_FLAG           (rxr_data_start_flag),
         .RXR_DATA_START_OFFSET         (rxr_data_start_offset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .RXR_DATA_END_FLAG             (rxr_data_end_flag),
         .RXR_DATA_END_OFFSET           (rxr_data_end_offset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .RXR_META_FDWBE                (rxr_meta_fdwbe[`SIG_FBE_W-1:0]),
         .RXR_META_LDWBE                (rxr_meta_ldwbe[`SIG_LBE_W-1:0]),
         .RXR_META_TC                   (rxr_meta_tc[`SIG_TC_W-1:0]),
         .RXR_META_ATTR                 (rxr_meta_attr[`SIG_ATTR_W-1:0]),
         .RXR_META_TAG                  (rxr_meta_tag[`SIG_TAG_W-1:0]),
         .RXR_META_TYPE                 (rxr_meta_type[`SIG_TYPE_W-1:0]),
         .RXR_META_ADDR                 (rxr_meta_addr[`SIG_ADDR_W-1:0]),
         .RXR_META_BAR_DECODED          (rxr_meta_bar_decoded[`SIG_BARDECODE_W-1:0]),
         .RXR_META_REQUESTER_ID         (rxr_meta_requester_id[`SIG_REQID_W-1:0]),
         .RXR_META_LENGTH               (rxr_meta_length[`SIG_LEN_W-1:0]),
         .RXR_META_EP                   (rxr_meta_ep),

         .TXC_DATA_READY                (txc_data_ready),
         .TXC_META_READY                (txc_meta_ready),
         .TXC_SENT                      (txc_sent),

         .TXR_DATA_READY                (txr_data_ready),
         .TXR_META_READY                (txr_meta_ready),
         .TXR_SENT                      (txr_sent),
         // Unconnected Outputs
         .TX_TLP                        (wTxTlp_nc),
         .TX_TLP_VALID                  (wTxTlpValid_nc),
         .TX_TLP_START_FLAG             (wTxTlpStartFlag_nc),
         .TX_TLP_START_OFFSET           (wTxTlpStartOffset_nc),
         .TX_TLP_END_FLAG               (wTxTlpEndFlag_nc),
         .TX_TLP_END_OFFSET             (wTxTlpEndOffset_nc),

         .RX_TLP_READY                  (wRxTlpReady_nc),
         // Inputs
         .CLK                           (clk),
         .RST_IN                        (rst_in),

         .CONFIG_COMPLETER_ID           (config_completer_id[`SIG_CPLID_W-1:0]),

         .TXC_DATA_VALID                (txc_data_valid),
         .TXC_DATA                      (txc_data[C_PCI_DATA_WIDTH-1:0]),
         .TXC_DATA_START_FLAG           (txc_data_start_flag),
         .TXC_DATA_START_OFFSET         (txc_data_start_offset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TXC_DATA_END_FLAG             (txc_data_end_flag),
         .TXC_DATA_END_OFFSET           (txc_data_end_offset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TXC_META_VALID                (txc_meta_valid),
         .TXC_META_FDWBE                (txc_meta_fdwbe[`SIG_FBE_W-1:0]),
         .TXC_META_LDWBE                (txc_meta_ldwbe[`SIG_LBE_W-1:0]),
         .TXC_META_ADDR                 (txc_meta_addr[`SIG_LOWADDR_W-1:0]),
         .TXC_META_TYPE                 (txc_meta_type[`SIG_TYPE_W-1:0]),
         .TXC_META_LENGTH               (txc_meta_length[`SIG_LEN_W-1:0]),
         .TXC_META_BYTE_COUNT           (txc_meta_byte_count[`SIG_BYTECNT_W-1:0]),
         .TXC_META_TAG                  (txc_meta_tag[`SIG_TAG_W-1:0]),
         .TXC_META_REQUESTER_ID         (txc_meta_requester_id[`SIG_REQID_W-1:0]),
         .TXC_META_TC                   (txc_meta_tc[`SIG_TC_W-1:0]),
         .TXC_META_ATTR                 (txc_meta_attr[`SIG_ATTR_W-1:0]),
         .TXC_META_EP                   (txc_meta_ep),

         .TXR_DATA_VALID                (txr_data_valid),
         .TXR_DATA                      (txr_data[C_PCI_DATA_WIDTH-1:0]),
         .TXR_DATA_START_FLAG           (txr_data_start_flag),
         .TXR_DATA_START_OFFSET         (txr_data_start_offset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TXR_DATA_END_FLAG             (txr_data_end_flag),
         .TXR_DATA_END_OFFSET           (txr_data_end_offset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TXR_META_VALID                (txr_meta_valid),
         .TXR_META_FDWBE                (txr_meta_fdwbe[`SIG_FBE_W-1:0]),
         .TXR_META_LDWBE                (txr_meta_ldwbe[`SIG_LBE_W-1:0]),
         .TXR_META_ADDR                 (txr_meta_addr[`SIG_ADDR_W-1:0]),
         .TXR_META_LENGTH               (txr_meta_length[`SIG_LEN_W-1:0]),
         .TXR_META_TAG                  (txr_meta_tag[`SIG_TAG_W-1:0]),
         .TXR_META_TC                   (txr_meta_tc[`SIG_TC_W-1:0]),
         .TXR_META_ATTR                 (txr_meta_attr[`SIG_ATTR_W-1:0]),
         .TXR_META_TYPE                 (txr_meta_type[`SIG_TYPE_W-1:0]),
         .TXR_META_EP                   (txr_meta_ep),
         // Unconnected Inputs
         .RX_TLP                        (wRxTlp_nc),
         .RX_TLP_VALID                  (wRxTlpValid_nc),
         .RX_TLP_START_FLAG             (wRxTlpStartFlag_nc),
         .RX_TLP_START_OFFSET           (wRxTlpStartOffset_nc),
         .RX_TLP_END_FLAG               (wRxTlpEndFlag_nc),
         .RX_TLP_END_OFFSET             (wRxTlpEndOffset_nc),
         .RX_TLP_BAR_DECODE             (wRxTlpBarDecode_nc),

         .TX_TLP_READY                  (wTxTlpReady_nc),
         /*AUTOINST*/
         // Outputs
         .M_AXIS_CQ_TREADY              (M_AXIS_CQ_TREADY),
         .M_AXIS_RC_TREADY              (M_AXIS_RC_TREADY),
         .S_AXIS_CC_TVALID              (S_AXIS_CC_TVALID),
         .S_AXIS_CC_TLAST               (S_AXIS_CC_TLAST),
         .S_AXIS_CC_TDATA               (S_AXIS_CC_TDATA[C_PCI_DATA_WIDTH-1:0]),
         .S_AXIS_CC_TKEEP               (S_AXIS_CC_TKEEP[(C_PCI_DATA_WIDTH/32)-1:0]),
         .S_AXIS_CC_TUSER               (S_AXIS_CC_TUSER[`SIG_CC_TUSER_W-1:0]),
         .S_AXIS_RQ_TVALID              (S_AXIS_RQ_TVALID),
         .S_AXIS_RQ_TLAST               (S_AXIS_RQ_TLAST),
         .S_AXIS_RQ_TDATA               (S_AXIS_RQ_TDATA[C_PCI_DATA_WIDTH-1:0]),
         .S_AXIS_RQ_TKEEP               (S_AXIS_RQ_TKEEP[(C_PCI_DATA_WIDTH/32)-1:0]),
         .S_AXIS_RQ_TUSER               (S_AXIS_RQ_TUSER[`SIG_RQ_TUSER_W-1:0]),
         // Inputs
         .M_AXIS_CQ_TVALID              (M_AXIS_CQ_TVALID),
         .M_AXIS_CQ_TLAST               (M_AXIS_CQ_TLAST),
         .M_AXIS_CQ_TDATA               (M_AXIS_CQ_TDATA[C_PCI_DATA_WIDTH-1:0]),
         .M_AXIS_CQ_TKEEP               (M_AXIS_CQ_TKEEP[(C_PCI_DATA_WIDTH/32)-1:0]),
         .M_AXIS_CQ_TUSER               (M_AXIS_CQ_TUSER[`SIG_CQ_TUSER_W-1:0]),
         .M_AXIS_RC_TVALID              (M_AXIS_RC_TVALID),
         .M_AXIS_RC_TLAST               (M_AXIS_RC_TLAST),
         .M_AXIS_RC_TDATA               (M_AXIS_RC_TDATA[C_PCI_DATA_WIDTH-1:0]),
         .M_AXIS_RC_TKEEP               (M_AXIS_RC_TKEEP[(C_PCI_DATA_WIDTH/32)-1:0]),
         .M_AXIS_RC_TUSER               (M_AXIS_RC_TUSER[`SIG_RC_TUSER_W-1:0]),
         .S_AXIS_CC_TREADY              (S_AXIS_CC_TREADY),
         .S_AXIS_RQ_TREADY              (S_AXIS_RQ_TREADY));

    riffa
        #(.C_TAG_WIDTH                  (C_LOG_NUM_TAGS),/* TODO: Standardize declaration*/
          /*AUTOINSTPARAM*/
          // Parameters
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
          .C_NUM_CHNL                   (C_NUM_CHNL),
          .C_MAX_READ_REQ_BYTES         (C_MAX_READ_REQ_BYTES),
          .C_VENDOR                     (C_VENDOR),
          .C_FPGA_NAME                  (C_FPGA_NAME))
    riffa_inst
        (// Outputs
         .TXC_DATA                      (txc_data[C_PCI_DATA_WIDTH-1:0]),
         .TXC_DATA_VALID                (txc_data_valid),
         .TXC_DATA_START_FLAG           (txc_data_start_flag),
         .TXC_DATA_START_OFFSET         (txc_data_start_offset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TXC_DATA_END_FLAG             (txc_data_end_flag),
         .TXC_DATA_END_OFFSET           (txc_data_end_offset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TXC_META_VALID                (txc_meta_valid),
         .TXC_META_FDWBE                (txc_meta_fdwbe[`SIG_FBE_W-1:0]),
         .TXC_META_LDWBE                (txc_meta_ldwbe[`SIG_LBE_W-1:0]),
         .TXC_META_ADDR                 (txc_meta_addr[`SIG_LOWADDR_W-1:0]),
         .TXC_META_TYPE                 (txc_meta_type[`SIG_TYPE_W-1:0]),
         .TXC_META_LENGTH               (txc_meta_length[`SIG_LEN_W-1:0]),
         .TXC_META_BYTE_COUNT           (txc_meta_byte_count[`SIG_BYTECNT_W-1:0]),
         .TXC_META_TAG                  (txc_meta_tag[`SIG_TAG_W-1:0]),
         .TXC_META_REQUESTER_ID         (txc_meta_requester_id[`SIG_REQID_W-1:0]),
         .TXC_META_TC                   (txc_meta_tc[`SIG_TC_W-1:0]),
         .TXC_META_ATTR                 (txc_meta_attr[`SIG_ATTR_W-1:0]),
         .TXC_META_EP                   (txc_meta_ep),

         .TXR_DATA_VALID                (txr_data_valid),
         .TXR_DATA                      (txr_data[C_PCI_DATA_WIDTH-1:0]),
         .TXR_DATA_START_FLAG           (txr_data_start_flag),
         .TXR_DATA_START_OFFSET         (txr_data_start_offset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TXR_DATA_END_FLAG             (txr_data_end_flag),
         .TXR_DATA_END_OFFSET           (txr_data_end_offset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TXR_META_VALID                (txr_meta_valid),
         .TXR_META_FDWBE                (txr_meta_fdwbe[`SIG_FBE_W-1:0]),
         .TXR_META_LDWBE                (txr_meta_ldwbe[`SIG_LBE_W-1:0]),
         .TXR_META_ADDR                 (txr_meta_addr[`SIG_ADDR_W-1:0]),
         .TXR_META_LENGTH               (txr_meta_length[`SIG_LEN_W-1:0]),
         .TXR_META_TAG                  (txr_meta_tag[`SIG_TAG_W-1:0]),
         .TXR_META_TC                   (txr_meta_tc[`SIG_TC_W-1:0]),
         .TXR_META_ATTR                 (txr_meta_attr[`SIG_ATTR_W-1:0]),
         .TXR_META_TYPE                 (txr_meta_type[`SIG_TYPE_W-1:0]),
         .TXR_META_EP                   (txr_meta_ep),

         .INTR_MSI_REQUEST              (intr_msi_request),
         // Inputs
         .CLK                           (clk),
         .RST_IN                        (rst_in),
         .RXR_DATA                      (rxr_data[C_PCI_DATA_WIDTH-1:0]),
         .RXR_DATA_VALID                (rxr_data_valid),
         .RXR_DATA_START_FLAG           (rxr_data_start_flag),
         .RXR_DATA_START_OFFSET         (rxr_data_start_offset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .RXR_DATA_WORD_ENABLE          (rxr_data_word_enable[(C_PCI_DATA_WIDTH/32)-1:0]),
         .RXR_DATA_END_FLAG             (rxr_data_end_flag),
         .RXR_DATA_END_OFFSET           (rxr_data_end_offset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .RXR_META_FDWBE                (rxr_meta_fdwbe[`SIG_FBE_W-1:0]),
         .RXR_META_LDWBE                (rxr_meta_ldwbe[`SIG_LBE_W-1:0]),
         .RXR_META_TC                   (rxr_meta_tc[`SIG_TC_W-1:0]),
         .RXR_META_ATTR                 (rxr_meta_attr[`SIG_ATTR_W-1:0]),
         .RXR_META_TAG                  (rxr_meta_tag[`SIG_TAG_W-1:0]),
         .RXR_META_TYPE                 (rxr_meta_type[`SIG_TYPE_W-1:0]),
         .RXR_META_ADDR                 (rxr_meta_addr[`SIG_ADDR_W-1:0]),
         .RXR_META_BAR_DECODED          (rxr_meta_bar_decoded[`SIG_BARDECODE_W-1:0]),
         .RXR_META_REQUESTER_ID         (rxr_meta_requester_id[`SIG_REQID_W-1:0]),
         .RXR_META_LENGTH               (rxr_meta_length[`SIG_LEN_W-1:0]),
         .RXR_META_EP                   (rxr_meta_ep),

         .RXC_DATA_VALID                (rxc_data_valid),
         .RXC_DATA                      (rxc_data[C_PCI_DATA_WIDTH-1:0]),
         .RXC_DATA_START_FLAG           (rxc_data_start_flag),
         .RXC_DATA_START_OFFSET         (rxc_data_start_offset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .RXC_DATA_WORD_ENABLE          (rxc_data_word_enable[(C_PCI_DATA_WIDTH/32)-1:0]),
         .RXC_DATA_END_FLAG             (rxc_data_end_flag),
         .RXC_DATA_END_OFFSET           (rxc_data_end_offset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .RXC_META_FDWBE                (rxc_meta_fdwbe[`SIG_FBE_W-1:0]),
         .RXC_META_LDWBE                (rxc_meta_ldwbe[`SIG_LBE_W-1:0]),
         .RXC_META_TAG                  (rxc_meta_tag[`SIG_TAG_W-1:0]),
         .RXC_META_ADDR                 (rxc_meta_addr[`SIG_LOWADDR_W-1:0]),
         .RXC_META_TYPE                 (rxc_meta_type[`SIG_TYPE_W-1:0]),
         .RXC_META_LENGTH               (rxc_meta_length[`SIG_LEN_W-1:0]),
         .RXC_META_BYTES_REMAINING      (rxc_meta_bytes_remaining[`SIG_BYTECNT_W-1:0]),
         .RXC_META_COMPLETER_ID         (rxc_meta_completer_id[`SIG_CPLID_W-1:0]),
         .RXC_META_EP                   (rxc_meta_ep),

         .TXC_DATA_READY                (txc_data_ready),
         .TXC_META_READY                (txc_meta_ready),
         .TXC_SENT                      (txc_sent),         

         .TXR_DATA_READY                (txr_data_ready),
         .TXR_META_READY                (txr_meta_ready),
         .TXR_SENT                      (txr_sent),

         .CONFIG_COMPLETER_ID           (config_completer_id[`SIG_CPLID_W-1:0]),
         .CONFIG_BUS_MASTER_ENABLE      (config_bus_master_enable),
         .CONFIG_LINK_WIDTH             (config_link_width[`SIG_LINKWIDTH_W-1:0]),
         .CONFIG_LINK_RATE              (config_link_rate[`SIG_LINKRATE_W-1:0]),
         .CONFIG_MAX_READ_REQUEST_SIZE  (config_max_read_request_size[`SIG_MAXREAD_W-1:0]),
         .CONFIG_MAX_PAYLOAD_SIZE       (config_max_payload_size[`SIG_MAXPAYLOAD_W-1:0]),
         .CONFIG_INTERRUPT_MSIENABLE    (config_interrupt_msienable),
         .CONFIG_CPL_BOUNDARY_SEL       (config_cpl_boundary_sel),
         .CONFIG_MAX_CPL_DATA           (config_max_cpl_data[`SIG_FC_CPLD_W-1:0]),
         .CONFIG_MAX_CPL_HDR            (config_max_cpl_hdr[`SIG_FC_CPLH_W-1:0]),
        
         .INTR_MSI_RDY                  (intr_msi_rdy),

         /*AUTOINST*/
         // Outputs
         .RST_OUT                       (RST_OUT),
         .CHNL_RX                       (CHNL_RX[C_NUM_CHNL-1:0]),
         .CHNL_RX_LAST                  (CHNL_RX_LAST[C_NUM_CHNL-1:0]),
         .CHNL_RX_LEN                   (CHNL_RX_LEN[(C_NUM_CHNL*32)-1:0]),
         .CHNL_RX_OFF                   (CHNL_RX_OFF[(C_NUM_CHNL*31)-1:0]),
         .CHNL_RX_DATA                  (CHNL_RX_DATA[(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0]),
         .CHNL_RX_DATA_VALID            (CHNL_RX_DATA_VALID[C_NUM_CHNL-1:0]),
         .CHNL_TX_ACK                   (CHNL_TX_ACK[C_NUM_CHNL-1:0]),
         .CHNL_TX_DATA_REN              (CHNL_TX_DATA_REN[C_NUM_CHNL-1:0]),
         // Inputs
         .CHNL_RX_CLK                   (CHNL_RX_CLK[C_NUM_CHNL-1:0]),
         .CHNL_RX_ACK                   (CHNL_RX_ACK[C_NUM_CHNL-1:0]),
         .CHNL_RX_DATA_REN              (CHNL_RX_DATA_REN[C_NUM_CHNL-1:0]),
         .CHNL_TX_CLK                   (CHNL_TX_CLK[C_NUM_CHNL-1:0]),
         .CHNL_TX                       (CHNL_TX[C_NUM_CHNL-1:0]),
         .CHNL_TX_LAST                  (CHNL_TX_LAST[C_NUM_CHNL-1:0]),
         .CHNL_TX_LEN                   (CHNL_TX_LEN[(C_NUM_CHNL*32)-1:0]),
         .CHNL_TX_OFF                   (CHNL_TX_OFF[(C_NUM_CHNL*31)-1:0]),
         .CHNL_TX_DATA                  (CHNL_TX_DATA[(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0]),
         .CHNL_TX_DATA_VALID            (CHNL_TX_DATA_VALID[C_NUM_CHNL-1:0]));

endmodule
// Local Variables:
// verilog-library-directories:("../../engine/" "../../riffa/")
// End:

