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
// Filename:            riffa_wrapper_zc706.v
// Version:             1.00.a
// Verilog Standard:    Verilog-2001
// Description:         RIFFA wrapper for the ZC706 Development board.
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`include "trellis.vh"
`include "riffa.vh"
`include "xilinx.vh"
`include "ultrascale.vh"
`include "functions.vh"
`timescale 1ps / 1ps
module riffa_wrapper_zc706
    #(// Number of RIFFA Channels
      parameter C_NUM_CHNL = 1,
      // Bit-Width from Vivado IP Generator
      parameter C_PCI_DATA_WIDTH = 128,
      // 4-Byte Name for this FPGA
      parameter C_MAX_PAYLOAD_BYTES = 256,
      parameter C_LOG_NUM_TAGS = 5,
      parameter C_FPGA_ID = "Z706") 
    (// Interface: Xilinx RX 
     input [C_PCI_DATA_WIDTH-1:0]                 M_AXIS_RX_TDATA,
     input [(C_PCI_DATA_WIDTH/8)-1:0]             M_AXIS_RX_TKEEP,
     input                                        M_AXIS_RX_TLAST,
     input                                        M_AXIS_RX_TVALID,
     output                                       M_AXIS_RX_TREADY,
     input [`SIG_XIL_RX_TUSER_W-1:0]              M_AXIS_RX_TUSER,
     output                                       RX_NP_OK,
     output                                       RX_NP_REQ,

     // Interface: Xilinx TX
     output [C_PCI_DATA_WIDTH-1:0]                S_AXIS_TX_TDATA,
     output [(C_PCI_DATA_WIDTH/8)-1:0]            S_AXIS_TX_TKEEP,
     output                                       S_AXIS_TX_TLAST,
     output                                       S_AXIS_TX_TVALID,
     input                                        S_AXIS_TX_TREADY,
     output [`SIG_XIL_TX_TUSER_W-1:0]             S_AXIS_TX_TUSER,
     output                                       TX_CFG_GNT,

     // Interface: Xilinx Configuration 
     input [`SIG_BUSID_W-1:0]                     CFG_BUS_NUMBER,
     input [`SIG_DEVID_W-1:0]                     CFG_DEVICE_NUMBER,
     input [`SIG_FNID_W-1:0]                      CFG_FUNCTION_NUMBER,
     input [`SIG_CFGREG_W-1:0]                    CFG_COMMAND,
     input [`SIG_CFGREG_W-1:0]                    CFG_DCOMMAND,
     input [`SIG_CFGREG_W-1:0]                    CFG_LSTATUS,
     input [`SIG_CFGREG_W-1:0]                    CFG_LCOMMAND,

     // Interface: Xilinx Flow Control
     input [`SIG_FC_CPLD_W-1:0]                   FC_CPLD,
     input [`SIG_FC_CPLH_W-1:0]                   FC_CPLH,
     output [`SIG_FC_SEL_W-1:0]                   FC_SEL,
    
     // Interface: Xilinx Interrupt
     input                                        CFG_INTERRUPT_MSIEN,
     input                                        CFG_INTERRUPT_RDY,
     output                                       CFG_INTERRUPT,

     input                                        USER_CLK,
     input                                        USER_RESET,

    
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
     output [C_NUM_CHNL-1:0]                      CHNL_TX_DATA_REN); // Channel write data has been recieved
     
    localparam C_FPGA_NAME = "REGT"; // This is not yet exposed in the driver
    localparam C_MAX_READ_REQ_BYTES = C_MAX_PAYLOAD_BYTES * 2;
    // ALTERA, XILINX or ULTRASCALE
    localparam C_VENDOR = "XILINX";

    localparam C_KEEP_WIDTH = C_PCI_DATA_WIDTH / 32;
    localparam C_PIPELINE_OUTPUT = 1;
    localparam C_PIPELINE_INPUT = 1;
    localparam C_DEPTH_PACKETS = 4;
    

    wire                                          clk;
    wire                                          rst_in;

    wire                                          done_txc_rst;
    wire                                          done_txr_rst;
    wire                                          done_rxr_rst;
    wire                                          done_rxc_rst;
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

    // Classic Interface Wires
    wire                                          rx_tlp_ready;
    wire [C_PCI_DATA_WIDTH-1:0]                   rx_tlp;
    wire                                          rx_tlp_end_flag;
    wire [`SIG_OFFSET_W-1:0]                      rx_tlp_end_offset;
    wire                                          rx_tlp_start_flag;
    wire [`SIG_OFFSET_W-1:0]                      rx_tlp_start_offset;
    wire                                          rx_tlp_valid;
    wire [`SIG_BARDECODE_W-1:0]                   rx_tlp_bar_decode;
    
    wire                                          tx_tlp_ready;
    wire [C_PCI_DATA_WIDTH-1:0]                   tx_tlp;
    wire                                          tx_tlp_end_flag;
    wire [`SIG_OFFSET_W-1:0]                      tx_tlp_end_offset;
    wire                                          tx_tlp_start_flag;
    wire [`SIG_OFFSET_W-1:0]                      tx_tlp_start_offset;
    wire                                          tx_tlp_valid;
    
    // Unconnected Wires (Used in ultrascale interface)
    // Interface: RQ (TXC)
    wire                                          s_axis_rq_tlast_nc;
    wire [C_PCI_DATA_WIDTH-1:0]                   s_axis_rq_tdata_nc;
    wire [`SIG_RQ_TUSER_W-1:0]                    s_axis_rq_tuser_nc;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]              s_axis_rq_tkeep_nc;
    wire                                          s_axis_rq_tready_nc = 0;
    wire                                          s_axis_rq_tvalid_nc;
    // Interface: RC (RXC)
    wire [C_PCI_DATA_WIDTH-1:0]                   m_axis_rc_tdata_nc = 0;
    wire [`SIG_RC_TUSER_W-1:0]                    m_axis_rc_tuser_nc = 0;
    wire                                          m_axis_rc_tlast_nc = 0;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]              m_axis_rc_tkeep_nc = 0;
    wire                                          m_axis_rc_tvalid_nc = 0;
    wire                                          m_axis_rc_tready_nc;
    // Interface: CQ (RXR)
    wire [C_PCI_DATA_WIDTH-1:0]                   m_axis_cq_tdata_nc = 0;
    wire [`SIG_CQ_TUSER_W-1:0]                    m_axis_cq_tuser_nc = 0;
    wire                                          m_axis_cq_tlast_nc = 0;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]              m_axis_cq_tkeep_nc = 0;
    wire                                          m_axis_cq_tvalid_nc = 0;
    wire                                          m_axis_cq_tready_nc = 0;
    // Interface: CC (TXC)
    wire [C_PCI_DATA_WIDTH-1:0]                   s_axis_cc_tdata_nc;
    wire [`SIG_CC_TUSER_W-1:0]                    s_axis_cc_tuser_nc;
    wire                                          s_axis_cc_tlast_nc;
    wire [(C_PCI_DATA_WIDTH/32)-1:0]              s_axis_cc_tkeep_nc;
    wire                                          s_axis_cc_tvalid_nc;
    wire                                          s_axis_cc_tready_nc = 0;

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

    reg                                           rRxTlpValid;
    reg                                           rRxTlpEndFlag;

    assign clk = USER_CLK;
    assign rst_in = USER_RESET;

    translation_xilinx
        #(/*AUTOINSTPARAM*/
          // Parameters
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH))
    trans
        (// Outputs
         .RX_TLP                        (rx_tlp[C_PCI_DATA_WIDTH-1:0]),
         .RX_TLP_VALID                  (rx_tlp_valid),
         .RX_TLP_START_FLAG             (rx_tlp_start_flag),
         .RX_TLP_START_OFFSET           (rx_tlp_start_offset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .RX_TLP_END_FLAG               (rx_tlp_end_flag),
         .RX_TLP_END_OFFSET             (rx_tlp_end_offset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .RX_TLP_BAR_DECODE             (rx_tlp_bar_decode[`SIG_BARDECODE_W-1:0]),
         .TX_TLP_READY                  (tx_tlp_ready),
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
         // Inputs
         .CLK                           (clk),
         .RST_IN                        (rst_in),
         .RX_TLP_READY                  (rx_tlp_ready),
         .TX_TLP                        (tx_tlp[C_PCI_DATA_WIDTH-1:0]),
         .TX_TLP_VALID                  (tx_tlp_valid),
         .TX_TLP_START_FLAG             (tx_tlp_start_flag),
         .TX_TLP_START_OFFSET           (tx_tlp_start_offset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .TX_TLP_END_FLAG               (tx_tlp_end_flag),
         .TX_TLP_END_OFFSET             (tx_tlp_end_offset[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
         .INTR_MSI_REQUEST              (intr_msi_request),
         /*AUTOINST*/
         // Outputs
         .M_AXIS_RX_TREADY              (M_AXIS_RX_TREADY),
         .RX_NP_OK                      (RX_NP_OK),
         .RX_NP_REQ                     (RX_NP_REQ),
         .S_AXIS_TX_TDATA               (S_AXIS_TX_TDATA[C_PCI_DATA_WIDTH-1:0]),
         .S_AXIS_TX_TKEEP               (S_AXIS_TX_TKEEP[(C_PCI_DATA_WIDTH/8)-1:0]),
         .S_AXIS_TX_TLAST               (S_AXIS_TX_TLAST),
         .S_AXIS_TX_TVALID              (S_AXIS_TX_TVALID),
         .S_AXIS_TX_TUSER               (S_AXIS_TX_TUSER[`SIG_XIL_TX_TUSER_W-1:0]),
         .TX_CFG_GNT                    (TX_CFG_GNT),
         .FC_SEL                        (FC_SEL[`SIG_FC_SEL_W-1:0]),
         .CFG_INTERRUPT                 (CFG_INTERRUPT),
         // Inputs
         .M_AXIS_RX_TDATA               (M_AXIS_RX_TDATA[C_PCI_DATA_WIDTH-1:0]),
         .M_AXIS_RX_TKEEP               (M_AXIS_RX_TKEEP[(C_PCI_DATA_WIDTH/8)-1:0]),
         .M_AXIS_RX_TLAST               (M_AXIS_RX_TLAST),
         .M_AXIS_RX_TVALID              (M_AXIS_RX_TVALID),
         .M_AXIS_RX_TUSER               (M_AXIS_RX_TUSER[`SIG_XIL_RX_TUSER_W-1:0]),
         .S_AXIS_TX_TREADY              (S_AXIS_TX_TREADY),
         .CFG_BUS_NUMBER                (CFG_BUS_NUMBER[`SIG_BUSID_W-1:0]),
         .CFG_DEVICE_NUMBER             (CFG_DEVICE_NUMBER[`SIG_DEVID_W-1:0]),
         .CFG_FUNCTION_NUMBER           (CFG_FUNCTION_NUMBER[`SIG_FNID_W-1:0]),
         .CFG_COMMAND                   (CFG_COMMAND[`SIG_CFGREG_W-1:0]),
         .CFG_DCOMMAND                  (CFG_DCOMMAND[`SIG_CFGREG_W-1:0]),
         .CFG_LSTATUS                   (CFG_LSTATUS[`SIG_CFGREG_W-1:0]),
         .CFG_LCOMMAND                  (CFG_LCOMMAND[`SIG_CFGREG_W-1:0]),
         .FC_CPLD                       (FC_CPLD[`SIG_FC_CPLD_W-1:0]),
         .FC_CPLH                       (FC_CPLH[`SIG_FC_CPLH_W-1:0]),
         .CFG_INTERRUPT_MSIEN           (CFG_INTERRUPT_MSIEN),
         .CFG_INTERRUPT_RDY             (CFG_INTERRUPT_RDY));

    engine_layer
        #(// Parameters
          .C_MAX_PAYLOAD_DWORDS         (C_MAX_PAYLOAD_BYTES/4),
          /*AUTOINSTPARAM*/
          // Parameters
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
          .C_LOG_NUM_TAGS               (C_LOG_NUM_TAGS),
          .C_PIPELINE_INPUT             (C_PIPELINE_INPUT),
          .C_PIPELINE_OUTPUT            (C_PIPELINE_OUTPUT),
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
         .RST_LOGIC                     (RST_OUT),
         // Unconnected Outputs
         .TX_TLP                        (tx_tlp),
         .TX_TLP_VALID                  (tx_tlp_valid),
         .TX_TLP_START_FLAG             (tx_tlp_start_flag),
         .TX_TLP_START_OFFSET           (tx_tlp_start_offset),
         .TX_TLP_END_FLAG               (tx_tlp_end_flag),
         .TX_TLP_END_OFFSET             (tx_tlp_end_offset),

         .RX_TLP_READY                  (rx_tlp_ready),
         // Inputs
         .CLK_BUS                       (clk),
         .RST_BUS                       (rst_in),

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
         .RX_TLP                        (rx_tlp),
         .RX_TLP_VALID                  (rx_tlp_valid),
         .RX_TLP_START_FLAG             (rx_tlp_start_flag),
         .RX_TLP_START_OFFSET           (rx_tlp_start_offset),
         .RX_TLP_END_FLAG               (rx_tlp_end_flag),
         .RX_TLP_END_OFFSET             (rx_tlp_end_offset),
         .RX_TLP_BAR_DECODE             (rx_tlp_bar_decode),

         .TX_TLP_READY                  (tx_tlp_ready),
         .DONE_TXC_RST                  (done_txc_rst),
         .DONE_TXR_RST                  (done_txr_rst),
         .DONE_RXR_RST                  (done_rxc_rst),
         .DONE_RXC_RST                  (done_rxr_rst),
         // Outputs
         .M_AXIS_CQ_TREADY              (m_axis_cq_tready_nc),
         .M_AXIS_RC_TREADY              (m_axis_rc_tready_nc),
         .S_AXIS_CC_TVALID              (s_axis_cc_tvalid_nc),
         .S_AXIS_CC_TLAST               (s_axis_cc_tlast_nc),
         .S_AXIS_CC_TDATA               (s_axis_cc_tdata_nc[C_PCI_DATA_WIDTH-1:0]),
         .S_AXIS_CC_TKEEP               (s_axis_cc_tkeep_nc[(C_PCI_DATA_WIDTH/32)-1:0]),
         .S_AXIS_CC_TUSER               (s_axis_cc_tuser_nc[`SIG_CC_TUSER_W-1:0]),
         .S_AXIS_RQ_TVALID              (s_axis_rq_tvalid_nc),
         .S_AXIS_RQ_TLAST               (s_axis_rq_tlast_nc),
         .S_AXIS_RQ_TDATA               (s_axis_rq_tdata_nc[C_PCI_DATA_WIDTH-1:0]),
         .S_AXIS_RQ_TKEEP               (s_axis_rq_tkeep_nc[(C_PCI_DATA_WIDTH/32)-1:0]),
         .S_AXIS_RQ_TUSER               (s_axis_rq_tuser_nc[`SIG_RQ_TUSER_W-1:0]),
         // Inputs
         .M_AXIS_CQ_TVALID              (m_axis_cq_tvalid_nc),
         .M_AXIS_CQ_TLAST               (m_axis_cq_tlast_nc),
         .M_AXIS_CQ_TDATA               (m_axis_cq_tdata_nc[C_PCI_DATA_WIDTH-1:0]),
         .M_AXIS_CQ_TKEEP               (m_axis_cq_tkeep_nc[(C_PCI_DATA_WIDTH/32)-1:0]),
         .M_AXIS_CQ_TUSER               (m_axis_cq_tuser_nc[`SIG_CQ_TUSER_W-1:0]),
         .M_AXIS_RC_TVALID              (m_axis_rc_tvalid_nc),
         .M_AXIS_RC_TLAST               (m_axis_rc_tlast_nc),
         .M_AXIS_RC_TDATA               (m_axis_rc_tdata_nc[C_PCI_DATA_WIDTH-1:0]),
         .M_AXIS_RC_TKEEP               (m_axis_rc_tkeep_nc[(C_PCI_DATA_WIDTH/32)-1:0]),
         .M_AXIS_RC_TUSER               (m_axis_rc_tuser_nc[`SIG_RC_TUSER_W-1:0]),
         .S_AXIS_CC_TREADY              (s_axis_cc_tready_nc),
         .S_AXIS_RQ_TREADY              (s_axis_rq_tready_nc)
         /*AUTOINST*/);

    riffa
        #(.C_TAG_WIDTH                  (C_LOG_NUM_TAGS),/* TODO: Standardize declaration*/
          /*AUTOINSTPARAM*/
          // Parameters
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
          .C_NUM_CHNL                   (C_NUM_CHNL),
          .C_MAX_READ_REQ_BYTES         (C_MAX_READ_REQ_BYTES),
          .C_VENDOR                     (C_VENDOR),
          .C_FPGA_NAME                  (C_FPGA_NAME),
          .C_FPGA_ID                    (C_FPGA_ID),
          .C_DEPTH_PACKETS              (C_DEPTH_PACKETS))
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

         .DONE_TXC_RST                  (done_txc_rst),
         .DONE_TXR_RST                  (done_txr_rst),
         .RST_BUS                       (rst_in),
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
// verilog-library-directories:("../../riffa_hdl/")
// End:
