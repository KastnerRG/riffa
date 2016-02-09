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
// Filename:            ADM7V3_Gen2x8If128.v
// Version:             1.00.a
// Verilog Standard:    Verilog-2001
// Description:         Top level module for RIFFA 2.2 reference design for the
//                      the Xilinx 7V3 Development Board.
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`include "functions.vh"
`include "riffa.vh"
`include "ultrascale.vh"
`timescale 1ps / 1ps
module ADM7V3_Gen2x8If128
    #(// Number of RIFFA Channels
      parameter C_NUM_CHNL = 1,
      // Number of PCIe Lanes
      parameter C_NUM_LANES =  8,
      // Settings from Vivado IP Generator
      parameter C_PCI_DATA_WIDTH = 128,
      parameter C_MAX_PAYLOAD_BYTES = 256,
      parameter C_LOG_NUM_TAGS = 6
      ) 
    (output [(C_NUM_LANES - 1) : 0] PCI_EXP_TXP,
     output [(C_NUM_LANES - 1) : 0] PCI_EXP_TXN,
     input [(C_NUM_LANES - 1) : 0]  PCI_EXP_RXP,
     input [(C_NUM_LANES - 1) : 0]  PCI_EXP_RXN,

     output [5:0]                   LED,
     input                          PCIE_REFCLK_P,
     input                          PCIE_REFCLK_N,
     input                          PCIE_RESET_N
     );

    // Clocks, etc
    wire                            user_lnk_up;
    wire                            user_clk;
    wire                            user_reset;
    wire                            pcie_refclk;
    wire                            pcie_reset_n;

    // Interface: RQ (TXC)
    wire                            s_axis_rq_tlast;
    wire [C_PCI_DATA_WIDTH-1:0]     s_axis_rq_tdata;
    wire [`SIG_RQ_TUSER_W-1:0]      s_axis_rq_tuser;
    wire [(C_PCI_DATA_WIDTH/32)-1:0] s_axis_rq_tkeep;
    wire                             s_axis_rq_tready;
    wire                             s_axis_rq_tvalid;
    // Interface: RC (RXC)
    wire [C_PCI_DATA_WIDTH-1:0]      m_axis_rc_tdata;
    wire [`SIG_RC_TUSER_W-1:0]       m_axis_rc_tuser;
    wire                             m_axis_rc_tlast;
    wire [(C_PCI_DATA_WIDTH/32)-1:0] m_axis_rc_tkeep;
    wire                             m_axis_rc_tvalid;
    wire                             m_axis_rc_tready;
    // Interface: CQ (RXR)
    wire [C_PCI_DATA_WIDTH-1:0]      m_axis_cq_tdata;
    wire [`SIG_CQ_TUSER_W-1:0]       m_axis_cq_tuser;
    wire                             m_axis_cq_tlast;
    wire [(C_PCI_DATA_WIDTH/32)-1:0] m_axis_cq_tkeep;
    wire                             m_axis_cq_tvalid;
    wire                             m_axis_cq_tready;
    // Interface: CC (TXC)
    wire [C_PCI_DATA_WIDTH-1:0]      s_axis_cc_tdata;
    wire [`SIG_CC_TUSER_W-1:0]       s_axis_cc_tuser;
    wire                             s_axis_cc_tlast;
    wire [(C_PCI_DATA_WIDTH/32)-1:0] s_axis_cc_tkeep;
    wire                             s_axis_cc_tvalid;
    wire                             s_axis_cc_tready;

    // Configuration (CFG) Interface                                           
    wire [3:0]                       pcie_rq_seq_num;
    wire                             pcie_rq_seq_num_vld;
    wire [5:0]                       pcie_rq_tag;
    wire                             pcie_rq_tag_vld;
    wire                             pcie_cq_np_req;
    wire [5:0]                       pcie_cq_np_req_count;

    wire                             cfg_phy_link_down;
    wire [3:0]                       cfg_negotiated_width; // CONFIG_LINK_WIDTH
    wire [2:0]                       cfg_current_speed; // CONFIG_LINK_RATE
    wire [2:0]                       cfg_max_payload; // CONFIG_MAX_PAYLOAD
    wire [2:0]                       cfg_max_read_req; // CONFIG_MAX_READ_REQUEST
    wire [7:0]                       cfg_function_status; // [2] = CONFIG_BUS_MASTER_ENABLE
    wire [5:0]                       cfg_function_power_state; // Ignorable but not removable
    wire [11:0]                      cfg_vf_status; // Ignorable but not removable
    wire [17:0]                      cfg_vf_power_state; // Ignorable but not removable
    wire [1:0]                       cfg_link_power_state; // Ignorable but not removable

    // Error Reporting Interface
    wire                             cfg_err_cor_out;
    wire                             cfg_err_nonfatal_out;
    wire                             cfg_err_fatal_out;

    wire                             cfg_ltr_enable;
    wire [5:0]                       cfg_ltssm_state;// TODO: Connect to LED's
    wire [1:0]                       cfg_rcb_status;
    wire [1:0]                       cfg_dpa_substate_change;
    wire [1:0]                       cfg_obff_enable;
    wire                             cfg_pl_status_change;

    wire [1:0]                       cfg_tph_requester_enable;
    wire [5:0]                       cfg_tph_st_mode;
    wire [5:0]                       cfg_vf_tph_requester_enable;
    wire [17:0]                      cfg_vf_tph_st_mode;
    wire [7:0]                       cfg_fc_ph;
    wire [11:0]                      cfg_fc_pd;
    wire [7:0]                       cfg_fc_nph;
    wire [11:0]                      cfg_fc_npd;
    wire [7:0]                       cfg_fc_cplh;
    wire [11:0]                      cfg_fc_cpld;
    wire [2:0]                       cfg_fc_sel;

    // Interrupt Interface Signals
    wire [3:0]                       cfg_interrupt_int;
    wire [1:0]                       cfg_interrupt_pending;
    wire                             cfg_interrupt_sent;
    wire [1:0]                       cfg_interrupt_msi_enable;
    wire [5:0]                       cfg_interrupt_msi_vf_enable;
    wire [5:0]                       cfg_interrupt_msi_mmenable;
    wire                             cfg_interrupt_msi_mask_update;
    wire [31:0]                      cfg_interrupt_msi_data;
    wire [3:0]                       cfg_interrupt_msi_select;
    wire [31:0]                      cfg_interrupt_msi_int;
    wire [63:0]                      cfg_interrupt_msi_pending_status;
    wire                             cfg_interrupt_msi_sent;
    wire                             cfg_interrupt_msi_fail;
    wire [2:0]                       cfg_interrupt_msi_attr;
    wire                             cfg_interrupt_msi_tph_present;
    wire [1:0]                       cfg_interrupt_msi_tph_type;
    wire [8:0]                       cfg_interrupt_msi_tph_st_tag;
    wire [2:0]                       cfg_interrupt_msi_function_number;

    wire                             rst_out;
    wire [C_NUM_CHNL-1:0]            chnl_rx_clk; 
    wire [C_NUM_CHNL-1:0]            chnl_rx; 
    wire [C_NUM_CHNL-1:0]            chnl_rx_ack; 
    wire [C_NUM_CHNL-1:0]            chnl_rx_last; 
    wire [(C_NUM_CHNL*`SIG_CHNL_LENGTH_W)-1:0] chnl_rx_len; 
    wire [(C_NUM_CHNL*`SIG_CHNL_OFFSET_W)-1:0] chnl_rx_off; 
    wire [(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0]   chnl_rx_data; 
    wire [C_NUM_CHNL-1:0]                      chnl_rx_data_valid; 
    wire [C_NUM_CHNL-1:0]                      chnl_rx_data_ren;

    wire [C_NUM_CHNL-1:0]                      chnl_tx_clk; 
    wire [C_NUM_CHNL-1:0]                      chnl_tx; 
    wire [C_NUM_CHNL-1:0]                      chnl_tx_ack;
    wire [C_NUM_CHNL-1:0]                      chnl_tx_last; 
    wire [(C_NUM_CHNL*`SIG_CHNL_LENGTH_W)-1:0] chnl_tx_len; 
    wire [(C_NUM_CHNL*`SIG_CHNL_OFFSET_W)-1:0] chnl_tx_off; 
    wire [(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0]   chnl_tx_data; 
    wire [C_NUM_CHNL-1:0]                      chnl_tx_data_valid; 
    wire [C_NUM_CHNL-1:0]                      chnl_tx_data_ren;

    genvar                                     chnl;

    IBUF 
        #()  
    pci_reset_n_ibuf 
        (.O(pcie_reset_n), 
         .I(PCIE_RESET_N));

    IBUFDS_GTE2 
        #()
    refclk_ibuf 
        (.O(pcie_refclk), 
         .ODIV2(), 
         .I(PCIE_REFCLK_P), 
         .CEB(1'b0), 
         .IB(PCIE_REFCLK_N));

    OBUF 
        #() 
    led_0_obuf 
        (.O(LED[0]),
         .I(cfg_ltssm_state[0]));
    OBUF 
        #() 
    led_1_obuf 
        (.O(LED[1]),
         .I(cfg_ltssm_state[1]));
    OBUF 
        #() 
    led_2_obuf 
        (.O(LED[2]),
         .I(cfg_ltssm_state[2]));
    OBUF 
        #() 
    led_3_obuf 
        (.O(LED[3]),
         .I(cfg_ltssm_state[3]));
    OBUF 
        #() 
    led_4_obuf 
        (.O(LED[4]),
         .I(cfg_ltssm_state[4]));
    OBUF 
        #() 
    led_5_obuf 
        (.O(LED[5]),
         .I(cfg_ltssm_state[5]));
    OBUF 
        #() 
    led_6_obuf 
        (.O(LED[6]),
         .I(pcie_reset_n));
    OBUF 
        #() 
    led_7_obuf 
        (.O(LED[7]),
         .I(rst_out));

    // Core Top Level Wrapper
    PCIeGen2x8If128  
        #()
    pcie3_7x_0_i 
        (//---------------------------------------------------------------------
         //  PCI Express (pci_exp) Interface                                    
         //---------------------------------------------------------------------
         .pci_exp_txn                                    ( PCI_EXP_TXN ),
         .pci_exp_txp                                    ( PCI_EXP_TXP ),
         .pci_exp_rxn                                    ( PCI_EXP_RXN ),
         .pci_exp_rxp                                    ( PCI_EXP_RXP ),

         //---------------------------------------------------------------------
         //  AXI Interface                                                      
         //---------------------------------------------------------------------
         .user_clk                                       ( user_clk ),
         .user_reset                                     ( user_reset ),
         .user_lnk_up                                    ( user_lnk_up ),
         .user_app_rdy                                   ( ),

         .s_axis_rq_tlast                                ( s_axis_rq_tlast ),
         .s_axis_rq_tdata                                ( s_axis_rq_tdata ),
         .s_axis_rq_tuser                                ( s_axis_rq_tuser ),
         .s_axis_rq_tkeep                                ( s_axis_rq_tkeep ),
         .s_axis_rq_tready                               ( s_axis_rq_tready ),
         .s_axis_rq_tvalid                               ( s_axis_rq_tvalid ),

         .m_axis_rc_tdata                                ( m_axis_rc_tdata ),
         .m_axis_rc_tuser                                ( m_axis_rc_tuser ),
         .m_axis_rc_tlast                                ( m_axis_rc_tlast ),
         .m_axis_rc_tkeep                                ( m_axis_rc_tkeep ),
         .m_axis_rc_tvalid                               ( m_axis_rc_tvalid ),
         .m_axis_rc_tready                               ( {22{m_axis_rc_tready}} ),

         .m_axis_cq_tdata                                ( m_axis_cq_tdata ),
         .m_axis_cq_tuser                                ( m_axis_cq_tuser ),
         .m_axis_cq_tlast                                ( m_axis_cq_tlast ),
         .m_axis_cq_tkeep                                ( m_axis_cq_tkeep ),
         .m_axis_cq_tvalid                               ( m_axis_cq_tvalid ),
         .m_axis_cq_tready                               ( {22{m_axis_cq_tready}} ),

         .s_axis_cc_tdata                                ( s_axis_cc_tdata ),
         .s_axis_cc_tuser                                ( s_axis_cc_tuser ),
         .s_axis_cc_tlast                                ( s_axis_cc_tlast ),
         .s_axis_cc_tkeep                                ( s_axis_cc_tkeep ),
         .s_axis_cc_tvalid                               ( s_axis_cc_tvalid ),
         .s_axis_cc_tready                               ( s_axis_cc_tready ),

         //---------------------------------------------------------------------
         //  Configuration (CFG) Interface                                      
         //---------------------------------------------------------------------
         .pcie_rq_seq_num                                ( pcie_rq_seq_num ),
         .pcie_rq_seq_num_vld                            ( pcie_rq_seq_num_vld ),
         .pcie_rq_tag                                    ( pcie_rq_tag ),
         .pcie_rq_tag_vld                                ( pcie_rq_tag_vld ),
         .pcie_cq_np_req                                 ( pcie_cq_np_req ),
         .pcie_cq_np_req_count                           ( pcie_cq_np_req_count ),
         .cfg_phy_link_down                              ( cfg_phy_link_down ),
         .cfg_phy_link_status                            ( cfg_phy_link_status),
         .cfg_negotiated_width                           ( cfg_negotiated_width ),
         .cfg_current_speed                              ( cfg_current_speed ),
         .cfg_max_payload                                ( cfg_max_payload ),
         .cfg_max_read_req                               ( cfg_max_read_req ),
         .cfg_function_status                            ( cfg_function_status ),
         .cfg_function_power_state                       ( cfg_function_power_state ),
         .cfg_vf_status                                  ( cfg_vf_status ),
         .cfg_vf_power_state                             ( cfg_vf_power_state ),
         .cfg_link_power_state                           ( cfg_link_power_state ),
         // Error Reporting Interface
         .cfg_err_cor_out                                ( cfg_err_cor_out ),
         .cfg_err_nonfatal_out                           ( cfg_err_nonfatal_out ),
         .cfg_err_fatal_out                              ( cfg_err_fatal_out ),
         .cfg_ltr_enable                                 ( cfg_ltr_enable ),
         .cfg_ltssm_state                                ( cfg_ltssm_state ),
         .cfg_rcb_status                                 ( cfg_rcb_status ),
         .cfg_dpa_substate_change                        ( cfg_dpa_substate_change ),
         .cfg_obff_enable                                ( cfg_obff_enable ),
         .cfg_pl_status_change                           ( cfg_pl_status_change ),
         .cfg_tph_requester_enable                       ( cfg_tph_requester_enable ),
         .cfg_tph_st_mode                                ( cfg_tph_st_mode ),
         .cfg_vf_tph_requester_enable                    ( cfg_vf_tph_requester_enable ),
         .cfg_vf_tph_st_mode                             ( cfg_vf_tph_st_mode ),
         .cfg_fc_ph                                      ( cfg_fc_ph ),
         .cfg_fc_pd                                      ( cfg_fc_pd ),
         .cfg_fc_nph                                     ( cfg_fc_nph ),
         .cfg_fc_npd                                     ( cfg_fc_npd ),
         .cfg_fc_cplh                                    ( cfg_fc_cplh ),
         .cfg_fc_cpld                                    ( cfg_fc_cpld ),
         .cfg_fc_sel                                     ( cfg_fc_sel ),
         //---------------------------------------------------------------------
         // EP Only                                                             
         //---------------------------------------------------------------------
         // Interrupt Interface Signals
         .cfg_interrupt_int                              ( cfg_interrupt_int ),
         .cfg_interrupt_pending                          ( cfg_interrupt_pending ),
         .cfg_interrupt_sent                             ( cfg_interrupt_sent ),
         .cfg_interrupt_msi_enable                       ( cfg_interrupt_msi_enable ),
         .cfg_interrupt_msi_vf_enable                    ( cfg_interrupt_msi_vf_enable ),
         .cfg_interrupt_msi_mmenable                     ( cfg_interrupt_msi_mmenable ),
         .cfg_interrupt_msi_mask_update                  ( cfg_interrupt_msi_mask_update ),
         .cfg_interrupt_msi_data                         ( cfg_interrupt_msi_data ),
         .cfg_interrupt_msi_select                       ( cfg_interrupt_msi_select ),
         .cfg_interrupt_msi_int                          ( cfg_interrupt_msi_int ),
         .cfg_interrupt_msi_pending_status               ( cfg_interrupt_msi_pending_status ),
         .cfg_interrupt_msi_sent                         ( cfg_interrupt_msi_sent ),
         .cfg_interrupt_msi_fail                         ( cfg_interrupt_msi_fail ),
         .cfg_interrupt_msi_attr                         ( cfg_interrupt_msi_attr ),
         .cfg_interrupt_msi_tph_present                  ( cfg_interrupt_msi_tph_present ),
         .cfg_interrupt_msi_tph_type                     ( cfg_interrupt_msi_tph_type ),
         .cfg_interrupt_msi_tph_st_tag                   ( cfg_interrupt_msi_tph_st_tag ),
         .cfg_interrupt_msi_function_number              ( cfg_interrupt_msi_function_number ),

         //---------------------------------------------------------------------
         //  System(SYS) Interface                                              
         //---------------------------------------------------------------------
         .sys_clk                                        (pcie_refclk),
         .sys_reset                                      (~pcie_reset_n));

    riffa_wrapper_adm7V3
        #(/*AUTOINSTPARAM*/
          // Parameters
          .C_NUM_CHNL                   (C_NUM_CHNL),
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
          .C_MAX_PAYLOAD_BYTES          (C_MAX_PAYLOAD_BYTES),
          .C_LOG_NUM_TAGS               (C_LOG_NUM_TAGS))
    riffa
        (// Outputs
         .M_AXIS_CQ_TREADY              (m_axis_cq_tready),
         .M_AXIS_RC_TREADY              (m_axis_rc_tready),
         .S_AXIS_CC_TVALID              (s_axis_cc_tvalid),
         .S_AXIS_CC_TLAST               (s_axis_cc_tlast),
         .S_AXIS_CC_TDATA               (s_axis_cc_tdata[C_PCI_DATA_WIDTH-1:0]),
         .S_AXIS_CC_TKEEP               (s_axis_cc_tkeep[(C_PCI_DATA_WIDTH/32)-1:0]),
         .S_AXIS_CC_TUSER               (s_axis_cc_tuser[`SIG_CC_TUSER_W-1:0]),
         .S_AXIS_RQ_TVALID              (s_axis_rq_tvalid),
         .S_AXIS_RQ_TLAST               (s_axis_rq_tlast),
         .S_AXIS_RQ_TDATA               (s_axis_rq_tdata[C_PCI_DATA_WIDTH-1:0]),
         .S_AXIS_RQ_TKEEP               (s_axis_rq_tkeep[(C_PCI_DATA_WIDTH/32)-1:0]),
         .S_AXIS_RQ_TUSER               (s_axis_rq_tuser[`SIG_RQ_TUSER_W-1:0]),
         .USER_CLK                      (user_clk),
         .USER_RESET                    (user_reset),
         .CFG_INTERRUPT_INT             (cfg_interrupt_int[3:0]),
         .CFG_INTERRUPT_PENDING         (cfg_interrupt_pending[1:0]),
         .CFG_INTERRUPT_MSI_SELECT      (cfg_interrupt_msi_select[3:0]),
         .CFG_INTERRUPT_MSI_INT         (cfg_interrupt_msi_int[31:0]),
         .CFG_INTERRUPT_MSI_PENDING_STATUS(cfg_interrupt_msi_pending_status[63:0]),
         .CFG_INTERRUPT_MSI_ATTR        (cfg_interrupt_msi_attr[2:0]),
         .CFG_INTERRUPT_MSI_TPH_PRESENT (cfg_interrupt_msi_tph_present),
         .CFG_INTERRUPT_MSI_TPH_TYPE    (cfg_interrupt_msi_tph_type[1:0]),
         .CFG_INTERRUPT_MSI_TPH_ST_TAG  (cfg_interrupt_msi_tph_st_tag[8:0]),
         .CFG_INTERRUPT_MSI_FUNCTION_NUMBER(cfg_interrupt_msi_function_number[2:0]),
         .CFG_FC_SEL                    (cfg_fc_sel[2:0]),
         .PCIE_CQ_NP_REQ                (pcie_cq_np_req),
         .RST_OUT                       (rst_out),
         .CHNL_RX                       (chnl_rx[C_NUM_CHNL-1:0]),
         .CHNL_RX_LAST                  (chnl_rx_last[C_NUM_CHNL-1:0]),
         .CHNL_RX_LEN                   (chnl_rx_len[(C_NUM_CHNL*`SIG_CHNL_LENGTH_W)-1:0]),
         .CHNL_RX_OFF                   (chnl_rx_off[(C_NUM_CHNL*`SIG_CHNL_OFFSET_W)-1:0]),
         .CHNL_RX_DATA                  (chnl_rx_data[(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0]),
         .CHNL_RX_DATA_VALID            (chnl_rx_data_valid[C_NUM_CHNL-1:0]),
         .CHNL_TX_ACK                   (chnl_tx_ack[C_NUM_CHNL-1:0]),
         .CHNL_TX_DATA_REN              (chnl_tx_data_ren[C_NUM_CHNL-1:0]),
         // Inputs
         .M_AXIS_CQ_TVALID              (m_axis_cq_tvalid),
         .M_AXIS_CQ_TLAST               (m_axis_cq_tlast),
         .M_AXIS_CQ_TDATA               (m_axis_cq_tdata[C_PCI_DATA_WIDTH-1:0]),
         .M_AXIS_CQ_TKEEP               (m_axis_cq_tkeep[(C_PCI_DATA_WIDTH/32)-1:0]),
         .M_AXIS_CQ_TUSER               (m_axis_cq_tuser[`SIG_CQ_TUSER_W-1:0]),
         .M_AXIS_RC_TVALID              (m_axis_rc_tvalid),
         .M_AXIS_RC_TLAST               (m_axis_rc_tlast),
         .M_AXIS_RC_TDATA               (m_axis_rc_tdata[C_PCI_DATA_WIDTH-1:0]),
         .M_AXIS_RC_TKEEP               (m_axis_rc_tkeep[(C_PCI_DATA_WIDTH/32)-1:0]),
         .M_AXIS_RC_TUSER               (m_axis_rc_tuser[`SIG_RC_TUSER_W-1:0]),
         .S_AXIS_CC_TREADY              (s_axis_cc_tready),
         .S_AXIS_RQ_TREADY              (s_axis_rq_tready),
         .CFG_INTERRUPT_MSI_ENABLE      (cfg_interrupt_msi_enable[1:0]),
         .CFG_INTERRUPT_MSI_MASK_UPDATE (cfg_interrupt_msi_mask_update),
         .CFG_INTERRUPT_MSI_DATA        (cfg_interrupt_msi_data[31:0]),
         .CFG_INTERRUPT_MSI_SENT        (cfg_interrupt_msi_sent),
         .CFG_INTERRUPT_MSI_FAIL        (cfg_interrupt_msi_fail),
         .CFG_FC_CPLH                   (cfg_fc_cplh[7:0]),
         .CFG_FC_CPLD                   (cfg_fc_cpld[11:0]),
         .CFG_NEGOTIATED_WIDTH          (cfg_negotiated_width[3:0]),
         .CFG_CURRENT_SPEED             (cfg_current_speed[2:0]),
         .CFG_MAX_PAYLOAD               (cfg_max_payload[2:0]),
         .CFG_MAX_READ_REQ              (cfg_max_read_req[2:0]),
         .CFG_FUNCTION_STATUS           (cfg_function_status[7:0]),
         .CFG_RCB_STATUS                (cfg_rcb_status[1:0]),
         .CHNL_RX_CLK                   (chnl_rx_clk[C_NUM_CHNL-1:0]),
         .CHNL_RX_ACK                   (chnl_rx_ack[C_NUM_CHNL-1:0]),
         .CHNL_RX_DATA_REN              (chnl_rx_data_ren[C_NUM_CHNL-1:0]),
         .CHNL_TX_CLK                   (chnl_tx_clk[C_NUM_CHNL-1:0]),
         .CHNL_TX                       (chnl_tx[C_NUM_CHNL-1:0]),
         .CHNL_TX_LAST                  (chnl_tx_last[C_NUM_CHNL-1:0]),
         .CHNL_TX_LEN                   (chnl_tx_len[(C_NUM_CHNL*`SIG_CHNL_LENGTH_W)-1:0]),
         .CHNL_TX_OFF                   (chnl_tx_off[(C_NUM_CHNL*`SIG_CHNL_OFFSET_W)-1:0]),
         .CHNL_TX_DATA                  (chnl_tx_data[(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0]),
         .CHNL_TX_DATA_VALID            (chnl_tx_data_valid[C_NUM_CHNL-1:0])
         /*AUTOINST*/);

    generate
        for (chnl = 0; chnl < C_NUM_CHNL; chnl = chnl + 1) begin : test_channels
            chnl_tester 
                    #(/*AUTOINSTPARAM*/
                      // Parameters
                      .C_PCI_DATA_WIDTH (C_PCI_DATA_WIDTH)) 
            module1 
                    (.CLK(user_clk),
                     .RST(rst_out),    // riffa_reset includes riffa_endpoint resets
                     // Rx interface
                     .CHNL_RX_CLK(chnl_rx_clk[chnl]), 
                     .CHNL_RX(chnl_rx[chnl]), 
                     .CHNL_RX_ACK(chnl_rx_ack[chnl]), 
                     .CHNL_RX_LAST(chnl_rx_last[chnl]), 
                     .CHNL_RX_LEN(chnl_rx_len[32*chnl +:32]), 
                     .CHNL_RX_OFF(chnl_rx_off[31*chnl +:31]), 
                     .CHNL_RX_DATA(chnl_rx_data[C_PCI_DATA_WIDTH*chnl +:C_PCI_DATA_WIDTH]), 
                     .CHNL_RX_DATA_VALID(chnl_rx_data_valid[chnl]), 
                     .CHNL_RX_DATA_REN(chnl_rx_data_ren[chnl]),
                     // Tx interface
                     .CHNL_TX_CLK(chnl_tx_clk[chnl]), 
                     .CHNL_TX(chnl_tx[chnl]), 
                     .CHNL_TX_ACK(chnl_tx_ack[chnl]), 
                     .CHNL_TX_LAST(chnl_tx_last[chnl]), 
                     .CHNL_TX_LEN(chnl_tx_len[32*chnl +:32]), 
                     .CHNL_TX_OFF(chnl_tx_off[31*chnl +:31]), 
                     .CHNL_TX_DATA(chnl_tx_data[C_PCI_DATA_WIDTH*chnl +:C_PCI_DATA_WIDTH]), 
                     .CHNL_TX_DATA_VALID(chnl_tx_data_valid[chnl]), 
                     .CHNL_TX_DATA_REN(chnl_tx_data_ren[chnl])
                     /*AUTOINST*/);    
        end
    endgenerate
endmodule
// Local Variables:
// verilog-library-directories:("../../../../riffa_hdl/" "../../")
// End:

