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
// Filename:            DE5QGen1x8If64.v
// Version:             
// Verilog Standard:    Verilog-2001
// Description:         Top level module for RIFFA 2.2 reference design for the
//                      the Altera Stratix V Avalong Streaming Interface to PCI
//                      Express module and the Terasic DE5 Development Board.
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`include "functions.vh"
`include "riffa.vh"
`include "altera.vh"
`timescale 1ps / 1ps
module DE5QGen1x8If64_CLK
    #(// Number of RIFFA Channels
      parameter C_NUM_CHNL = 12,
      // Number of PCIe Lanes
      parameter C_NUM_LANES =  8,
      // Settings from Quartus IP Library
      parameter C_PCI_DATA_WIDTH = 64,
      parameter C_MAX_PAYLOAD_BYTES = 256,
      parameter C_LOG_NUM_TAGS = 5
      )
    (
     // ----------LEDs----------
     output [7:0]            LED,

     // ----------PCIE----------
     input                   PCIE_RESET_N,
     input                   PCIE_REFCLK,

     // ----------PCIE Serial RX----------
     input [C_NUM_LANES-1:0] PCIE_RX_IN,

     // ----------PCIE Serial TX----------
     output [C_NUM_LANES-1:0]  PCIE_TX_OUT,

     // ----------Oscillators----------
     input                   OSC_BANK3D_50MHZ
     );

    wire                     npor;
    wire                     pin_perst;

    // ----------TL Config interface----------
    wire [3:0]               tl_cfg_add;
    wire [31:0]              tl_cfg_ctl;
    wire [52:0]              tl_cfg_sts;

    // ----------Rx/TX Interfaces----------
    wire [0:0]               rx_st_sop;
    wire [0:0]               rx_st_eop;
    wire [0:0]               rx_st_err;
    wire [0:0]               rx_st_valid;
    wire                     rx_st_ready;
    wire [C_PCI_DATA_WIDTH-1:0] rx_st_data;
    wire [0:0]                  rx_st_empty;
    
    wire [0:0]                  tx_st_sop;
    wire [0:0]                  tx_st_eop;
    wire [0:0]                  tx_st_err;
    wire [0:0]                  tx_st_valid;
    wire                        tx_st_ready;
    wire [C_PCI_DATA_WIDTH-1:0] tx_st_data;
    wire [0:0]                  tx_st_empty;
    
    // ----------Clocks & Locks----------
    wire                        pld_clk;
    wire                        coreclkout_hip;
    wire                        refclk;
    wire                        pld_core_ready;
    wire                        reset_status;
    wire                        serdes_pll_locked;
    wire                        riffa_5_clk;
    wire                        riffa_10_clk;
    wire                        riffa_25_clk;
    wire                        riffa_50_clk;
    wire                        riffa_75_clk;
    wire                        riffa_100_clk;
    wire                        riffa_125_clk;
    wire                        riffa_150_clk;
    wire                        riffa_175_clk;
    wire                        riffa_200_clk;
    wire                        riffa_225_clk;
    wire                        riffa_250_clk;

    // ----------Interrupt Interfaces----------
    wire                        app_msi_req;
    wire                        app_msi_ack;
    
    // ----------Reconfiguration Controller signals----------
    wire                        mgmt_clk_clk;
    wire                        mgmt_rst_reset;

    // ----------Reconfiguration Driver Signals----------
    wire                        reconfig_xcvr_clk;
    wire                        reconfig_xcvr_rst;

    wire [7:0]                  rx_in;
    wire [7:0]                  tx_out;
    
    // ------------Status Interface------------
    wire                        derr_cor_ext_rcv;
    wire                        derr_cor_ext_rpl;
    wire                        derr_rpl;
    wire                        dlup;
    wire                        dlup_exit;
    wire                        ev128ns;
    wire                        ev1us;
    wire                        hotrst_exit;
    wire [3:0]                  int_status;
    wire                        l2_exit;
    wire [3:0]                  lane_act;
    wire [4:0]                  ltssmstate;
    wire                        rx_par_err;
    wire [1:0]                  tx_par_err;
    wire                        cfg_par_err;
    wire [7:0]                  ko_cpl_spc_header;
    wire [11:0]                 ko_cpl_spc_data;

    // ----------Clocks----------
    assign pld_clk = coreclkout_hip;
    assign mgmt_clk_clk = PCIE_REFCLK;
    assign reconfig_xcvr_clk = PCIE_REFCLK;
    assign refclk = PCIE_REFCLK;
    assign pld_core_ready = serdes_pll_locked;
    
    // ----------Resets----------
    assign reconfig_xcvr_rst = 1'b0;
    assign mgmt_rst_reset = 1'b0;
    assign pin_perst = PCIE_RESET_N;
    assign npor = PCIE_RESET_N;

    // ----------LED's----------
    assign LED[7:0] = 8'hff;
    QSysDE5QGen1x8If64_CLK
        pcie_system_inst
            (
             // Outputs
             .rx_st_startofpacket              (rx_st_sop[0:0]),
             .rx_st_endofpacket                (rx_st_eop[0:0]),
             .rx_st_valid                      (rx_st_valid[0:0]),
             .rx_st_data                       (rx_st_data[63:0]),
             .tx_st_ready                      (tx_st_ready),
             .pciehip_reset_status             (reset_status),
             .pciehip_serdes_pll_locked        (serdes_pll_locked),
             .pciecfg_tl_cfg_add               (tl_cfg_add[3:0]),
             .pciecfg_tl_cfg_ctl               (tl_cfg_ctl[31:0]),
             .pciecfg_tl_cfg_sts               (tl_cfg_sts[52:0]),
             .pciecoreclk_clk                  (coreclkout_hip),
             .pcieserial_tx_out0               (PCIE_TX_OUT[0]),
             .pcieserial_tx_out1               (PCIE_TX_OUT[1]),
             .pcieserial_tx_out2               (PCIE_TX_OUT[2]),
             .pcieserial_tx_out3               (PCIE_TX_OUT[3]),
             .pcieserial_tx_out4               (PCIE_TX_OUT[4]),
             .pcieserial_tx_out5               (PCIE_TX_OUT[5]),
             .pcieserial_tx_out6               (PCIE_TX_OUT[6]),
             .pcieserial_tx_out7               (PCIE_TX_OUT[7]),
             .pciemsi_app_int_ack              (app_int_ack),
             .pciemsi_app_msi_ack              (app_msi_ack),
             .pciestat_derr_cor_ext_rcv        (derr_cor_ext_rcv),
             .pciestat_derr_cor_ext_rpl        (derr_cor_ext_rpl),
             .pciestat_derr_rpl                (derr_rpl),
             .pciestat_dlup                    (dlup),
             .pciestat_dlup_exit               (dlup_exit),
             .pciestat_ev128ns                 (ev128ns),
             .pciestat_ev1us                   (ev1us),
             .pciestat_hotrst_exit             (hotrst_exit),
             .pciestat_int_status              (int_status),
             .pciestat_l2_exit                 (l2_exit),
             .pciestat_lane_act                (lane_act),
             .pciestat_ltssmstate              (ltssmstate),
             .pciestat_rx_par_err              (rx_par_err),
             .pciestat_tx_par_err              (tx_par_err),
             .pciestat_cfg_par_err             (cfg_par_err),
             .pciestat_ko_cpl_spc_header       (ko_cpl_spc_header),
             .pciestat_ko_cpl_spc_data         (ko_cpl_spc_data),
             .riffa_5_clk                      (riffa_5_clk),
             .riffa_10_clk                     (riffa_10_clk),
             .riffa_25_clk                     (riffa_25_clk),
             .riffa_50_clk                     (riffa_50_clk),
             .riffa_75_clk                     (riffa_75_clk),
             .riffa_100_clk                    (riffa_100_clk),
             .riffa_125_clk                    (riffa_125_clk),
             .riffa_150_clk                    (riffa_150_clk),
             .riffa_175_clk                    (riffa_175_clk),
             .riffa_200_clk                    (riffa_200_clk),
             .riffa_225_clk                    (riffa_225_clk),
             .riffa_250_clk                    (riffa_250_clk),  
             // Inputs
             .rx_st_ready                      (rx_st_ready),
             .tx_st_startofpacket              (tx_st_sop[0:0]),
             .tx_st_endofpacket                (tx_st_eop[0:0]),
             .tx_st_valid                      (tx_st_valid[0:0]),
             .tx_st_data                       (tx_st_data[63:0]),
             .pciehip_pld_core_ready           (pld_core_ready),
             .pcienpor_npor                    (npor),
             .pcienpor_pin_perst               (pin_perst),
             .pcierefclk_clk                   (refclk),
             .reconfigrefclk_clk               (reconfig_xcvr_clk),
             .pciepld_clk                      (pld_clk),
             .reconfigrst_reset                (reconfig_xcvr_rst),
             .mgmtrst_reset                    (mgmt_rst_reset),
             .mgmtclk_clk                      (mgmt_clk_clk),
             .reconfigpldclk_clk               (pld_clk),
             .pcieserial_rx_in0                (PCIE_RX_IN[0]),
             .pcieserial_rx_in1                (PCIE_RX_IN[1]),
             .pcieserial_rx_in2                (PCIE_RX_IN[2]),
             .pcieserial_rx_in3                (PCIE_RX_IN[3]),
             .pcieserial_rx_in4                (PCIE_RX_IN[4]),
             .pcieserial_rx_in5                (PCIE_RX_IN[5]),
             .pcieserial_rx_in6                (PCIE_RX_IN[6]),
             .pcieserial_rx_in7                (PCIE_RX_IN[7]),
             .pciemsi_app_msi_req              (app_msi_req),
             .drvstat_derr_cor_ext_rcv         (derr_cor_ext_rcv),
             .drvstat_derr_cor_ext_rpl         (derr_cor_ext_rpl),
             .drvstat_derr_rpl                 (derr_rpl),
             .drvstat_dlup                     (dlup),
             .drvstat_dlup_exit                (dlup_exit),
             .drvstat_ev128ns                  (ev128ns),
             .drvstat_ev1us                    (ev1us),
             .drvstat_hotrst_exit              (hotrst_exit),
             .drvstat_int_status               (int_status),
             .drvstat_l2_exit                  (l2_exit),
             .drvstat_lane_act                 (lane_act),
             .drvstat_ltssmstate               (ltssmstate),
             .drvstat_rx_par_err               (rx_par_err),
             .drvstat_tx_par_err               (tx_par_err),
             .drvstat_cfg_par_err              (cfg_par_err),
             .drvstat_ko_cpl_spc_header        (ko_cpl_spc_header),
             .drvstat_ko_cpl_spc_data          (ko_cpl_spc_data),
             .pllrefclk_clk                    (OSC_BANK3D_50MHZ),
             .pllrst_reset_n                   (1'b1));

    // -------------------- END ALTERA IP INSTANTIATION  --------------------
    // -------------------- BEGIN RIFFA INSTANTAION --------------------

    // RIFFA channel interface
    wire                        rst_out;
    wire [C_NUM_CHNL-1:0]       chnl_rx_clk;
    wire [C_NUM_CHNL-1:0]       chnl_rx;
    wire [C_NUM_CHNL-1:0]       chnl_rx_ack;
    wire [C_NUM_CHNL-1:0]       chnl_rx_last;
    wire [(C_NUM_CHNL*32)-1:0]  chnl_rx_len;
    wire [(C_NUM_CHNL*31)-1:0]  chnl_rx_off;
    wire [(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0] chnl_rx_data;
    wire [C_NUM_CHNL-1:0]                    chnl_rx_data_valid;
    wire [C_NUM_CHNL-1:0]                    chnl_rx_data_ren;
    
    wire [C_NUM_CHNL-1:0]                    chnl_tx_clk;
    wire [C_NUM_CHNL-1:0]                    chnl_tx;
    wire [C_NUM_CHNL-1:0]                    chnl_tx_ack;
    wire [C_NUM_CHNL-1:0]                    chnl_tx_last;
    wire [(C_NUM_CHNL*32)-1:0]               chnl_tx_len;
    wire [(C_NUM_CHNL*31)-1:0]               chnl_tx_off;
    wire [(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0] chnl_tx_data;
    wire [C_NUM_CHNL-1:0]                    chnl_tx_data_valid;
    wire [C_NUM_CHNL-1:0]                    chnl_tx_data_ren;

    wire                                     chnl_reset;
    wire                                     chnl_clk;
    wire                                     riffa_reset;
    wire                                     riffa_clk;
    assign riffa_reset = reset_status;
    assign riffa_clk = pld_clk;
    assign chnl_clk = pld_clk;
    assign chnl_reset = rst_out;
    
    riffa_wrapper_de5
        #(/*AUTOINSTPARAM*/
          // Parameters
          .C_LOG_NUM_TAGS               (C_LOG_NUM_TAGS),
          .C_NUM_CHNL                   (C_NUM_CHNL),
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
          .C_MAX_PAYLOAD_BYTES          (C_MAX_PAYLOAD_BYTES))
    riffa
        (
         // Outputs
         .RX_ST_READY                   (rx_st_ready),
         .TX_ST_DATA                    (tx_st_data[C_PCI_DATA_WIDTH-1:0]),
         .TX_ST_VALID                   (tx_st_valid[0:0]),
         .TX_ST_EOP                     (tx_st_eop[0:0]),
         .TX_ST_SOP                     (tx_st_sop[0:0]),
         .TX_ST_EMPTY                   (tx_st_empty[0:0]),
         .APP_MSI_REQ                   (app_msi_req),
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
         .RX_ST_DATA                    (rx_st_data[C_PCI_DATA_WIDTH-1:0]),
         .RX_ST_EOP                     (rx_st_eop[0:0]),
         .RX_ST_SOP                     (rx_st_sop[0:0]),
         .RX_ST_VALID                   (rx_st_valid[0:0]),
         .RX_ST_EMPTY                   (rx_st_empty[0:0]),
         .TX_ST_READY                   (tx_st_ready),
         .TL_CFG_CTL                    (tl_cfg_ctl[`SIG_CFG_CTL_W-1:0]),
         .TL_CFG_ADD                    (tl_cfg_add[`SIG_CFG_ADD_W-1:0]),
         .TL_CFG_STS                    (tl_cfg_sts[`SIG_CFG_STS_W-1:0]),
         .KO_CPL_SPC_HEADER             (ko_cpl_spc_header[`SIG_KO_CPLH_W-1:0]),
         .KO_CPL_SPC_DATA               (ko_cpl_spc_data[`SIG_KO_CPLD_W-1:0]),
         .APP_MSI_ACK                   (app_msi_ack),
         .PLD_CLK                       (pld_clk),
         .RESET_STATUS                  (reset_status),
         .CHNL_RX_CLK                   (chnl_rx_clk[C_NUM_CHNL-1:0]),
         .CHNL_RX_ACK                   (chnl_rx_ack[C_NUM_CHNL-1:0]),
         .CHNL_RX_DATA_REN              (chnl_rx_data_ren[C_NUM_CHNL-1:0]),
         .CHNL_TX_CLK                   (chnl_tx_clk[C_NUM_CHNL-1:0]),
         .CHNL_TX                       (chnl_tx[C_NUM_CHNL-1:0]),
         .CHNL_TX_LAST                  (chnl_tx_last[C_NUM_CHNL-1:0]),
         .CHNL_TX_LEN                   (chnl_tx_len[(C_NUM_CHNL*`SIG_CHNL_LENGTH_W)-1:0]),
         .CHNL_TX_OFF                   (chnl_tx_off[(C_NUM_CHNL*`SIG_CHNL_OFFSET_W)-1:0]),
         .CHNL_TX_DATA                  (chnl_tx_data[(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0]),
         .CHNL_TX_DATA_VALID            (chnl_tx_data_valid[C_NUM_CHNL-1:0]));

    // --------------------  END RIFFA INSTANTAION --------------------
    chnl_tester 
        #(.C_PCI_DATA_WIDTH(C_PCI_DATA_WIDTH)) 
    chnl_tester_5mhz
        (.CLK(riffa_5_clk),
         .RST(rst_out),    // riffa_reset includes riffa_endpoint resets
         // Rx interface
         .CHNL_RX_CLK(chnl_rx_clk[0]), 
         .CHNL_RX(chnl_rx[0]), 
         .CHNL_RX_ACK(chnl_rx_ack[0]), 
         .CHNL_RX_LAST(chnl_rx_last[0]), 
         .CHNL_RX_LEN(chnl_rx_len[32*0 +:32]), 
         .CHNL_RX_OFF(chnl_rx_off[31*0 +:31]), 
         .CHNL_RX_DATA(chnl_rx_data[C_PCI_DATA_WIDTH*0 +:C_PCI_DATA_WIDTH]), 
         .CHNL_RX_DATA_VALID(chnl_rx_data_valid[0]), 
         .CHNL_RX_DATA_REN(chnl_rx_data_ren[0]),
         // Tx interface
         .CHNL_TX_CLK(chnl_tx_clk[0]), 
         .CHNL_TX(chnl_tx[0]), 
         .CHNL_TX_ACK(chnl_tx_ack[0]), 
         .CHNL_TX_LAST(chnl_tx_last[0]), 
         .CHNL_TX_LEN(chnl_tx_len[32*0 +:32]), 
         .CHNL_TX_OFF(chnl_tx_off[31*0 +:31]), 
         .CHNL_TX_DATA(chnl_tx_data[C_PCI_DATA_WIDTH*0 +:C_PCI_DATA_WIDTH]), 
         .CHNL_TX_DATA_VALID(chnl_tx_data_valid[0]), 
         .CHNL_TX_DATA_REN(chnl_tx_data_ren[0]));

    chnl_tester 
        #(.C_PCI_DATA_WIDTH(C_PCI_DATA_WIDTH)) 
    chnl_tester_10mhz
        (.CLK(riffa_10_clk),
         .RST(rst_out),    // riffa_reset includes riffa_endpoint resets
         // Rx interface
         .CHNL_RX_CLK(chnl_rx_clk[1]), 
         .CHNL_RX(chnl_rx[1]), 
         .CHNL_RX_ACK(chnl_rx_ack[1]), 
         .CHNL_RX_LAST(chnl_rx_last[1]), 
         .CHNL_RX_LEN(chnl_rx_len[32*1 +:32]), 
         .CHNL_RX_OFF(chnl_rx_off[31*1 +:31]), 
         .CHNL_RX_DATA(chnl_rx_data[C_PCI_DATA_WIDTH*1 +:C_PCI_DATA_WIDTH]), 
         .CHNL_RX_DATA_VALID(chnl_rx_data_valid[1]), 
         .CHNL_RX_DATA_REN(chnl_rx_data_ren[1]),
         // Tx interface
         .CHNL_TX_CLK(chnl_tx_clk[1]), 
         .CHNL_TX(chnl_tx[1]), 
         .CHNL_TX_ACK(chnl_tx_ack[1]), 
         .CHNL_TX_LAST(chnl_tx_last[1]), 
         .CHNL_TX_LEN(chnl_tx_len[32*1 +:32]), 
         .CHNL_TX_OFF(chnl_tx_off[31*1 +:31]), 
         .CHNL_TX_DATA(chnl_tx_data[C_PCI_DATA_WIDTH*1 +:C_PCI_DATA_WIDTH]), 
         .CHNL_TX_DATA_VALID(chnl_tx_data_valid[1]), 
         .CHNL_TX_DATA_REN(chnl_tx_data_ren[1]));

    chnl_tester 
        #(.C_PCI_DATA_WIDTH(C_PCI_DATA_WIDTH)) 
    chnl_tester_25mhz
        (.CLK(riffa_25_clk),
         .RST(rst_out),    // riffa_reset includes riffa_endpoint resets
         // Rx interface
         .CHNL_RX_CLK(chnl_rx_clk[2]), 
         .CHNL_RX(chnl_rx[2]), 
         .CHNL_RX_ACK(chnl_rx_ack[2]), 
         .CHNL_RX_LAST(chnl_rx_last[2]), 
         .CHNL_RX_LEN(chnl_rx_len[32*2 +:32]), 
         .CHNL_RX_OFF(chnl_rx_off[31*2 +:31]), 
         .CHNL_RX_DATA(chnl_rx_data[C_PCI_DATA_WIDTH*2 +:C_PCI_DATA_WIDTH]), 
         .CHNL_RX_DATA_VALID(chnl_rx_data_valid[2]), 
         .CHNL_RX_DATA_REN(chnl_rx_data_ren[2]),
         // Tx interface
         .CHNL_TX_CLK(chnl_tx_clk[2]), 
         .CHNL_TX(chnl_tx[2]), 
         .CHNL_TX_ACK(chnl_tx_ack[2]), 
         .CHNL_TX_LAST(chnl_tx_last[2]), 
         .CHNL_TX_LEN(chnl_tx_len[32*2 +:32]), 
         .CHNL_TX_OFF(chnl_tx_off[31*2 +:31]), 
         .CHNL_TX_DATA(chnl_tx_data[C_PCI_DATA_WIDTH*2 +:C_PCI_DATA_WIDTH]), 
         .CHNL_TX_DATA_VALID(chnl_tx_data_valid[2]), 
         .CHNL_TX_DATA_REN(chnl_tx_data_ren[2]));

    chnl_tester 
        #(.C_PCI_DATA_WIDTH(C_PCI_DATA_WIDTH)) 
    chnl_tester_50mhz
        (.CLK(riffa_50_clk),
         .RST(rst_out),    // riffa_reset includes riffa_endpoint resets
         // Rx interface
         .CHNL_RX_CLK(chnl_rx_clk[3]), 
         .CHNL_RX(chnl_rx[3]), 
         .CHNL_RX_ACK(chnl_rx_ack[3]), 
         .CHNL_RX_LAST(chnl_rx_last[3]), 
         .CHNL_RX_LEN(chnl_rx_len[32*3 +:32]), 
         .CHNL_RX_OFF(chnl_rx_off[31*3 +:31]), 
         .CHNL_RX_DATA(chnl_rx_data[C_PCI_DATA_WIDTH*3 +:C_PCI_DATA_WIDTH]), 
         .CHNL_RX_DATA_VALID(chnl_rx_data_valid[3]), 
         .CHNL_RX_DATA_REN(chnl_rx_data_ren[3]),
         // Tx interface
         .CHNL_TX_CLK(chnl_tx_clk[3]), 
         .CHNL_TX(chnl_tx[3]), 
         .CHNL_TX_ACK(chnl_tx_ack[3]), 
         .CHNL_TX_LAST(chnl_tx_last[3]), 
         .CHNL_TX_LEN(chnl_tx_len[32*3 +:32]), 
         .CHNL_TX_OFF(chnl_tx_off[31*3 +:31]), 
         .CHNL_TX_DATA(chnl_tx_data[C_PCI_DATA_WIDTH*3 +:C_PCI_DATA_WIDTH]), 
         .CHNL_TX_DATA_VALID(chnl_tx_data_valid[3]), 
         .CHNL_TX_DATA_REN(chnl_tx_data_ren[3]));

    chnl_tester 
        #(.C_PCI_DATA_WIDTH(C_PCI_DATA_WIDTH)) 
    chnl_tester_75mhz
        (.CLK(riffa_75_clk),
         .RST(rst_out),    // riffa_reset includes riffa_endpoint resets
         // Rx interface
         .CHNL_RX_CLK(chnl_rx_clk[4]), 
         .CHNL_RX(chnl_rx[4]), 
         .CHNL_RX_ACK(chnl_rx_ack[4]), 
         .CHNL_RX_LAST(chnl_rx_last[4]), 
         .CHNL_RX_LEN(chnl_rx_len[32*4 +:32]), 
         .CHNL_RX_OFF(chnl_rx_off[31*4 +:31]), 
         .CHNL_RX_DATA(chnl_rx_data[C_PCI_DATA_WIDTH*4 +:C_PCI_DATA_WIDTH]), 
         .CHNL_RX_DATA_VALID(chnl_rx_data_valid[4]), 
         .CHNL_RX_DATA_REN(chnl_rx_data_ren[4]),
         // Tx interface
         .CHNL_TX_CLK(chnl_tx_clk[4]), 
         .CHNL_TX(chnl_tx[4]), 
         .CHNL_TX_ACK(chnl_tx_ack[4]), 
         .CHNL_TX_LAST(chnl_tx_last[4]), 
         .CHNL_TX_LEN(chnl_tx_len[32*4 +:32]), 
         .CHNL_TX_OFF(chnl_tx_off[31*4 +:31]), 
         .CHNL_TX_DATA(chnl_tx_data[C_PCI_DATA_WIDTH*4 +:C_PCI_DATA_WIDTH]), 
         .CHNL_TX_DATA_VALID(chnl_tx_data_valid[4]), 
         .CHNL_TX_DATA_REN(chnl_tx_data_ren[4]));

    chnl_tester 
        #(.C_PCI_DATA_WIDTH(C_PCI_DATA_WIDTH)) 
    chnl_tester_100mhz
        (.CLK(riffa_100_clk),
         .RST(rst_out),    // riffa_reset includes riffa_endpoint resets
         // Rx interface
         .CHNL_RX_CLK(chnl_rx_clk[5]), 
         .CHNL_RX(chnl_rx[5]), 
         .CHNL_RX_ACK(chnl_rx_ack[5]), 
         .CHNL_RX_LAST(chnl_rx_last[5]), 
         .CHNL_RX_LEN(chnl_rx_len[32*5 +:32]), 
         .CHNL_RX_OFF(chnl_rx_off[31*5 +:31]), 
         .CHNL_RX_DATA(chnl_rx_data[C_PCI_DATA_WIDTH*5 +:C_PCI_DATA_WIDTH]), 
         .CHNL_RX_DATA_VALID(chnl_rx_data_valid[5]), 
         .CHNL_RX_DATA_REN(chnl_rx_data_ren[5]),
         // Tx interface
         .CHNL_TX_CLK(chnl_tx_clk[5]), 
         .CHNL_TX(chnl_tx[5]), 
         .CHNL_TX_ACK(chnl_tx_ack[5]), 
         .CHNL_TX_LAST(chnl_tx_last[5]), 
         .CHNL_TX_LEN(chnl_tx_len[32*5 +:32]), 
         .CHNL_TX_OFF(chnl_tx_off[31*5 +:31]), 
         .CHNL_TX_DATA(chnl_tx_data[C_PCI_DATA_WIDTH*5 +:C_PCI_DATA_WIDTH]), 
         .CHNL_TX_DATA_VALID(chnl_tx_data_valid[5]), 
         .CHNL_TX_DATA_REN(chnl_tx_data_ren[5]));

    chnl_tester 
        #(.C_PCI_DATA_WIDTH(C_PCI_DATA_WIDTH)) 
    chnl_tester_125mhz
        (.CLK(riffa_125_clk),
         .RST(rst_out),    // riffa_reset includes riffa_endpoint resets
         // Rx interface
         .CHNL_RX_CLK(chnl_rx_clk[6]), 
         .CHNL_RX(chnl_rx[6]), 
         .CHNL_RX_ACK(chnl_rx_ack[6]), 
         .CHNL_RX_LAST(chnl_rx_last[6]), 
         .CHNL_RX_LEN(chnl_rx_len[32*6 +:32]), 
         .CHNL_RX_OFF(chnl_rx_off[31*6 +:31]), 
         .CHNL_RX_DATA(chnl_rx_data[C_PCI_DATA_WIDTH*6 +:C_PCI_DATA_WIDTH]), 
         .CHNL_RX_DATA_VALID(chnl_rx_data_valid[6]), 
         .CHNL_RX_DATA_REN(chnl_rx_data_ren[6]),
         // Tx interface
         .CHNL_TX_CLK(chnl_tx_clk[6]), 
         .CHNL_TX(chnl_tx[6]), 
         .CHNL_TX_ACK(chnl_tx_ack[6]), 
         .CHNL_TX_LAST(chnl_tx_last[6]), 
         .CHNL_TX_LEN(chnl_tx_len[32*6 +:32]), 
         .CHNL_TX_OFF(chnl_tx_off[31*6 +:31]), 
         .CHNL_TX_DATA(chnl_tx_data[C_PCI_DATA_WIDTH*6 +:C_PCI_DATA_WIDTH]), 
         .CHNL_TX_DATA_VALID(chnl_tx_data_valid[6]), 
         .CHNL_TX_DATA_REN(chnl_tx_data_ren[6]));

    chnl_tester 
        #(.C_PCI_DATA_WIDTH(C_PCI_DATA_WIDTH)) 
    chnl_tester_150mhz
        (.CLK(riffa_150_clk),
         .RST(rst_out),    // riffa_reset includes riffa_endpoint resets
         // Rx interface
         .CHNL_RX_CLK(chnl_rx_clk[7]), 
         .CHNL_RX(chnl_rx[7]), 
         .CHNL_RX_ACK(chnl_rx_ack[7]), 
         .CHNL_RX_LAST(chnl_rx_last[7]), 
         .CHNL_RX_LEN(chnl_rx_len[32*7 +:32]), 
         .CHNL_RX_OFF(chnl_rx_off[31*7 +:31]), 
         .CHNL_RX_DATA(chnl_rx_data[C_PCI_DATA_WIDTH*7 +:C_PCI_DATA_WIDTH]), 
         .CHNL_RX_DATA_VALID(chnl_rx_data_valid[7]), 
         .CHNL_RX_DATA_REN(chnl_rx_data_ren[7]),
         // Tx interface
         .CHNL_TX_CLK(chnl_tx_clk[7]), 
         .CHNL_TX(chnl_tx[7]), 
         .CHNL_TX_ACK(chnl_tx_ack[7]), 
         .CHNL_TX_LAST(chnl_tx_last[7]), 
         .CHNL_TX_LEN(chnl_tx_len[32*7 +:32]), 
         .CHNL_TX_OFF(chnl_tx_off[31*7 +:31]), 
         .CHNL_TX_DATA(chnl_tx_data[C_PCI_DATA_WIDTH*7 +:C_PCI_DATA_WIDTH]), 
         .CHNL_TX_DATA_VALID(chnl_tx_data_valid[7]), 
         .CHNL_TX_DATA_REN(chnl_tx_data_ren[7]));

    chnl_tester 
        #(.C_PCI_DATA_WIDTH(C_PCI_DATA_WIDTH)) 
    chnl_tester_175mhz
        (.CLK(riffa_175_clk),
         .RST(rst_out),    // riffa_reset includes riffa_endpoint resets
         // Rx interface
         .CHNL_RX_CLK(chnl_rx_clk[8]), 
         .CHNL_RX(chnl_rx[8]), 
         .CHNL_RX_ACK(chnl_rx_ack[8]), 
         .CHNL_RX_LAST(chnl_rx_last[8]), 
         .CHNL_RX_LEN(chnl_rx_len[32*8 +:32]), 
         .CHNL_RX_OFF(chnl_rx_off[31*8 +:31]), 
         .CHNL_RX_DATA(chnl_rx_data[C_PCI_DATA_WIDTH*8 +:C_PCI_DATA_WIDTH]), 
         .CHNL_RX_DATA_VALID(chnl_rx_data_valid[8]), 
         .CHNL_RX_DATA_REN(chnl_rx_data_ren[8]),
         // Tx interface
         .CHNL_TX_CLK(chnl_tx_clk[8]), 
         .CHNL_TX(chnl_tx[8]), 
         .CHNL_TX_ACK(chnl_tx_ack[8]), 
         .CHNL_TX_LAST(chnl_tx_last[8]), 
         .CHNL_TX_LEN(chnl_tx_len[32*8 +:32]), 
         .CHNL_TX_OFF(chnl_tx_off[31*8 +:31]), 
         .CHNL_TX_DATA(chnl_tx_data[C_PCI_DATA_WIDTH*8 +:C_PCI_DATA_WIDTH]), 
         .CHNL_TX_DATA_VALID(chnl_tx_data_valid[8]), 
         .CHNL_TX_DATA_REN(chnl_tx_data_ren[8]));

    chnl_tester 
        #(.C_PCI_DATA_WIDTH(C_PCI_DATA_WIDTH)) 
    chnl_tester_200mhz
        (.CLK(riffa_200_clk),
         .RST(rst_out),    // riffa_reset includes riffa_endpoint resets
         // Rx interface
         .CHNL_RX_CLK(chnl_rx_clk[9]), 
         .CHNL_RX(chnl_rx[9]), 
         .CHNL_RX_ACK(chnl_rx_ack[9]), 
         .CHNL_RX_LAST(chnl_rx_last[9]), 
         .CHNL_RX_LEN(chnl_rx_len[32*9 +:32]), 
         .CHNL_RX_OFF(chnl_rx_off[31*9 +:31]), 
         .CHNL_RX_DATA(chnl_rx_data[C_PCI_DATA_WIDTH*9 +:C_PCI_DATA_WIDTH]), 
         .CHNL_RX_DATA_VALID(chnl_rx_data_valid[9]), 
         .CHNL_RX_DATA_REN(chnl_rx_data_ren[9]),
         // Tx interface
         .CHNL_TX_CLK(chnl_tx_clk[9]), 
         .CHNL_TX(chnl_tx[9]), 
         .CHNL_TX_ACK(chnl_tx_ack[9]), 
         .CHNL_TX_LAST(chnl_tx_last[9]), 
         .CHNL_TX_LEN(chnl_tx_len[32*9 +:32]), 
         .CHNL_TX_OFF(chnl_tx_off[31*9 +:31]), 
         .CHNL_TX_DATA(chnl_tx_data[C_PCI_DATA_WIDTH*9 +:C_PCI_DATA_WIDTH]), 
         .CHNL_TX_DATA_VALID(chnl_tx_data_valid[9]), 
         .CHNL_TX_DATA_REN(chnl_tx_data_ren[9]));

    chnl_tester 
        #(.C_PCI_DATA_WIDTH(C_PCI_DATA_WIDTH)) 
    chnl_tester_225mhz
        (.CLK(riffa_225_clk),
         .RST(rst_out),    // riffa_reset includes riffa_endpoint resets
         // Rx interface
         .CHNL_RX_CLK(chnl_rx_clk[10]), 
         .CHNL_RX(chnl_rx[10]), 
         .CHNL_RX_ACK(chnl_rx_ack[10]), 
         .CHNL_RX_LAST(chnl_rx_last[10]), 
         .CHNL_RX_LEN(chnl_rx_len[32*10 +:32]), 
         .CHNL_RX_OFF(chnl_rx_off[31*10 +:31]), 
         .CHNL_RX_DATA(chnl_rx_data[C_PCI_DATA_WIDTH*10 +:C_PCI_DATA_WIDTH]), 
         .CHNL_RX_DATA_VALID(chnl_rx_data_valid[10]), 
         .CHNL_RX_DATA_REN(chnl_rx_data_ren[10]),
         // Tx interface
         .CHNL_TX_CLK(chnl_tx_clk[10]), 
         .CHNL_TX(chnl_tx[10]), 
         .CHNL_TX_ACK(chnl_tx_ack[10]), 
         .CHNL_TX_LAST(chnl_tx_last[10]), 
         .CHNL_TX_LEN(chnl_tx_len[32*10 +:32]), 
         .CHNL_TX_OFF(chnl_tx_off[31*10 +:31]), 
         .CHNL_TX_DATA(chnl_tx_data[C_PCI_DATA_WIDTH*10 +:C_PCI_DATA_WIDTH]), 
         .CHNL_TX_DATA_VALID(chnl_tx_data_valid[10]), 
         .CHNL_TX_DATA_REN(chnl_tx_data_ren[10]));

    chnl_tester 
        #(.C_PCI_DATA_WIDTH(C_PCI_DATA_WIDTH)) 
    chnl_tester_250mhz
        (.CLK(riffa_250_clk),
         .RST(rst_out),    // riffa_reset includes riffa_endpoint resets
         // Rx interface
         .CHNL_RX_CLK(chnl_rx_clk[11]), 
         .CHNL_RX(chnl_rx[11]), 
         .CHNL_RX_ACK(chnl_rx_ack[11]), 
         .CHNL_RX_LAST(chnl_rx_last[11]), 
         .CHNL_RX_LEN(chnl_rx_len[32*11 +:32]), 
         .CHNL_RX_OFF(chnl_rx_off[31*11 +:31]), 
         .CHNL_RX_DATA(chnl_rx_data[C_PCI_DATA_WIDTH*11 +:C_PCI_DATA_WIDTH]), 
         .CHNL_RX_DATA_VALID(chnl_rx_data_valid[11]), 
         .CHNL_RX_DATA_REN(chnl_rx_data_ren[11]),
         // Tx interface
         .CHNL_TX_CLK(chnl_tx_clk[11]), 
         .CHNL_TX(chnl_tx[11]), 
         .CHNL_TX_ACK(chnl_tx_ack[11]), 
         .CHNL_TX_LAST(chnl_tx_last[11]), 
         .CHNL_TX_LEN(chnl_tx_len[32*11 +:32]), 
         .CHNL_TX_OFF(chnl_tx_off[31*11 +:31]), 
         .CHNL_TX_DATA(chnl_tx_data[C_PCI_DATA_WIDTH*11 +:C_PCI_DATA_WIDTH]), 
         .CHNL_TX_DATA_VALID(chnl_tx_data_valid[11]), 
         .CHNL_TX_DATA_REN(chnl_tx_data_ren[11]));

endmodule
