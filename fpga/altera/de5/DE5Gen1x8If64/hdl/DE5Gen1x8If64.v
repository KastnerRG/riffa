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
// Filename:            DE5Gen1x8If64
// Version:             
// Verilog Standard:    Verilog-2001
// Description:         Top level module for RIFFA 2.2 reference design for the
//                      the Altera Stratix V Hard IP for PCI Express
//                      module and the Terasic DE5 net Development Board.
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`include "functions.vh"
`include "riffa.vh"
`include "altera.vh"
`timescale 1ps / 1ps
module DE5Gen1x8If64
    #(// Number of RIFFA Channels
      parameter C_NUM_CHNL = 1,
      // Number of PCIe Lanes
      parameter C_NUM_LANES =  8,
      // Settings from Quartus IP Library
      parameter C_PCI_DATA_WIDTH = 64,
      parameter C_MAX_PAYLOAD_BYTES = 256,
      parameter C_LOG_NUM_TAGS = 5
      )
    (
     // ----------LEDs----------
     output [7:0]             LED,

     // ----------PCIE----------
     input                    PCIE_RESET_N,
     input                    PCIE_REFCLK,

     // ----------PCIE Serial RX----------
     input [C_NUM_LANES-1:0]  PCIE_RX_IN,

     // ----------PCIE Serial TX----------
     output [C_NUM_LANES-1:0] PCIE_TX_OUT,

     // ----------Oscillators----------
     input                    OSC_BANK3D_50MHZ
     );

    wire                      npor;
    wire                      pin_perst;

    // ----------LMI Interface----------
    wire [11:0]               lmi_addr;
    wire [31:0]               lmi_din;
    wire                      lmi_rden;
    wire                      lmi_wren;
    wire                      lmi_ack;
    wire [31:0]               lmi_dout;

    // ----------TL Config interface----------
    wire [3:0]                tl_cfg_add;
    wire [31:0]               tl_cfg_ctl;
    wire [52:0]               tl_cfg_sts;

    // ----------Rx/TX Interfaces----------
    wire [0:0]                rx_st_sop;
    wire [0:0]                rx_st_eop;
    wire [0:0]                rx_st_err;
    wire [0:0]                rx_st_valid;
    wire [0:0]                rx_st_empty;
    wire                      rx_st_ready;
    wire [63:0]               rx_st_data;

    wire [7:0]                rx_st_bar;
    wire                      rx_st_mask;

    wire [0:0]                tx_st_sop;
    wire [0:0]                tx_st_eop;
    wire [0:0]                tx_st_err;
    wire [0:0]                tx_st_valid;
    wire [0:0]                tx_st_empty;
    wire                      tx_st_ready;
    wire [63:0]               tx_st_data;

    // ----------Clocks----------
    wire                      pld_clk;
    wire                      coreclkout_hip;
    wire                      refclk;
    wire                      pld_core_ready;
    wire                      reset_status;
    wire                      serdes_pll_locked;
    wire                      pld_clk_inuse;

    // ----------Reconfiguration busses----------
    wire [699:0]              reconfig_to_xcvr;
    wire [505:0]              reconfig_from_xcvr;

    // ----------Interrupt Interfaces----------
    wire                      app_int_sts;
    wire [4:0]                app_msi_num;
    wire                      app_msi_req;
    wire [2:0]                app_msi_tc;
    wire                      app_int_ack;
    wire                      app_msi_ack;

    // ----------Link status signals----------
    wire                      derr_cor_ext_rcv;
    wire                      derr_cor_ext_rpl;
    wire                      derr_rpl;
    wire                      dlup;
    wire                      dlup_exit;
    wire                      ev128ns;
    wire                      ev1us;
    wire                      hotrst_exit;
    wire [3:0]                int_status;
    wire                      l2_exit;
    wire [3:0]                lane_act;
    wire [4:0]                ltssmstate;
    wire                      rx_par_err;
    wire [1:0]                tx_par_err;
    wire                      cfg_par_err;
    wire [1:0]                currentspeed;
    wire [7:0]                ko_cpl_spc_header;
    wire [11:0]               ko_cpl_spc_data;

    // ----------Link Status Signals (Driver)----------
    wire                      derr_cor_ext_rcv_drv;
    wire                      derr_cor_ext_rpl_drv;
    wire                      derr_rpl_drv;
    wire                      dlup_drv;
    wire                      dlup_exit_drv;
    wire                      ev128ns_drv;
    wire                      ev1us_drv;
    wire                      hotrst_exit_drv;
    wire [3:0]                int_status_drv;
    wire                      l2_exit_drv;
    wire [3:0]                lane_act_drv;
    wire [4:0]                ltssmstate_drv;
    wire                      rx_par_err_drv;
    wire [1:0]                tx_par_err_drv;
    wire                      cfg_par_err_drv;
    wire [7:0]                ko_cpl_spc_header_drv;
    wire [11:0]               ko_cpl_spc_data_drv;


    // ----------Reconfiguration Controller signals----------
    wire                      reconfig_busy;
    wire                      mgmt_clk_clk;
    wire                      mgmt_rst_reset;

    wire [6:0]                reconfig_mgmt_address;
    wire                      reconfig_mgmt_read;
    wire [31:0]               reconfig_mgmt_readdata;
    wire                      reconfig_mgmt_waitrequest;
    wire                      reconfig_mgmt_write;
    wire [31:0]               reconfig_mgmt_writedata;

    // ----------Reconfiguration Driver signals----------
    wire                      reconfig_xcvr_clk;
    wire                      reconfig_xcvr_rst;

    wire [7:0]                rx_in;
    wire [7:0]                tx_out;

    // ----------Serial interfaces----------
    assign rx_in = PCIE_RX_IN;
    assign PCIE_TX_OUT = tx_out;

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

    // -------------------- BEGIN ALTERA IP INSTANTIATION  --------------------
    // Transciever driver (Required for Gen1)
    altpcie_reconfig_driver 
        #(.number_of_reconfig_interfaces(10),
          .gen123_lane_rate_mode_hwtcl("Gen1 (2.5 Gbps)"), // This must be changed between generations
          .INTENDED_DEVICE_FAMILY("Stratix V"))
    XCVRDriverGen1x8_inst
        (
         // Outputs
         .reconfig_mgmt_address            (reconfig_mgmt_address[6:0]),
         .reconfig_mgmt_read               (reconfig_mgmt_read),
         .reconfig_mgmt_write              (reconfig_mgmt_write),
         .reconfig_mgmt_writedata          (reconfig_mgmt_writedata[31:0]),
        
         // Inputs
         .pld_clk                          (pld_clk),
        
         .reconfig_xcvr_rst                (reconfig_xcvr_rst),
         .reconfig_mgmt_readdata           (reconfig_mgmt_readdata[31:0]),
         .reconfig_mgmt_waitrequest        (reconfig_mgmt_waitrequest),
         .reconfig_xcvr_clk                (reconfig_xcvr_clk),
         .reconfig_busy                    (reconfig_busy),

         // Link Status signals
         .derr_cor_ext_rcv_drv             (derr_cor_ext_rcv_drv),
         .derr_cor_ext_rpl_drv             (derr_cor_ext_rpl_drv),
         .derr_rpl_drv                     (derr_rpl_drv),
         .dlup_drv                         (dlup_drv),
         .dlup_exit_drv                    (dlup_exit_drv),
         .ev128ns_drv                      (ev128ns_drv),
         .ev1us_drv                        (ev1us_drv),
         .hotrst_exit_drv                  (hotrst_exit_drv),
         .int_status_drv                   (int_status_drv[3:0]),
         .l2_exit_drv                      (l2_exit_drv),
         .lane_act_drv                     (lane_act_drv[3:0]),
         .ltssmstate_drv                   (ltssmstate_drv[4:0]),
         .rx_par_err_drv                   (rx_par_err_drv),
         .tx_par_err_drv                   (tx_par_err_drv[1:0]),
         .cfg_par_err_drv                  (cfg_par_err_drv),
         .ko_cpl_spc_header_drv            (ko_cpl_spc_header_drv[7:0]),
         .ko_cpl_spc_data_drv              (ko_cpl_spc_data_drv[11:0]),
         .currentspeed                     (currentspeed[1:0]));

    assign derr_cor_ext_rcv_drv = derr_cor_ext_rcv;
    assign derr_cor_ext_rpl_drv = derr_cor_ext_rpl;
    assign derr_rpl_drv = derr_rpl;
    assign dlup_drv = dlup;
    assign dlup_exit_drv = dlup_exit;
    assign ev128ns_drv = ev128ns;
    assign ev1us_drv = ev1us;
    assign hotrst_exit_drv = hotrst_exit;
    assign int_status_drv = int_status;
    assign l2_exit_drv = l2_exit;
    assign lane_act_drv = lane_act;
    assign ltssmstate_drv = ltssmstate;
    assign rx_par_err_drv = rx_par_err;
    assign tx_par_err_drv = tx_par_err;
    assign cfg_par_err_drv = cfg_par_err;
    assign ko_cpl_spc_header_drv = ko_cpl_spc_header;
    assign ko_cpl_spc_data_drv = ko_cpl_spc_data;

    XCVRCtrlGen1x8 XCVRCtrlGen1x8_inst
        (
         // Outputs
         .reconfig_busy                    (reconfig_busy),
         .reconfig_mgmt_readdata           (reconfig_mgmt_readdata[31:0]),
         .reconfig_mgmt_waitrequest        (reconfig_mgmt_waitrequest),
         .reconfig_to_xcvr                 (reconfig_to_xcvr[699:0]),
         // Inputs
         .mgmt_clk_clk                     (mgmt_clk_clk),
         .mgmt_rst_reset                   (mgmt_rst_reset),
         .reconfig_mgmt_address            (reconfig_mgmt_address[6:0]),
         .reconfig_mgmt_read               (reconfig_mgmt_read),
         .reconfig_mgmt_write              (reconfig_mgmt_write),
         .reconfig_mgmt_writedata          (reconfig_mgmt_writedata[31:0]),
         .reconfig_from_xcvr               (reconfig_from_xcvr[459:0]));


    // PCIE Core
    PCIeGen1x8If64 PCIeGen1x8If64_inst
        (
         // Outputs
         // Local Management Interface
         .lmi_ack                          (lmi_ack),
         .lmi_dout                         (lmi_dout[31:0]),
         .tl_cfg_add                       (tl_cfg_add[3:0]),
         .tl_cfg_ctl                       (tl_cfg_ctl[31:0]),
         .tl_cfg_sts                       (tl_cfg_sts[52:0]),
         
         // RX Interface      
         .rx_st_sop                        (rx_st_sop[0:0]),
         .rx_st_eop                        (rx_st_eop[0:0]),
         .rx_st_err                        (rx_st_err[0:0]),
         .rx_st_valid                      (rx_st_valid[0:0]),
         .rx_st_data                       (rx_st_data[63:0]),
         .rx_st_bar                        (rx_st_bar[7:0]),
         // TX Interface      
         .tx_st_ready                      (tx_st_ready),
         
         .coreclkout_hip                   (coreclkout_hip),
         .reset_status                     (reset_status),
         .serdes_pll_locked                (serdes_pll_locked),
         .pld_clk_inuse                    (pld_clk_inuse),

         // Reconfiguration Interface
         .reconfig_from_xcvr               (reconfig_from_xcvr[459:0]),
         
         .tx_out0                          (tx_out[0]),
         .tx_out1                          (tx_out[1]),
         .tx_out2                          (tx_out[2]),
         .tx_out3                          (tx_out[3]),
         .tx_out4                          (tx_out[4]),
         .tx_out5                          (tx_out[5]),
         .tx_out6                          (tx_out[6]),
         .tx_out7                          (tx_out[7]),

         .app_int_ack                      (app_int_ack),
         .app_msi_ack                      (app_msi_ack),

         // Link status signals
         .derr_cor_ext_rcv                 (derr_cor_ext_rcv),
         .derr_cor_ext_rpl                 (derr_cor_ext_rpl),
         .derr_rpl                         (derr_rpl),
         .dlup                             (dlup),
         .dlup_exit                        (dlup_exit),
         .ev128ns                          (ev128ns),
         .ev1us                            (ev1us),
         .hotrst_exit                      (hotrst_exit),
         .int_status                       (int_status[3:0]),
         .l2_exit                          (l2_exit),
         .lane_act                         (lane_act[3:0]),
         .ltssmstate                       (ltssmstate[4:0]),
         .rx_par_err                       (rx_par_err),
         .tx_par_err                       (tx_par_err[1:0]),
         .cfg_par_err                      (cfg_par_err),
         .ko_cpl_spc_header                (ko_cpl_spc_header[7:0]),
         .ko_cpl_spc_data                  (ko_cpl_spc_data[11:0]),
         .currentspeed                     (currentspeed[1:0]),
         
         // Inputs
         // Resets
         .npor                             (npor),
         .pin_perst                        (pin_perst),

         // Clocks
         .pld_clk                          (pld_clk),
         .refclk                           (refclk),
         .pld_core_ready                   (pld_core_ready),

         // Local management Interface
         .lmi_addr                         (lmi_addr[11:0]),
         .lmi_din                          (lmi_din[31:0]),
         .lmi_rden                         (lmi_rden),
         .lmi_wren                         (lmi_wren),
         
         // RX Interface
         .rx_st_ready                      (rx_st_ready),
         .rx_st_mask                       (rx_st_mask),

         // TX Interface
         .tx_st_sop                        (tx_st_sop[0:0]),
         .tx_st_eop                        (tx_st_eop[0:0]),
         .tx_st_err                        (tx_st_err[0:0]),
         .tx_st_valid                      (tx_st_valid[0:0]),
         .tx_st_data                       (tx_st_data[63:0]),

         // Reconfiguration Interface
         .reconfig_to_xcvr                 (reconfig_to_xcvr[699:0]),

         // RX Serial interface
         .rx_in0                           (rx_in[0]),
         .rx_in1                           (rx_in[1]),
         .rx_in2                           (rx_in[2]),
         .rx_in3                           (rx_in[3]),
         .rx_in4                           (rx_in[4]),
         .rx_in5                           (rx_in[5]),
         .rx_in6                           (rx_in[6]),
         .rx_in7                           (rx_in[7]),

         // Interrupt Interface
         .app_int_sts                      (app_int_sts),
         .app_msi_num                      (app_msi_num[4:0]),
         .app_msi_req                      (app_msi_req),
         .app_msi_tc                       (app_msi_tc[2:0]),
         .simu_mode_pipe                   (1'b0));

    // -------------------- END ALTERA IP INSTANTIATION  --------------------
    // -------------------- BEGIN RIFFA INSTANTAION --------------------

    // RIFFA channel interface
    wire                      rst_out;
    wire [C_NUM_CHNL-1:0]     chnl_rx_clk;
    wire [C_NUM_CHNL-1:0]     chnl_rx;
    wire [C_NUM_CHNL-1:0]     chnl_rx_ack;
    wire [C_NUM_CHNL-1:0]     chnl_rx_last;
    wire [(C_NUM_CHNL*32)-1:0] chnl_rx_len;
    wire [(C_NUM_CHNL*31)-1:0] chnl_rx_off;
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
    // --------------------  BEGIN USER CODE  --------------------
    genvar                                   i;
    generate
        for (i = 0; i < C_NUM_CHNL; i = i + 1) begin : test_channels
            // Instantiate and assign modules to RIFFA channels. Users should 
            // replace the chnl_tester instantiation with their own core.
            chnl_tester 
                 #(
                   .C_PCI_DATA_WIDTH(C_PCI_DATA_WIDTH)
                   )
            chnl_tester_i
                 (

                  .CLK(chnl_clk),
                  .RST(chnl_reset), // chnl_reset includes riffa_endpoint resets
                  // Rx interface
                  .CHNL_RX_CLK(chnl_rx_clk[i]), 
                  .CHNL_RX(chnl_rx[i]), 
                  .CHNL_RX_ACK(chnl_rx_ack[i]), 
                  .CHNL_RX_LAST(chnl_rx_last[i]), 
                  .CHNL_RX_LEN(chnl_rx_len[`SIG_CHNL_LENGTH_W*i +:`SIG_CHNL_LENGTH_W]), 
                  .CHNL_RX_OFF(chnl_rx_off[`SIG_CHNL_OFFSET_W*i +:`SIG_CHNL_OFFSET_W]), 
                  .CHNL_RX_DATA(chnl_rx_data[C_PCI_DATA_WIDTH*i +:C_PCI_DATA_WIDTH]), 
                  .CHNL_RX_DATA_VALID(chnl_rx_data_valid[i]), 
                  .CHNL_RX_DATA_REN(chnl_rx_data_ren[i]),
                  // Tx interface
                  .CHNL_TX_CLK(chnl_tx_clk[i]), 
                  .CHNL_TX(chnl_tx[i]), 
                  .CHNL_TX_ACK(chnl_tx_ack[i]), 
                  .CHNL_TX_LAST(chnl_tx_last[i]), 
                  .CHNL_TX_LEN(chnl_tx_len[`SIG_CHNL_LENGTH_W*i +:`SIG_CHNL_LENGTH_W]), 
                  .CHNL_TX_OFF(chnl_tx_off[`SIG_CHNL_OFFSET_W*i +:`SIG_CHNL_OFFSET_W]), 
                  .CHNL_TX_DATA(chnl_tx_data[C_PCI_DATA_WIDTH*i +:C_PCI_DATA_WIDTH]), 
                  .CHNL_TX_DATA_VALID(chnl_tx_data_valid[i]), 
                  .CHNL_TX_DATA_REN(chnl_tx_data_ren[i])
                  );    
        end
    endgenerate
    // --------------------  END USER CODE  --------------------
endmodule
