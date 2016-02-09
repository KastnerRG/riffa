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
// Filename:            DE2Gen1x1If64.v
// Version:             
// Verilog Standard:    Verilog-2001
// Description:         Top level module for RIFFA 2.2 reference design for the
//                      the Altera Cyclone IV IP Compiler for PCI Express
//                      module and the Terasic DE2i Development Board.
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`include "functions.vh"
`include "riffa.vh"
`include "altera.vh"
module DE2Gen1x1If64
    #(// Number of RIFFA Channels
      parameter C_NUM_CHNL = 1,
      // Number of PCIe Lanes
      parameter C_NUM_LANES =  1,
      // Settings from Quartus IP Library
      parameter C_PCI_DATA_WIDTH = 64,
      parameter C_MAX_PAYLOAD_BYTES = 256,
      parameter C_LOG_NUM_TAGS = 5)
   (input         CLK1_50,
    input         CLK2_50,
    input         CLK3_50,
   input         PCIE_RESET_N,
   input         PCIE_REFCLK,
   input [0:0]   PCIE_RX_IN,
   output [0:0]  PCIE_TX_OUT,
   // ----------LEDs ----------
   output [8:0]  LED_G,
   output [17:0] LED_R);
    // ----------PLL Signals----------
    wire                      clk50;
    wire                      clk125;
    wire                      clk250;
    wire                      locked;
    wire                      inclk0;

    // ----------PCIe Core Signals----------
    // ----------PCIe Clocks----------
    wire                      pld_clk;
    wire                      reconfig_clk;
    wire                      core_clk_out;
    wire                      fixedclk_serdes;
    wire                      refclk;
    wire                      rc_pll_locked;
    wire                      cal_blk_clk;

    // ----------PCIe Resets----------
    wire [  4: 0]             pex_msi_num;
    wire                      pll_powerdown;
    wire                      reset_status;
    wire                      crst;
    wire                      npor;
    wire                      srst;
    wire                      gxb_powerdown;

    // ----------PCIe Transaction layer configuration ----------
    wire [  3: 0]             tl_cfg_add;
    wire [ 31: 0]             tl_cfg_ctl;
    wire                      tl_cfg_ctl_wr;
    wire [ 52: 0]             tl_cfg_sts;
    wire                      tl_cfg_sts_wr;
    wire [ 19: 0]             ko_cpl_spc_vc0;
    
    // ----------PCIe Local Management Interface----------
    wire                      lmi_ack;
    wire [ 31: 0]             lmi_dout;
    wire [ 11: 0]             lmi_addr;
    wire [ 31: 0]             lmi_din;
    wire                      lmi_rden;
    wire                      lmi_wren;

    // ----------PCIe Interrupt Interface   ----------
    wire                      app_int_ack;
    wire                      app_msi_ack;
    wire                      app_int_sts;
    wire                      app_msi_req;

    // ----------PCIe Status Signals----------
    wire                      hotrst_exit;
    wire                      l2_exit;
    wire [3:0]                lane_act;
    wire [4:0]                ltssm;
    wire                      pme_to_sr;
    wire                      suc_spd_neg;

    // ----------PCIe RX Interface----------
    wire                      rx_st_mask0;
    wire [  7: 0]             rx_st_bardec0;
    wire [ 15: 0]             rx_st_be0;
    wire [0:0]                rx_st_sop0;
    wire [0:0]                rx_st_eop0;
    wire [0:0]                rx_st_err0;
    wire [0:0]                rx_st_valid0;
    wire [0:0]                rx_st_empty0;
    wire                      rx_st_ready0;
    wire [C_PCI_DATA_WIDTH-1:0] rx_st_data0;

    // ----------PCIe TX Interface----------
    wire [0:0]                   tx_st_sop0;
    wire [0:0]                   tx_st_eop0;
    wire [0:0]                   tx_st_err0;
    wire [0:0]                   tx_st_valid0;
    wire [0:0]                   tx_st_empty0;
    wire                         tx_st_ready0;
    wire [C_PCI_DATA_WIDTH-1:0]  tx_st_data0;

   // ----------ALTGX Signals----------
   wire                         busy;
   wire                         busy_altgxb_reconfig;
   wire [4:0]                   reconfig_fromgxb;
   wire [3:0]                   reconfig_togxb;

    // ----------Resets ----------
    reg [4:0]                    rRstCtr,_rRstCtr;
    reg [2:0]                    rRstSync,_rRstSync;
    wire                         wSyncRst;
    
    always @(*) begin
        _rRstSync = {rRstSync[1:0], ~npor};
        _rRstCtr = rRstCtr;
        if (rRstSync[2]) begin
            _rRstCtr = 0;
        end else if (~rRstCtr[4]) begin
            _rRstCtr = rRstCtr + 1;
        end
    end
    
    always @(posedge pld_clk) begin
        rRstSync <= _rRstSync;
        rRstCtr <= _rRstCtr;
    end

    assign wSyncRst = ~ rRstCtr[4];
    assign srst = wSyncRst;
    assign crst = wSyncRst;

    // ----------PLL assignments----------
    assign inclk0 = CLK1_50;
    assign fixedclk_serdes = clk125;
    assign reconfig_clk = clk50;

    // ----------PCIe Resets----------
    assign npor = PCIE_RESET_N;
    assign gxb_powerdown = ~ npor;
    assign pll_powerdown = ~ npor;
    
    // ----------PCIe Clocks / PLLs----------
    assign refclk = PCIE_REFCLK;
    assign pld_clk = core_clk_out;
    assign cal_blk_clk = reconfig_clk;

    // ----------ALTGX----------
    assign busy = busy_altgxb_reconfig;

    // -------------------- BEGIN ALTERA IP INSTANTIATION  --------------------
    ALTPLL50I50O125O250O ALTPLL50I50O125O250O_inst
        (
         // Outputs
         .c0                               (clk50),
         .c1                               (clk125),
         .c2                               (clk250),
         .locked                           (locked),
         // Inputs
         .inclk0                           (inclk0));

   ALTGXPCIeGen1x1
     altgx_inst
       (
        // Outputs
        .busy                           (busy),
        .reconfig_togxb                 (reconfig_togxb[3:0]),
        // Inputs
        .reconfig_clk                   (reconfig_clk),
        .reconfig_fromgxb               (reconfig_fromgxb[4:0]));

   PCIeGen1x1If64 
     pcie_inst
       (
        // Outputs
        .app_int_ack                      (app_int_ack),
        .app_msi_ack                      (app_msi_ack),
        .core_clk_out                     (core_clk_out),
        .hotrst_exit                      (hotrst_exit),
        .l2_exit                          (l2_exit),
        .lane_act                         (lane_act[3:0]),
        .lmi_ack                          (lmi_ack),
        .lmi_dout                         (lmi_dout[31:0]),
        .ltssm                            (ltssm[4:0]),
        .rc_pll_locked                    (rc_pll_locked),
        .reconfig_fromgxb                 (reconfig_fromgxb[4:0]),
        .reset_status                     (reset_status),
        .rx_st_bardec0                    (rx_st_bardec0[7:0]),
        .rx_st_be0                        (rx_st_be0[7:0]),
        .rx_st_data0                      (rx_st_data0[C_PCI_DATA_WIDTH-1:0]),
        .rx_st_eop0                       (rx_st_eop0),
        .rx_st_err0                       (rx_st_err0),
        .rx_st_sop0                       (rx_st_sop0),
        .rx_st_valid0                     (rx_st_valid0),
        .suc_spd_neg                      (suc_spd_neg),// Gen 2 successful
        .tl_cfg_add                       (tl_cfg_add[3:0]), 
        .tl_cfg_ctl                       (tl_cfg_ctl[31:0]),
        .tl_cfg_ctl_wr                    (tl_cfg_ctl_wr),
        .tl_cfg_sts                       (tl_cfg_sts[52:0]),
        .tl_cfg_sts_wr                    (tl_cfg_sts_wr),
        .ko_cpl_spc_vc0                   (ko_cpl_spc_vc0),
        .tx_out0                          (PCIE_TX_OUT[0]),
        .tx_st_ready0                     (tx_st_ready0),
        // Inputs
        .app_int_sts                      (app_int_sts),
        .app_msi_num                      (5'b00000),
        .app_msi_req                      (app_msi_req),
        .app_msi_tc                       (3'b000),
        .busy_altgxb_reconfig             (busy_altgxb_reconfig),
        .cal_blk_clk                      (cal_blk_clk),
        .crst                             (crst),
        .fixedclk_serdes                  (fixedclk_serdes),
        .gxb_powerdown                    (gxb_powerdown),
        .pll_powerdown                    (pll_powerdown),
        .lmi_addr                         (lmi_addr[11:0]),
        .lmi_din                          (lmi_din[31:0]),
        .lmi_rden                         (lmi_rden),
        .lmi_wren                         (lmi_wren),
        .npor                             (npor),
        .pex_msi_num                      (pex_msi_num[4:0]),
        .pld_clk                          (pld_clk),
        .reconfig_clk                     (reconfig_clk),
        .reconfig_togxb                   (reconfig_togxb[3:0]),
        .refclk                           (refclk),
        .rx_in0                           (PCIE_RX_IN[0]),
        .rx_st_ready0                     (rx_st_ready0),
        .srst                             (srst),
        .tx_st_data0                      (tx_st_data0[C_PCI_DATA_WIDTH-1:0]),
        .tx_st_eop0                       (tx_st_eop0),
        .tx_st_err0                       (tx_st_err0),
        .tx_st_sop0                       (tx_st_sop0),
        .tx_st_valid0                     (tx_st_valid0));

    // -------------------- END ALTERA IP INSTANTIATION  --------------------
    // -------------------- BEGIN RIFFA INSTANTAION --------------------

    // ----------RIFFA channel interface----------
    wire [C_NUM_CHNL-1:0]        chnl_rx_clk;
    wire [C_NUM_CHNL-1:0]        chnl_rx;
    wire [C_NUM_CHNL-1:0]        chnl_rx_ack;
    wire [C_NUM_CHNL-1:0]        chnl_rx_last;
    wire [(C_NUM_CHNL*32)-1:0]   chnl_rx_len;
    wire [(C_NUM_CHNL*31)-1:0]   chnl_rx_off;
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
    wire                                     rst_out;

    assign chnl_clk = pld_clk;
    assign chnl_reset = rst_out;
    
    riffa_wrapper_de2i
        #(/*AUTOINSTPARAM*/
          // Parameters
          .C_LOG_NUM_TAGS               (C_LOG_NUM_TAGS),
          .C_NUM_CHNL                   (C_NUM_CHNL),
          .C_PCI_DATA_WIDTH             (C_PCI_DATA_WIDTH),
          .C_MAX_PAYLOAD_BYTES          (C_MAX_PAYLOAD_BYTES))
    riffa
        (// Outputs
         .RX_ST_READY                   (rx_st_ready0),
         .TX_ST_DATA                    (tx_st_data0[C_PCI_DATA_WIDTH-1:0]),
         .TX_ST_VALID                   (tx_st_valid0[0:0]),
         .TX_ST_EOP                     (tx_st_eop0[0:0]),
         .TX_ST_SOP                     (tx_st_sop0[0:0]),
         .TX_ST_EMPTY                   (tx_st_empty0[0:0]),
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
         .RX_ST_DATA                    (rx_st_data0[C_PCI_DATA_WIDTH-1:0]),
         .RX_ST_EOP                     (rx_st_eop0[0:0]),
         .RX_ST_SOP                     (rx_st_sop0[0:0]),
         .RX_ST_VALID                   (rx_st_valid0[0:0]),
         .RX_ST_EMPTY                   (rx_st_empty0[0:0]),
         .TX_ST_READY                   (tx_st_ready0),
         .TL_CFG_CTL                    (tl_cfg_ctl[`SIG_CFG_CTL_W-1:0]),
         .TL_CFG_ADD                    (tl_cfg_add[`SIG_CFG_ADD_W-1:0]),
         .TL_CFG_STS                    (tl_cfg_sts[`SIG_CFG_STS_W-1:0]),
         .KO_CPL_SPC_HEADER             (ko_cpl_spc_vc0[7:0]),
         .KO_CPL_SPC_DATA               (ko_cpl_spc_vc0[19:8]),
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
                 #(.C_PCI_DATA_WIDTH(C_PCI_DATA_WIDTH))
            chnl_tester_i
                 (.CLK(chnl_clk),
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
                  .CHNL_TX_DATA_REN(chnl_tx_data_ren[i]));    
        end
    endgenerate
    // --------------------  END USER CODE  --------------------
endmodule // DE2i_PCIe
