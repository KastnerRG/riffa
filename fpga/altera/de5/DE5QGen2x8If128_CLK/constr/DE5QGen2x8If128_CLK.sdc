# ----------------------------------------------------------------------
# Copyright (c) 2016, The Regents of the University of California All
# rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
# 
#     * Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials provided
#       with the distribution.
# 
#     * Neither the name of The Regents of the University of California
#       nor the names of its contributors may be used to endorse or
#       promote products derived from this software without specific
#       prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL REGENTS OF THE
# UNIVERSITY OF CALIFORNIA BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
# TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
# USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
# ----------------------------------------------------------------------
#----------------------------------------------------------------------------
# Filename:            DE5QGen2x8If128.sdc (Qsys)
# Version:             1.00.a
# Verilog Standard:    Verilog-2001
# Description:         Synopsys Design Constraints for the DE5 board.
# These design constrains constrain the PCIE_REFCLK, and 50 MHz Clock Input
# Author:              Dustin Richmond (@darichmond)
#-----------------------------------------------------------------------------
create_clock -name PCIE_REFCLK -period 10.000 [get_ports {PCIE_REFCLK}]
create_clock -name osc_50MHz -period 20.000 [get_ports {OSC_BANK3D_50MHZ}]
create_clock -name riffa_5_clk -period 200 [get_pins {pcie_system_inst|pll_0|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}]
create_clock -name riffa_10_clk -period 100 [get_pins {pcie_system_inst|pll_0|altera_pll_i|general[1].gpll~PLL_OUTPUT_COUNTER|divclk}]
create_clock -name riffa_25_clk -period 40 [get_pins {pcie_system_inst|pll_0|altera_pll_i|general[2].gpll~PLL_OUTPUT_COUNTER|divclk}]
create_clock -name riffa_50_clk -period 20 [get_pins {pcie_system_inst|pll_0|altera_pll_i|general[3].gpll~PLL_OUTPUT_COUNTER|divclk}]
create_clock -name riffa_75_clk -period 13.3 [get_pins {pcie_system_inst|pll_0|altera_pll_i|general[4].gpll~PLL_OUTPUT_COUNTER|divclk}]
create_clock -name riffa_100_clk -period 10 [get_pins {pcie_system_inst|pll_0|altera_pll_i|general[5].gpll~PLL_OUTPUT_COUNTER|divclk}]
create_clock -name riffa_125_clk -period 8 [get_pins {pcie_system_inst|pll_0|altera_pll_i|general[6].gpll~PLL_OUTPUT_COUNTER|divclk}]
create_clock -name riffa_150_clk -period 6.66 [get_pins {pcie_system_inst|pll_0|altera_pll_i|general[7].gpll~PLL_OUTPUT_COUNTER|divclk}]
create_clock -name riffa_175_clk -period 6 [get_pins {pcie_system_inst|pll_0|altera_pll_i|general[8].gpll~PLL_OUTPUT_COUNTER|divclk}]
create_clock -name riffa_200_clk -period 5.33 [get_pins {pcie_system_inst|pll_0|altera_pll_i|general[9].gpll~PLL_OUTPUT_COUNTER|divclk}]
create_clock -name riffa_225_clk -period 4.66 [get_pins {pcie_system_inst|pll_0|altera_pll_i|general[10].gpll~PLL_OUTPUT_COUNTER|divclk}]
create_clock -name riffa_250_clk -period 4 [get_pins {pcie_system_inst|pll_0|altera_pll_i|general[11].gpll~PLL_OUTPUT_COUNTER|divclk}]
create_clock -name user_clk -period 4 [get_pins {pcie_system_inst|pciegen2x8if128|altpcie_hip_256_pipen1b|stratixv_hssi_gen3_pcie_hip|coreclkout}]
set_clock_groups -asynchronous -group {user_clk} -group {riffa_5_clk} -group {riffa_10_clk} -group {riffa_25_clk} -group {riffa_50_clk} -group {riffa_75_clk} -group {riffa_100_clk} -group {riffa_125_clk} -group {riffa_150_clk} -group {riffa_175_clk} -group {riffa_200_clk} -group {riffa_225_clk} -group {riffa_250_clk}

################################################################################
# 13.1 Workround for http://www.altera.com/support/kdb/solutions/rd12162013_581.html?GSA_pos=1&WT.oss_r=1&WT.oss=adce_off_r
################################################################################

set_false_path -to [get_registers *|*.adce_off_r[0]]
set_false_path -to [get_registers *|*.adce_on_rr[0]]
set_false_path -to [get_registers *|reset_sync_pldclk_r[*]]

################################################################################
# End Workround 
################################################################################

derive_pll_clocks -create_base_clocks
derive_clock_uncertainty

################################################################################
# Imports from Example Design (altera/13.1/ip/altera/altera_pcie/altera_pcie_hip_ast_ed/)
################################################################################

######################################################################
# HIP Soft reset controller SDC constraints
set_false_path -to   [get_registers *altpcie_rs_serdes|fifo_err_sync_r[0]]
set_false_path -from [get_registers *sv_xcvr_pipe_native*] -to [get_registers *altpcie_rs_serdes|*]

# HIP testin pins SDC constraints
set_false_path -from [get_pins -compatibility_mode *hip_ctrl*]

######################################################################
# Constraints for CV SIG asynchonous logic
set_false_path -from [get_keepers {*|altpcie_hip_256_pipen1b:altpcie_hip_256_pipen1b|hip_eq_dprio:sigtesten.hip_eq_dprio_inst|altpcie_hip_vecsync2:vecsync_ltssm|altpcie_hip_vecsync:altpcie_hip_vecsync2|data_in_d0[*]}] -to [get_keepers {*|altpcie_hip_256_pipen1b:altpcie_hip_256_pipen1b|hip_eq_dprio:sigtesten.hip_eq_dprio_inst|altpcie_hip_vecsync2:vecsync_ltssm|altpcie_hip_vecsync:altpcie_hip_vecsync2|data_out[*]}]

set_false_path -from [get_keepers {*|altpcie_hip_256_pipen1b:altpcie_hip_256_pipen1b|hip_eq_dprio:sigtesten.hip_eq_dprio_inst|altpcie_hip_vecsync2:vecsync_ltssm|altpcie_hip_vecsync:altpcie_hip_vecsync2|req_wr_clk}] -to [get_keepers {*|altpcie_hip_256_pipen1b:altpcie_hip_256_pipen1b|hip_eq_dprio:sigtesten.hip_eq_dprio_inst|altpcie_hip_vecsync2:vecsync_ltssm|altpcie_hip_vecsync:altpcie_hip_vecsync2|altpcie_hip_bitsync:u_req_rd_clk|sync_regs[*]}]

set_false_path -from [get_keepers {*|altpcie_hip_256_pipen1b:altpcie_hip_256_pipen1b|hip_eq_dprio:sigtesten.hip_eq_dprio_inst|altpcie_hip_vecsync2:vecsync_tls|altpcie_hip_vecsync:altpcie_hip_vecsync2|req_rd_clk_d0}] -to [get_keepers {*|altpcie_hip_256_pipen1b:altpcie_hip_256_pipen1b|hip_eq_dprio:sigtesten.hip_eq_dprio_inst|altpcie_hip_vecsync2:vecsync_tls|altpcie_hip_vecsync:altpcie_hip_vecsync2|altpcie_hip_bitsync:u_ack_wr_clk|sync_regs[*]}]

set_false_path -from [get_keepers {*|altpcie_hip_256_pipen1b:altpcie_hip_256_pipen1b|hip_eq_dprio:sigtesten.hip_eq_dprio_inst|altpcie_hip_vecsync2:vecsync_tls|altpcie_hip_vecsync:altpcie_hip_vecsync2|req_wr_clk}] -to [get_keepers {*|altpcie_hip_256_pipen1b:altpcie_hip_256_pipen1b|hip_eq_dprio:sigtesten.hip_eq_dprio_inst|altpcie_hip_vecsync2:vecsync_tls|altpcie_hip_vecsync:altpcie_hip_vecsync2|altpcie_hip_bitsync:u_req_rd_clk|sync_regs[*]}]

set_false_path -from [get_keepers {*|altpcie_hip_256_pipen1b:altpcie_hip_256_pipen1b|hip_eq_dprio:sigtesten.hip_eq_dprio_inst|altpcie_hip_vecsync2:vecsync_tls|altpcie_hip_vecsync:altpcie_hip_vecsync2|data_in_d0[*]}] -to [get_keepers {*|altpcie_hip_256_pipen1b:altpcie_hip_256_pipen1b|hip_eq_dprio:sigtesten.hip_eq_dprio_inst|altpcie_hip_vecsync2:vecsync_tls|altpcie_hip_vecsync:altpcie_hip_vecsync2|data_out[*]}]

set_false_path -from [get_keepers {*|altpcie_hip_256_pipen1b:altpcie_hip_256_pipen1b|hip_eq_dprio:sigtesten.hip_eq_dprio_inst|altpcie_hip_vecsync2:vecsync_ltssm|altpcie_hip_vecsync:altpcie_hip_vecsync2|req_rd_clk_d0}] -to [get_keepers {*|altpcie_hip_256_pipen1b:altpcie_hip_256_pipen1b|hip_eq_dprio:sigtesten.hip_eq_dprio_inst|altpcie_hip_vecsync2:vecsync_ltssm|altpcie_hip_vecsync:altpcie_hip_vecsync2|altpcie_hip_bitsync:u_ack_wr_clk|sync_regs[*]}]


set_false_path -from [get_keepers {*|altpcie_hip_256_pipen1b:altpcie_hip_256_pipen1b|stratixv_hssi_gen3_pcie_hip~SYNC_DATA_REG122}] -to [get_keepers {*|altpcie_hip_256_pipen1b:altpcie_hip_256_pipen1b|test_out[*]}]

set_false_path -from [get_keepers {*|altpcie_hip_256_pipen1b:altpcie_hip_256_pipen1b|stratixv_hssi_gen3_pcie_hip~SYNC_DATA_REG122}] -to [get_keepers {*|altpcie_hip_256_pipen1b:altpcie_hip_256_pipen1b|hd_altpe3_hip_eq_bypass_ph3:sigtesten.ep_eq_bypass_ph3_inst|test_dbg_eqout[*]}]
                                                                                                                                                
set_false_path -from [get_keepers {*|altpcie_hip_256_pipen1b:altpcie_hip_256_pipen1b|stratixv_hssi_gen3_pcie_hip~SYNC_DATA_REG122}] -to [get_keepers {*|altpcie_hip_256_pipen1b:altpcie_hip_256_pipen1b|hd_altpe3_hip_eq_bypass_ph3:sigtesten.ep_eq_bypass_ph3_inst|test_dbg_eqber[*]}]

set_false_path -from [get_keepers {*|altpcie_hip_256_pipen1b:altpcie_hip_256_pipen1b|stratixv_hssi_gen3_pcie_hip~SYNC_DATA_REG122}] -to [get_keepers {*|altpcie_hip_256_pipen1b:altpcie_hip_256_pipen1b|hd_altpe3_hip_eq_bypass_ph3:sigtesten.ep_eq_bypass_ph3_inst|test_dbg_farend_lf[*]}]

set_false_path -from [get_keepers {*|altpcie_hip_256_pipen1b:altpcie_hip_256_pipen1b|stratixv_hssi_gen3_pcie_hip~SYNC_DATA_REG122}] -to [get_keepers {*|altpcie_hip_256_pipen1b:altpcie_hip_256_pipen1b|hd_altpe3_hip_eq_bypass_ph3:sigtesten.ep_eq_bypass_ph3_inst|test_dbg_farend_fs[*]}]