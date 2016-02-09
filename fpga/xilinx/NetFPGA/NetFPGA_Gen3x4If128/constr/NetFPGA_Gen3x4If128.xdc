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
# Filename:            NetFPGA_Top.xdc
# Version:             1.00.a
# Verilog Standard:    Verilog-2001
# Description:         Xilinx Design Constraints for the NetFPGA board.
# These constrain the PCIE_REFCLK, its DSBUF, LED Pins, and PCIE_RESET_N pin
#
# Author:              Dustin Richmond (@darichmond)
#-----------------------------------------------------------------------------
#
#########################################################################################################################
# User Constraints
#########################################################################################################################

###############################################################################
# User Time Names / User Time Groups / Time Specs
###############################################################################

###############################################################################
# User Physical Constraints
###############################################################################

set_property PACKAGE_PIN AR22 [get_ports {LED[0]}]
set_property PACKAGE_PIN AR23 [get_ports {LED[1]}]

set_property IOSTANDARD LVCMOS15 [get_ports {LED[0]}]
set_property IOSTANDARD LVCMOS15 [get_ports {LED[1]}]

set_false_path -to [get_ports -filter NAME=~LED*]

#########################################################################################################################
# End User Constraints
#########################################################################################################################
#
#
#
#########################################################################################################################
# PCIE Core Constraints
#########################################################################################################################

# SYS reset (input) signal.  The sys_reset_n signal should be
# obtained from the PCI Express interface if possible.  For
# slot based form factors, a system reset signal is usually
# present on the connector.  For cable based form factors, a
# system reset signal may not be available.  In this case, the
# system reset signal must be generated locally by some form of
# supervisory circuit.  You may change the IOSTANDARD and LOC
# to suit your requirements and VCCO voltage banking rules.
# Some 7 series devices do not have 3.3 V I/Os available.
# Therefore the appropriate level shift is required to operate
# with these devices that contain only 1.8 V banks.

set_property PACKAGE_PIN AY35 [get_ports PCIE_RESET_N]
set_property IOSTANDARD LVCMOS15 [get_ports PCIE_RESET_N]
set_property PULLUP true [get_ports PCIE_RESET_N]

# SYS clock 100 MHz (input) signal. The sys_clk_p and sys_clk_n
# signals are the PCI Express reference clock. Virtex-7 GT
# Transceiver architecture requires the use of a dedicated clock
# resources (FPGA input pins) associated with each GT Transceiver.
# To use these pins an IBUFDS primitive (refclk_ibuf) is
# instantiated in user's design.
# Please refer to the Virtex-7 GT Transceiver User Guide
# (UG) for guidelines regarding clock resource selection.
set_property LOC IBUFDS_GTE2_X1Y11 [get_cells refclk_ibuf]

###############################################################################
# Timing Constraints
###############################################################################
create_clock -period 10.000 -name pcie_refclk [get_pins refclk_ibuf/O]

###############################################################################
# Physical Constraints
###############################################################################

set_false_path -from [get_ports PCIE_RESET_N]
###############################################################################
# End
###############################################################################



#set_property MARK_DEBUG true [get_nets {pcie3_7x_0_i/inst/cfg_interrupt_msi_int[0]}]
#set_property MARK_DEBUG true [get_nets {pcie3_7x_0_i/inst/cfg_interrupt_msi_pending_status[0]}]
#set_property MARK_DEBUG true [get_nets pcie3_7x_0_i/inst/cfg_interrupt_msi_fail]
#set_property MARK_DEBUG true [get_nets pcie3_7x_0_i/inst/cfg_interrupt_msi_sent]
#set_property MARK_DEBUG true [get_nets riffa/riffa_inst/reg_inst/wChnlRxDoneReady]
#set_property MARK_DEBUG true [get_nets riffa/riffa_inst/reg_inst/wChnlRxLenValid]
#set_property MARK_DEBUG true [get_nets riffa/riffa_inst/reg_inst/wChnlRxOfflastValid]
#set_property MARK_DEBUG true [get_nets riffa/riffa_inst/reg_inst/wChnlSgRxAddrHiValid]
#set_property MARK_DEBUG true [get_nets riffa/riffa_inst/reg_inst/wChnlSgRxAddrLoValid]
#set_property MARK_DEBUG true [get_nets riffa/riffa_inst/reg_inst/wChnlSgRxLenValid]
#set_property MARK_DEBUG true [get_nets riffa/riffa_inst/reg_inst/wChnlSgTxAddrHiValid]
#set_property MARK_DEBUG true [get_nets riffa/riffa_inst/reg_inst/wChnlSgTxAddrLoValid]
#set_property MARK_DEBUG true [get_nets riffa/riffa_inst/reg_inst/wChnlSgTxLenValid]
#set_property MARK_DEBUG true [get_nets riffa/riffa_inst/reg_inst/wChnlTxDoneReady]
#set_property MARK_DEBUG true [get_nets riffa/riffa_inst/reg_inst/wChnlTxLenReady]
#set_property MARK_DEBUG true [get_nets riffa/riffa_inst/reg_inst/wTransDoneRst]
#set_property MARK_DEBUG true [get_nets riffa/riffa_inst/intr/cfg_interrupt_msi_fail]
#set_property MARK_DEBUG true [get_nets riffa/riffa_inst/intr/cfg_interrupt_msi_sent]
#set_property MARK_DEBUG true [get_nets {riffa/riffa_inst/intr/wIntrVectorReady[0]}]
#set_property MARK_DEBUG true [get_nets {riffa/riffa_inst/intr/wIntrVectorReady[1]}]
#set_property MARK_DEBUG true [get_nets riffa/txr_sent]
#create_debug_core u_ila_0_0 ila
#set_property ALL_PROBE_SAME_MU true [get_debug_cores u_ila_0_0]
#set_property ALL_PROBE_SAME_MU_CNT 2 [get_debug_cores u_ila_0_0]
#set_property C_ADV_TRIGGER false [get_debug_cores u_ila_0_0]
#set_property C_DATA_DEPTH 2048 [get_debug_cores u_ila_0_0]
#set_property C_EN_STRG_QUAL true [get_debug_cores u_ila_0_0]
#set_property C_INPUT_PIPE_STAGES 2 [get_debug_cores u_ila_0_0]
#set_property C_TRIGIN_EN false [get_debug_cores u_ila_0_0]
#set_property C_TRIGOUT_EN false [get_debug_cores u_ila_0_0]
#set_property port_width 1 [get_debug_ports u_ila_0_0/clk]
#connect_debug_port u_ila_0_0/clk [get_nets [list user_clk]]
#set_property port_width 2 [get_debug_ports u_ila_0_0/probe0]
#connect_debug_port u_ila_0_0/probe0 [get_nets [list {riffa/riffa_inst/intr/wIntrVectorReady[0]} {riffa/riffa_inst/intr/wIntrVectorReady[1]}]]
#create_debug_port u_ila_0_0 probe
#set_property port_width 1 [get_debug_ports u_ila_0_0/probe1]
#connect_debug_port u_ila_0_0/probe1 [get_nets [list {pcie3_7x_0_i/inst/cfg_interrupt_msi_pending_status[0]}]]
#create_debug_port u_ila_0_0 probe
#set_property port_width 1 [get_debug_ports u_ila_0_0/probe2]
#connect_debug_port u_ila_0_0/probe2 [get_nets [list {pcie3_7x_0_i/inst/cfg_interrupt_msi_int[0]}]]
#create_debug_port u_ila_0_0 probe
#set_property port_width 1 [get_debug_ports u_ila_0_0/probe3]
#connect_debug_port u_ila_0_0/probe3 [get_nets [list riffa/riffa_inst/intr/cfg_interrupt_msi_fail]]
#create_debug_port u_ila_0_0 probe
#set_property port_width 1 [get_debug_ports u_ila_0_0/probe4]
#connect_debug_port u_ila_0_0/probe4 [get_nets [list pcie3_7x_0_i/inst/cfg_interrupt_msi_fail]]
#create_debug_port u_ila_0_0 probe
#set_property port_width 1 [get_debug_ports u_ila_0_0/probe5]
#connect_debug_port u_ila_0_0/probe5 [get_nets [list pcie3_7x_0_i/inst/cfg_interrupt_msi_sent]]
#create_debug_port u_ila_0_0 probe
#set_property port_width 1 [get_debug_ports u_ila_0_0/probe6]
#connect_debug_port u_ila_0_0/probe6 [get_nets [list riffa/riffa_inst/intr/cfg_interrupt_msi_sent]]
#create_debug_port u_ila_0_0 probe
#set_property port_width 1 [get_debug_ports u_ila_0_0/probe7]
#connect_debug_port u_ila_0_0/probe7 [get_nets [list riffa/txr_sent]]
#create_debug_port u_ila_0_0 probe
#set_property port_width 1 [get_debug_ports u_ila_0_0/probe8]
#connect_debug_port u_ila_0_0/probe8 [get_nets [list riffa/riffa_inst/reg_inst/wChnlRxDoneReady]]
#create_debug_port u_ila_0_0 probe
#set_property port_width 1 [get_debug_ports u_ila_0_0/probe9]
#connect_debug_port u_ila_0_0/probe9 [get_nets [list riffa/riffa_inst/reg_inst/wChnlRxLenValid]]
#create_debug_port u_ila_0_0 probe
#set_property port_width 1 [get_debug_ports u_ila_0_0/probe10]
#connect_debug_port u_ila_0_0/probe10 [get_nets [list riffa/riffa_inst/reg_inst/wChnlRxOfflastValid]]
#create_debug_port u_ila_0_0 probe
#set_property port_width 1 [get_debug_ports u_ila_0_0/probe11]
#connect_debug_port u_ila_0_0/probe11 [get_nets [list riffa/riffa_inst/reg_inst/wChnlSgRxAddrHiValid]]
#create_debug_port u_ila_0_0 probe
#set_property port_width 1 [get_debug_ports u_ila_0_0/probe12]
#connect_debug_port u_ila_0_0/probe12 [get_nets [list riffa/riffa_inst/reg_inst/wChnlSgRxAddrLoValid]]
#create_debug_port u_ila_0_0 probe
#set_property port_width 1 [get_debug_ports u_ila_0_0/probe13]
#connect_debug_port u_ila_0_0/probe13 [get_nets [list riffa/riffa_inst/reg_inst/wChnlSgRxLenValid]]
#create_debug_port u_ila_0_0 probe
#set_property port_width 1 [get_debug_ports u_ila_0_0/probe14]
#connect_debug_port u_ila_0_0/probe14 [get_nets [list riffa/riffa_inst/reg_inst/wChnlSgTxAddrHiValid]]
#create_debug_port u_ila_0_0 probe
#set_property port_width 1 [get_debug_ports u_ila_0_0/probe15]
#connect_debug_port u_ila_0_0/probe15 [get_nets [list riffa/riffa_inst/reg_inst/wChnlSgTxAddrLoValid]]
#create_debug_port u_ila_0_0 probe
#set_property port_width 1 [get_debug_ports u_ila_0_0/probe16]
#connect_debug_port u_ila_0_0/probe16 [get_nets [list riffa/riffa_inst/reg_inst/wChnlSgTxLenValid]]
#create_debug_port u_ila_0_0 probe
#set_property port_width 1 [get_debug_ports u_ila_0_0/probe17]
#connect_debug_port u_ila_0_0/probe17 [get_nets [list riffa/riffa_inst/reg_inst/wChnlTxDoneReady]]
#create_debug_port u_ila_0_0 probe
#set_property port_width 1 [get_debug_ports u_ila_0_0/probe18]
#connect_debug_port u_ila_0_0/probe18 [get_nets [list riffa/riffa_inst/reg_inst/wChnlTxLenReady]]
#create_debug_port u_ila_0_0 probe
#set_property port_width 1 [get_debug_ports u_ila_0_0/probe19]
#connect_debug_port u_ila_0_0/probe19 [get_nets [list riffa/riffa_inst/reg_inst/wTransDoneRst]]
#set_property MARK_DEBUG true [get_nets riffa/engine_layer_inst/rx_engine_ultrascale_inst/RXR_DATA_VALID]
#set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
#set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
#set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
#connect_debug_port dbg_hub/clk [get_nets user_clk]
