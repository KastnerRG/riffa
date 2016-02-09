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
# Filename:            DE4Gen2x8If128.sdc
# Version:             1.00.a
# Verilog Standard:    Verilog-2001
# Description:         Synopsys Design Constraints for the DE4 board.
# These design constrains constrain the PCIE_REFCLK, and 50 MHz Clock Input
# Author:              Dustin Richmond (@darichmond)
#-----------------------------------------------------------------------------
# Oscillator clk (50 MHz Input)
create_clock -name OSCILLATOR_CLK -period 20 [get_ports {OSC_50_BANK2}]

# Refclk (100 MHz differential input)
create_clock -period "100 MHz" -name {refclk} [get_ports {PCIE_REFCLK}]

# 50 MHZ PLL Clock
create_generated_clock -name clk50 -source [get_ports {OSC_50_BANK2}] [get_nets {*|altpll_component|auto_generated|wire_pll1_clk[0]}]

# 125 MHZ PLL Clock
create_generated_clock -name clk125 -multiply_by 5 -divide_by 2 -source [get_ports {OSC_50_BANK2}] [get_nets {*|altpll_component|auto_generated|wire_pll1_clk[1]}]

# 250 MHZ PLL Clock
create_generated_clock -name clk250 -multiply_by 5 -source [get_ports {OSC_50_BANK2}] [get_nets {*|altpll_component|auto_generated|wire_pll1_clk[2]}]

derive_pll_clocks 
derive_clock_uncertainty


# Imported from IP Compiler user guide
set_clock_groups -exclusive -group [get_clocks { refclk*clkout }] -group [get_clocks { *div0*coreclkout}]
set_clock_groups -exclusive -group [get_clocks { *central_clk_div0* }] -group [get_clocks { *_hssi_pcie_hip* }] -group [get_clocks { *central_clk_div1* }]

