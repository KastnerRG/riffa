# Oscillator Clocks
create_clock -name CLK1_50 -period 20 [get_ports {CLK1_50}]
create_clock -name CLK2_50 -period 20 [get_ports {CLK2_50}]
create_clock -name CLK3_50 -period 20 [get_ports {CLK3_50}]

# Refclk (100 MHz differential input)
create_clock -period "100 MHz" -name {refclk} [get_ports {PCIE_REFCLK}]

# 50 MHZ PLL Clock
create_generated_clock -name clk50 -source [get_ports {CLK1_50}] [get_nets {*|altpll_component|auto_generated|wire_pll1_clk[0]}]

# 125 MHZ PLL Clock
create_generated_clock -name clk125 -multiply_by 5 -divide_by 2 -source [get_ports {CLK1_50}] [get_nets {*|altpll_component|auto_generated|wire_pll1_clk[1]}]

# 250 MHZ PLL Clock
create_generated_clock -name clk250 -multiply_by 5 -source [get_ports {CLK1_50}] [get_nets {*|altpll_component|auto_generated|wire_pll1_clk[2]}]

derive_pll_clocks 
derive_clock_uncertainty

# Imported from IP Compiler user guide
set_clock_groups -exclusive -group [get_clocks { refclk*clkout }] -group [get_clocks { *div0*coreclkout}]
set_clock_groups -exclusive -group [get_clocks { *central_clk_div0* }] -group [get_clocks { *_hssi_pcie_hip* }] -group [get_clocks { *central_clk_div1* }]
