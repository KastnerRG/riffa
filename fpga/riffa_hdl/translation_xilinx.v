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
/*
 Filename: translation_layer.v
 Version: 1.0
 Verilog Standard: Verilog-2001

 Description: The translation layer provides a uniform interface for all classic
 PCIe interfaces, such as all Altera devices, and all Xilinx devices (pre VC709).
 
 Notes: Any modifications to this file should meet the conditions set
 forth in the "Trellis Style Guide"

 Author: Dustin Richmond (@darichmond) 
 Co-Authors:
 */
`include "trellis.vh" // Defines the user-facing signal widths.
`include "xilinx.vh"
module translation_xilinx
    #(
      parameter C_PCI_DATA_WIDTH = 256
      )
    (
     input                                    CLK,
     input                                    RST_IN,

     // Interface: Xilinx RX 
     input [C_PCI_DATA_WIDTH-1:0]             M_AXIS_RX_TDATA,
     input [(C_PCI_DATA_WIDTH/8)-1:0]         M_AXIS_RX_TKEEP,
     input                                    M_AXIS_RX_TLAST,
     input                                    M_AXIS_RX_TVALID,
     output                                   M_AXIS_RX_TREADY,
     input [`SIG_XIL_RX_TUSER_W-1:0]          M_AXIS_RX_TUSER,
     output                                   RX_NP_OK,
     output                                   RX_NP_REQ,

     // Interface: Xilinx TX
     output [C_PCI_DATA_WIDTH-1:0]            S_AXIS_TX_TDATA,
     output [(C_PCI_DATA_WIDTH/8)-1:0]        S_AXIS_TX_TKEEP,
     output                                   S_AXIS_TX_TLAST,
     output                                   S_AXIS_TX_TVALID,
     input                                    S_AXIS_TX_TREADY,
     output [`SIG_XIL_TX_TUSER_W-1:0]         S_AXIS_TX_TUSER,
     output                                   TX_CFG_GNT,

     // Interface: Xilinx Configuration 
     input [`SIG_BUSID_W-1:0]                 CFG_BUS_NUMBER,
     input [`SIG_DEVID_W-1:0]                 CFG_DEVICE_NUMBER,
     input [`SIG_FNID_W-1:0]                  CFG_FUNCTION_NUMBER,
     input [`SIG_CFGREG_W-1:0]                CFG_COMMAND,
     input [`SIG_CFGREG_W-1:0]                CFG_DCOMMAND,
     input [`SIG_CFGREG_W-1:0]                CFG_LSTATUS,
     input [`SIG_CFGREG_W-1:0]                CFG_LCOMMAND,

     // Interface: Xilinx Flow Control
     input [`SIG_FC_CPLD_W-1:0]               FC_CPLD,
     input [`SIG_FC_CPLH_W-1:0]               FC_CPLH,
     output [`SIG_FC_SEL_W-1:0]               FC_SEL,
    
     // Interface: Xilinx Interrupt
     input                                    CFG_INTERRUPT_MSIEN,
     input                                    CFG_INTERRUPT_RDY,
     output                                   CFG_INTERRUPT,
    
     // Interface: RX Classic
     output [C_PCI_DATA_WIDTH-1:0]            RX_TLP,
     output                                   RX_TLP_VALID,
     output                                   RX_TLP_START_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0] RX_TLP_START_OFFSET,
     output                                   RX_TLP_END_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0] RX_TLP_END_OFFSET,
     output [`SIG_BARDECODE_W-1:0]            RX_TLP_BAR_DECODE,
     input                                    RX_TLP_READY,

     // Interface: TX Classic
     output                                   TX_TLP_READY,
     input [C_PCI_DATA_WIDTH-1:0]             TX_TLP,
     input                                    TX_TLP_VALID,
     input                                    TX_TLP_START_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]  TX_TLP_START_OFFSET,
     input                                    TX_TLP_END_FLAG,
     input [clog2s(C_PCI_DATA_WIDTH/32)-1:0]  TX_TLP_END_OFFSET,

     // Interface: Configuration
     output [`SIG_CPLID_W-1:0]                CONFIG_COMPLETER_ID,
     output                                   CONFIG_BUS_MASTER_ENABLE, 
     output [`SIG_LINKWIDTH_W-1:0]            CONFIG_LINK_WIDTH,
     output [`SIG_LINKRATE_W-1:0]             CONFIG_LINK_RATE,
     output [`SIG_MAXREAD_W-1:0]              CONFIG_MAX_READ_REQUEST_SIZE, 
     output [`SIG_MAXPAYLOAD_W-1:0]           CONFIG_MAX_PAYLOAD_SIZE,
     output                                   CONFIG_INTERRUPT_MSIENABLE,
     output                                   CONFIG_CPL_BOUNDARY_SEL,

     // Interface: Flow Control
     output [`SIG_FC_CPLD_W-1:0]              CONFIG_MAX_CPL_DATA,
     output [`SIG_FC_CPLH_W-1:0]              CONFIG_MAX_CPL_HDR,

     // Interface: Interrupt     
     output                                   INTR_MSI_RDY, // High when interrupt is able to be sent
     input                                    INTR_MSI_REQUEST // High to request interrupt
     );
    /* 
     Notes on the Configuration Interface:
     Link Width (cfg_lstatus[9:4]): 000001=x1, 000010=x2, 000100=x4, 001000=x8, 001100=x12, 010000=x16
     Link Rate (cfg_lstatus[3:0]): 0001=2.5GT/s, 0010=5.0GT/s, 0011=8.0GT/s
     Max Read Request Size (cfg_dcommand[14:12]): 000=128B, 001=256B, 010=512B, 011=1024B, 100=2048B, 101=4096B
     Max Payload Size (cfg_dcommand[7:5]): 000=128B, 001=256B, 010=512B, 011=1024B
     Bus Master Enable (cfg_command[2]): 1=Enabled, 0=Disabled
     Read Completion Boundary (cfg_lcommand[3]): 0=64 bytes, 1=128 bytes
     MSI Enable (cfg_msicsr[0]): 1=Enabled, 0=Disabled
     
     Notes on the Flow Control Interface:
     FC_CPLD (Xilinx) Receive credit limit for data 
     FC_CPLH (Xilinx) Receive credit limit for headers 
     FC_SEL (Xilinx Only) Selects the correct output on the FC_* signals

     Notes on the TX Interface:
     TX_CFG_GNT (Xilinx): 1=Always allow core to transmit internally generated TLPs
     
     Notes on the RX Interface:
     RX_NP_OK (Xilinx): 1=Always allow non posted transactions
     */
    
    /*AUTOWIRE*/

    reg rRxTlpValid;
    reg rRxTlpEndFlag;

    // Rx Interface (From PCIe Core)
    assign RX_TLP = M_AXIS_RX_TDATA;
    assign RX_TLP_VALID = M_AXIS_RX_TVALID;

    // Rx Interface (To PCIe Core)
    assign M_AXIS_RX_TREADY =  RX_TLP_READY;

    // TX Interface (From PCIe Core)
    assign TX_TLP_READY = S_AXIS_TX_TREADY;

    // TX Interface (TO PCIe Core)
    assign S_AXIS_TX_TDATA = TX_TLP;
    assign S_AXIS_TX_TVALID = TX_TLP_VALID;
    assign S_AXIS_TX_TLAST = TX_TLP_END_FLAG;

    // Configuration Interface
    assign CONFIG_COMPLETER_ID = {CFG_BUS_NUMBER,CFG_DEVICE_NUMBER,CFG_FUNCTION_NUMBER};
    assign CONFIG_BUS_MASTER_ENABLE = CFG_COMMAND[`CFG_COMMAND_BUSMSTR_R];
    assign CONFIG_LINK_WIDTH = CFG_LSTATUS[`CFG_LSTATUS_LWIDTH_R];
    assign CONFIG_LINK_RATE = CFG_LSTATUS[`CFG_LSTATUS_LRATE_R];
    assign CONFIG_MAX_READ_REQUEST_SIZE = CFG_DCOMMAND[`CFG_DCOMMAND_MAXREQ_R];
    assign CONFIG_MAX_PAYLOAD_SIZE = CFG_DCOMMAND[`CFG_DCOMMAND_MAXPAY_R];
    assign CONFIG_INTERRUPT_MSIENABLE = CFG_INTERRUPT_MSIEN;
    assign CONFIG_CPL_BOUNDARY_SEL = CFG_LCOMMAND[`CFG_LCOMMAND_RCB_R];
    assign CONFIG_MAX_CPL_DATA = FC_CPLD;
    assign CONFIG_MAX_CPL_HDR = FC_CPLH;

    assign FC_SEL = `SIG_FC_SEL_RX_MAXALLOC_V;
    assign RX_NP_OK = 1'b1;
    assign RX_NP_REQ = 1'b1;
    assign TX_CFG_GNT = 1'b1;
    
    // Interrupt interface
    assign CFG_INTERRUPT = INTR_MSI_REQUEST;
    assign INTR_MSI_RDY = CFG_INTERRUPT_RDY;
    generate
        if (C_PCI_DATA_WIDTH == 9'd32) begin : gen_xilinx_32
            assign RX_TLP_START_FLAG = ~rRxTlpValid | rRxTlpEndFlag;
            assign RX_TLP_START_OFFSET = {clog2s(C_PCI_DATA_WIDTH/32){1'b0}};
            assign RX_TLP_END_OFFSET = 0;
            assign RX_TLP_END_FLAG = M_AXIS_RX_TLAST;
            assign S_AXIS_TX_TKEEP = 4'hF;
        end else if (C_PCI_DATA_WIDTH == 9'd64) begin : gen_xilinx_64
            assign RX_TLP_START_FLAG = ~rRxTlpValid | rRxTlpEndFlag;
            assign RX_TLP_START_OFFSET = {clog2s(C_PCI_DATA_WIDTH/32){1'b0}};
            assign RX_TLP_END_OFFSET = M_AXIS_RX_TKEEP[4];
            assign RX_TLP_END_FLAG = M_AXIS_RX_TLAST;
            assign S_AXIS_TX_TKEEP = {{4{TX_TLP_END_OFFSET | ~TX_TLP_END_FLAG}},4'hF};
        end else if (C_PCI_DATA_WIDTH == 9'd128) begin : gen_xilinx_128
            assign RX_TLP_END_OFFSET = M_AXIS_RX_TUSER[20:19];
            assign RX_TLP_END_FLAG = M_AXIS_RX_TUSER[21];
            assign RX_TLP_START_FLAG = M_AXIS_RX_TUSER[14];
            assign RX_TLP_START_OFFSET = M_AXIS_RX_TUSER[13:12];
            assign S_AXIS_TX_TKEEP = {{4{~TX_TLP_END_FLAG | (TX_TLP_END_OFFSET == 2'b11)}},
                                      {4{~TX_TLP_END_FLAG | (TX_TLP_END_OFFSET >= 2'b10)}},
                                      {4{~TX_TLP_END_FLAG | (TX_TLP_END_OFFSET >= 2'b01)}},
                                      {4{1'b1}}};// TODO: More efficient if we use masks...
        end else if (C_PCI_DATA_WIDTH == 9'd256) begin : x256
            // Not possible...
        end
    endgenerate

    always @(posedge CLK) begin
        rRxTlpValid <= RX_TLP_VALID;
        rRxTlpEndFlag <= RX_TLP_END_FLAG;
    end
endmodule // translation_layer

