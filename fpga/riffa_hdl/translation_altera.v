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

 Description: The translation layer provides a uniform interface for all Altera 
 PCIe interfaces
 
 Notes: Any modifications to this file should meet the conditions set
 forth in the "Trellis Style Guide"

 Author: Dustin Richmond (@darichmond) 
 Co-Authors:
 */
`include "trellis.vh" // Defines the user-facing signal widths.
`include "altera.vh"
module translation_altera
    #(
      parameter C_PCI_DATA_WIDTH = 128
      )
    (
     input                          CLK,
     input                          RST_IN,

     // Interface: Altera RX 
     input [C_PCI_DATA_WIDTH-1:0]   RX_ST_DATA,
     input [0:0]                    RX_ST_EOP,
     input [0:0]                    RX_ST_SOP,
     input [0:0]                    RX_ST_VALID,
     output                         RX_ST_READY,
     input [0:0]                    RX_ST_EMPTY,

     // Interface: Altera TX
     output [C_PCI_DATA_WIDTH-1:0]  TX_ST_DATA,
     output [0:0]                   TX_ST_VALID,
     input                          TX_ST_READY,
     output [0:0]                   TX_ST_EOP,
     output [0:0]                   TX_ST_SOP,
     output [0:0]                   TX_ST_EMPTY,

     // Interface: Altera Config
     input [`SIG_CFG_CTL_W-1:0]     TL_CFG_CTL,
     input [`SIG_CFG_ADD_W-1:0]     TL_CFG_ADD,
     input [`SIG_CFG_STS_W-1:0]     TL_CFG_STS,

     // Interface: Altera Flow Control
     input [`SIG_FC_CPLH_W-1:0]     KO_CPL_SPC_HEADER,
     input [`SIG_FC_CPLD_W-1:0]     KO_CPL_SPC_DATA,
    
     // Interface: Altera Interrupt
     input                          APP_MSI_ACK,
     output                         APP_MSI_REQ,

     // Interface: RX Classic
     output [C_PCI_DATA_WIDTH-1:0]  RX_TLP,
     output                         RX_TLP_VALID,
     output                         RX_TLP_START_FLAG,
     output [`SIG_OFFSET_W-1:0]     RX_TLP_START_OFFSET,
     output                         RX_TLP_END_FLAG,
     output [`SIG_OFFSET_W-1:0]     RX_TLP_END_OFFSET,
     output [`SIG_BARDECODE_W-1:0]  RX_TLP_BAR_DECODE,
     input                          RX_TLP_READY,

     // Interface: TX Classic
     output                         TX_TLP_READY,
     input [C_PCI_DATA_WIDTH-1:0]   TX_TLP,
     input                          TX_TLP_VALID,
     input                          TX_TLP_START_FLAG,
     input [`SIG_OFFSET_W-1:0]      TX_TLP_START_OFFSET,
     input                          TX_TLP_END_FLAG,
     input [`SIG_OFFSET_W-1:0]      TX_TLP_END_OFFSET,

     // Interface: Configuration
     output [`SIG_CPLID_W-1:0]      CONFIG_COMPLETER_ID,
     output                         CONFIG_BUS_MASTER_ENABLE, 
     output [`SIG_LINKWIDTH_W-1:0]  CONFIG_LINK_WIDTH,
     output [`SIG_LINKRATE_W-1:0]   CONFIG_LINK_RATE,
     output [`SIG_MAXREAD_W-1:0]    CONFIG_MAX_READ_REQUEST_SIZE, 
     output [`SIG_MAXPAYLOAD_W-1:0] CONFIG_MAX_PAYLOAD_SIZE,
     output                         CONFIG_INTERRUPT_MSIENABLE,
     output                         CONFIG_CPL_BOUNDARY_SEL,
     output [`SIG_FC_CPLD_W-1:0]    CONFIG_MAX_CPL_DATA, // Receive credit limit for data
     output [`SIG_FC_CPLH_W-1:0]    CONFIG_MAX_CPL_HDR, // Receive credit limit for headers

     // Interface: Interrupt     
     output                         INTR_MSI_RDY, // High when interrupt is able to be sent
     input                          INTR_MSI_REQUEST // High to request interrupt
     );
    
    localparam C_ALTERA_TX_READY_LATENCY = 1;// TODO: HMmmmmm 2?
    localparam C_OFFSET_WIDTH  = clog2s(C_PCI_DATA_WIDTH/32);

    reg [C_PCI_DATA_WIDTH-1:0]      rRxStData;
    reg                             rRxStValid;
    reg                             rRxStEop;
    reg                             rRxStSop;

    reg [`SIG_CFG_ADD_W-1:0]        rTlCfgAdd,_rTlCfgAdd;
    reg [`SIG_CFG_CTL_W-1:0]        rTlCfgCtl,_rTlCfgCtl;
    reg [`SIG_CFG_STS_W-1:0]        rTlCfgSts,_rTlCfgSts;

    reg [`SIG_CPLID_W-1:0]          rCfgCompleterId;
    reg                             rCfgBusMstrEnable;
    reg [`SIG_MAXREAD_W-1:0]        rCfgMaxReadRequestSize;
    reg [`SIG_MAXPAYLOAD_W-1:0]     rCfgMaxPayloadSize;
    reg                             rCfgInterruptMsienable;
    reg                             rReadCompletionBoundarySel;

    reg [C_ALTERA_TX_READY_LATENCY-1:0] rTxStReady, _rTxStReady;

    // Rx Interface (To PCIe Core)
    assign RX_ST_READY = RX_TLP_READY;

    // Rx Interface (From PCIe Core)
    assign RX_TLP = RX_ST_DATA;
    assign RX_TLP_VALID = RX_ST_VALID;
    assign RX_TLP_END_FLAG = RX_ST_EOP;
    assign RX_TLP_END_OFFSET = {3'b000,RX_ST_EMPTY};
    assign RX_TLP_START_FLAG = RX_ST_SOP;
    assign RX_TLP_START_OFFSET = 0;

    // TX Interface (From PCIe Core)
    assign TX_TLP_READY = rTxStReady[C_ALTERA_TX_READY_LATENCY-1];

    // TX Interface (To PCIe Core)
    assign TX_ST_DATA = TX_TLP;
    assign TX_ST_VALID = TX_TLP_VALID & TX_TLP_READY;
    assign TX_ST_EOP = TX_TLP_END_FLAG;
    assign TX_ST_SOP = TX_TLP_START_FLAG;
    
    // Configuration Interface
    assign CONFIG_COMPLETER_ID = rCfgCompleterId; 
    assign CONFIG_BUS_MASTER_ENABLE = rCfgBusMstrEnable;
    assign CONFIG_LINK_WIDTH = rTlCfgSts[`TLSTS_LWIDTH_R]; 
    assign CONFIG_LINK_RATE = rTlCfgSts[`TLSTS_LRATE_R];
    assign CONFIG_MAX_READ_REQUEST_SIZE = rCfgMaxReadRequestSize;
    assign CONFIG_MAX_PAYLOAD_SIZE = rCfgMaxPayloadSize;
    assign CONFIG_INTERRUPT_MSIENABLE = rCfgInterruptMsienable;
    assign CONFIG_CPL_BOUNDARY_SEL = rReadCompletionBoundarySel;
    assign CONFIG_MAX_CPL_HDR = KO_CPL_SPC_HEADER;
    assign CONFIG_MAX_CPL_DATA = KO_CPL_SPC_DATA;

    // Interrupt interface 
    assign APP_MSI_REQ = INTR_MSI_REQUEST;
    assign INTR_MSI_RDY = APP_MSI_ACK;

    always @(*) begin
        _rTxStReady = (rTxStReady << 1) | TX_ST_READY;
    end

    always @(posedge CLK) begin
        rTxStReady <= _rTxStReady;
    end

    always @(*) begin
        _rTlCfgCtl = TL_CFG_CTL;
        _rTlCfgAdd = TL_CFG_ADD;
        _rTlCfgSts = TL_CFG_STS;
    end

    always @(posedge CLK) begin // Should be the same clock as pld_clk
        rTlCfgAdd <= _rTlCfgAdd;
        rTlCfgCtl <= _rTlCfgCtl;
        rTlCfgSts <= _rTlCfgSts;
        
        if(rTlCfgAdd == `TLCFG_DEVCTL_I) begin
            rCfgMaxReadRequestSize <= rTlCfgCtl[`TLCTL_MAXREQ_R];
            rCfgMaxPayloadSize <= rTlCfgCtl[`TLCTL_MAXPAY_R];
        end
        
        if(rTlCfgAdd == `TLCFG_LNKCTL_I) begin
            rReadCompletionBoundarySel <= rTlCfgCtl[`TLCTL_RCB_R];
        end
        if(rTlCfgAdd == `TLCFG_PRMCMD_I) begin
            rCfgBusMstrEnable <= rTlCfgCtl[`TLCTL_BUSMSTR_R];
        end
        if(rTlCfgAdd == `TLCFG_MSICSR_I) begin
            rCfgInterruptMsienable <= rTlCfgCtl[`TLCTL_MSIENABLE_R];
        end
        if(rTlCfgAdd == `TLCFG_BUSDEV_I) begin
            rCfgCompleterId <= {rTlCfgCtl[`TLCTL_BUSDEV_R],3'b0};
        end
    end // always @ (posedge CLK)
    generate
        if (C_PCI_DATA_WIDTH == 9'd32) begin : a32
            // Not possible...
        end else if (C_PCI_DATA_WIDTH == 9'd64) begin : a64
            assign TX_ST_EMPTY = 0;
        end else if (C_PCI_DATA_WIDTH == 9'd128) begin : a128
            assign TX_ST_EMPTY = ~TX_TLP_END_OFFSET[1] & TX_ST_EOP;
        end else if (C_PCI_DATA_WIDTH == 9'd256) begin : a256
            assign TX_ST_EMPTY = TX_TLP_END_OFFSET[2];
        end
    endgenerate
endmodule // translation_layer

