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
// ----------------------------------------------------------------------
// Filename:            Filename: tx_multiplexer.v
// Version:             Version: 1.0
// Verilog Standard:    Verilog-2005
// Description: the TX Multiplexer services read and write requests from
// RIFFA channels in round robin order.
// Author: Dustin Richmond (@darichmond) 
// ----------------------------------------------------------------------
`include "trellis.vh"
module tx_multiplexer
    #(
      parameter C_PCI_DATA_WIDTH = 128,
      parameter C_NUM_CHNL = 12,
      parameter C_TAG_WIDTH = 5,
      parameter C_VENDOR = "ALTERA",
      parameter C_DEPTH_PACKETS = 10
      )
    (
     input                                     CLK,
     input                                     RST_IN,

     input [C_NUM_CHNL-1:0]                    WR_REQ, // Write request
     input [(C_NUM_CHNL*`SIG_ADDR_W)-1:0]      WR_ADDR, // Write address
     input [(C_NUM_CHNL*`SIG_LEN_W)-1:0]       WR_LEN, // Write data length
     input [(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0] WR_DATA, // Write data
     output [C_NUM_CHNL-1:0]                   WR_DATA_REN, // Write data read enable
     output [C_NUM_CHNL-1:0]                   WR_ACK, // Write request has been accepted
     output [C_NUM_CHNL-1:0]                   WR_SENT, // Write Reuqest has been sent to the core
     
     input [C_NUM_CHNL-1:0]                    RD_REQ, // Read request
     input [(C_NUM_CHNL*2)-1:0]                RD_SG_CHNL, // Read request channel for scatter gather lists
     input [(C_NUM_CHNL*`SIG_ADDR_W)-1:0]      RD_ADDR, // Read request address
     input [(C_NUM_CHNL*`SIG_LEN_W)-1:0]       RD_LEN, // Read request length
     output [C_NUM_CHNL-1:0]                   RD_ACK, // Read request has been accepted

     output [5:0]                              INT_TAG, // Internal tag to exchange with external
     output                                    INT_TAG_VALID, // High to signal tag exchange 
     input [C_TAG_WIDTH-1:0]                   EXT_TAG, // External tag to provide in exchange for internal tag
     input                                     EXT_TAG_VALID, // High to signal external tag is valid

     output                                    TX_ENG_RD_REQ_SENT, // Read completion request issued
     input                                     RXBUF_SPACE_AVAIL,
     
     // Interface: TXR Engine
     output                                    TXR_DATA_VALID,
     output [C_PCI_DATA_WIDTH-1:0]             TXR_DATA,
     output                                    TXR_DATA_START_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0]  TXR_DATA_START_OFFSET,
     output                                    TXR_DATA_END_FLAG,
     output [clog2s(C_PCI_DATA_WIDTH/32)-1:0]  TXR_DATA_END_OFFSET,
     input                                     TXR_DATA_READY,

     output                                    TXR_META_VALID,
     output [`SIG_FBE_W-1:0]                   TXR_META_FDWBE, 
     output [`SIG_LBE_W-1:0]                   TXR_META_LDWBE,
     output [`SIG_ADDR_W-1:0]                  TXR_META_ADDR,
     output [`SIG_LEN_W-1:0]                   TXR_META_LENGTH,
     output [`SIG_TAG_W-1:0]                   TXR_META_TAG,
     output [`SIG_TC_W-1:0]                    TXR_META_TC,
     output [`SIG_ATTR_W-1:0]                  TXR_META_ATTR,
     output [`SIG_TYPE_W-1:0]                  TXR_META_TYPE,
     output                                    TXR_META_EP,
     input                                     TXR_META_READY,
     input                                     TXR_SENT);

    wire [C_NUM_CHNL-1:0]                      wAckRdData;
    wire                                       wAckValid;
                                      
    reg [C_NUM_CHNL-1:0]                       rAckWrData; // Registered fifo input (only write acks)
    reg [C_NUM_CHNL-1:0]                       rAckRdData; // Registered fifo output (only write acks)
    reg                                        rAckWrEn,_rAckWrEn; // Fifo write enable (RD or WR_ACK)
    reg                                        rAckRdEn; // Fifo read enable (TXR_SENT)

    always @(*) begin
        _rAckWrEn = (WR_ACK != 0) | (RD_ACK != 0);
    end

    always @(posedge CLK) begin
        rAckWrData <= WR_ACK;
        rAckWrEn <= _rAckWrEn;
    end    

    always @(posedge CLK) begin
        rAckRdEn <= TXR_SENT;
        if(rAckRdEn & wAckValid) begin
            rAckRdData <= wAckRdData;//
        end else begin
            rAckRdData <= 0;
        end
    end

    assign WR_SENT = rAckRdData;
    
    fifo
        #(// Parameters
          .C_WIDTH                      (C_NUM_CHNL),
          .C_DEPTH                      (C_DEPTH_PACKETS*8), // This is an extremely conservative estimate...
          .C_DELAY                      (0)
          /*AUTOINSTPARAM*/)
    req_ack_fifo
        (// Outputs
         .WR_READY                      (),
         .RD_DATA                       (wAckRdData),
         .RD_VALID                      (wAckValid),
         // Inputs
         .WR_DATA                       (rAckWrData),
         .WR_VALID                      (rAckWrEn),
         .RD_READY                      (rAckRdEn),
         .RST                           (RST_IN),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    generate
        if(C_PCI_DATA_WIDTH == 32) begin
            tx_multiplexer_32 
                #(/*AUTOINSTPARAM*/
                  // Parameters
                  .C_PCI_DATA_WIDTH     (C_PCI_DATA_WIDTH),
                  .C_NUM_CHNL           (C_NUM_CHNL),
                  .C_TAG_WIDTH          (C_TAG_WIDTH),
                  .C_VENDOR             (C_VENDOR))
            tx_mux 
                (/*AUTOINST*/
                 // Outputs
                 .WR_DATA_REN           (WR_DATA_REN[C_NUM_CHNL-1:0]),
                 .WR_ACK                (WR_ACK[C_NUM_CHNL-1:0]),
                 .RD_ACK                (RD_ACK[C_NUM_CHNL-1:0]),
                 .INT_TAG               (INT_TAG[5:0]),
                 .INT_TAG_VALID         (INT_TAG_VALID),
                 .TX_ENG_RD_REQ_SENT    (TX_ENG_RD_REQ_SENT),
                 .TXR_DATA_VALID        (TXR_DATA_VALID),
                 .TXR_DATA              (TXR_DATA[C_PCI_DATA_WIDTH-1:0]),
                 .TXR_DATA_START_FLAG   (TXR_DATA_START_FLAG),
                 .TXR_DATA_START_OFFSET (TXR_DATA_START_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .TXR_DATA_END_FLAG     (TXR_DATA_END_FLAG),
                 .TXR_DATA_END_OFFSET   (TXR_DATA_END_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .TXR_META_VALID        (TXR_META_VALID),
                 .TXR_META_FDWBE        (TXR_META_FDWBE[`SIG_FBE_W-1:0]),
                 .TXR_META_LDWBE        (TXR_META_LDWBE[`SIG_LBE_W-1:0]),
                 .TXR_META_ADDR         (TXR_META_ADDR[`SIG_ADDR_W-1:0]),
                 .TXR_META_LENGTH       (TXR_META_LENGTH[`SIG_LEN_W-1:0]),
                 .TXR_META_TAG          (TXR_META_TAG[`SIG_TAG_W-1:0]),
                 .TXR_META_TC           (TXR_META_TC[`SIG_TC_W-1:0]),
                 .TXR_META_ATTR         (TXR_META_ATTR[`SIG_ATTR_W-1:0]),
                 .TXR_META_TYPE         (TXR_META_TYPE[`SIG_TYPE_W-1:0]),
                 .TXR_META_EP           (TXR_META_EP),
                 // Inputs
                 .CLK                   (CLK),
                 .RST_IN                (RST_IN),
                 .WR_REQ                (WR_REQ[C_NUM_CHNL-1:0]),
                 .WR_ADDR               (WR_ADDR[(C_NUM_CHNL*`SIG_ADDR_W)-1:0]),
                 .WR_LEN                (WR_LEN[(C_NUM_CHNL*`SIG_LEN_W)-1:0]),
                 .WR_DATA               (WR_DATA[(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0]),
                 .RD_REQ                (RD_REQ[C_NUM_CHNL-1:0]),
                 .RD_SG_CHNL            (RD_SG_CHNL[(C_NUM_CHNL*2)-1:0]),
                 .RD_ADDR               (RD_ADDR[(C_NUM_CHNL*`SIG_ADDR_W)-1:0]),
                 .RD_LEN                (RD_LEN[(C_NUM_CHNL*`SIG_LEN_W)-1:0]),
                 .EXT_TAG               (EXT_TAG[C_TAG_WIDTH-1:0]),
                 .EXT_TAG_VALID         (EXT_TAG_VALID),
                 .RXBUF_SPACE_AVAIL     (RXBUF_SPACE_AVAIL),
                 .TXR_DATA_READY        (TXR_DATA_READY),
                 .TXR_META_READY        (TXR_META_READY));
            
        end else if(C_PCI_DATA_WIDTH == 64) begin

            tx_multiplexer_64 
                #(/*AUTOINSTPARAM*/
                  // Parameters
                  .C_PCI_DATA_WIDTH     (C_PCI_DATA_WIDTH),
                  .C_NUM_CHNL           (C_NUM_CHNL),
                  .C_TAG_WIDTH          (C_TAG_WIDTH),
                  .C_VENDOR             (C_VENDOR))
            tx_mux 
                (/*AUTOINST*/
                 // Outputs
                 .WR_DATA_REN           (WR_DATA_REN[C_NUM_CHNL-1:0]),
                 .WR_ACK                (WR_ACK[C_NUM_CHNL-1:0]),
                 .RD_ACK                (RD_ACK[C_NUM_CHNL-1:0]),
                 .INT_TAG               (INT_TAG[5:0]),
                 .INT_TAG_VALID         (INT_TAG_VALID),
                 .TX_ENG_RD_REQ_SENT    (TX_ENG_RD_REQ_SENT),
                 .TXR_DATA_VALID        (TXR_DATA_VALID),
                 .TXR_DATA              (TXR_DATA[C_PCI_DATA_WIDTH-1:0]),
                 .TXR_DATA_START_FLAG   (TXR_DATA_START_FLAG),
                 .TXR_DATA_START_OFFSET (TXR_DATA_START_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .TXR_DATA_END_FLAG     (TXR_DATA_END_FLAG),
                 .TXR_DATA_END_OFFSET   (TXR_DATA_END_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .TXR_META_VALID        (TXR_META_VALID),
                 .TXR_META_FDWBE        (TXR_META_FDWBE[`SIG_FBE_W-1:0]),
                 .TXR_META_LDWBE        (TXR_META_LDWBE[`SIG_LBE_W-1:0]),
                 .TXR_META_ADDR         (TXR_META_ADDR[`SIG_ADDR_W-1:0]),
                 .TXR_META_LENGTH       (TXR_META_LENGTH[`SIG_LEN_W-1:0]),
                 .TXR_META_TAG          (TXR_META_TAG[`SIG_TAG_W-1:0]),
                 .TXR_META_TC           (TXR_META_TC[`SIG_TC_W-1:0]),
                 .TXR_META_ATTR         (TXR_META_ATTR[`SIG_ATTR_W-1:0]),
                 .TXR_META_TYPE         (TXR_META_TYPE[`SIG_TYPE_W-1:0]),
                 .TXR_META_EP           (TXR_META_EP),
                 // Inputs
                 .CLK                   (CLK),
                 .RST_IN                (RST_IN),
                 .WR_REQ                (WR_REQ[C_NUM_CHNL-1:0]),
                 .WR_ADDR               (WR_ADDR[(C_NUM_CHNL*`SIG_ADDR_W)-1:0]),
                 .WR_LEN                (WR_LEN[(C_NUM_CHNL*`SIG_LEN_W)-1:0]),
                 .WR_DATA               (WR_DATA[(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0]),
                 .RD_REQ                (RD_REQ[C_NUM_CHNL-1:0]),
                 .RD_SG_CHNL            (RD_SG_CHNL[(C_NUM_CHNL*2)-1:0]),
                 .RD_ADDR               (RD_ADDR[(C_NUM_CHNL*`SIG_ADDR_W)-1:0]),
                 .RD_LEN                (RD_LEN[(C_NUM_CHNL*`SIG_LEN_W)-1:0]),
                 .EXT_TAG               (EXT_TAG[C_TAG_WIDTH-1:0]),
                 .EXT_TAG_VALID         (EXT_TAG_VALID),
                 .RXBUF_SPACE_AVAIL     (RXBUF_SPACE_AVAIL),
                 .TXR_DATA_READY        (TXR_DATA_READY),
                 .TXR_META_READY        (TXR_META_READY));

        end else if(C_PCI_DATA_WIDTH == 128) begin

            tx_multiplexer_128
                #(/*AUTOINSTPARAM*/
                  // Parameters
                  .C_PCI_DATA_WIDTH     (C_PCI_DATA_WIDTH),
                  .C_NUM_CHNL           (C_NUM_CHNL),
                  .C_TAG_WIDTH          (C_TAG_WIDTH),
                  .C_VENDOR             (C_VENDOR))
            tx_mux_128_inst
                (/*AUTOINST*/
                 // Outputs
                 .WR_DATA_REN           (WR_DATA_REN[C_NUM_CHNL-1:0]),
                 .WR_ACK                (WR_ACK[C_NUM_CHNL-1:0]),
                 .RD_ACK                (RD_ACK[C_NUM_CHNL-1:0]),
                 .INT_TAG               (INT_TAG[5:0]),
                 .INT_TAG_VALID         (INT_TAG_VALID),
                 .TX_ENG_RD_REQ_SENT    (TX_ENG_RD_REQ_SENT),
                 .TXR_DATA_VALID        (TXR_DATA_VALID),
                 .TXR_DATA              (TXR_DATA[C_PCI_DATA_WIDTH-1:0]),
                 .TXR_DATA_START_FLAG   (TXR_DATA_START_FLAG),
                 .TXR_DATA_START_OFFSET (TXR_DATA_START_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .TXR_DATA_END_FLAG     (TXR_DATA_END_FLAG),
                 .TXR_DATA_END_OFFSET   (TXR_DATA_END_OFFSET[clog2s(C_PCI_DATA_WIDTH/32)-1:0]),
                 .TXR_META_VALID        (TXR_META_VALID),
                 .TXR_META_FDWBE        (TXR_META_FDWBE[`SIG_FBE_W-1:0]),
                 .TXR_META_LDWBE        (TXR_META_LDWBE[`SIG_LBE_W-1:0]),
                 .TXR_META_ADDR         (TXR_META_ADDR[`SIG_ADDR_W-1:0]),
                 .TXR_META_LENGTH       (TXR_META_LENGTH[`SIG_LEN_W-1:0]),
                 .TXR_META_TAG          (TXR_META_TAG[`SIG_TAG_W-1:0]),
                 .TXR_META_TC           (TXR_META_TC[`SIG_TC_W-1:0]),
                 .TXR_META_ATTR         (TXR_META_ATTR[`SIG_ATTR_W-1:0]),
                 .TXR_META_TYPE         (TXR_META_TYPE[`SIG_TYPE_W-1:0]),
                 .TXR_META_EP           (TXR_META_EP),
                 // Inputs
                 .CLK                   (CLK),
                 .RST_IN                (RST_IN),
                 .WR_REQ                (WR_REQ[C_NUM_CHNL-1:0]),
                 .WR_ADDR               (WR_ADDR[(C_NUM_CHNL*`SIG_ADDR_W)-1:0]),
                 .WR_LEN                (WR_LEN[(C_NUM_CHNL*`SIG_LEN_W)-1:0]),
                 .WR_DATA               (WR_DATA[(C_NUM_CHNL*C_PCI_DATA_WIDTH)-1:0]),
                 .RD_REQ                (RD_REQ[C_NUM_CHNL-1:0]),
                 .RD_SG_CHNL            (RD_SG_CHNL[(C_NUM_CHNL*2)-1:0]),
                 .RD_ADDR               (RD_ADDR[(C_NUM_CHNL*`SIG_ADDR_W)-1:0]),
                 .RD_LEN                (RD_LEN[(C_NUM_CHNL*`SIG_LEN_W)-1:0]),
                 .EXT_TAG               (EXT_TAG[C_TAG_WIDTH-1:0]),
                 .EXT_TAG_VALID         (EXT_TAG_VALID),
                 .RXBUF_SPACE_AVAIL     (RXBUF_SPACE_AVAIL),
                 .TXR_DATA_READY        (TXR_DATA_READY),
                 .TXR_META_READY        (TXR_META_READY));

        end 
    endgenerate
endmodule
// Local Variables:
// verilog-library-directories:("." "registers/" "../common/")
// End:
