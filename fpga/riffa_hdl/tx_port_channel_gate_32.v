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
// Filename:            tx_port_channel_gate_32.v
// Version:             1.00.a
// Verilog Standard:    Verilog-2001
// Description:         Captures transaction open/close events as well as data
// and passes it to the RD_CLK domain through the async_fifo. CHNL_TX_DATA_REN can
// only be high after CHNL_TX goes high and after the CHNL_TX_ACK pulse. When
// CHNL_TX drops, the channel closes (until the next transaction -- signaled by
// CHNL_TX going up again).
// Author:              Matt Jacobsen
// History:             @mattj: Version 2.0
//-----------------------------------------------------------------------------
`define S_TXPORTGATE32_IDLE     2'b00
`define S_TXPORTGATE32_OPENING  2'b01
`define S_TXPORTGATE32_OPEN     2'b10
`define S_TXPORTGATE32_CLOSED   2'b11

`timescale 1ns/1ns
module tx_port_channel_gate_32 
    #(parameter C_DATA_WIDTH = 9'd32,
      // Local parameters
      parameter C_FIFO_DEPTH = 8,
      parameter C_FIFO_DATA_WIDTH = C_DATA_WIDTH + 1)
    (input                          RST,

     input                          RD_CLK, // FIFO read clock
     output [C_FIFO_DATA_WIDTH-1:0] RD_DATA, // FIFO read data
     output                         RD_EMPTY, // FIFO is empty
     input                          RD_EN, // FIFO read enable

     input                          CHNL_CLK, // Channel write clock
     input                          CHNL_TX, // Channel write receive signal
     output                         CHNL_TX_ACK, // Channel write acknowledgement signal
     input                          CHNL_TX_LAST, // Channel last write
     input [31:0]                   CHNL_TX_LEN, // Channel write length (in 32 bit words)
     input [30:0]                   CHNL_TX_OFF, // Channel write offset
     input [C_DATA_WIDTH-1:0]       CHNL_TX_DATA, // Channel write data
     input                          CHNL_TX_DATA_VALID, // Channel write data valid
     output                         CHNL_TX_DATA_REN); // Channel write data has been recieved

    (* syn_encoding = "user" *)
    (* fsm_encoding = "user" *)
    reg [1:0]                       rState=`S_TXPORTGATE32_IDLE, _rState=`S_TXPORTGATE32_IDLE;
    reg                             rFifoWen=0, _rFifoWen=0;
    reg [C_FIFO_DATA_WIDTH-1:0]     rFifoData=0, _rFifoData=0;
    wire                            wFifoFull;

    reg                             rChnlTx=0, _rChnlTx=0;
    reg                             rChnlLast=0, _rChnlLast=0;
    reg [31:0]                      rChnlLen=0, _rChnlLen=0;
    reg [30:0]                      rChnlOff=0, _rChnlOff=0;
    reg                             rAck=0, _rAck=0;
    reg                             rPause=0, _rPause=0;
    reg                             rClosed=0, _rClosed=0;
    reg                             rOpen=0, _rOpen=0;

    assign CHNL_TX_ACK = rAck;
    assign CHNL_TX_DATA_REN = (rOpen & !wFifoFull); // S_TXPORTGATE32_OPEN

    // Buffer the input signals that come from outside the tx_port.
    always @ (posedge CHNL_CLK) begin
        rChnlTx <= #1 (RST ? 1'd0 : _rChnlTx);
        rChnlLast <= #1 _rChnlLast;
        rChnlLen <= #1 _rChnlLen;
        rChnlOff <= #1 _rChnlOff;
    end

    always @ (*) begin
        _rChnlTx = CHNL_TX;
        _rChnlLast = CHNL_TX_LAST;
        _rChnlLen = CHNL_TX_LEN;
        _rChnlOff = CHNL_TX_OFF;
    end

    // FIFO for temporarily storing data from the channel.
    (* RAM_STYLE="DISTRIBUTED" *)
    async_fifo 
        #(.C_WIDTH(C_FIFO_DATA_WIDTH), 
          .C_DEPTH(C_FIFO_DEPTH)) 
    fifo 
        (.WR_CLK(CHNL_CLK),
         .WR_RST(RST),
         .WR_EN(rFifoWen),
         .WR_DATA(rFifoData),
         .WR_FULL(wFifoFull),
         .RD_CLK(RD_CLK),
         .RD_RST(RST),
         .RD_EN(RD_EN),
         .RD_DATA(RD_DATA),
         .RD_EMPTY(RD_EMPTY));

    // Pass the transaction open event, transaction data, and the transaction
    // close event through to the RD_CLK domain via the async_fifo.
    always @ (posedge CHNL_CLK) begin
        rState <= #1 (RST ? `S_TXPORTGATE32_IDLE : _rState);
        rFifoWen <= #1 (RST ? 1'd0 : _rFifoWen);
        rFifoData <= #1 _rFifoData;
        rAck <= #1 (RST ? 1'd0 : _rAck);
        rPause <= #1 (RST ? 1'd0 : _rPause);
        rClosed <= #1 (RST ? 1'd0 : _rClosed);
        rOpen <= #1 (RST ? 1'd0 : _rOpen);
    end

    always @ (*) begin
        _rState = rState;
        _rFifoWen = rFifoWen;
        _rFifoData = rFifoData;
        _rPause = rPause;
        _rAck = rAck;
        _rClosed = rClosed;
        _rOpen = rOpen;
        case (rState)

            `S_TXPORTGATE32_IDLE: begin // Write the len
                _rPause = 0;
                _rClosed = 0;
                _rOpen = 0;
                if (!wFifoFull) begin
                    _rFifoWen = rChnlTx;
                    _rFifoData = {1'd1, rChnlLen};
                    if (rChnlTx)
                        _rState = `S_TXPORTGATE32_OPENING;
                end
            end

            `S_TXPORTGATE32_OPENING: begin // Write the off, last
                // rClosed catches a transfer that opens and subsequently closes
                // without writing data
                _rClosed = (rClosed | !rChnlTx);
                if (!wFifoFull) begin
                    _rAck = rChnlTx;
                    _rFifoData = {1'd1, rChnlOff, rChnlLast};
                    if (rClosed | !rChnlTx)
                        _rState = `S_TXPORTGATE32_CLOSED;
                    else begin
                        _rState = `S_TXPORTGATE32_OPEN;
                        _rOpen = CHNL_TX & rChnlTx;
                    end
                end
            end

            `S_TXPORTGATE32_OPEN: begin // Copy channel data into the FIFO
                _rAck = 0;
                if (!wFifoFull) begin
                    // CHNL_TX_DATA_VALID & CHNL_TX_DATA should really be buffered
                    // but the VALID+REN model seem to make this difficult.
                    _rFifoWen = CHNL_TX_DATA_VALID;
                    _rFifoData = {1'd0, CHNL_TX_DATA};
                end
                if (!rChnlTx)
                    _rState = `S_TXPORTGATE32_CLOSED;
                _rOpen = CHNL_TX & rChnlTx;
            end
            
            `S_TXPORTGATE32_CLOSED: begin // Write the end marker (twice)
                _rAck = 0;
                if (!wFifoFull) begin
                    _rPause = 1;
                    _rFifoWen = 1;
                    _rFifoData = {1'd1, {C_DATA_WIDTH{1'd0}}};
                    if (rPause)
                        _rState = `S_TXPORTGATE32_IDLE;
                end
            end
            
        endcase

    end

endmodule
