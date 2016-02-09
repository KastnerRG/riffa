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
// Filename:			rx_port_requester.v
// Version:				1.00.a
// Verilog Standard:	Verilog-2001
// Description:			Issues read requests to the tx_engine for the rx_port
// and sg_list_requester modules in the rx_port. Expects those modules to update
// their address and length values after every request issued. Also expects them
// to update their space available values within 6 cycles of a change to the
// RX_LEN.
// Author:				Matt Jacobsen
// History:				@mattj: Version 2.0
//-----------------------------------------------------------------------------
`define S_RXPORTREQ_RX_TX		2'b00
`define S_RXPORTREQ_TX_RX		2'b01
`define S_RXPORTREQ_ISSUE		2'b10

`timescale 1ns/1ns
module rx_port_requester_mux (
	input RST,
	input CLK,
	
	input SG_RX_REQ,				// Scatter gather RX read request
	input [9:0] SG_RX_LEN,			// Scatter gather RX read request length
	input [63:0] SG_RX_ADDR,		// Scatter gather RX read request address
	output SG_RX_REQ_PROC,			// Scatter gather RX read request processing

	input SG_TX_REQ,				// Scatter gather TX read request
	input [9:0] SG_TX_LEN,			// Scatter gather TX read request length
	input [63:0] SG_TX_ADDR,		// Scatter gather TX read request address
	output SG_TX_REQ_PROC,			// Scatter gather TX read request processing
	
	input MAIN_REQ,					// Main read request
	input [9:0] MAIN_LEN,			// Main read request length
	input [63:0] MAIN_ADDR,			// Main read request address
	output MAIN_REQ_PROC,			// Main read request processing

	output RX_REQ,					// Read request
	input RX_REQ_ACK,				// Read request accepted
	output [1:0] RX_REQ_TAG,		// Read request data tag 
	output [63:0] RX_REQ_ADDR,		// Read request address
	output [9:0] RX_REQ_LEN,		// Read request length

	output REQ_ACK					// Request accepted
);

reg									rRxReqAck=0, _rRxReqAck=0;

(* syn_encoding = "user" *)
(* fsm_encoding = "user" *)
reg		[1:0]						rState=`S_RXPORTREQ_RX_TX, _rState=`S_RXPORTREQ_RX_TX;
reg		[9:0]						rLen=0, _rLen=0;
reg		[63:0]						rAddr=64'd0, _rAddr=64'd0;
reg									rSgRxAck=0, _rSgRxAck=0;
reg									rSgTxAck=0, _rSgTxAck=0;
reg									rMainAck=0, _rMainAck=0;
reg									rAck=0, _rAck=0;


assign SG_RX_REQ_PROC = rSgRxAck;
assign SG_TX_REQ_PROC = rSgTxAck;
assign MAIN_REQ_PROC = rMainAck;

assign RX_REQ = rState[1]; // S_RXPORTREQ_ISSUE
assign RX_REQ_TAG = {rSgTxAck, rSgRxAck};
assign RX_REQ_ADDR = rAddr;
assign RX_REQ_LEN = rLen;

assign REQ_ACK = rAck;


// Buffer signals that come from outside the rx_port.
always @ (posedge CLK) begin
	rRxReqAck <= #1 (RST ? 1'd0 : _rRxReqAck);
end

always @ (*) begin
	_rRxReqAck = RX_REQ_ACK;
end


// Handle issuing read requests. Scatter gather requests are processed
// with higher priority than the main channel.
always @ (posedge CLK) begin
	rState <= #1 (RST ? `S_RXPORTREQ_RX_TX : _rState);
	rLen <= #1 _rLen;
	rAddr <= #1 _rAddr;
	rSgRxAck <= #1 _rSgRxAck;
	rSgTxAck <= #1 _rSgTxAck;
	rMainAck <= #1 _rMainAck;
	rAck <= #1 _rAck;
end

always @ (*) begin
	_rState = rState;
	_rLen = rLen;
	_rAddr = rAddr;
	_rSgRxAck = rSgRxAck;
	_rSgTxAck = rSgTxAck;
	_rMainAck = rMainAck;
	_rAck = rAck;
	case (rState)

	`S_RXPORTREQ_RX_TX: begin // Wait for a new read request
		if (SG_RX_REQ) begin
			_rLen = SG_RX_LEN;
			_rAddr = SG_RX_ADDR;
			_rSgRxAck = 1;
			_rAck = 1;
			_rState = `S_RXPORTREQ_ISSUE;
		end
		else if (SG_TX_REQ) begin
			_rLen = SG_TX_LEN;
			_rAddr = SG_TX_ADDR;
			_rSgTxAck = 1;
			_rAck = 1;
			_rState = `S_RXPORTREQ_ISSUE;
		end
		else if (MAIN_REQ) begin
			_rLen = MAIN_LEN;
			_rAddr = MAIN_ADDR;
			_rMainAck = 1;
			_rAck = 1;
			_rState = `S_RXPORTREQ_ISSUE;
		end
		else begin
			_rState = `S_RXPORTREQ_TX_RX;
		end
	end

	`S_RXPORTREQ_TX_RX: begin // Wait for a new read request
		if (SG_TX_REQ) begin
			_rLen = SG_TX_LEN;
			_rAddr = SG_TX_ADDR;
			_rSgTxAck = 1;
			_rAck = 1;
			_rState = `S_RXPORTREQ_ISSUE;
		end
		else if (SG_RX_REQ) begin
			_rLen = SG_RX_LEN;
			_rAddr = SG_RX_ADDR;
			_rSgRxAck = 1;
			_rAck = 1;
			_rState = `S_RXPORTREQ_ISSUE;
		end
		else if (MAIN_REQ) begin
			_rLen = MAIN_LEN;
			_rAddr = MAIN_ADDR;
			_rMainAck = 1;
			_rAck = 1;
			_rState = `S_RXPORTREQ_ISSUE;
		end
		else begin
			_rState = `S_RXPORTREQ_RX_TX;
		end
	end

	`S_RXPORTREQ_ISSUE: begin // Issue the request
		_rAck = 0;
		if (rRxReqAck) begin
			_rSgRxAck = 0;
			_rSgTxAck = 0;
			_rMainAck = 0;
			if (rSgRxAck)
				_rState = `S_RXPORTREQ_TX_RX;
			else
				_rState = `S_RXPORTREQ_RX_TX;
		end
	end

	default: begin
		_rState = `S_RXPORTREQ_RX_TX;
	end

	endcase
end

endmodule
