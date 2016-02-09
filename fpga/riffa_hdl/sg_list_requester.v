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
// Filename:			sg_list_requester.v
// Version:				1.00.a
// Verilog Standard:	Verilog-2001
// Description:			Receives scatter gather address/length info and requests
// the scatter gather data from the RX engine. Monitors the state of the scatter
// gather FIFO to make sure it can accommodate the requested data. Also signals
// when the entire scatter gather data has been received so the buffer can be
// overwritten with new data.
// Author:				Matt Jacobsen
// History:				@mattj: Version 2.0
//-----------------------------------------------------------------------------
`define S_SGREQ_IDLE	8'b0000_0001
`define S_SGREQ_WAIT_0	8'b0000_0010
`define S_SGREQ_WAIT_1	8'b0000_0100
`define S_SGREQ_CHECK	8'b0000_1000
`define S_SGREQ_ISSUE	8'b0001_0000
`define S_SGREQ_UPDATE	8'b0010_0000
`define S_SGREQ_COUNT	8'b0100_0000
`define S_SGREQ_FLUSH	8'b1000_0000

`timescale 1ns/1ns
module sg_list_requester #(
	parameter C_FIFO_DATA_WIDTH = 9'd64,
	parameter C_FIFO_DEPTH = 1024,
	parameter C_MAX_READ_REQ = 2,				// Max read: 000=128B, 001=256B, 010=512B, 011=1024B, 100=2048B, 101=4096B
	// Local parameters
	parameter C_FIFO_DEPTH_WIDTH = clog2((2**clog2(C_FIFO_DEPTH))+1),
	parameter C_WORDS_PER_ELEM = 4,
	parameter C_MAX_ELEMS = 200,
	parameter C_MAX_ENTRIES = (C_MAX_ELEMS*C_WORDS_PER_ELEM),
	parameter C_FIFO_COUNT_THRESH = C_FIFO_DEPTH - C_MAX_ENTRIES
)
(
	input CLK,
	input RST,
	input [2:0] CONFIG_MAX_READ_REQUEST_SIZE,			// Maximum read payload: 000=128B, 001=256B, 010=512B, 011=1024B, 100=2048B, 101=4096B

	input USER_RST,								// User reset, should clear FIFO data too
	
	output BUF_RECVD,							// Signals when scatter gather buffer received
	input [31:0] BUF_DATA,						// Buffer data
	input BUF_LEN_VALID,						// Buffer length valid
	input BUF_ADDR_HI_VALID,					// Buffer high address valid
	input BUF_ADDR_LO_VALID,					// Buffer low address valid

	input [C_FIFO_DEPTH_WIDTH-1:0] FIFO_COUNT,	// Scatter gather FIFO count
	output FIFO_FLUSH,							// Scatter gather FIFO flush request
	input FIFO_FLUSHED,							// Scatter gather FIFO flushed
	output FIFO_RST,							// Scatter gather FIFO data reset request

	output RX_REQ,								// Issue a read request
	output [63:0] RX_ADDR,						// Request address
	output [9:0] RX_LEN,						// Request length
	input RX_REQ_ACK,							// Request has been issued
	input RX_DONE								// Request has completed (data received)
);

`include "functions.vh"

reg		[31:0]			rData=0, _rData=0;
reg						rAddrHiValid=0, _rAddrHiValid=0;
reg						rAddrLoValid=0, _rAddrLoValid=0;
reg						rLenValid=0, _rLenValid=0;

(* syn_encoding = "user" *)
(* fsm_encoding = "user" *)
reg		[7:0]			rState=`S_SGREQ_IDLE, _rState=`S_SGREQ_IDLE;
reg						rDone=0, _rDone=0;
reg						rDelay=0, _rDelay=0;
reg		[2:0]			rCarry=0, _rCarry=0;
reg		[3:0]			rValsProp=0, _rValsProp=0;
reg		[63:0]			rAddr=64'd0, _rAddr=64'd0;
reg		[31:0]			rBufWords=0, _rBufWords=0;
reg		[10:0]			rPageRem=0, _rPageRem=0;
reg						rPageSpill=0, _rPageSpill=0;
reg		[10:0]			rPreLen=0, _rPreLen=0;
reg		[2:0]			rMaxPayloadTrain=0, _rMaxPayloadTrain=0;
reg		[2:0]			rMaxPayloadShift=0, _rMaxPayloadShift=0;
reg		[9:0]			rMaxPayload=0, _rMaxPayload=0;
reg						rPayloadSpill=0, _rPayloadSpill=0;
reg		[9:0]			rLen=0, _rLen=0;
reg						rBufWordsEQ0Hi=0, _rBufWordsEQ0Hi=0;
reg						rBufWordsEQ0Lo=0, _rBufWordsEQ0Lo=0;
reg						rUserRst=0, _rUserRst=0;

reg						rRecvdAll=0, _rRecvdAll=0;
reg		[10:0]			rAckCount=0, _rAckCount=0;


assign BUF_RECVD = rDone;

assign FIFO_FLUSH = rState[7]; // S_SGREQ_FLUSH
assign FIFO_RST = (rUserRst & rState[0]); // S_SGREQ_IDLE

assign RX_ADDR = rAddr;
assign RX_LEN = rLen;
assign RX_REQ = rState[4]; // S_SGREQ_ISSUE


// Buffer signals coming from outside the rx_port.
always @ (posedge CLK) begin
	rData <= #1 _rData;
	rAddrHiValid <= #1 _rAddrHiValid;
	rAddrLoValid <= #1 _rAddrLoValid;
	rLenValid <= #1 _rLenValid;
end

always @ (*) begin
	_rData = BUF_DATA;
	_rAddrHiValid = BUF_ADDR_HI_VALID;
	_rAddrLoValid = BUF_ADDR_LO_VALID;
	_rLenValid = BUF_LEN_VALID;
end


// Handle requesting the next scatter gather buffer data.
wire [9:0] wAddrLoInv = ~rAddr[11:2];
always @ (posedge CLK) begin
	rState <= #1 (RST ? `S_SGREQ_IDLE : _rState);
	rDone <= #1 (RST ? 1'd0 : _rDone);
	rDelay <= #1 _rDelay;
	rAddr <= #1 _rAddr;
	rCarry <= #1 _rCarry;
	rBufWords <= #1 _rBufWords;
	rValsProp <= #1 _rValsProp;
	rPageRem <= #1 _rPageRem;
	rPageSpill <= #1 _rPageSpill;
	rPreLen <= #1 _rPreLen;
	rMaxPayloadTrain <= #1 _rMaxPayloadTrain;
	rMaxPayloadShift <= #1 _rMaxPayloadShift;
	rMaxPayload <= #1 _rMaxPayload;
	rPayloadSpill <= #1 _rPayloadSpill;
	rLen <= #1 _rLen;
	rBufWordsEQ0Hi <= #1 _rBufWordsEQ0Hi;
	rBufWordsEQ0Lo <= #1 _rBufWordsEQ0Lo;
	rUserRst <= #1 (RST ? 1'd0 : _rUserRst);
end

always @ (*) begin
	_rState = rState;
	_rDone = rDone;
	_rDelay = rDelay;

	_rUserRst = ((rUserRst & !rState[0]) | USER_RST);
	_rValsProp = ((rValsProp<<1) | RX_REQ_ACK);
	{_rCarry[0], _rAddr[15:0]} = (rAddrLoValid ? rData[15:0] : (rAddr[15:0] + ({12{RX_REQ_ACK}} & {2'b0,rLen}<<2)));
	{_rCarry[1], _rAddr[31:16]} = (rAddrLoValid ? rData[31:16] : (rAddr[31:16] + rCarry[0]));
	{_rCarry[2], _rAddr[47:32]} = (rAddrHiValid ? rData[15:0] : (rAddr[47:32] + rCarry[1]));
				 _rAddr[63:48] = (rAddrHiValid ? rData[31:16] : (rAddr[63:48] + rCarry[2]));
	_rBufWords = (rLenValid ? rData : rBufWords) - ({10{RX_REQ_ACK}} & rLen);
	_rPageRem = (wAddrLoInv + 1'd1);	
	_rPageSpill = (rBufWords > rPageRem);	
	_rPreLen = (rPageSpill ? rPageRem : rBufWords[10:0]);			
	_rMaxPayloadTrain = (CONFIG_MAX_READ_REQUEST_SIZE > 3'd4 ? 3'd4 : CONFIG_MAX_READ_REQUEST_SIZE);
	_rMaxPayloadShift = (C_MAX_READ_REQ[2:0] < rMaxPayloadTrain ? C_MAX_READ_REQ[2:0] : rMaxPayloadTrain);
	_rMaxPayload = (6'd32<<rMaxPayloadShift);
	_rPayloadSpill = (rPreLen > rMaxPayload);
	_rLen = (rPayloadSpill ? rMaxPayload : rPreLen[9:0]);
	_rBufWordsEQ0Hi = (16'd0 == rBufWords[31:16]);
	_rBufWordsEQ0Lo = (16'd0 == rBufWords[15:0]);

	case (rState)

	`S_SGREQ_IDLE: begin // Wait for addr & length
		_rDone = 0;
		if (rLenValid)
			_rState = `S_SGREQ_WAIT_0;
	end

	`S_SGREQ_WAIT_0: begin // Wait 1 cycle for values to propagate
		_rDelay = 0;
		_rState = `S_SGREQ_WAIT_1;
	end

	`S_SGREQ_WAIT_1: begin // Wait 2 cycles for values to propagate
		_rDelay = 1;
		if (rDelay)
			_rState = `S_SGREQ_CHECK;
	end

	`S_SGREQ_CHECK: begin // Wait for space to be made available
		if (FIFO_COUNT < C_FIFO_COUNT_THRESH)
			_rState = `S_SGREQ_ISSUE;
		else if (rUserRst)
			_rState = `S_SGREQ_COUNT;
	end
	
	`S_SGREQ_ISSUE: begin // Wait for read request to be serviced
		if (RX_REQ_ACK)
			_rState = `S_SGREQ_UPDATE;
	end

	`S_SGREQ_UPDATE: begin // Update the address and length
		if (rUserRst | (rBufWordsEQ0Hi & rBufWordsEQ0Lo))
			_rState = `S_SGREQ_COUNT;
		else if (rValsProp[3])
			_rState = `S_SGREQ_ISSUE;
	end

	`S_SGREQ_COUNT: begin // Wait for read data to arrive
		if (rRecvdAll)
			_rState = `S_SGREQ_FLUSH;
	end

	`S_SGREQ_FLUSH: begin // Wait for read data to arrive
		if (FIFO_FLUSHED) begin
			_rDone = !rUserRst;
			_rState = `S_SGREQ_IDLE;
		end
	end

	default: begin
		_rState = `S_SGREQ_IDLE;
	end

	endcase
end


// Keep track of requests made and requests completed so we know when all
// the outstanding data has been received.
always @ (posedge CLK) begin
	rAckCount <= #1 (RST ? 10'd0 : _rAckCount);
	rRecvdAll <= #1 _rRecvdAll;
end

always @ (*) begin
	// Track REQ_DONE and SG_DONE to maintain an outstanding request count.
	_rRecvdAll = (rAckCount == 10'd0);
	if (rState[0]) // S_SGREQ_IDLE
		_rAckCount = 0;
	else
		_rAckCount = rAckCount + RX_REQ_ACK - RX_DONE; 
end


endmodule
