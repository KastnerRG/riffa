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
// Filename:			rx_port_reader.v
// Version:				1.00.a
// Verilog Standard:	Verilog-2001
// Description:			Handles the RX lifecycle and issuing requests for receiving
// data input. 
//						for the RIFFA channel.
// Author:				Matt Jacobsen
// History:				@mattj: Version 2.0
//-----------------------------------------------------------------------------
`define S_RXPORTRD_MAIN_IDLE		6'b00_0001
`define S_RXPORTRD_MAIN_CHECK		6'b00_0010
`define S_RXPORTRD_MAIN_READ		6'b00_0100
`define S_RXPORTRD_MAIN_FLUSH		6'b00_1000
`define S_RXPORTRD_MAIN_DONE		6'b01_0000
`define S_RXPORTRD_MAIN_RESET		6'b10_0000

`define S_RXPORTRD_RX_IDLE		8'b0000_0001
`define S_RXPORTRD_RX_BUF		8'b0000_0010
`define S_RXPORTRD_RX_ADJ_0		8'b0000_0100
`define S_RXPORTRD_RX_ADJ_1		8'b0000_1000
`define S_RXPORTRD_RX_ISSUE		8'b0001_0000
`define S_RXPORTRD_RX_WAIT_0	8'b0010_0000
`define S_RXPORTRD_RX_WAIT_1	8'b0100_0000
`define S_RXPORTRD_RX_DONE		8'b1000_0000

`timescale 1ns/1ns
module rx_port_reader #(
	parameter C_DATA_WIDTH = 9'd64,
	parameter C_FIFO_DEPTH = 1024,
	parameter C_MAX_READ_REQ = 2,				// Max read: 000=128B, 001=256B, 010=512B, 011=1024B, 100=2048B, 101=4096B
	// Local parameters
	parameter C_DATA_WORD_WIDTH = clog2((C_DATA_WIDTH/32)+1),
	parameter C_FIFO_WORDS = (C_DATA_WIDTH/32)*C_FIFO_DEPTH
)
(
	input CLK,
	input RST,
	input [2:0] CONFIG_MAX_READ_REQUEST_SIZE,			// Maximum read payload: 000=128B, 001=256B, 010=512B, 011=1024B, 100=2048B, 101=4096B
	
	input [31:0] TXN_DATA,						// Read transaction data
	input TXN_LEN_VALID,						// Read transaction length valid
	input TXN_OFF_LAST_VALID,					// Read transaction offset/last valid
	output [31:0] TXN_DONE_LEN,					// Read transaction actual transfer length
	output TXN_DONE,							// Read transaction done
	output TXN_ERR,								// Read transaction completed with error
	input TXN_DONE_ACK,							// Read transaction actual transfer length read

	output TXN_DATA_FLUSH,						// Request that all data in the packer be flushed
	input TXN_DATA_FLUSHED,						// All data in the packer has been flushed

	output RX_REQ,								// Issue a read request
	output [63:0] RX_ADDR,						// Request address
	output [9:0] RX_LEN,						// Request length
	input RX_REQ_ACK,							// Request has been accepted

	input [C_DATA_WORD_WIDTH-1:0] RX_DATA_EN,	// Incoming read data enable
	input RX_DONE,								// Incoming read completed
	input RX_ERR,								// Incoming read completed with error
	input SG_DONE,								// Incoming scatter gather read completed
	input SG_ERR,								// Incoming scatter gather read completed with error

	input [63:0] SG_ELEM_ADDR,					// Scatter gather element address
	input [31:0] SG_ELEM_LEN,					// Scatter gather element length (in words)
	input SG_ELEM_RDY,							// Scatter gather element ready
	output SG_ELEM_REN,							// Scatter gather element read enable
	output SG_RST,								// Scatter gather reset

	output CHNL_RX,								// Signal channel RX
	output [31:0] CHNL_RX_LEN,					// Channel RX length
	output CHNL_RX_LAST,						// Channel RX last
	output [30:0] CHNL_RX_OFF,					// Channel RX offset
	input CHNL_RX_RECVD,						// Channel RX received
	input CHNL_RX_ACK_RECVD,					// Channel RX acknowledgment received
	input [31:0] CHNL_RX_CONSUMED				// Channel words consumed in current RX
);

`include "functions.vh"


reg		[31:0]						rTxnData=0, _rTxnData=0;
reg									rTxnOffLastValid=0, _rTxnOffLastValid=0;
reg									rTxnLenValid=0, _rTxnLenValid=0;
reg		[C_DATA_WORD_WIDTH-1:0]		rRxDataEn=0, _rRxDataEn=0;

(* syn_encoding = "user" *)
(* fsm_encoding = "user" *)
reg		[5:0]						rMainState=`S_RXPORTRD_MAIN_IDLE, _rMainState=`S_RXPORTRD_MAIN_IDLE;
reg		[31:0]						rOffLast=0, _rOffLast=0;
reg		[31:0]						rReadWords=0, _rReadWords=0;
reg									rReadWordsZero=0, _rReadWordsZero=0;
reg		[0:0]						rStart=0, _rStart=0;
reg		[3:0]						rFlushed=0, _rFlushed=0;
reg		[31:0]						rDoneLen=0, _rDoneLen=0;
reg									rTxnDone=0, _rTxnDone=0;

(* syn_encoding = "user" *)
(* fsm_encoding = "user" *)
reg		[7:0]						rRxState=`S_RXPORTRD_RX_IDLE, _rRxState=`S_RXPORTRD_RX_IDLE;
reg									rSgRen=0, _rSgRen=0;
reg		[31:0]						rWords=0, _rWords=0;
reg		[31:0]						rBufWords=0, _rBufWords=0;
reg		[31:0]						rBufWordsInit=0, _rBufWordsInit=0;
reg									rLargeBuf=0, _rLargeBuf=0;
reg		[63:0]						rAddr=64'd0, _rAddr=64'd0;
reg		[3:0]						rValsProp=0, _rValsProp=0;
reg		[2:0]						rCarry=0, _rCarry=0;
reg									rCopyBufWords=0, _rCopyBufWords=0;
reg									rUseInit=0, _rUseInit=0;
reg		[10:0]						rPageRem=0, _rPageRem=0;
reg									rPageSpill=0, _rPageSpill=0;
reg									rPageSpillInit=0, _rPageSpillInit=0;
reg		[10:0]						rPreLen=0, _rPreLen=0;
reg		[2:0]						rMaxPayloadTrain=0, _rMaxPayloadTrain=0;
reg		[2:0]						rMaxPayloadShift=0, _rMaxPayloadShift=0;
reg		[9:0]						rMaxPayload=0, _rMaxPayload=0;
reg									rPayloadSpill=0, _rPayloadSpill=0;
reg									rMaxLen=0, _rMaxLen=0;
reg		[9:0]						rLen=0, _rLen=0;
reg									rLenEQWordsHi=0, _rLenEQWordsHi=0;
reg									rLenEQWordsLo=0, _rLenEQWordsLo=0;
reg									rLenEQBufWordsHi=0, _rLenEQBufWordsHi=0;
reg									rLenEQBufWordsLo=0, _rLenEQBufWordsLo=0;

reg		[31:0]						rRecvdWords=0, _rRecvdWords=0;
reg		[31:0]						rReqdWords=0, _rReqdWords=0;
reg		[31:0]						rRequestingWords=0, _rRequestingWords=0;
reg		[31:0]						rAvailWords=0, _rAvailWords=0;
reg		[31:0]						rPartWords=0, _rPartWords=0;
reg		[10:0]						rAckCount=0, _rAckCount=0;
reg									rAckCountEQ0=0, _rAckCountEQ0=0;
reg									rLastDoneRead=1, _rLastDoneRead=1;
reg									rTxnDoneAck=0, _rTxnDoneAck=0;
reg									rPartWordsRecvd=0, _rPartWordsRecvd=0;
reg									rCarryInv=0, _rCarryInv=0;
reg									rSpaceAvail=0, _rSpaceAvail=0;

reg									rPartialDone=0, _rPartialDone=0;
reg									rReqPartialDone=0, _rReqPartialDone=0;

reg									rErr=0, _rErr=0;


assign TXN_DONE_LEN = rDoneLen;
assign TXN_DONE = (rTxnDone | rPartialDone);
assign TXN_ERR = rErr;

assign TXN_DATA_FLUSH = rMainState[3]; // S_RXPORTRD_MAIN_FLUSH

assign RX_REQ = (rRxState[4] & rSpaceAvail); // S_RXPORTRD_RX_ISSUE
assign RX_ADDR = rAddr;
assign RX_LEN = rLen;

assign SG_ELEM_REN = rSgRen;
assign SG_RST = rMainState[1]; // S_RXPORTRD_MAIN_CHECK

assign CHNL_RX = (rMainState[2] | rMainState[3] | rMainState[4]); // S_RXPORTRD_MAIN_READ | S_RXPORTRD_MAIN_FLUSH | S_RXPORTRD_MAIN_DONE
assign CHNL_RX_LEN = rReadWords;
assign CHNL_RX_LAST = rOffLast[0];
assign CHNL_RX_OFF = rOffLast[31:1];


// Buffer signals that come from outside the rx_port.
always @ (posedge CLK) begin
	rTxnData <= #1 _rTxnData;
	rTxnOffLastValid <= #1 _rTxnOffLastValid;
	rTxnLenValid <= #1 _rTxnLenValid;
	rTxnDoneAck <= #1 (RST ? 1'd0 : _rTxnDoneAck);	
	rRxDataEn <= #1 _rRxDataEn;
end

always @ (*) begin
	_rTxnData = TXN_DATA;
	_rTxnOffLastValid = TXN_OFF_LAST_VALID;
	_rTxnLenValid = TXN_LEN_VALID;
	_rTxnDoneAck = TXN_DONE_ACK;	
	_rRxDataEn = RX_DATA_EN;
end


// Handle RX lifecycle.
always @ (posedge CLK) begin
	rMainState <= #1 (RST ? `S_RXPORTRD_MAIN_IDLE : _rMainState);
	rOffLast <= #1 _rOffLast;
	rReadWords <= #1 _rReadWords;
	rReadWordsZero <= #1 _rReadWordsZero;
	rStart <= #1 _rStart;
	rFlushed <= #1 _rFlushed;
	rDoneLen <= #1 (RST ? 0 : _rDoneLen);
	rTxnDone <= #1 _rTxnDone;
end

always @ (*) begin
	_rMainState = rMainState;
	_rDoneLen = rDoneLen;
	_rTxnDone = rTxnDone;
	
	_rOffLast = (rTxnOffLastValid ? rTxnData : rOffLast);
	_rReadWords = (rMainState[0] & rTxnLenValid ? rTxnData : rReadWords);
	_rReadWordsZero = (rReadWords == 0);
	_rStart = ((rStart<<1) | rTxnLenValid);
	_rFlushed = ((rFlushed<<1) | TXN_DATA_FLUSHED);

	case (rMainState)

	`S_RXPORTRD_MAIN_IDLE: begin // Wait for new read transaction offset/last & length
		_rTxnDone = 0;
		if (rStart[0])
			_rMainState = `S_RXPORTRD_MAIN_CHECK;
	end

	`S_RXPORTRD_MAIN_CHECK: begin // See if we should start a transaction
		if (!rReadWordsZero)
			_rMainState = `S_RXPORTRD_MAIN_READ;
		else if (rOffLast[0])
			_rMainState = `S_RXPORTRD_MAIN_FLUSH;
		else
			_rMainState = `S_RXPORTRD_MAIN_IDLE;
	end
	
	`S_RXPORTRD_MAIN_READ: begin // Issue read transfers, wait for data to arrive
		if (rRxState[7] & rLastDoneRead) begin // S_RXPORTRD_RX_DONE
			_rDoneLen = rRecvdWords;
			_rMainState = `S_RXPORTRD_MAIN_FLUSH;
		end
	end

	`S_RXPORTRD_MAIN_FLUSH: begin // Wait for data to be flushed
		if (rFlushed[3])
			_rMainState = `S_RXPORTRD_MAIN_DONE;
	end

	`S_RXPORTRD_MAIN_DONE: begin // Wait for RX to be received and ackd in the channel
		if (CHNL_RX_RECVD & CHNL_RX_ACK_RECVD)
			_rMainState = `S_RXPORTRD_MAIN_RESET;
	end

	`S_RXPORTRD_MAIN_RESET: begin // Wait until RX has dropped in the channel
		if (!CHNL_RX_RECVD) begin
			_rTxnDone = 1;
			_rMainState = `S_RXPORTRD_MAIN_IDLE;
		end
	end

	default: begin
		_rMainState = `S_RXPORTRD_MAIN_IDLE;
	end

	endcase
end


// Issue the read requests at the buffer level. Decrement the amount requested
// after every request. Continue until all words have been requested.
wire [9:0] wAddrLoInv = ~rAddr[11:2];
always @ (posedge CLK) begin
	rRxState <= #1 (RST ? `S_RXPORTRD_RX_IDLE : _rRxState);
	rSgRen <= #1 (RST ? 1'd0: _rSgRen);
	rWords <= #1 _rWords;
	rBufWords <= #1 _rBufWords;
	rBufWordsInit <= #1 _rBufWordsInit;
	rLargeBuf <= #1 _rLargeBuf;
	rAddr <= #1 _rAddr;
	rCarry <= #1 _rCarry;
	rValsProp <= #1 _rValsProp;
	rPageRem <= #1 _rPageRem;
	rPageSpill <= #1 _rPageSpill;
	rPageSpillInit <= #1 _rPageSpillInit;
	rCopyBufWords <= #1 _rCopyBufWords;
	rUseInit <= #1 _rUseInit;
	rPreLen <= #1 _rPreLen;
	rMaxPayloadTrain <= #1 _rMaxPayloadTrain;
	rMaxPayloadShift <= #1 _rMaxPayloadShift;
	rMaxPayload <= #1 _rMaxPayload;
	rPayloadSpill <= #1 _rPayloadSpill;
	rMaxLen <= #1 _rMaxLen;
	rLen <= #1 _rLen;
	rLenEQWordsHi <= #1 _rLenEQWordsHi;
	rLenEQWordsLo <= #1 _rLenEQWordsLo;
	rLenEQBufWordsHi <= #1 _rLenEQBufWordsHi;
	rLenEQBufWordsLo <= #1 _rLenEQBufWordsLo;
end

always @ (*) begin
	_rRxState = rRxState;
	_rCopyBufWords = rCopyBufWords;
	_rUseInit = rUseInit;
	_rSgRen = rSgRen;

	_rValsProp = ((rValsProp<<1) | rRxState[2]); // S_RXPORTRD_RX_ADJ_0
	_rLargeBuf = (SG_ELEM_LEN > rWords);
   	{_rCarry[0], _rAddr[15:0]} = (rRxState[1] ? SG_ELEM_ADDR[15:0] : (rAddr[15:0] + ({12{RX_REQ_ACK}} & {rLen,2'd0}))); 
	{_rCarry[1], _rAddr[31:16]} = (rRxState[1] ? SG_ELEM_ADDR[31:16] : (rAddr[31:16] + rCarry[0]));
	{_rCarry[2], _rAddr[47:32]} = (rRxState[1] ? SG_ELEM_ADDR[47:32] : (rAddr[47:32] + rCarry[1]));
				 _rAddr[63:48] = (rRxState[1] ? SG_ELEM_ADDR[63:48] : (rAddr[63:48] + rCarry[2]));
	_rWords = (rRxState[0] ? rReadWords : (rWords - ({10{RX_REQ_ACK}} & rLen)));
	_rBufWordsInit = (rLargeBuf ? rWords : SG_ELEM_LEN);
    _rBufWords = (rCopyBufWords ? rBufWordsInit : rBufWords) - ({10{RX_REQ_ACK}} & rLen);
	_rPageRem = (wAddrLoInv + 1'd1);	
	_rPageSpillInit = (rBufWordsInit > rPageRem);	
	_rPageSpill = (rBufWords > rPageRem);	
	_rPreLen = ((rPageSpillInit & rUseInit) | (rPageSpill & !rUseInit) ? rPageRem : rBufWords[10:0]);
	_rMaxPayloadTrain = (CONFIG_MAX_READ_REQUEST_SIZE > 3'd4 ? 3'd4 : CONFIG_MAX_READ_REQUEST_SIZE);
	_rMaxPayloadShift = (C_MAX_READ_REQ[2:0] < rMaxPayloadTrain ? C_MAX_READ_REQ[2:0] : rMaxPayloadTrain);
	_rMaxPayload = (6'd32<<rMaxPayloadShift);
	_rPayloadSpill = (rPreLen > rMaxPayload);
	_rMaxLen = ((rMaxLen & !rValsProp[2]) | RX_REQ_ACK);
	_rLen = (rPayloadSpill | rMaxLen ? rMaxPayload : rPreLen[9:0]);
	_rLenEQWordsHi = (16'd0 == rWords[31:16]);
	_rLenEQWordsLo = ({6'd0, rLen} == rWords[15:0]);
	_rLenEQBufWordsHi = (16'd0 == rBufWords[31:16]);
	_rLenEQBufWordsLo = ({6'd0, rLen} == rBufWords[15:0]);

	case (rRxState)

	`S_RXPORTRD_RX_IDLE: begin // Wait for a new read transaction
		if (rMainState[2]) // S_RXPORTRD_MAIN_READ
			_rRxState = `S_RXPORTRD_RX_BUF;
	end

	`S_RXPORTRD_RX_BUF: begin // Wait for buffer length and address
		if (SG_ELEM_RDY) begin
			_rSgRen = 1;
			_rRxState = `S_RXPORTRD_RX_ADJ_0;
		end
		else if (rErr) begin
			_rRxState = `S_RXPORTRD_RX_WAIT_0;
		end
	end

	`S_RXPORTRD_RX_ADJ_0: begin // Fix for large buffer
		_rSgRen = 0;
		_rCopyBufWords = rSgRen;
		_rRxState = `S_RXPORTRD_RX_ADJ_1;
	end

	// (bufwords and pagerem valid here) 
	`S_RXPORTRD_RX_ADJ_1: begin // Wait for the value to propagate 
		// Check for page boundary crossing
		// Fix for page boundary crossing
		// Check for max read payload
		// Fix for max read payload
		_rCopyBufWords = 0;
		_rUseInit = rCopyBufWords;
		if (rValsProp[3])
			_rRxState = `S_RXPORTRD_RX_ISSUE;
	end

	`S_RXPORTRD_RX_ISSUE: begin // Wait for the request to be accepted
		if (RX_REQ_ACK) begin
			if (rErr | (rLenEQWordsHi & rLenEQWordsLo))
				_rRxState = `S_RXPORTRD_RX_WAIT_0;
			else if (rLenEQBufWordsHi & rLenEQBufWordsLo)
				_rRxState = `S_RXPORTRD_RX_BUF;
			else
				_rRxState = `S_RXPORTRD_RX_ADJ_0;
		end
	end

	`S_RXPORTRD_RX_WAIT_0: begin // Wait for rAckCount to update
		_rRxState = `S_RXPORTRD_RX_WAIT_1;
	end

	`S_RXPORTRD_RX_WAIT_1: begin // Wait for requested data to arrive
		if (rAckCountEQ0)
			_rRxState = `S_RXPORTRD_RX_DONE;
	end

	`S_RXPORTRD_RX_DONE: begin // Signal done
		if (rMainState[3]) // S_RXPORTRD_MAIN_FLUSH
			_rRxState = `S_RXPORTRD_RX_IDLE;
	end
	
	default: begin
		_rRxState = `S_RXPORTRD_RX_IDLE;
	end
	
	endcase
end


// Count the data.
always @ (posedge CLK) begin
	rRecvdWords <= #1 _rRecvdWords;
	rReqdWords <= #1 _rReqdWords;
	rPartWords <= #1 _rPartWords;
	rAckCount <= #1 _rAckCount;
	rAckCountEQ0 <= #1 _rAckCountEQ0;
	rPartWordsRecvd <= #1 _rPartWordsRecvd;
	rRequestingWords <= #1 _rRequestingWords;
	rAvailWords <= #1 _rAvailWords;
	rCarryInv <= #1 _rCarryInv;
	rSpaceAvail <= #1 _rSpaceAvail;
	rLastDoneRead <= #1 (RST ? 1'd1 : _rLastDoneRead);	
end

always @ (*) begin
	// Count words as they arrive (words from the rx_engine directly).
	if (rMainState[0]) // S_RXPORTRD_MAIN_IDLE
		_rRecvdWords = #1 0;
	else
		_rRecvdWords = #1 rRecvdWords + rRxDataEn;

	// Count words as they are requested.
	if (rMainState[0]) // S_RXPORTRD_MAIN_IDLE
		_rReqdWords = #1 0;
	else
      _rReqdWords = #1 rReqdWords + ({10{RX_REQ_ACK}} & rLen);

	// Track outstanding requests
	if (rMainState[0]) // S_RXPORTRD_MAIN_IDLE
		_rAckCount = 0;
	else
		_rAckCount = rAckCount + RX_REQ_ACK - RX_DONE;
	_rAckCountEQ0 = (rAckCount == 11'd0);

	// Track when the user reads the actual transfer amount.
	_rLastDoneRead = (rTxnDone ? 1'd0 : (rLastDoneRead | rTxnDoneAck));

	// Track the amount of words that are expected to arrive.
	_rPartWords = #1 (rTxnLenValid ? rTxnData : rPartWords);

	// Compare counts.
	_rPartWordsRecvd = (rRecvdWords >= rPartWords);
	_rRequestingWords = rReqdWords + rLen;
	{_rCarryInv, _rAvailWords[15:0]} = {1'd1, rRequestingWords[15:0]} - CHNL_RX_CONSUMED[15:0];
	_rAvailWords[31:16] = rRequestingWords[31:16] - CHNL_RX_CONSUMED[31:16] - !rCarryInv;
	_rSpaceAvail = (rAvailWords <= C_FIFO_WORDS);
end


// Facilitate sending a TXN_DONE when we receive a TXN_ACK after the transaction
// has begun sending. This will happen when the workstation detects that it has 
// sent/used all its currently mapped scatter gather elements, but it's not enough 
// to complete the transaction. The TXN_DONE will let the workstation know it can
// release the current scatter gather mappings and allocate new ones.
always @ (posedge CLK) begin
	rPartialDone <= #1 _rPartialDone;
	rReqPartialDone <= #1 (RST ? 1'd0 : _rReqPartialDone);
end

always @ (*) begin
	// Signal TXN_DONE after we've recieved the (seemingly superfluous) TXN_ACK 
	// and received the corresponding amount of words.
	_rPartialDone = (rReqPartialDone & rPartWordsRecvd);
	
	// Keep track of (seemingly superfluous) TXN_ACK requests.
	if ((rReqPartialDone & rPartWordsRecvd) | rMainState[0]) // S_RXPORTRD_MAIN_IDLE
		_rReqPartialDone = 0;
	else
		_rReqPartialDone = (rReqPartialDone | rTxnLenValid);
end


// Handle errors in the main data or scatter gather data.
always @ (posedge CLK) begin
	rErr <= #1 (RST ? 1'd0 : _rErr);
end

always @ (*) begin
	// Keep track of errors if we encounter them.
	if (rMainState[0]) // S_RXPORTRD_MAIN_IDLE
		_rErr = 0;
	else
		_rErr = (rErr | (RX_DONE & RX_ERR) | (SG_DONE & SG_ERR));
end



/*
wire [35:0] wControl0;
chipscope_icon_1 cs_icon(
	.CONTROL0(wControl0)
);

chipscope_ila_t8_512 a0(
	.CLK(CLK), 
	.CONTROL(wControl0), 
	.TRIG0({TXN_LEN_VALID | TXN_DONE_ACK | TXN_DONE | TXN_ERR, 1'd0, rMainState}),
	.DATA({176'd0,
			64'd0, // 64
			rAddr, // 64
			SG_ELEM_RDY, // 1
			1'd0, // 1
			1'd0, // 1
			1'd0, // 1
			rSgRen, // 1
			1'd0, // 1
			rLastDoneRead, // 1
			rLen, // 10
			rWords, // 32
			rAckCount, // 11
			rPartWords, // 32
			rPartWordsRecvd, // 1
			rReqPartialDone, // 1
			rPartialDone, // 1
			rTxnDone, // 1
			rRxState, // 8
			rRecvdWords, // 32
			rReadWords, // 32
			TXN_LEN_VALID, // 1
			TXN_DONE_ACK, // 1
			rDoneLen, // 32
			rMainState}) // 6
);
*/

endmodule
