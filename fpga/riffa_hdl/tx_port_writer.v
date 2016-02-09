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
// Filename:			tx_port_writer.v
// Version:				1.00.a
// Verilog Standard:	Verilog-2001
// Description:			Handles receiving new transaction events and data, and
// making requests to tx engine. 
//						for the RIFFA channel.
// Author:				Matt Jacobsen
// History:				@mattj: Version 2.0
//-----------------------------------------------------------------------------
`define S_TXPORTWR_MAIN_IDLE		8'b0000_0001
`define S_TXPORTWR_MAIN_CHECK		8'b0000_0010
`define S_TXPORTWR_MAIN_SIG_NEW		8'b0000_0100
`define S_TXPORTWR_MAIN_NEW_ACK		8'b0000_1000
`define S_TXPORTWR_MAIN_WRITE		8'b0001_0000
`define S_TXPORTWR_MAIN_DONE		8'b0010_0000
`define S_TXPORTWR_MAIN_SIG_DONE	8'b0100_0000
`define S_TXPORTWR_MAIN_RESET		8'b1000_0000

`define S_TXPORTWR_TX_IDLE			8'b0000_0001
`define S_TXPORTWR_TX_BUF			8'b0000_0010
`define S_TXPORTWR_TX_ADJ_0			8'b0000_0100
`define S_TXPORTWR_TX_ADJ_1			8'b0000_1000
`define S_TXPORTWR_TX_ADJ_2			8'b0001_0000
`define S_TXPORTWR_TX_CHECK_DATA	8'b0010_0000
`define S_TXPORTWR_TX_WRITE			8'b0100_0000
`define S_TXPORTWR_TX_WRITE_REM		8'b1000_0000

`timescale 1ns/1ns
module tx_port_writer (
	input CLK,
	input RST,
	input [2:0] CONFIG_MAX_PAYLOAD_SIZE,	// Maximum write payload: 000=128B, 001=256B, 010=512B, 011=1024B

	output TXN,							// Write transaction notification
	input TXN_ACK,						// Write transaction acknowledged
	output [31:0] TXN_LEN,				// Write transaction length
	output [31:0] TXN_OFF_LAST,			// Write transaction offset/last
	output [31:0] TXN_DONE_LEN,			// Write transaction actual transfer length
	output TXN_DONE,					// Write transaction done
	output TXN_ERR,						// Write transaction encountered an error
	input TXN_DONE_ACK,					// Write transaction actual transfer length read

	input NEW_TXN,						// Transaction parameters are valid
	output NEW_TXN_ACK,					// Transaction parameter read, continue
	input NEW_TXN_LAST,					// Channel last write
	input [31:0] NEW_TXN_LEN,			// Channel write length (in 32 bit words)
	input [30:0] NEW_TXN_OFF,			// Channel write offset
	input [31:0] NEW_TXN_WORDS_RECVD,	// Count of data words received in transaction
	input NEW_TXN_DONE,					// Transaction is closed

	input [63:0] SG_ELEM_ADDR,			// Scatter gather element address
	input [31:0] SG_ELEM_LEN,			// Scatter gather element length (in words)
	input SG_ELEM_RDY,					// Scatter gather element ready
	input SG_ELEM_EMPTY,				// Scatter gather elements empty
	output SG_ELEM_REN,					// Scatter gather element read enable
	output SG_RST,						// Scatter gather data reset
	input SG_ERR,						// Scatter gather read encountered an error
	
	output TX_REQ,						// Outgoing write request
	input TX_REQ_ACK,					// Outgoing write request acknowledged
	output [63:0] TX_ADDR,				// Outgoing write high address
	output [9:0] TX_LEN,				// Outgoing write length (in 32 bit words)
	output TX_LAST,						// Outgoing write is last request for transaction
	input TX_SENT						// Outgoing write complete
);

`include "functions.vh"

(* syn_encoding = "user" *)
(* fsm_encoding = "user" *)
reg		[7:0]						rMainState=`S_TXPORTWR_MAIN_IDLE, _rMainState=`S_TXPORTWR_MAIN_IDLE;
reg		[31:0]						rOffLast=0, _rOffLast=0;
reg									rWordsEQ0=0, _rWordsEQ0=0;
reg									rStarted=0, _rStarted=0;
reg		[31:0]						rDoneLen=0, _rDoneLen=0;
reg									rSgErr=0, _rSgErr=0;
reg									rTxErrd=0, _rTxErrd=0;
reg									rTxnAck=0, _rTxnAck=0;

(* syn_encoding = "user" *)
(* fsm_encoding = "user" *)
reg		[7:0]						rTxState=`S_TXPORTWR_TX_IDLE, _rTxState=`S_TXPORTWR_TX_IDLE;
reg		[31:0]						rSentWords=0, _rSentWords=0;
reg		[31:0]						rWords=0, _rWords=0;
reg		[31:0]						rBufWords=0, _rBufWords=0;
reg		[31:0]						rBufWordsInit=0, _rBufWordsInit=0;
reg									rLargeBuf=0, _rLargeBuf=0;
reg		[63:0]						rAddr=64'd0, _rAddr=64'd0;
reg		[2:0]						rCarry=0, _rCarry=0;
reg									rValsPropagated=0, _rValsPropagated=0;
reg		[5:0]						rValsProp=0, _rValsProp=0;
reg									rCopyBufWords=0, _rCopyBufWords=0;
reg									rUseInit=0, _rUseInit=0;
reg		[10:0]						rPageRem=0, _rPageRem=0;
reg									rPageSpill=0, _rPageSpill=0;
reg									rPageSpillInit=0, _rPageSpillInit=0;
reg		[10:0]						rPreLen=0, _rPreLen=0;
reg		[2:0]						rMaxPayloadSize=0, _rMaxPayloadSize=0;
reg		[2:0]						rMaxPayloadShift=0, _rMaxPayloadShift=0;
reg		[9:0]						rMaxPayload=0, _rMaxPayload=0;
reg									rPayloadSpill=0, _rPayloadSpill=0;
reg									rMaxLen=1, _rMaxLen=1;
reg		[9:0]						rLen=0, _rLen=0;
reg		[31:0]						rSendingWords=0, _rSendingWords=0;
reg									rAvail=0, _rAvail=0;
reg		[1:0]						rTxnDone=0, _rTxnDone=0;
reg		[9:0]						rLastLen=0, _rLastLen=0;
reg									rLastLenEQ0=0, _rLastLenEQ0=0;
reg									rLenEQWords=0, _rLenEQWords=0;
reg									rLenEQBufWords=0, _rLenEQBufWords=0;

reg									rNotRequesting=1, _rNotRequesting=1;
reg		[63:0]						rReqAddr=64'd0, _rReqAddr=64'd0;
reg		[9:0]						rReqLen=0, _rReqLen=0;
reg									rReqLast=0, _rReqLast=0;
reg									rTxReqAck=0, _rTxReqAck=0;

reg									rDone=0, _rDone=0;
reg		[9:0]						rAckCount=0, _rAckCount=0;
reg									rTxSent=0, _rTxSent=0;
reg									rLastDoneRead=1, _rLastDoneRead=1;
reg									rTxnDoneAck=0, _rTxnDoneAck=0;

reg									rReqPartialDone=0, _rReqPartialDone=0;
reg									rPartialDone=0, _rPartialDone=0;


assign NEW_TXN_ACK = rMainState[1]; // S_TXPORTWR_MAIN_CHECK

assign TXN = rMainState[2]; // S_TXPORTWR_MAIN_SIG_NEW
assign TXN_DONE = (rMainState[6] | rPartialDone); // S_TXPORTWR_MAIN_SIG_DONE
assign TXN_LEN = rWords;
assign TXN_OFF_LAST = rOffLast;
assign TXN_DONE_LEN = rDoneLen;
assign TXN_ERR = rTxErrd;

assign SG_ELEM_REN = rTxState[2]; // S_TXPORTWR_TX_ADJ_0
assign SG_RST = rMainState[3]; // S_TXPORTWR_MAIN_NEW_ACK

assign TX_REQ = !rNotRequesting;
assign TX_ADDR = rReqAddr; 
assign TX_LEN = rReqLen;
assign TX_LAST = rReqLast;


// Buffer the input signals that come from outside the tx_port.
always @ (posedge CLK) begin
	rTxnAck <= #1 (RST ? 1'd0 : _rTxnAck);
	rTxnDoneAck <= #1 (RST ? 1'd0 : _rTxnDoneAck);
	rSgErr <= #1 (RST ? 1'd0 : _rSgErr);
	rTxReqAck <= #1 (RST ? 1'd0 : _rTxReqAck);
	rTxSent <= #1 (RST ? 1'd0 : _rTxSent);
end

always @ (*) begin
	_rTxnAck = TXN_ACK;
	_rTxnDoneAck = TXN_DONE_ACK;
	_rSgErr = SG_ERR;
	_rTxReqAck = TX_REQ_ACK;
	_rTxSent = TX_SENT;
end
	

// Wait for a NEW_TXN request. Then request transfers until all the data is sent
// or until the specified length is reached. Then signal TXN_DONE.
always @ (posedge CLK) begin
	rMainState <= #1 (RST ? `S_TXPORTWR_MAIN_IDLE : _rMainState);
	rOffLast <= #1 _rOffLast;
	rWordsEQ0 <= #1 _rWordsEQ0;
	rStarted <= #1 _rStarted;
	rDoneLen <= #1 (RST ? 0 : _rDoneLen);
	rTxErrd <= #1 (RST ? 1'd0 : _rTxErrd);
end

always @ (*) begin
	_rMainState = rMainState;
	_rOffLast = rOffLast;
	_rWordsEQ0 = rWordsEQ0;
	_rStarted = rStarted;
	_rDoneLen = rDoneLen;
	_rTxErrd = rTxErrd;
	case (rMainState)

	`S_TXPORTWR_MAIN_IDLE: begin // Wait for channel write request
		_rStarted = 0;
		_rWordsEQ0 = (NEW_TXN_LEN == 0);
		_rOffLast = {NEW_TXN_OFF, NEW_TXN_LAST};
		if (NEW_TXN)
			_rMainState = `S_TXPORTWR_MAIN_CHECK;
	end

	`S_TXPORTWR_MAIN_CHECK: begin // Continue with transaction?
		if (rOffLast[0] | !rWordsEQ0)
			_rMainState = `S_TXPORTWR_MAIN_SIG_NEW;
		else
			_rMainState = `S_TXPORTWR_MAIN_RESET;
	end

	`S_TXPORTWR_MAIN_SIG_NEW: begin // Signal new write
		_rMainState = `S_TXPORTWR_MAIN_NEW_ACK;
	end

	`S_TXPORTWR_MAIN_NEW_ACK: begin // Wait for acknowledgement
		if (rTxnAck) // ACK'd on PC read of TXN length
			_rMainState = (rWordsEQ0 ? `S_TXPORTWR_MAIN_SIG_DONE : `S_TXPORTWR_MAIN_WRITE);
	end

	`S_TXPORTWR_MAIN_WRITE: begin // Start writing and wait for all writes to complete
		_rStarted = (rStarted | rTxState[1]); // S_TXPORTWR_TX_BUF
		_rTxErrd = (rTxErrd | rSgErr);
		if (rTxState[0] & rStarted) // S_TXPORTWR_TX_IDLE
			_rMainState = `S_TXPORTWR_MAIN_DONE;
	end

	`S_TXPORTWR_MAIN_DONE: begin // Wait for the last transaction to complete
		if (rDone & rLastDoneRead) begin
			_rDoneLen = rSentWords;
			_rMainState = `S_TXPORTWR_MAIN_SIG_DONE;
		end
	end

	`S_TXPORTWR_MAIN_SIG_DONE: begin // Signal the done port
		_rTxErrd = 0;
		_rMainState = `S_TXPORTWR_MAIN_RESET;
	end

	`S_TXPORTWR_MAIN_RESET: begin // Wait for the channel tx to drop
		if (NEW_TXN_DONE)
			_rMainState = `S_TXPORTWR_MAIN_IDLE;
	end
	
	default: begin
		_rMainState = `S_TXPORTWR_MAIN_IDLE;
	end
	
	endcase
end


// Manage sending TX requests to the TX engine. Transfers will be limited
// by each scatter gather buffer's size, max payload size, and must not
// cross a (4KB) page boundary. The request is only made if there is sufficient
// data already written to the buffer.
wire [9:0] wLastLen = (NEW_TXN_WORDS_RECVD - rSentWords);
wire [9:0] wAddrLoInv = ~rAddr[11:2];
wire [10:0] wPageRem = (wAddrLoInv + 1'd1);	
always @ (posedge CLK) begin
	rTxState <= #1 (RST | rSgErr ? `S_TXPORTWR_TX_IDLE : _rTxState);
	rSentWords <= #1 (rMainState[0] ? 0 : _rSentWords);
	rWords <= #1 _rWords;
	rBufWords <= #1 _rBufWords;
	rBufWordsInit <= #1 _rBufWordsInit;
	rAddr <= #1 _rAddr;
	rCarry <= #1 _rCarry;
	rValsPropagated <= #1 _rValsPropagated;
	rValsProp <= #1 _rValsProp;
	rLargeBuf <= #1 _rLargeBuf;
	rPageRem <= #1 _rPageRem;
	rPageSpill <= #1 _rPageSpill;
	rPageSpillInit <= #1 _rPageSpillInit;
	rCopyBufWords <= #1 _rCopyBufWords;
	rUseInit <= #1 _rUseInit;
	rPreLen <= #1 _rPreLen;
	rMaxPayloadSize <= #1 _rMaxPayloadSize;
	rMaxPayloadShift <= #1 _rMaxPayloadShift;
	rMaxPayload <= #1 _rMaxPayload;
	rPayloadSpill <= #1 _rPayloadSpill;
	rMaxLen <= #1 (RST ? 1'd1 : _rMaxLen);
	rLen <= #1 _rLen;
	rSendingWords <= #1 _rSendingWords;
	rAvail <= #1 _rAvail;
	rTxnDone <= #1 _rTxnDone;
	rLastLen <= #1 _rLastLen;
	rLastLenEQ0 <= #1 _rLastLenEQ0;
	rLenEQWords <= #1 _rLenEQWords;
	rLenEQBufWords <= #1 _rLenEQBufWords;
end

always @ (*) begin
	_rTxState = rTxState;
	_rCopyBufWords = rCopyBufWords;
	_rUseInit = rUseInit;
	
	_rValsProp = ((rValsProp<<1) | rTxState[3]); // S_TXPORTWR_TX_ADJ_1
	_rValsPropagated = (rValsProp == 6'd0);
	_rLargeBuf = (SG_ELEM_LEN > rWords);
	{_rCarry[0], _rAddr[15:0]} = (rTxState[1] ? SG_ELEM_ADDR[15:0] : (rAddr[15:0] + ({12{rTxState[6]}} & {rLen, 2'd0}))); // S_TXPORTWR_TX_WRITE
	{_rCarry[1], _rAddr[31:16]} = (rTxState[1] ? SG_ELEM_ADDR[31:16] : (rAddr[31:16] + rCarry[0]));
	{_rCarry[2], _rAddr[47:32]} = (rTxState[1] ? SG_ELEM_ADDR[47:32] : (rAddr[47:32] + rCarry[1]));
				 _rAddr[63:48] = (rTxState[1] ? SG_ELEM_ADDR[63:48] : (rAddr[63:48] + rCarry[2]));
	_rSentWords = (rTxState[7] ? NEW_TXN_WORDS_RECVD : rSentWords) + ({10{rTxState[6]}} & rLen); // S_TXPORTWR_TX_WRITE
	_rWords = (NEW_TXN_ACK ? NEW_TXN_LEN : (rWords - ({10{rTxState[6]}} & rLen))); // S_TXPORTWR_TX_WRITE
	_rBufWordsInit = (rLargeBuf ? rWords : SG_ELEM_LEN); 
	_rBufWords = (rCopyBufWords ? rBufWordsInit : rBufWords) - ({10{rTxState[6]}} & rLen); // S_TXPORTWR_TX_WRITE
	_rPageRem = wPageRem;	
	_rPageSpillInit = (rBufWordsInit > wPageRem);	
	_rPageSpill = (rBufWords > wPageRem);	
	_rPreLen = ((rPageSpillInit & rUseInit) | (rPageSpill & !rUseInit) ? rPageRem : rBufWords[10:0]);
	_rMaxPayloadSize = CONFIG_MAX_PAYLOAD_SIZE;
	_rMaxPayloadShift = (rMaxPayloadSize > 3'd4 ? 3'd4 : rMaxPayloadSize);
	_rMaxPayload = (6'd32<<rMaxPayloadShift);
	_rPayloadSpill = (rPreLen > rMaxPayload);
	_rMaxLen = ((rMaxLen & !rValsProp[1]) | rTxState[6]); // S_TXPORTWR_TX_WRITE
	_rLen = (rPayloadSpill | rMaxLen ? rMaxPayload : rPreLen[9:0]);
	_rSendingWords = rSentWords + rLen;
	_rAvail = (NEW_TXN_WORDS_RECVD >= rSendingWords);
	_rTxnDone = ((rTxnDone<<1) | NEW_TXN_DONE);
	_rLastLen = wLastLen;
	_rLastLenEQ0 = (rLastLen == 10'd0);
	_rLenEQWords = (rLen == rWords);
	_rLenEQBufWords = (rLen == rBufWords);
	
	case (rTxState)

	`S_TXPORTWR_TX_IDLE: begin // Wait for channel write request
		if (rMainState[4] & !rStarted) // S_TXPORTWR_MAIN_WRITE
			_rTxState = `S_TXPORTWR_TX_BUF;
	end

	`S_TXPORTWR_TX_BUF: begin // Wait for buffer length and address
		if (SG_ELEM_RDY)
			_rTxState = `S_TXPORTWR_TX_ADJ_0;
	end

	`S_TXPORTWR_TX_ADJ_0: begin // Fix for large buffer
		_rCopyBufWords = 1;
		_rTxState = `S_TXPORTWR_TX_ADJ_1;
	end

	`S_TXPORTWR_TX_ADJ_1: begin // Check for page boundary crossing
		_rCopyBufWords = 0;
		_rUseInit = rCopyBufWords;
		_rTxState = `S_TXPORTWR_TX_ADJ_2;
	end

	`S_TXPORTWR_TX_ADJ_2: begin // Wait for values to propagate
		// Fix for page boundary crossing
		// Check for max payload
		// Fix for max payload
		_rUseInit = 0;
		if (rValsProp[2])
			_rTxState = `S_TXPORTWR_TX_CHECK_DATA;
	end

	`S_TXPORTWR_TX_CHECK_DATA: begin // Wait for available data
		if (rNotRequesting) begin
			if (rAvail)
				_rTxState = `S_TXPORTWR_TX_WRITE;
			else if (rValsPropagated & rTxnDone[1])
				_rTxState = (rLastLenEQ0 ? `S_TXPORTWR_TX_IDLE : `S_TXPORTWR_TX_WRITE_REM);
		end
	end

	`S_TXPORTWR_TX_WRITE: begin // Send len and repeat or finish?
		if (rLenEQWords)
			_rTxState = `S_TXPORTWR_TX_IDLE;
		else if (rLenEQBufWords)
			_rTxState = `S_TXPORTWR_TX_BUF;
		else
			_rTxState = `S_TXPORTWR_TX_ADJ_1;
	end

	`S_TXPORTWR_TX_WRITE_REM: begin // Send remaining data and finish
		_rTxState = `S_TXPORTWR_TX_IDLE;
	end
	
	default: begin
		_rTxState = `S_TXPORTWR_TX_IDLE;
	end
	
	endcase
end

// Request TX transfers separately so that the TX FSM can continue calculating
// the next set of request parameters without having to wait for the TX_REQ_ACK.
always @ (posedge CLK) begin
	rAckCount <= #1 (RST ? 10'd0 : _rAckCount);
	rNotRequesting <= #1 (RST ? 1'd1 : _rNotRequesting);
	rReqAddr <= #1 _rReqAddr;
	rReqLen <= #1 _rReqLen;
	rReqLast <= #1 _rReqLast;
	rDone <= #1 _rDone;
	rLastDoneRead <= #1 (RST ? 1'd1 : _rLastDoneRead);
end

always @ (*) begin
	// Start signaling when the TX FSM is ready.
	if (rTxState[6] | rTxState[7]) // S_TXPORTWR_TX_WRITE
		_rNotRequesting = 0;
	else
		_rNotRequesting = (rNotRequesting | rTxReqAck);

	// Pass off the rAddr & rLen when ready and wait for TX_REQ_ACK.
	if (rTxState[6]) begin // S_TXPORTWR_TX_WRITE
		_rReqAddr = rAddr;
		_rReqLen = rLen;
		_rReqLast = rLenEQWords;
	end
	else if (rTxState[7]) begin // S_TXPORTWR_TX_WRITE_REM
		_rReqAddr = rAddr;
		_rReqLen = rLastLen;
		_rReqLast = 1;
	end
	else begin
		_rReqAddr = rReqAddr;
		_rReqLen = rReqLen;
		_rReqLast = rReqLast;
	end

	// Track TX_REQ_ACK and TX_SENT to determine when the transaction is over.
	_rDone = (rAckCount == 10'd0);
	if (rMainState[0]) // S_TXPORTWR_MAIN_IDLE
		_rAckCount = 0;
	else
		_rAckCount = rAckCount + rTxState[6] + rTxState[7] - rTxSent; // S_TXPORTWR_TX_WRITE, S_TXPORTWR_TX_WRITE_REM
		
	// Track when the user reads the actual transfer amount.
	_rLastDoneRead = (rMainState[6] ? 1'd0 : (rLastDoneRead | rTxnDoneAck)); // S_TXPORTWR_MAIN_SIG_DONE
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
	// Signal TXN_DONE after we've recieved the (seemingly superfluous) TXN_ACK, 
	// we have no outstanding transfer requests, we're not currently requesting a
	// transfer, and there are no more scatter gather elements.
	_rPartialDone = (rReqPartialDone & rDone & rNotRequesting & SG_ELEM_EMPTY & rTxState[1]); // S_TXPORTWR_TX_BUF
	
	// Keep track of (seemingly superfluous) TXN_ACK requests.
	if ((rReqPartialDone & rDone & rNotRequesting & SG_ELEM_EMPTY & rTxState[1]) | rMainState[0]) // S_TXPORTWR_MAIN_IDLE
		_rReqPartialDone = 0;
	else
		_rReqPartialDone = (rReqPartialDone | (rTxnAck & !rMainState[3])); // !S_TXPORTWR_MAIN_NEW_ACK
end



/*
wire [35:0] wControl0;
chipscope_icon_1 cs_icon(
	.CONTROL0(wControl0)
);

chipscope_ila_t8_512 a0(
	.CLK(CLK), 
	.CONTROL(wControl0), 
	.TRIG0({rTxState[6] | rTxState[7] | rTxSent, rAckCount[6:0]}),
	.DATA({280'd0,
			NEW_TXN_WORDS_RECVD, // 32
			rSendingWords, // 32
			rAvail, // 1
			rNotRequesting, // 1
			NEW_TXN_LAST, // 1
			NEW_TXN_LEN, // 32
			NEW_TXN_OFF, // 31
			NEW_TXN, // 1
			rAckCount, // 10
			rLastDoneRead, // 1
			rWords, // 32
			rBufWords, // 32
			rLen, // 10
			rTxState, // 8
			rMainState}) // 8
);
*/
endmodule
