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
// Filename:			tx_port_monitor_32.v
// Version:				1.00.a
// Verilog Standard:	Verilog-2001
// Description:			Detects transaction open/close events from the stream
// of data from the tx_port_channel_gate. Filters out events and passes data
// onto the tx_port_buffer. 
// Author:				Matt Jacobsen
// History:				@mattj: Version 2.0
//-----------------------------------------------------------------------------
`define S_TXPORTMON32_NEXT	6'b00_0001
`define S_TXPORTMON32_EVT_2	6'b00_0010
`define S_TXPORTMON32_TXN	6'b00_0100
`define S_TXPORTMON32_READ	6'b00_1000
`define S_TXPORTMON32_END_0	6'b01_0000
`define S_TXPORTMON32_END_1	6'b10_0000

`timescale 1ns/1ns
module tx_port_monitor_32 #(
	parameter C_DATA_WIDTH = 9'd32,
	parameter C_FIFO_DEPTH = 512,
	// Local parameters
	parameter C_FIFO_DEPTH_THRESH = (C_FIFO_DEPTH - 4),
	parameter C_FIFO_DEPTH_WIDTH = clog2((2**clog2(C_FIFO_DEPTH))+1),
	parameter C_VALID_HIST = 1
)
(
	input RST,
	input CLK,

	input [C_DATA_WIDTH:0] EVT_DATA,			// Event data from tx_port_channel_gate
	input EVT_DATA_EMPTY,						// Event data FIFO is empty
	output EVT_DATA_RD_EN,						// Event data FIFO read enable

	output [C_DATA_WIDTH-1:0] WR_DATA,			// Output data
	output WR_EN,								// Write enable for output data
	input [C_FIFO_DEPTH_WIDTH-1:0] WR_COUNT,	// Output FIFO count

	output TXN,									// Transaction parameters are valid
	input ACK,									// Transaction parameter read, continue
	output LAST,								// Channel last write
	output [31:0] LEN,							// Channel write length (in 32 bit words)
	output [30:0] OFF,							// Channel write offset
	output [31:0] WORDS_RECVD,					// Count of data words received in transaction
	output DONE,								// Transaction is closed

	input TX_ERR								// Transaction encountered an error
);

`include "functions.vh"

(* syn_encoding = "user" *)
(* fsm_encoding = "user" *)
reg 	[5:0]				rState=`S_TXPORTMON32_NEXT, _rState=`S_TXPORTMON32_NEXT;
reg 						rRead=0, _rRead=0;
reg 	[C_VALID_HIST-1:0]	rDataValid={C_VALID_HIST{1'd0}}, _rDataValid={C_VALID_HIST{1'd0}};
reg 						rEvent=0, _rEvent=0;
reg 	[31:0]				rReadOffLast=0, _rReadOffLast=0;
reg 	[31:0]				rReadLen=0, _rReadLen=0;
reg 						rReadCount=0, _rReadCount=0;
reg 	[31:0]				rWordsRecvd=0, _rWordsRecvd=0;
reg 	[31:0]				rWordsRecvdAdv=0, _rWordsRecvdAdv=0;
reg 						rAlmostAllRecvd=0, _rAlmostAllRecvd=0;
reg 						rAlmostFull=0, _rAlmostFull=0;
reg 						rLenEQ0Hi=0, _rLenEQ0Hi=0;
reg 						rLenEQ0Lo=0, _rLenEQ0Lo=0;
reg 						rLenLE1Lo=0, _rLenLE1Lo=0;
reg							rTxErr=0, _rTxErr=0;

wire wEventData = (rDataValid[0] & EVT_DATA[C_DATA_WIDTH]);
wire wPayloadData = (rDataValid[0] & !EVT_DATA[C_DATA_WIDTH] & rState[3]); // S_TXPORTMON32_READ
wire wAllWordsRecvd = ((rAlmostAllRecvd | (rLenEQ0Hi & rLenLE1Lo)) & wPayloadData);

assign EVT_DATA_RD_EN = rRead;

assign WR_DATA = EVT_DATA[C_DATA_WIDTH-1:0];
assign WR_EN = wPayloadData;

assign TXN = rState[2]; // S_TXPORTMON32_TXN
assign LAST = rReadOffLast[0];
assign OFF = rReadOffLast[31:1];
assign LEN = rReadLen;
assign WORDS_RECVD = rWordsRecvd;
assign DONE = !rState[3]; // !S_TXPORTMON32_READ




// Buffer the input signals that come from outside the tx_port.
always @ (posedge CLK) begin
	rTxErr <= #1 (RST ? 1'd0 : _rTxErr);
end

always @ (*) begin
	_rTxErr = TX_ERR;
end


// Transaction monitoring FSM.
always @ (posedge CLK) begin
	rState <= #1 (RST ? `S_TXPORTMON32_NEXT : _rState);
end

always @ (*) begin
	_rState = rState;
	case (rState)

	`S_TXPORTMON32_NEXT: begin // Read, wait for start of transaction event
		if (rEvent)
			_rState = `S_TXPORTMON32_TXN;
	end

	`S_TXPORTMON32_EVT_2: begin // Read, wait for start of transaction event
		if (rEvent)
			_rState = `S_TXPORTMON32_TXN;
	end

	`S_TXPORTMON32_TXN: begin // Don't read, wait until transaction has been acknowledged
		if (ACK)
			_rState = ((rLenEQ0Hi && rLenEQ0Lo) ? `S_TXPORTMON32_END_0 : `S_TXPORTMON32_READ);
	end

	`S_TXPORTMON32_READ: begin // Continue reading, wait for end of transaction event or all expected data
		if (rEvent)
			_rState = `S_TXPORTMON32_END_1;
		else if (wAllWordsRecvd | rTxErr)
			_rState = `S_TXPORTMON32_END_0;
	end
	
	`S_TXPORTMON32_END_0: begin // Continue reading, wait for first end of transaction event
		if (rEvent)
			_rState = `S_TXPORTMON32_END_1;
	end

	`S_TXPORTMON32_END_1: begin // Continue reading, wait for second end of transaction event
		if (rEvent)
			_rState = `S_TXPORTMON32_NEXT;
	end

	default: begin
		_rState = `S_TXPORTMON32_NEXT;
	end

	endcase	
end


// Manage reading from the FIFO and tracking amounts read.
always @ (posedge CLK) begin
	rRead <= #1 (RST ? 1'd0 : _rRead);
	rDataValid <= #1 (RST ? {C_VALID_HIST{1'd0}} : _rDataValid);
	rEvent <= #1 (RST ? 1'd0 : _rEvent);
	rReadOffLast <= #1 _rReadOffLast;
	rReadLen <= #1 _rReadLen;
	rReadCount <= #1 (RST ? 1'd0 : _rReadCount);
	rWordsRecvd <= #1 _rWordsRecvd;
	rWordsRecvdAdv <= #1 _rWordsRecvdAdv;
	rAlmostAllRecvd <= #1 _rAlmostAllRecvd;
	rAlmostFull <= #1 _rAlmostFull;
	rLenEQ0Hi <= #1 _rLenEQ0Hi;
	rLenEQ0Lo <= #1 _rLenEQ0Lo;
	rLenLE1Lo <= #1 _rLenLE1Lo;
end

always @ (*) begin
	// Don't get to the full point in the output FIFO
	_rAlmostFull = (WR_COUNT >= C_FIFO_DEPTH_THRESH);

	// Track read history so we know when data is valid
	_rDataValid = ((rDataValid<<1) | (rRead & !EVT_DATA_EMPTY));

	// Read until we get a (valid) event
	_rRead = (!rState[2] & !(rState[1] & (rEvent | wEventData | ~EVT_DATA_EMPTY)) & !wEventData & !rAlmostFull); // !S_TXPORTMON32_TXN

	// Track detected events
	_rEvent = wEventData;

	// Save event data when valid
	if (wEventData) begin
		_rReadOffLast = (rReadCount ? EVT_DATA[C_DATA_WIDTH-1:0] : rReadOffLast);
		_rReadLen = (!rReadCount ? EVT_DATA[C_DATA_WIDTH-1:0] : rReadLen);
		_rReadCount = rReadCount + 1'd1;
	end
	else begin
		_rReadOffLast = rReadOffLast;
		_rReadLen = rReadLen;
		_rReadCount = rReadCount;
	end
	
	// If LEN == 0, we don't want to send any data to the output
	_rLenEQ0Hi = (LEN[31:16] == 16'd0);
	_rLenEQ0Lo = (LEN[15:0] == 16'd0);

	// If LEN <= 1, we want to trigger the almost all received flag
	_rLenLE1Lo = (LEN[15:0] <= 16'd1);
	
	// Count received non-event data
	_rWordsRecvd = (ACK ? 0 : rWordsRecvd + wPayloadData);
	_rWordsRecvdAdv = (ACK ? 2*(C_DATA_WIDTH/32) : rWordsRecvdAdv + wPayloadData);
	_rAlmostAllRecvd = ((rWordsRecvdAdv >= LEN) && wPayloadData);
end

/*
wire [35:0] wControl0;
chipscope_icon_1 cs_icon(
	.CONTROL0(wControl0)
);

chipscope_ila_t8_512 a0(
	.CLK(CLK), 
	.CONTROL(wControl0), 
	.TRIG0({TXN, wPayloadData, wEventData, rState}),
	.DATA({297'd0,
			WR_COUNT, // 10
			wPayloadData, // 1
			EVT_DATA_RD_EN, // 1
			RST, // 1
			rTxErr, // 1
			wEventData, // 1
			rReadData, // 64
			OFF, // 31
			LEN, // 32
			LAST, // 1
			TXN, // 1
			EVT_DATA_EMPTY, // 1
			EVT_DATA, // 65
			rState}) // 5
);
*/

endmodule
