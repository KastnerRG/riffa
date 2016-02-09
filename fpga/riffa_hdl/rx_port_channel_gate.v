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
// Filename:			rx_port_channel_gate.v
// Version:				1.00.a
// Verilog Standard:	Verilog-2001
// Description:			Provides cross domain synchronization for the CHNL_RX* 
// signals between the CHNL_CLK and CLK domains. 
// Author:				Matt Jacobsen
// History:				@mattj: Version 2.0
//-----------------------------------------------------------------------------

`timescale 1ns/1ns
module rx_port_channel_gate #(
	parameter C_DATA_WIDTH = 9'd64
)
(
	input RST,
	input CLK,

	input RX,								// Channel read signal (CLK)
	output RX_RECVD,						// Channel read received signal (CLK)
	output RX_ACK_RECVD,					// Channel read acknowledgment received signal (CLK)
	input RX_LAST,							// Channel last read (CLK)
	input [31:0] RX_LEN,					// Channel read length (CLK)
	input [30:0] RX_OFF,					// Channel read offset (CLK)
	output [31:0] RX_CONSUMED,				// Channel words consumed (CLK)
	
	input [C_DATA_WIDTH-1:0] RD_DATA,		// FIFO read data (CHNL_CLK)
	input RD_EMPTY,							// FIFO is empty (CHNL_CLK)
	output RD_EN,							// FIFO read enable (CHNL_CLK)

	input CHNL_CLK,							// Channel read clock
	output CHNL_RX,							// Channel read receive signal (CHNL_CLK)
	input CHNL_RX_ACK,						// Channle read received signal (CHNL_CLK)
	output CHNL_RX_LAST,					// Channel last read (CHNL_CLK)
	output [31:0] CHNL_RX_LEN,				// Channel read length (CHNL_CLK)
	output [30:0] CHNL_RX_OFF,				// Channel read offset (CHNL_CLK)
	output [C_DATA_WIDTH-1:0] CHNL_RX_DATA,	// Channel read data (CHNL_CLK)
	output CHNL_RX_DATA_VALID,				// Channel read data valid (CHNL_CLK)
	input CHNL_RX_DATA_REN					// Channel read data has been recieved (CHNL_CLK)
);

reg								rAckd=0, _rAckd=0;
reg								rChnlRxAck=0, _rChnlRxAck=0;

reg		[31:0]					rConsumed=0, _rConsumed=0;
reg		[31:0]					rConsumedStable=0, _rConsumedStable=0;
reg		[31:0]					rConsumedSample=0, _rConsumedSample=0;
reg								rCountRead=0, _rCountRead=0;
wire							wCountRead;
wire							wCountStable;
wire							wDataRead = (CHNL_RX_DATA_REN & CHNL_RX_DATA_VALID);


assign RX_CONSUMED = rConsumedSample;

assign RD_EN = CHNL_RX_DATA_REN;

assign CHNL_RX_LAST = RX_LAST;
assign CHNL_RX_LEN = RX_LEN;
assign CHNL_RX_OFF = RX_OFF;
assign CHNL_RX_DATA = RD_DATA;
assign CHNL_RX_DATA_VALID = !RD_EMPTY;


// Buffer the input signals that come from outside the rx_port.
always @ (posedge CHNL_CLK) begin
	rChnlRxAck <= #1 (RST ? 1'd0 : _rChnlRxAck);
end

always @ (*) begin
	_rChnlRxAck = CHNL_RX_ACK;
end


// Signal receive into the channel domain.
cross_domain_signal rxSig (
	.CLK_A(CLK), 
	.CLK_A_SEND(RX), 
	.CLK_A_RECV(RX_RECVD), 
	.CLK_B(CHNL_CLK), 
	.CLK_B_RECV(CHNL_RX), 
	.CLK_B_SEND(CHNL_RX)
);

// Signal acknowledgment of receive into the CLK domain.
syncff rxAckSig (.CLK(CLK), .IN_ASYNC(rAckd), .OUT_SYNC(RX_ACK_RECVD));


// Capture CHNL_RX_ACK and reset only after the CHNL_RX drops.
always @ (posedge CHNL_CLK) begin
	rAckd <= #1 (RST ? 1'd0 : _rAckd);
end

always @ (*) begin
	_rAckd = (CHNL_RX & (rAckd | rChnlRxAck));
end


// Count the words consumed by the channel and pass it into the CLK domain.
always @ (posedge CHNL_CLK) begin
	rConsumed <= #1 _rConsumed;
	rConsumedStable <= #1 _rConsumedStable;
	rCountRead <= #1 (RST ? 1'd0 : _rCountRead);
end

always @ (*) begin
	_rConsumed = (!CHNL_RX ? 0 : rConsumed + (wDataRead*(C_DATA_WIDTH/32)));
	_rConsumedStable = (wCountRead | rCountRead ? rConsumedStable : rConsumed);
	_rCountRead = !wCountRead;
end

always @ (posedge CLK) begin
	rConsumedSample <= #1 _rConsumedSample;
end

always @ (*) begin
	_rConsumedSample = (wCountStable ? rConsumedStable : rConsumedSample);
end

// Determine when it's safe to update the count in the CLK domain.
cross_domain_signal countSync (
	.CLK_A(CHNL_CLK), 
	.CLK_A_SEND(rCountRead), 
	.CLK_A_RECV(wCountRead), 
	.CLK_B(CLK), 
	.CLK_B_RECV(wCountStable), 
	.CLK_B_SEND(wCountStable)
);


endmodule
