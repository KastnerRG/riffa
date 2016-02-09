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
// Filename:			tx_port_buffer_128.v
// Version:				1.00.a
// Verilog Standard:	Verilog-2001
// Description:			Wraps a FIFO for saving channel data and provides a 
// registered read output. Retains unread words from reads that are a length 
// which is not a multiple of the data bus width (C_FIFO_DATA_WIDTH). Data is
// available 5 cycles after RD_EN is asserted (not 1, like a traditional FIFO).
// Author:				Matt Jacobsen
// History:				@mattj: Version 2.0
//-----------------------------------------------------------------------------

`timescale 1ns/1ns
module tx_port_buffer_128 #(
	parameter C_FIFO_DATA_WIDTH = 9'd128,
	parameter C_FIFO_DEPTH = 512,
	// Local parameters
	parameter C_FIFO_DEPTH_WIDTH = clog2((2**clog2(C_FIFO_DEPTH))+1),
	parameter C_RD_EN_HIST = 2,
	parameter C_FIFO_RD_EN_HIST = 2,
	parameter C_CONSUME_HIST = 3,
	parameter C_COUNT_HIST = 3,
	parameter C_LEN_LAST_HIST = 1
)
(
	input RST,
	input CLK,

	input LEN_VALID,							// Transfer length is valid
	input [1:0] LEN_LSB,						// LSBs of transfer length
	input LEN_LAST,								// Last transfer in transaction

	input [C_FIFO_DATA_WIDTH-1:0] WR_DATA,		// Input data
	input WR_EN,								// Input data write enable
	output [C_FIFO_DEPTH_WIDTH-1:0] WR_COUNT,	// Input data write count

	output [C_FIFO_DATA_WIDTH-1:0] RD_DATA,		// Output data
	input RD_EN									// Output data read enable
);

`include "functions.vh"

reg 	[1:0]						rRdPtr=0, _rRdPtr=0;
reg 	[1:0]						rWrPtr=0, _rWrPtr=0;
reg 	[3:0]						rLenLSB0=0, _rLenLSB0=0;
reg 	[3:0]						rLenLSB1=0, _rLenLSB1=0;
reg 	[3:0]						rLenLast=0, _rLenLast=0;
reg									rLenValid=0, _rLenValid=0;

reg									rRen=0, _rRen=0;	
reg 	[2:0]						rCount=0, _rCount=0;
reg 	[(C_COUNT_HIST*3)-1:0]		rCountHist={C_COUNT_HIST{3'd0}}, _rCountHist={C_COUNT_HIST{3'd0}};
reg 	[C_LEN_LAST_HIST-1:0]		rLenLastHist={C_LEN_LAST_HIST{1'd0}}, _rLenLastHist={C_LEN_LAST_HIST{1'd0}};
reg 	[C_RD_EN_HIST-1:0]			rRdEnHist={C_RD_EN_HIST{1'd0}}, _rRdEnHist={C_RD_EN_HIST{1'd0}};
reg 								rFifoRdEn=0, _rFifoRdEn=0;
reg 	[C_FIFO_RD_EN_HIST-1:0]		rFifoRdEnHist={C_FIFO_RD_EN_HIST{1'd0}}, _rFifoRdEnHist={C_FIFO_RD_EN_HIST{1'd0}};
reg 	[(C_CONSUME_HIST*3)-1:0]	rConsumedHist={C_CONSUME_HIST{3'd0}}, _rConsumedHist={C_CONSUME_HIST{3'd0}};
reg		[C_FIFO_DATA_WIDTH-1:0]		rFifoData={C_FIFO_DATA_WIDTH{1'd0}}, _rFifoData={C_FIFO_DATA_WIDTH{1'd0}};
reg		[223:0]						rData=224'd0, _rData=224'd0;

wire	[C_FIFO_DATA_WIDTH-1:0]		wFifoData;

assign RD_DATA = rData[0 +:C_FIFO_DATA_WIDTH];


// Buffer the input signals that come from outside the tx_port.
always @ (posedge CLK) begin
	rLenValid <= #1 (RST ? 1'd0 : _rLenValid);
	rRen <= #1 (RST ? 1'd0 : _rRen);
end

always @ (*) begin
	_rLenValid = LEN_VALID;
	_rRen = RD_EN;
end


// FIFO for storing data from the channel.
(* RAM_STYLE="BLOCK" *)
sync_fifo #(.C_WIDTH(C_FIFO_DATA_WIDTH), .C_DEPTH(C_FIFO_DEPTH), .C_PROVIDE_COUNT(1)) fifo (
	.CLK(CLK),
	.RST(RST),
	.WR_EN(WR_EN),
	.WR_DATA(WR_DATA),
	.FULL(),
	.COUNT(WR_COUNT),
	.RD_EN(rFifoRdEn),
	.RD_DATA(wFifoData),
	.EMPTY()
);


// Manage shifting of data in from the FIFO and shifting of data out once
// it is consumed. We'll keep 7 words of output registers to hold an input
// packet with up to 3 extra words of unread data.
wire [1:0] wLenLSB = {rLenLSB1[rRdPtr], rLenLSB0[rRdPtr]};
wire wLenLast = rLenLast[rRdPtr];
wire wAfterEnd = (!rRen & rRdEnHist[0]);
    // consumed = 4 if RD+2
    // consumed = remainder if EOP on RD+1 (~rRen & rRdEnHist[0])
    // consumed = 4 if EOP on RD+3 and LAST on RD+3
wire [2:0] wConsumed = ({(rRdEnHist[0] | (!rRdEnHist[0] & rRdEnHist[1] & rLenLastHist[0])),2'd0}) - ({2{wAfterEnd}} & wLenLSB);

always @ (posedge CLK) begin
	rCount <= #1 (RST ? 2'd0 : _rCount);
	rCountHist <= #1 _rCountHist;
	rRdEnHist <= #1 (RST ? {C_RD_EN_HIST{1'd0}} : _rRdEnHist);
	rFifoRdEn <= #1 (RST ? 1'd0 : _rFifoRdEn);
	rFifoRdEnHist <= #1 (RST ? {C_FIFO_RD_EN_HIST{1'd0}} : _rFifoRdEnHist);
	rConsumedHist <= #1 _rConsumedHist;
	rLenLastHist <= #1 (RST ? {C_LEN_LAST_HIST{1'd0}} : _rLenLastHist);
	rFifoData <= #1 _rFifoData;
	rData <= #1 _rData;
end

always @ (*) begin
	// Keep track of words in our buffer. Subtract 4 when we reach 4 on RD_EN.
	// Add wLenLSB when we finish a sequence of RD_EN that read 1, 2, or 3 words.
    // rCount + remainder
	_rCount = rCount + ({2{(wAfterEnd & !wLenLast)}} & wLenLSB) - ({(rRen & rCount[2]), 2'd0}) - ({3{(wAfterEnd & wLenLast)}} & rCount);
	_rCountHist = ((rCountHist<<3) | rCount);

	// Track read enables in the pipeline.
	_rRdEnHist = ((rRdEnHist<<1) | rRen);
	_rFifoRdEnHist = ((rFifoRdEnHist<<1) | rFifoRdEn);
	
	// Track delayed length last value
	_rLenLastHist = ((rLenLastHist<<1) | wLenLast);

	// Calculate the amount to shift out each RD_EN. This is always 4 unless it's
	// the last RD_EN in the sequence and the read words length is 1, 2, or 3.
	_rConsumedHist = ((rConsumedHist<<3) | wConsumed);

	// Read from the FIFO unless we have 4 words cached.
	_rFifoRdEn = (!rCount[2] & rRen);

	// Buffer the FIFO data.
	_rFifoData = wFifoData;

	// Shift the buffered FIFO data into and the consumed data out of the output register.
	if (rFifoRdEnHist[1])
		_rData = ((rData>>({rConsumedHist[8:6], 5'd0})) | (rFifoData<<({rCountHist[7:6], 5'd0})));
	else
		_rData = (rData>>({rConsumedHist[8:6], 5'd0}));
end


// Buffer up to 4 length LSB values for use to detect unread data that was
// part of a consumed packet. Should only need 2. This is basically a FIFO.
always @ (posedge CLK) begin
	rRdPtr <= #1 (RST ? 2'd0 : _rRdPtr);
	rWrPtr <= #1 (RST ? 2'd0 : _rWrPtr);
	rLenLSB0 <= #1 _rLenLSB0;
	rLenLSB1 <= #1 _rLenLSB1;
	rLenLast <= #1 _rLenLast;
end

always @ (*) begin
    _rRdPtr = (wAfterEnd ? rRdPtr + 1'd1 : rRdPtr);
    _rWrPtr = (rLenValid ? rWrPtr + 1'd1 : rWrPtr);
    _rLenLSB0 = rLenLSB0;
    _rLenLSB1 = rLenLSB1;
    if(rLenValid)
        {_rLenLSB1[rWrPtr], _rLenLSB0[rWrPtr]} = (~LEN_LSB + 1);
    _rLenLast = rLenLast;
    if(rLenValid)
        _rLenLast[rWrPtr] = LEN_LAST;
end

endmodule
