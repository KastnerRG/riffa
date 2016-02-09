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
// Filename:			tx_port_buffer_32.v
// Version:				1.00.a
// Verilog Standard:	Verilog-2001
// Description:			Wraps a FIFO for saving channel data and provides a 
// registered read output. Data is available 3 cycles after RD_EN is asserted 
// (not 1, like a traditional FIFO).
// Author:				Matt Jacobsen
// History:				@mattj: Version 2.0
//-----------------------------------------------------------------------------

`timescale 1ns/1ns
module tx_port_buffer_32 #(
	parameter C_FIFO_DATA_WIDTH = 9'd32,
	parameter C_FIFO_DEPTH = 512,
	// Local parameters
	parameter C_FIFO_DEPTH_WIDTH = clog2((2**clog2(C_FIFO_DEPTH))+1)
)
(
	input RST,
	input CLK,

	input [C_FIFO_DATA_WIDTH-1:0] WR_DATA,		// Input data
	input WR_EN,								// Input data write enable
	output [C_FIFO_DEPTH_WIDTH-1:0] WR_COUNT,	// Input data FIFO is full

	output [C_FIFO_DATA_WIDTH-1:0] RD_DATA,		// Output data
	input RD_EN									// Output data read enable
);

`include "functions.vh"

reg 								rFifoRdEn=0, _rFifoRdEn=0;
reg		[C_FIFO_DATA_WIDTH-1:0]		rFifoData={C_FIFO_DATA_WIDTH{1'd0}}, _rFifoData={C_FIFO_DATA_WIDTH{1'd0}};
wire	[C_FIFO_DATA_WIDTH-1:0]		wFifoData;

assign RD_DATA = rFifoData;


// Buffer the input signals that come from outside the tx_port.
always @ (posedge CLK) begin
	rFifoRdEn <= #1 (RST ? 1'd0 : _rFifoRdEn);
end

always @ (*) begin
	_rFifoRdEn = RD_EN;
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


// Buffer data from the FIFO.
always @ (posedge CLK) begin
	rFifoData <= #1 _rFifoData;
end

always @ (*) begin
	_rFifoData = wFifoData;
end


endmodule
