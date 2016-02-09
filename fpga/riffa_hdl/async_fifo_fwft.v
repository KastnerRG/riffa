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
// Filename:			async_fifo_fwft.v
// Version:				1.00.a
// Verilog Standard:	Verilog-2001
// Description:			An asynchronous capable parameterized FIFO. As with all
// first word fall through FIFOs, the RD_DATA will be valid when RD_EMPTY is 
// low. Asserting RD_EN will consume the current RD_DATA value and cause the 
// next value (if it exists) to appear on RD_DATA on the following cycle. Be sure 
// to check if RD_EMPTY is low each cycle to determine if RD_DATA is valid.
// Author:				Matt Jacobsen
// History:				@mattj: Version 2.0
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
module async_fifo_fwft #(
	parameter C_WIDTH = 32,	// Data bus width
	parameter C_DEPTH = 1024,	// Depth of the FIFO
	// Local parameters
	parameter C_REAL_DEPTH = 2**clog2(C_DEPTH),
	parameter C_DEPTH_BITS = clog2s(C_REAL_DEPTH),
	parameter C_DEPTH_P1_BITS = clog2s(C_REAL_DEPTH+1)
)
(
	input RD_CLK,							// Read clock
	input RD_RST,							// Read synchronous reset
	input WR_CLK,						 	// Write clock
	input WR_RST,							// Write synchronous reset
	input [C_WIDTH-1:0] WR_DATA, 			// Write data input (WR_CLK)
	input WR_EN, 							// Write enable, high active (WR_CLK)
	output [C_WIDTH-1:0] RD_DATA, 			// Read data output (RD_CLK)
	input RD_EN,							// Read enable, high active (RD_CLK)
	output WR_FULL, 						// Full condition (WR_CLK)
	output RD_EMPTY 						// Empty condition (RD_CLK)
);

`include "functions.vh"

reg		[C_WIDTH-1:0]			rData=0;
reg		[C_WIDTH-1:0]			rCache=0;
reg		[1:0]					rCount=0;
reg								rFifoDataValid=0;
reg								rDataValid=0;
reg								rCacheValid=0;
wire	[C_WIDTH-1:0]			wData;
wire							wEmpty;
wire							wRen = RD_EN || (rCount < 2'd2);


assign RD_DATA = rData;
assign RD_EMPTY = !rDataValid;


// Wrapped non-FWFT FIFO (synthesis attributes applied to this module will
// determine the memory option).
async_fifo #(.C_WIDTH(C_WIDTH), .C_DEPTH(C_DEPTH)) fifo (
	.WR_CLK(WR_CLK),
	.WR_RST(WR_RST),
	.RD_CLK(RD_CLK),
	.RD_RST(RD_RST),
	.WR_EN(WR_EN),
	.WR_DATA(WR_DATA),
	.WR_FULL(WR_FULL),
	.RD_EN(wRen),
	.RD_DATA(wData),
	.RD_EMPTY(wEmpty)
);

always @ (posedge RD_CLK) begin
	if (RD_RST) begin
		rCount <= #1 0;
		rDataValid <= #1 0;
		rCacheValid <= #1 0;
		rFifoDataValid <= #1 0;
	end
	else begin
		// Keep track of the count
		rCount <= #1 rCount + (wRen & !wEmpty) - (!RD_EMPTY & RD_EN);

		// Signals when wData from FIFO is valid
		rFifoDataValid <= #1 (wRen & !wEmpty);

		// Keep rData up to date
		if (rFifoDataValid) begin
			if (RD_EN | !rDataValid) begin
				rData <= #1 wData;
				rDataValid <= #1 1'd1;
				rCacheValid <= #1 1'd0;
			end
			else begin
				rCacheValid <= #1 1'd1;
			end
			rCache  <= #1 wData;
		end
		else begin
			if (RD_EN | !rDataValid) begin
				rData <= #1 rCache;
				rDataValid <= #1 rCacheValid;
				rCacheValid <= #1 1'd0;
			end
		end
	end
end
 
endmodule
 
