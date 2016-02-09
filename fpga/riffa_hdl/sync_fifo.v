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
// Filename:			sync_fifo.v
// Version:				1.00.a
// Verilog Standard:	Verilog-2001
// Description:			A synchronous capable parameterized FIFO. As with all
// traditional FIFOs, the RD_DATA will be valid one cycle following a RD_EN 
// assertion. EMPTY will remain low until the cycle following the last RD_EN 
// assertion. Note, that EMPTY may actually be high on the same cycle that 
// RD_DATA contains valid data.
// Author:				Matt Jacobsen
// History:				@mattj: Version 2.0
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
module sync_fifo #(
	parameter C_WIDTH = 32,	// Data bus width
	parameter C_DEPTH = 1024,	// Depth of the FIFO
	parameter C_PROVIDE_COUNT = 0, // Include code for counts
	// Local parameters
	parameter C_REAL_DEPTH = 2**clog2(C_DEPTH),
	parameter C_DEPTH_BITS = clog2s(C_REAL_DEPTH),
	parameter C_DEPTH_P1_BITS = clog2s(C_REAL_DEPTH+1)
)
(
	input CLK,								// Clock
	input RST, 								// Sync reset, active high
	input [C_WIDTH-1:0] WR_DATA, 			// Write data input
	input WR_EN, 							// Write enable, high active
	output [C_WIDTH-1:0] RD_DATA, 			// Read data output
	input RD_EN,							// Read enable, high active
	output FULL, 							// Full condition
	output EMPTY, 							// Empty condition
	output [C_DEPTH_P1_BITS-1:0] COUNT		// Data count
);

`include "functions.vh"

reg		[C_DEPTH_BITS:0]	rWrPtr=0, _rWrPtr=0;
reg		[C_DEPTH_BITS:0]	rWrPtrPlus1=1, _rWrPtrPlus1=1;
reg		[C_DEPTH_BITS:0]	rRdPtr=0, _rRdPtr=0;
reg		[C_DEPTH_BITS:0]	rRdPtrPlus1=1, _rRdPtrPlus1=1;
reg							rFull=0, _rFull=0;
reg							rEmpty=1, _rEmpty=1;

// Memory block (synthesis attributes applied to this module will
// determine the memory option).
ram_1clk_1w_1r #(.C_RAM_WIDTH(C_WIDTH), .C_RAM_DEPTH(C_REAL_DEPTH)) mem (
	.CLK(CLK),
	.ADDRA(rWrPtr[C_DEPTH_BITS-1:0]),
	.WEA(WR_EN & !rFull),
	.DINA(WR_DATA),
	.ADDRB(rRdPtr[C_DEPTH_BITS-1:0]),
	.DOUTB(RD_DATA)
);


// Write pointer logic.
always @ (posedge CLK) begin
	if (RST) begin
		rWrPtr <= #1 0;
		rWrPtrPlus1 <= #1 1;
	end
	else begin
		rWrPtr <= #1 _rWrPtr;
		rWrPtrPlus1 <= #1 _rWrPtrPlus1;
	end
end

always @ (*) begin
	if (WR_EN & !rFull) begin
		_rWrPtr = rWrPtrPlus1;
		_rWrPtrPlus1 = rWrPtrPlus1 + 1'd1;
	end
	else begin
		_rWrPtr = rWrPtr;
		_rWrPtrPlus1 = rWrPtrPlus1;
	end
end


// Read pointer logic.
always @ (posedge CLK) begin
	if (RST) begin
		rRdPtr <= #1 0;
		rRdPtrPlus1 <= #1 1;
	end
	else begin
		rRdPtr <= #1 _rRdPtr;
		rRdPtrPlus1 <= #1 _rRdPtrPlus1;
	end
end

always @ (*) begin
	if (RD_EN & !rEmpty) begin
		_rRdPtr = rRdPtrPlus1;
		_rRdPtrPlus1 = rRdPtrPlus1 + 1'd1;
	end
	else begin
		_rRdPtr = rRdPtr;
		_rRdPtrPlus1 = rRdPtrPlus1;
	end
end


// Calculate empty
assign EMPTY = rEmpty;

always @ (posedge CLK) begin
	rEmpty <= #1 (RST ? 1'd1 : _rEmpty);
end

always @ (*) begin
	_rEmpty = (rWrPtr == rRdPtr) || (RD_EN && !rEmpty && (rWrPtr == rRdPtrPlus1));
end


// Calculate full
assign FULL = rFull;

always @ (posedge CLK) begin
	rFull <= #1 (RST ? 1'd0 : _rFull);
end

always @ (*) begin
	_rFull = ((rWrPtr[C_DEPTH_BITS-1:0] == rRdPtr[C_DEPTH_BITS-1:0]) && (rWrPtr[C_DEPTH_BITS] != rRdPtr[C_DEPTH_BITS])) ||
	(WR_EN && (rWrPtrPlus1[C_DEPTH_BITS-1:0] == rRdPtr[C_DEPTH_BITS-1:0]) && (rWrPtrPlus1[C_DEPTH_BITS] != rRdPtr[C_DEPTH_BITS]));
end

generate
if (C_PROVIDE_COUNT) begin: provide_count
	reg [C_DEPTH_BITS:0] rCount=0, _rCount=0;

	assign COUNT = (rFull ? C_REAL_DEPTH[C_DEPTH_P1_BITS-1:0] : rCount);

	// Calculate read count
	always @ (posedge CLK) begin
		if (RST)
			rCount <= #1 0;
		else
			rCount <= #1 _rCount;
	end

	always @ (*) begin
		_rCount = (rWrPtr - rRdPtr);
	end
end
else begin: provide_no_count
	assign COUNT = 0;
end 
endgenerate
 
endmodule
 
