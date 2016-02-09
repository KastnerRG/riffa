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
// Filename:			async_fifo.v
// Version:				1.00.a
// Verilog Standard:	Verilog-2001
// Description:			Asynchronous capable parameterized FIFO. As with all
// traditional FIFOs, the RD_DATA will be valid one cycle following a RD_EN 
// assertion. RD_EMPTY will remain low until the cycle following the last RD_EN 
// assertion. Note, that RD_EMPTY may actually be high on the same cycle that 
// RD_DATA contains valid data.
// Author:				Matt Jacobsen
// History:				@mattj: Version 2.0
// Additional Comments: Based on design by CE Cummings in Simulation and 
// Synthesis Techniques for Asynchronous FIFO Design with Asynchronous Pointer 
// Comparisons
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
module async_fifo #(
	parameter C_WIDTH = 32,	// Data bus width
	parameter C_DEPTH = 1024,	// Depth of the FIFO
	// Local parameters
	parameter C_REAL_DEPTH = 2**clog2(C_DEPTH),
	parameter C_DEPTH_BITS = clog2(C_REAL_DEPTH),
	parameter C_DEPTH_P1_BITS = clog2(C_REAL_DEPTH+1)
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

wire						wCmpEmpty;
wire						wCmpFull;
wire	[C_DEPTH_BITS-1:0]	wWrPtr;
wire	[C_DEPTH_BITS-1:0]	wRdPtr;
wire	[C_DEPTH_BITS-1:0]	wWrPtrP1;
wire	[C_DEPTH_BITS-1:0]	wRdPtrP1;


// Memory block (synthesis attributes applied to this module will
// determine the memory option).
ram_2clk_1w_1r #(.C_RAM_WIDTH(C_WIDTH), .C_RAM_DEPTH(C_REAL_DEPTH)) mem (
	.CLKA(WR_CLK),
	.ADDRA(wWrPtr),
	.WEA(WR_EN & !WR_FULL),
	.DINA(WR_DATA),
	.CLKB(RD_CLK),
	.ADDRB(wRdPtr),
	.DOUTB(RD_DATA)
);


// Compare the pointers.
async_cmp #(.C_DEPTH_BITS(C_DEPTH_BITS)) asyncCompare (
	.WR_RST(WR_RST),
	.WR_CLK(WR_CLK),
	.RD_RST(RD_RST),
	.RD_CLK(RD_CLK),
	.RD_VALID(RD_EN & !RD_EMPTY),
	.WR_VALID(WR_EN & !WR_FULL),
	.EMPTY(wCmpEmpty), 
	.FULL(wCmpFull),
	.WR_PTR(wWrPtr), 
	.WR_PTR_P1(wWrPtrP1), 
	.RD_PTR(wRdPtr), 
	.RD_PTR_P1(wRdPtrP1)
);


// Calculate empty
rd_ptr_empty #(.C_DEPTH_BITS(C_DEPTH_BITS)) rdPtrEmpty (
	.RD_EMPTY(RD_EMPTY), 
	.RD_PTR(wRdPtr),
	.RD_PTR_P1(wRdPtrP1),
	.CMP_EMPTY(wCmpEmpty), 
	.RD_EN(RD_EN),
	.RD_CLK(RD_CLK), 
	.RD_RST(RD_RST)
);


// Calculate full
wr_ptr_full #(.C_DEPTH_BITS(C_DEPTH_BITS)) wrPtrFull (
	.WR_CLK(WR_CLK), 
	.WR_RST(WR_RST),
	.WR_EN(WR_EN),
	.WR_FULL(WR_FULL), 
	.WR_PTR(wWrPtr),
	.WR_PTR_P1(wWrPtrP1),
	.CMP_FULL(wCmpFull)
);
 
endmodule


module async_cmp #(
  parameter C_DEPTH_BITS = 4,
  // Local parameters
  parameter N = C_DEPTH_BITS-1
)
(
	input WR_RST,
	input WR_CLK,
	input RD_RST,
	input RD_CLK,
	input RD_VALID,
	input WR_VALID,
	output EMPTY, 
	output FULL, 
	input [C_DEPTH_BITS-1:0] WR_PTR, 
	input [C_DEPTH_BITS-1:0] RD_PTR, 
	input [C_DEPTH_BITS-1:0] WR_PTR_P1, 
	input [C_DEPTH_BITS-1:0] RD_PTR_P1
);
  
reg				rDir=0;
wire			wDirSet = (  (WR_PTR[N]^RD_PTR[N-1]) & ~(WR_PTR[N-1]^RD_PTR[N]));
wire			wDirClr = ((~(WR_PTR[N]^RD_PTR[N-1]) &  (WR_PTR[N-1]^RD_PTR[N])) | WR_RST);

reg				rRdValid=0;
reg				rEmpty=1;
reg				rFull=0;
wire			wATBEmpty = ((WR_PTR == RD_PTR_P1) && (RD_VALID | rRdValid));
wire			wATBFull = ((WR_PTR_P1 == RD_PTR) && WR_VALID);
wire			wEmpty = ((WR_PTR == RD_PTR) && !rDir);
wire			wFull = ((WR_PTR == RD_PTR) && rDir);

assign EMPTY = wATBEmpty || rEmpty;
assign FULL  = wATBFull || rFull;

always @(posedge wDirSet or posedge wDirClr)
if (wDirClr) 
	rDir <= 1'b0;
else
	rDir <= 1'b1;

always @(posedge RD_CLK) begin
	rEmpty <= (RD_RST ? 1'd1 : wEmpty);
	rRdValid <= (RD_RST ? 1'd0 : RD_VALID);
end

always @(posedge WR_CLK) begin
	rFull <= (WR_RST ? 1'd0 : wFull);
end

endmodule 
 
 
module rd_ptr_empty #(
	parameter C_DEPTH_BITS = 4
)
(
	input RD_CLK, 
	input RD_RST,
	input RD_EN, 
	output RD_EMPTY,
	output [C_DEPTH_BITS-1:0] RD_PTR,
	output [C_DEPTH_BITS-1:0] RD_PTR_P1,
	input CMP_EMPTY 
);

reg							rEmpty=1;
reg							rEmpty2=1;
reg		[C_DEPTH_BITS-1:0]	rRdPtr=0;
reg		[C_DEPTH_BITS-1:0]	rRdPtrP1=0;
reg		[C_DEPTH_BITS-1:0]	rBin=0;
reg		[C_DEPTH_BITS-1:0]	rBinP1=1;
wire	[C_DEPTH_BITS-1:0]	wGrayNext;
wire	[C_DEPTH_BITS-1:0]	wGrayNextP1;
wire	[C_DEPTH_BITS-1:0]	wBinNext;
wire	[C_DEPTH_BITS-1:0]	wBinNextP1;

assign RD_EMPTY = rEmpty;
assign RD_PTR = rRdPtr;
assign RD_PTR_P1 = rRdPtrP1;

// Gray coded pointer
always @(posedge RD_CLK or posedge RD_RST) begin
	if (RD_RST) begin
		rBin <= #1 0;
		rBinP1 <= #1 1;
		rRdPtr <= #1 0;
		rRdPtrP1 <= #1 0;
	end
	else begin
		rBin <= #1 wBinNext;
		rBinP1 <= #1 wBinNextP1;
		rRdPtr <= #1 wGrayNext;
		rRdPtrP1 <= #1 wGrayNextP1;
	end
end

// Increment the binary count if not empty
assign wBinNext = (!rEmpty ? rBin + RD_EN : rBin);
assign wBinNextP1 = (!rEmpty ? rBinP1 + RD_EN : rBinP1);
assign wGrayNext = ((wBinNext>>1) ^ wBinNext); // binary-to-gray conversion
assign wGrayNextP1 = ((wBinNextP1>>1) ^ wBinNextP1); // binary-to-gray conversion

always @(posedge RD_CLK) begin
	if (CMP_EMPTY)
		{rEmpty, rEmpty2} <= #1 2'b11;
	else
		{rEmpty, rEmpty2} <= #1 {rEmpty2, CMP_EMPTY};
end

endmodule
 
 
module wr_ptr_full #(
	parameter C_DEPTH_BITS = 4
)
(
	input WR_CLK, 
	input WR_RST,
	input WR_EN,
	output WR_FULL, 
	output [C_DEPTH_BITS-1:0] WR_PTR, 
	output [C_DEPTH_BITS-1:0] WR_PTR_P1, 
	input CMP_FULL
);

reg							rFull=0;
reg							rFull2=0;
reg		[C_DEPTH_BITS-1:0]	rPtr=0;
reg		[C_DEPTH_BITS-1:0]	rPtrP1=0;
reg		[C_DEPTH_BITS-1:0]	rBin=0;
reg		[C_DEPTH_BITS-1:0]	rBinP1=1;
wire	[C_DEPTH_BITS-1:0]	wGrayNext;
wire	[C_DEPTH_BITS-1:0]	wGrayNextP1;
wire	[C_DEPTH_BITS-1:0]	wBinNext;
wire	[C_DEPTH_BITS-1:0]	wBinNextP1;

assign WR_FULL = rFull;
assign WR_PTR = rPtr;
assign WR_PTR_P1 = rPtrP1;

// Gray coded pointer
always @(posedge WR_CLK or posedge WR_RST) begin
	if (WR_RST) begin
		rBin <= #1 0;
		rBinP1 <= #1 1;
		rPtr <= #1 0;
		rPtrP1 <= #1 0;
	end
	else begin
		rBin <= #1 wBinNext;
		rBinP1 <= #1 wBinNextP1;
		rPtr <= #1 wGrayNext;
		rPtrP1 <= #1 wGrayNextP1;
	end
end

// Increment the binary count if not full
assign wBinNext = (!rFull ? rBin + WR_EN : rBin);
assign wBinNextP1 = (!rFull ? rBinP1 + WR_EN : rBinP1);
assign wGrayNext = ((wBinNext>>1) ^ wBinNext); // binary-to-gray conversion
assign wGrayNextP1 = ((wBinNextP1>>1) ^ wBinNextP1); // binary-to-gray conversion

always @(posedge WR_CLK) begin
	if (WR_RST) 
		{rFull, rFull2} <= #1 2'b00;
	else if (CMP_FULL) 
		{rFull, rFull2} <= #1 2'b11;
	else
		{rFull, rFull2} <= #1 {rFull2, CMP_FULL};
end

endmodule

