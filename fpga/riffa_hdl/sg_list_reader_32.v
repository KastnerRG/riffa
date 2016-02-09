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
// Filename:			sg_list_reader_32.v
// Version:				1.00.a
// Verilog Standard:	Verilog-2001
// Description:			Reads data from the scatter gather list buffer.
// Author:				Matt Jacobsen
// History:				@mattj: Version 2.0
//-----------------------------------------------------------------------------
`define S_SGR32_RD_0		3'b000
`define S_SGR32_RD_1		3'b001
`define S_SGR32_RD_2		3'b010
`define S_SGR32_RD_3		3'b011
`define S_SGR32_RD_WAIT		3'b100

`define S_SGR32_CAP_0		3'b000
`define S_SGR32_CAP_1		3'b001
`define S_SGR32_CAP_2		3'b010
`define S_SGR32_CAP_3		3'b011
`define S_SGR32_CAP_RDY		3'b100

`timescale 1ns/1ns
module sg_list_reader_32 #(
	parameter C_DATA_WIDTH = 9'd32
)
(
	input CLK,
	input RST,

	input [C_DATA_WIDTH-1:0] BUF_DATA,	// Scatter gather buffer data 
	input BUF_DATA_EMPTY,				// Scatter gather buffer data empty
	output BUF_DATA_REN,				// Scatter gather buffer data read enable

	output VALID,						// Scatter gather element data is valid
	output EMPTY,						// Scatter gather elements empty
	input REN,							// Scatter gather element data read enable
	output [63:0] ADDR,					// Scatter gather element address
	output [31:0] LEN					// Scatter gather element length (in words)
);

(* syn_encoding = "user" *)
(* fsm_encoding = "user" *)
reg		[2:0]				rRdState=`S_SGR32_RD_0, _rRdState=`S_SGR32_RD_0;

(* syn_encoding = "user" *)
(* fsm_encoding = "user" *)
reg		[2:0]				rCapState=`S_SGR32_CAP_0, _rCapState=`S_SGR32_CAP_0;
reg		[C_DATA_WIDTH-1:0]	rData={C_DATA_WIDTH{1'd0}}, _rData={C_DATA_WIDTH{1'd0}};
reg		[63:0]				rAddr=64'd0, _rAddr=64'd0;
reg		[31:0]				rLen=0, _rLen=0;
reg							rFifoValid=0, _rFifoValid=0;
reg							rDataValid=0, _rDataValid=0;


assign BUF_DATA_REN = !rRdState[2]; // Not S_SGR32_RD_0
assign VALID = rCapState[2]; // S_SGR32_CAP_RDY
assign EMPTY = (BUF_DATA_EMPTY & !rRdState[2]); // Not S_SGR32_RD_0
assign ADDR = rAddr;
assign LEN = rLen;


// Capture address and length as it comes out of the FIFO
always @ (posedge CLK) begin
	rRdState <= #1 (RST ? `S_SGR32_RD_0 : _rRdState);
	rCapState <= #1 (RST ? `S_SGR32_CAP_0 : _rCapState);
	rData <= #1 _rData;
	rFifoValid <= #1 (RST ? 1'd0 : _rFifoValid);
	rDataValid <= #1 (RST ? 1'd0 : _rDataValid);
	rAddr <= #1 _rAddr;
	rLen <= #1 _rLen;
end

always @ (*) begin
	_rRdState = rRdState;
	_rCapState = rCapState;
	_rAddr = rAddr;
	_rLen = rLen;
	_rData = BUF_DATA;
	_rFifoValid = (BUF_DATA_REN & !BUF_DATA_EMPTY);
	_rDataValid = rFifoValid;

	case (rCapState)
	
	`S_SGR32_CAP_0: begin
		if (rDataValid) begin
			_rAddr[31:0] = rData;
			_rCapState = `S_SGR32_CAP_1;
		end
	end

	`S_SGR32_CAP_1: begin
		if (rDataValid) begin
			_rAddr[63:32] = rData;
			_rCapState = `S_SGR32_CAP_2;
		end
	end
	
	`S_SGR32_CAP_2: begin
		if (rDataValid) begin
			_rLen = rData;
			_rCapState = `S_SGR32_CAP_3;
		end
	end

	`S_SGR32_CAP_3: begin
		if (rDataValid)
			_rCapState = `S_SGR32_CAP_RDY;
	end

	`S_SGR32_CAP_RDY: begin
		if (REN)
			_rCapState = `S_SGR32_CAP_0;
	end

	default: begin
		_rCapState = `S_SGR32_CAP_0;
	end
	
	endcase

	case (rRdState)

	`S_SGR32_RD_0: begin // Read from the sg data FIFO
		if (!BUF_DATA_EMPTY)
			_rRdState = `S_SGR32_RD_1;
	end

	`S_SGR32_RD_1: begin // Read from the sg data FIFO
		if (!BUF_DATA_EMPTY)
			_rRdState = `S_SGR32_RD_2;
	end

	`S_SGR32_RD_2: begin // Read from the sg data FIFO
		if (!BUF_DATA_EMPTY)
			_rRdState = `S_SGR32_RD_3;
	end

	`S_SGR32_RD_3: begin // Read from the sg data FIFO
		if (!BUF_DATA_EMPTY)
			_rRdState = `S_SGR32_RD_WAIT;
	end

	`S_SGR32_RD_WAIT: begin // Wait for the data to be consumed
		if (REN)
			_rRdState = `S_SGR32_RD_0;
	end

	default: begin
		_rRdState = `S_SGR32_RD_0;
	end
	
	endcase
end

endmodule
