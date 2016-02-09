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
// Filename:			fifo_packer_32.v
// Version:				1.00.a
// Verilog Standard:	Verilog-2001
// Description:			Packs 32 bit received data into a 32 bit wide FIFO. 
// Assumes the FIFO always has room to accommodate the data.
// Author:				Matt Jacobsen
// History:				@mattj: Version 2.0
// Additional Comments: 
//-----------------------------------------------------------------------------

`timescale 1ns/1ns
module fifo_packer_32 (
	input CLK,
	input RST,
	input [31:0] DATA_IN,		// Incoming data
	input DATA_IN_EN,			// Incoming data enable
	input DATA_IN_DONE,			// Incoming data packet end
	input DATA_IN_ERR,			// Incoming data error
	input DATA_IN_FLUSH,		// End of incoming data
	output [31:0] PACKED_DATA,	// Outgoing data
	output PACKED_WEN,			// Outgoing data write enable
	output PACKED_DATA_DONE,	// End of outgoing data packet
	output PACKED_DATA_ERR,		// Error in outgoing data
	output PACKED_DATA_FLUSHED	// End of outgoing data
);

reg					rPackedDone=0, _rPackedDone=0;
reg					rPackedErr=0, _rPackedErr=0;
reg					rPackedFlush=0, _rPackedFlush=0;
reg					rPackedFlushed=0, _rPackedFlushed=0;
reg		[31:0]		rDataIn=64'd0, _rDataIn=64'd0;
reg					rDataInEn=0, _rDataInEn=0;


assign PACKED_DATA = rDataIn;
assign PACKED_WEN = rDataInEn;
assign PACKED_DATA_DONE = rPackedDone;
assign PACKED_DATA_ERR = rPackedErr;
assign PACKED_DATA_FLUSHED = rPackedFlushed;


// Buffers input data to ease timing.
always @ (posedge CLK) begin
	rPackedDone <= #1 (RST ? 1'd0 : _rPackedDone);
	rPackedErr <= #1 (RST ? 1'd0 : _rPackedErr);
	rPackedFlush <= #1 (RST ? 1'd0 : _rPackedFlush);
	rPackedFlushed <= #1 (RST ? 1'd0 : _rPackedFlushed);
	rDataIn <= #1 _rDataIn;
	rDataInEn <= #1 (RST ? 1'd0 : _rDataInEn);
end

always @ (*) begin
	// Buffer and mask the input data.
	_rDataIn = DATA_IN;
	_rDataInEn = DATA_IN_EN;

	// Track done/error/flush signals.
	_rPackedDone = DATA_IN_DONE;
	_rPackedErr = DATA_IN_ERR;
	_rPackedFlush = DATA_IN_FLUSH;
	_rPackedFlushed = rPackedFlush;
end



endmodule
