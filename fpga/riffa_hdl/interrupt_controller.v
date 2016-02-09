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
// Filename:			interrupt_controller.v
// Version:				1.00.a
// Verilog Standard:	Verilog-2001
// Description:			Signals an interrupt on the Xilnx PCIe Endpoint 
// 						interface. Supports single vector MSI or legacy based
// 						interrupts. 
//						When INTR is pulsed high, the interrupt will be issued
//						as soon as possible. If using legacy interrupts, the 
//						initial interrupt must be cleared by another request
//						(typically a PIO read or write request to the 
//						endpoint at some predetermined BAR address). Receipt of
//						the "clear" acknowledgment should cause INTR_LEGACY_CLR 
// 						input to pulse high. Thus completing the legacy 
//						interrupt cycle. If using MSI interrupts, no such
//						acknowldegment is necessary.
// Author:				Matt Jacobsen
// History:				@mattj: Version 2.0
//-----------------------------------------------------------------------------
`define S_INTRCTLR_IDLE				3'd0
`define S_INTRCLTR_WORKING			3'd1
`define S_INTRCLTR_COMPLETE			3'd2
`define S_INTRCLTR_CLEAR_LEGACY		3'd3
`define S_INTRCLTR_CLEARING_LEGACY	3'd4
`define S_INTRCLTR_DONE				3'd5

`timescale 1ns/1ns
module interrupt_controller (
	input CLK,						// System clock
	input RST,						// Async reset
	input INTR,						// Pulsed high to request an interrupt
	input INTR_LEGACY_CLR,			// Pulsed high to ack the legacy interrupt and clear it
	output INTR_DONE,				// Pulsed high to signal interrupt sent
	input CONFIG_INTERRUPT_MSIENABLE,	// 1 if MSI interrupts are enable, 0 if only legacy are supported
	output CFG_INTERRUPT_ASSERT,	// Legacy interrupt message type
	input INTR_MSI_RDY,		// High when interrupt is able to be sent
	output INTR_MSI_REQUEST			// High to request interrupt, when both INTR_MSI_RDY and INTR_MSI_REQUEST are high, interrupt is sent
);

reg		[2:0]	rState=`S_INTRCTLR_IDLE;
reg		[2:0]	rStateNext=`S_INTRCTLR_IDLE;
reg				rIntr=0;
reg				rIntrAssert=0;

assign INTR_DONE = (rState == `S_INTRCLTR_DONE);
assign INTR_MSI_REQUEST = rIntr;
assign CFG_INTERRUPT_ASSERT = rIntrAssert;

// Control sending interrupts.
always @(*) begin
	case (rState)

	`S_INTRCTLR_IDLE : begin
		if (INTR) begin
			rIntr = 1;
			rIntrAssert = !CONFIG_INTERRUPT_MSIENABLE;
			rStateNext = (INTR_MSI_RDY ? `S_INTRCLTR_COMPLETE : `S_INTRCLTR_WORKING);
		end 
		else begin
			rIntr = 0;
			rIntrAssert = 0;
			rStateNext = `S_INTRCTLR_IDLE;
		end
	end

	`S_INTRCLTR_WORKING : begin
		rIntr = 1;
		rIntrAssert = !CONFIG_INTERRUPT_MSIENABLE;
		rStateNext = (INTR_MSI_RDY ? `S_INTRCLTR_COMPLETE : `S_INTRCLTR_WORKING);
	end

	`S_INTRCLTR_COMPLETE : begin
		rIntr = 0;
		rIntrAssert = !CONFIG_INTERRUPT_MSIENABLE;
		rStateNext = (CONFIG_INTERRUPT_MSIENABLE ? `S_INTRCLTR_DONE : `S_INTRCLTR_CLEAR_LEGACY);
	end

	`S_INTRCLTR_CLEAR_LEGACY : begin
		if (INTR_LEGACY_CLR) begin
			rIntr = 1;
			rIntrAssert = 0;
			rStateNext = (INTR_MSI_RDY ? `S_INTRCLTR_DONE : `S_INTRCLTR_CLEARING_LEGACY);
		end 
		else begin
			rIntr = 0;
			rIntrAssert = 1;
			rStateNext = `S_INTRCLTR_CLEAR_LEGACY;
		end
	end

	`S_INTRCLTR_CLEARING_LEGACY : begin
		rIntr = 1;
		rIntrAssert = 0;
		rStateNext = (INTR_MSI_RDY ? `S_INTRCLTR_DONE : `S_INTRCLTR_CLEARING_LEGACY);
	end

	`S_INTRCLTR_DONE : begin
		rIntr = 0;
		rIntrAssert = 0;
		rStateNext = `S_INTRCTLR_IDLE;
	end
	
	default: begin
		rIntr = 0;
		rIntrAssert = 0;
		rStateNext = `S_INTRCTLR_IDLE;
	end
	
	endcase
end

// Update the state.
always @(posedge CLK) begin
	if (RST)
		rState <= #1 `S_INTRCTLR_IDLE;
	else
		rState <= #1 rStateNext;
end

endmodule

