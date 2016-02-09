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
// Filename:			interrupt.v
// Version:				1.00.a
// Verilog Standard:	Verilog-2001
// Description:			Manages the interrupt vector and sends interrupts.
// Author:				Matt Jacobsen
// History:				@mattj: Version 2.0
//-----------------------------------------------------------------------------
`define S_INTR_IDLE		2'd0
`define S_INTR_INTR		2'd1
`define S_INTR_CLR_0	2'd2
`define S_INTR_CLR_1	2'd3

`timescale 1ns/1ns
module interrupt #(
	parameter C_NUM_CHNL = 4'd12
)
(
	input CLK,
	input RST,
	input [C_NUM_CHNL-1:0] RX_SG_BUF_RECVD,	// The scatter gather data for a rx_port transaction has been read
	input [C_NUM_CHNL-1:0] RX_TXN_DONE,		// The rx_port transaction is done
	input [C_NUM_CHNL-1:0] TX_TXN,			// New tx_port transaction
	input [C_NUM_CHNL-1:0] TX_SG_BUF_RECVD,	// The scatter gather data for a tx_port transaction has been read
	input [C_NUM_CHNL-1:0] TX_TXN_DONE,		// The tx_port transaction is done
	input VECT_0_RST,						// Interrupt vector 0 reset
	input VECT_1_RST,						// Interrupt vector 1 reset
	input [31:0] VECT_RST,					// Interrupt vector reset value
	output [31:0] VECT_0,					// Interrupt vector 0
	output [31:0] VECT_1,					// Interrupt vector 1
	input INTR_LEGACY_CLR,					// Pulsed high to ack the legacy interrupt and clear it
	input CONFIG_INTERRUPT_MSIENABLE,		// 1 if MSI interrupts are enable, 0 if only legacy are supported
	input INTR_MSI_RDY,		 				// High when interrupt is able to be sent
	output INTR_MSI_REQUEST					// High to request interrupt, when both INTR_MSI_RDY and INTR_MSI_REQUEST are high, interrupt is sent
);

reg		[1:0]		rState=0;
reg		[31:0]		rVect0=0;
reg		[31:0]		rVect1=0;
wire	[31:0]		wVect0;
wire	[31:0]		wVect1;
wire				wIntr = (rState == `S_INTR_INTR);
wire				wIntrDone;

assign VECT_0 = rVect0;
assign VECT_1 = rVect1;

// Align the input signals to the interrupt vector. 
// VECT_0/VECT_1 are organized from right to left (LSB to MSB) as:
// [ 0] TX_TXN			for channel 0 in VECT_0, channel 6 in VECT_1
// [ 1] TX_SG_BUF_RECVD	for channel 0 in VECT_0, channel 6 in VECT_1
// [ 2] TX_TXN_DONE		for channel 0 in VECT_0, channel 6 in VECT_1
// [ 3] RX_SG_BUF_RECVD	for channel 0 in VECT_0, channel 6 in VECT_1
// [ 4] RX_TXN_DONE		for channel 0 in VECT_0, channel 6 in VECT_1
// ...
// [25] TX_TXN			for channel 5 in VECT_0, channel 11 in VECT_1
// [26] TX_SG_BUF_RECVD	for channel 5 in VECT_0, channel 11 in VECT_1
// [27] TX_TXN_DONE		for channel 5 in VECT_0, channel 11 in VECT_1
// [28] RX_SG_BUF_RECVD	for channel 5 in VECT_0, channel 11 in VECT_1
// [29] RX_TXN_DONE		for channel 5 in VECT_0, channel 11 in VECT_1
// Positions 30 - 31 in both VECT_0 and VECT_1 are zero.

genvar i;
generate
	for (i = 0; i < C_NUM_CHNL; i = i + 1) begin: vectMap
		if (i < 6) begin : vectMap0
			assign wVect0[(5*i)+0] = TX_TXN[i];
			assign wVect0[(5*i)+1] = TX_SG_BUF_RECVD[i];
			assign wVect0[(5*i)+2] = TX_TXN_DONE[i];
			assign wVect0[(5*i)+3] = RX_SG_BUF_RECVD[i];
			assign wVect0[(5*i)+4] = RX_TXN_DONE[i];
		end
		else begin : vectMap1
			assign wVect1[(5*(i-6))+0] = TX_TXN[i];
			assign wVect1[(5*(i-6))+1] = TX_SG_BUF_RECVD[i];
			assign wVect1[(5*(i-6))+2] = TX_TXN_DONE[i];
			assign wVect1[(5*(i-6))+3] = RX_SG_BUF_RECVD[i];
			assign wVect1[(5*(i-6))+4] = RX_TXN_DONE[i];
		end	
	end
	for (i = C_NUM_CHNL; i < 12; i = i + 1) begin: vectZero
		if (i < 6) begin : vectZero0
			assign wVect0[(5*i)+0] = 1'b0;
			assign wVect0[(5*i)+1] = 1'b0;
			assign wVect0[(5*i)+2] = 1'b0;
			assign wVect0[(5*i)+3] = 1'b0;
			assign wVect0[(5*i)+4] = 1'b0;
		end
		else begin : vectZero1
			assign wVect1[(5*(i-6))+0] = 1'b0;
			assign wVect1[(5*(i-6))+1] = 1'b0;
			assign wVect1[(5*(i-6))+2] = 1'b0;
			assign wVect1[(5*(i-6))+3] = 1'b0;
			assign wVect1[(5*(i-6))+4] = 1'b0;
		end	
	end
	assign wVect0[30] = 1'b0;
	assign wVect0[31] = 1'b0;
	assign wVect1[30] = 1'b0;
	assign wVect1[31] = 1'b0;
endgenerate

// Interrupt controller
interrupt_controller intrCtlr (
	.CLK(CLK),
	.RST(RST),
	.INTR(wIntr),
	.INTR_LEGACY_CLR(INTR_LEGACY_CLR),
	.INTR_DONE(wIntrDone),
	.CFG_INTERRUPT_ASSERT(),
	.CONFIG_INTERRUPT_MSIENABLE(CONFIG_INTERRUPT_MSIENABLE),
	.INTR_MSI_RDY(INTR_MSI_RDY),
	.INTR_MSI_REQUEST(INTR_MSI_REQUEST)
);

// Update the interrupt vector when new signals come in (pulse in) and on reset.
always @(posedge CLK) begin
	if (RST) begin
		rVect0 <= #1 0;
		rVect1 <= #1 0;
	end 
	else begin
		if (VECT_0_RST) begin
			rVect0 <= #1 (wVect0 | (rVect0 & ~VECT_RST));
			rVect1 <= #1 (wVect1 | rVect1);
		end
		else if (VECT_1_RST) begin
			rVect0 <= #1 (wVect0 | rVect0);
			rVect1 <= #1 (wVect1 | (rVect1 & ~VECT_RST));
		end
		else begin
			rVect0 <= #1 (wVect0 | rVect0);
			rVect1 <= #1 (wVect1 | rVect1);
		end
	end
end	

// Fire the interrupt when we have a non-zero vector.
always @(posedge CLK) begin
	if (RST) begin
		rState <= #1 `S_INTR_IDLE;
	end
	else begin
		case (rState)
		`S_INTR_IDLE :	rState <= #1 ((rVect0 | rVect1) == 0 ? `S_INTR_IDLE : `S_INTR_INTR);
		`S_INTR_INTR :	rState <= #1 (wIntrDone ? `S_INTR_CLR_0 : `S_INTR_INTR);
		`S_INTR_CLR_0 :	rState <= #1 (VECT_0_RST ? (C_NUM_CHNL > 6 ? `S_INTR_CLR_1 : `S_INTR_IDLE) : `S_INTR_CLR_0);
		`S_INTR_CLR_1 :	rState <= #1 (VECT_1_RST ? `S_INTR_IDLE : `S_INTR_CLR_1);
		endcase
	end
end	

endmodule
