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
// Filename:			channel_32.v
// Version:				1.00.a
// Verilog Standard:	Verilog-2001
// Description:			Represents a RIFFA channel. Contains a RX port and a 
// TX port.
// Author:				Matt Jacobsen
// History:				@mattj: Version 2.0
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
module channel_32 #(
	parameter C_DATA_WIDTH = 9'd32,
	parameter C_MAX_READ_REQ = 2,					// Max read: 000=128B, 001=256B, 010=512B, 011=1024B, 100=2048B, 101=4096B
	// Local parameters
	parameter C_RX_FIFO_DEPTH = 1024,
	parameter C_TX_FIFO_DEPTH = 512,
	parameter C_SG_FIFO_DEPTH = 1024,
	parameter C_DATA_WORD_WIDTH = clog2((C_DATA_WIDTH/32)+1)
)
(
	input CLK,
	input RST,
	input [2:0] CONFIG_MAX_READ_REQUEST_SIZE,		// Maximum read payload: 000=128B, 001=256B, 010=512B, 011=1024B, 100=2048B, 101=4096B
	input [2:0] CONFIG_MAX_PAYLOAD_SIZE,			// Maximum write payload: 000=128B, 001=256B, 010=512B, 011=1024B

	input [31:0] PIO_DATA,							// Single word programmed I/O data
	input [C_DATA_WIDTH-1:0] ENG_DATA,				// Main incoming data 

	output SG_RX_BUF_RECVD,							// Scatter gather RX buffer completely read (ready for next if applicable)
	input SG_RX_BUF_LEN_VALID,						// Scatter gather RX buffer length valid
	input SG_RX_BUF_ADDR_HI_VALID,					// Scatter gather RX buffer high address valid
	input SG_RX_BUF_ADDR_LO_VALID,					// Scatter gather RX buffer low address valid

	output SG_TX_BUF_RECVD,							// Scatter gather TX buffer completely read (ready for next if applicable)
	input SG_TX_BUF_LEN_VALID,						// Scatter gather TX buffer length valid
	input SG_TX_BUF_ADDR_HI_VALID,					// Scatter gather TX buffer high address valid
	input SG_TX_BUF_ADDR_LO_VALID,					// Scatter gather TX buffer low address valid

	input TXN_RX_LEN_VALID,							// Read transaction length valid
	input TXN_RX_OFF_LAST_VALID,					// Read transaction offset/last valid
	output [31:0] TXN_RX_DONE_LEN,					// Read transaction actual transfer length
	output TXN_RX_DONE,								// Read transaction done
	input TXN_RX_DONE_ACK,							// Read transaction actual transfer length read

	output TXN_TX,									// Write transaction notification
	input TXN_TX_ACK,								// Write transaction acknowledged
	output [31:0] TXN_TX_LEN,						// Write transaction length
	output [31:0] TXN_TX_OFF_LAST,					// Write transaction offset/last
	output [31:0] TXN_TX_DONE_LEN,					// Write transaction actual transfer length
	output TXN_TX_DONE,								// Write transaction done
	input TXN_TX_DONE_ACK,							// Write transaction actual transfer length read

	output RX_REQ,									// Read request
	input RX_REQ_ACK,								// Read request accepted
	output [1:0] RX_REQ_TAG,						// Read request data tag 
	output [63:0] RX_REQ_ADDR,						// Read request address
	output [9:0] RX_REQ_LEN,						// Read request length

	output TX_REQ,									// Outgoing write request
	input TX_REQ_ACK,								// Outgoing write request acknowledged
	output [63:0] TX_ADDR,							// Outgoing write high address
	output [9:0] TX_LEN,							// Outgoing write length (in 32 bit words)
	output [C_DATA_WIDTH-1:0] TX_DATA,				// Outgoing write data
	input TX_DATA_REN,								// Outgoing write data read enable
	input TX_SENT,									// Outgoing write complete

	input [C_DATA_WORD_WIDTH-1:0] MAIN_DATA_EN,		// Main incoming data enable
	input MAIN_DONE,								// Main incoming data complete
	input MAIN_ERR,									// Main incoming data completed with error

	input [C_DATA_WORD_WIDTH-1:0] SG_RX_DATA_EN,	// Scatter gather for RX incoming data enable
	input SG_RX_DONE,								// Scatter gather for RX incoming data complete
	input SG_RX_ERR,								// Scatter gather for RX incoming data completed with error

	input [C_DATA_WORD_WIDTH-1:0] SG_TX_DATA_EN,	// Scatter gather for TX incoming data enable
	input SG_TX_DONE,								// Scatter gather for TX incoming data complete
	input SG_TX_ERR,								// Scatter gather for TX incoming data completed with error

	input CHNL_RX_CLK,									// Channel read clock
	output CHNL_RX,									// Channel read receive signal
	input CHNL_RX_ACK,								// Channle read received signal
	output CHNL_RX_LAST,							// Channel last read
	output [31:0] CHNL_RX_LEN,						// Channel read length
	output [30:0] CHNL_RX_OFF,						// Channel read offset
	output [C_DATA_WIDTH-1:0] CHNL_RX_DATA,			// Channel read data
	output CHNL_RX_DATA_VALID,						// Channel read data valid
	input CHNL_RX_DATA_REN, 						// Channel read data has been recieved

	input CHNL_TX_CLK,								// Channel write clock
	input CHNL_TX,									// Channel write receive signal
	output CHNL_TX_ACK,								// Channel write acknowledgement signal
	input CHNL_TX_LAST,								// Channel last write
	input [31:0] CHNL_TX_LEN,						// Channel write length (in 32 bit words)
	input [30:0] CHNL_TX_OFF,						// Channel write offset
	input [C_DATA_WIDTH-1:0] CHNL_TX_DATA,			// Channel write data
	input CHNL_TX_DATA_VALID,						// Channel write data valid
	output CHNL_TX_DATA_REN							// Channel write data has been recieved
);

`include "functions.vh"


wire	[C_DATA_WIDTH-1:0]	wTxSgData;
wire						wTxSgDataEmpty;
wire						wTxSgDataRen;
wire						wTxSgDataErr;
wire						wTxSgDataRst;


// Receiving port (data to the channel)
rx_port_32 #(
	.C_DATA_WIDTH(C_DATA_WIDTH), 
	.C_MAIN_FIFO_DEPTH(C_RX_FIFO_DEPTH), 
	.C_SG_FIFO_DEPTH(C_SG_FIFO_DEPTH),
	.C_MAX_READ_REQ(C_MAX_READ_REQ)
) rxPort (
	.RST(RST), 
	.CLK(CLK), 
	.CONFIG_MAX_READ_REQUEST_SIZE(CONFIG_MAX_READ_REQUEST_SIZE), 
	
	.SG_RX_BUF_RECVD(SG_RX_BUF_RECVD),
	.SG_RX_BUF_DATA(PIO_DATA),
	.SG_RX_BUF_LEN_VALID(SG_RX_BUF_LEN_VALID),
	.SG_RX_BUF_ADDR_HI_VALID(SG_RX_BUF_ADDR_HI_VALID),
	.SG_RX_BUF_ADDR_LO_VALID(SG_RX_BUF_ADDR_LO_VALID),
	
	.SG_TX_BUF_RECVD(SG_TX_BUF_RECVD),
	.SG_TX_BUF_DATA(PIO_DATA),
	.SG_TX_BUF_LEN_VALID(SG_TX_BUF_LEN_VALID),
	.SG_TX_BUF_ADDR_HI_VALID(SG_TX_BUF_ADDR_HI_VALID),
	.SG_TX_BUF_ADDR_LO_VALID(SG_TX_BUF_ADDR_LO_VALID),
	
	.SG_DATA(wTxSgData),
	.SG_DATA_EMPTY(wTxSgDataEmpty),
	.SG_DATA_REN(wTxSgDataRen),
	.SG_RST(wTxSgDataRst),
	.SG_ERR(wTxSgDataErr),
	
	.TXN_DATA(PIO_DATA), 
	.TXN_LEN_VALID(TXN_RX_LEN_VALID), 
	.TXN_OFF_LAST_VALID(TXN_RX_OFF_LAST_VALID), 
	.TXN_DONE_LEN(TXN_RX_DONE_LEN),
	.TXN_DONE(TXN_RX_DONE),
	.TXN_DONE_ACK(TXN_RX_DONE_ACK),
	
	.RX_REQ(RX_REQ),
	.RX_REQ_ACK(RX_REQ_ACK),
	.RX_REQ_TAG(RX_REQ_TAG),
	.RX_REQ_ADDR(RX_REQ_ADDR),
	.RX_REQ_LEN(RX_REQ_LEN),

	.MAIN_DATA(ENG_DATA),
	.MAIN_DATA_EN(MAIN_DATA_EN), 
	.MAIN_DONE(MAIN_DONE), 
	.MAIN_ERR(MAIN_ERR),
	
	.SG_RX_DATA(ENG_DATA),
	.SG_RX_DATA_EN(SG_RX_DATA_EN), 
	.SG_RX_DONE(SG_RX_DONE), 
	.SG_RX_ERR(SG_RX_ERR),

	.SG_TX_DATA(ENG_DATA),
	.SG_TX_DATA_EN(SG_TX_DATA_EN), 
	.SG_TX_DONE(SG_TX_DONE), 
	.SG_TX_ERR(SG_TX_ERR),

	.CHNL_CLK(CHNL_RX_CLK), 
	.CHNL_RX(CHNL_RX), 
	.CHNL_RX_ACK(CHNL_RX_ACK), 
	.CHNL_RX_LAST(CHNL_RX_LAST), 
	.CHNL_RX_LEN(CHNL_RX_LEN), 
	.CHNL_RX_OFF(CHNL_RX_OFF), 
	.CHNL_RX_DATA(CHNL_RX_DATA), 
	.CHNL_RX_DATA_VALID(CHNL_RX_DATA_VALID), 
	.CHNL_RX_DATA_REN(CHNL_RX_DATA_REN)
);


// Sending port (data from the channel)
tx_port_32 #(
	.C_DATA_WIDTH(C_DATA_WIDTH), 
	.C_FIFO_DEPTH(C_TX_FIFO_DEPTH)
) txPort (
	.CLK(CLK), 
	.RST(RST), 
	.CONFIG_MAX_PAYLOAD_SIZE(CONFIG_MAX_PAYLOAD_SIZE), 
	
	.TXN(TXN_TX),
	.TXN_ACK(TXN_TX_ACK),
	.TXN_LEN(TXN_TX_LEN),
	.TXN_OFF_LAST(TXN_TX_OFF_LAST),
	.TXN_DONE_LEN(TXN_TX_DONE_LEN),
	.TXN_DONE(TXN_TX_DONE),
	.TXN_DONE_ACK(TXN_TX_DONE_ACK),
	
	.SG_DATA(wTxSgData),
	.SG_DATA_EMPTY(wTxSgDataEmpty),
	.SG_DATA_REN(wTxSgDataRen),
	.SG_RST(wTxSgDataRst),
	.SG_ERR(wTxSgDataErr),
	
	.TX_REQ(TX_REQ), 
	.TX_REQ_ACK(TX_REQ_ACK),
	.TX_ADDR(TX_ADDR), 
	.TX_LEN(TX_LEN), 
	.TX_DATA(TX_DATA),
	.TX_DATA_REN(TX_DATA_REN), 
	.TX_SENT(TX_SENT),

	.CHNL_CLK(CHNL_TX_CLK), 
	.CHNL_TX(CHNL_TX), 
	.CHNL_TX_ACK(CHNL_TX_ACK),
	.CHNL_TX_LAST(CHNL_TX_LAST), 
	.CHNL_TX_LEN(CHNL_TX_LEN), 
	.CHNL_TX_OFF(CHNL_TX_OFF), 
	.CHNL_TX_DATA(CHNL_TX_DATA), 
	.CHNL_TX_DATA_VALID(CHNL_TX_DATA_VALID), 
	.CHNL_TX_DATA_REN(CHNL_TX_DATA_REN)
);

endmodule
