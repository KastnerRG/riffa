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
// Filename:			cross_domain_signal.v
// Version:				1.00.a
// Verilog Standard:	Verilog-2001
// Description:			Send a signal from clock domain A into clock domain B
// and get the signal back into clock domain A. Domain A can know roughly when 
// the signal is received domain B. 
// Author:				Matt Jacobsen
// History:				@mattj: Version 2.0
//-----------------------------------------------------------------------------

`timescale 1ns/1ns
module cross_domain_signal (
	input CLK_A,		// Clock for domain A
	input CLK_A_SEND,	// Signal from domain A to domain B
	output CLK_A_RECV,	// Signal from domain B received in domain A 
	input CLK_B,		// Clock for domain B
	output CLK_B_RECV,	// Signal from domain A received in domain B
	input CLK_B_SEND	// Signal from domain B to domain A
);


// Sync level signals across domains.
syncff sigAtoB (.CLK(CLK_B), .IN_ASYNC(CLK_A_SEND), .OUT_SYNC(CLK_B_RECV));
syncff sigBtoA (.CLK(CLK_A), .IN_ASYNC(CLK_B_SEND), .OUT_SYNC(CLK_A_RECV));


endmodule
