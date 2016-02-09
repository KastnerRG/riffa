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
// Filename:            recv_credit_flow_ctrl.v
// Version:             1.00.a
// Verilog Standard:    Verilog-2001
// Description:     Monitors the receive completion credits for headers and
//          data to make sure the rx_port modules don't request too 
//          much data from the root complex, as this could result in
//          some data being dropped/lost.
// Author:      Matt Jacobsen
// Author:      Dustin Richmond
// History:     @mattj: Version 2.0
//-----------------------------------------------------------------------------
`timescale 1ns/1ns
module recv_credit_flow_ctrl
    (
     input        CLK,
     input        RST,
     input [2:0]  CONFIG_MAX_READ_REQUEST_SIZE, // Maximum read payload: 000=128B, 001=256B, 010=512B, 011=1024B, 100=2048B, 101=4096B
     input [11:0] CONFIG_MAX_CPL_DATA, // Receive credit limit for data
     input [7:0]  CONFIG_MAX_CPL_HDR, // Receive credit limit for headers
     input        CONFIG_CPL_BOUNDARY_SEL, // Read completion boundary (0=64 bytes, 1=128 bytes)w
     input        RX_ENG_RD_DONE, // Read completed
     input        TX_ENG_RD_REQ_SENT, // Read completion request issued
     output       RXBUF_SPACE_AVAIL // High if enough read completion credits exist to make a read completion request
     );

    reg           rCreditAvail=0;
    reg           rCplDAvail=0;
    reg           rCplHAvail=0;
    reg [12:0]    rMaxRecv=0;
    reg [11:0]    rCplDAmt=0;
    reg [7:0]     rCplHAmt=0;
    reg [11:0]    rCplD=0;
    reg [7:0]     rCplH=0;

    reg           rInfHCred; // TODO: Altera uses sideband signals (would have been more convenient, thanks Xilinx!)
    reg           rInfDCred; // TODO: Altera uses sideband signals (would have been more convenient, thanks Xilinx!)
    

    assign RXBUF_SPACE_AVAIL = rCreditAvail;

    // Determine the completions required for a max read completion request.
    always @(posedge CLK) begin
        rInfHCred <= (CONFIG_MAX_CPL_HDR == 0);
        rInfDCred <= (CONFIG_MAX_CPL_DATA == 0);
        rMaxRecv <= #1 (13'd128<<CONFIG_MAX_READ_REQUEST_SIZE);
        rCplHAmt <= #1 (rMaxRecv>>({2'b11, CONFIG_CPL_BOUNDARY_SEL}));
        rCplDAmt <= #1 (rMaxRecv>>4);
        rCplHAvail <= #1 (rCplH <= CONFIG_MAX_CPL_HDR);
        rCplDAvail <= #1 (rCplD <= CONFIG_MAX_CPL_DATA);
        rCreditAvail <= #1 ((rCplHAvail|rInfHCred) & (rCplDAvail | rInfDCred));
    end

    // Count the number of outstanding read completion requests.
    always @ (posedge CLK) begin
        if (RST) begin
            rCplH <= #1 0;
            rCplD <= #1 0;
        end
        else if (RX_ENG_RD_DONE & TX_ENG_RD_REQ_SENT) begin
            rCplH <= #1 rCplH;
            rCplD <= #1 rCplD;
        end
        else if (TX_ENG_RD_REQ_SENT) begin
            rCplH <= #1 rCplH + rCplHAmt;
            rCplD <= #1 rCplD + rCplDAmt;
        end
        else if (RX_ENG_RD_DONE) begin
            rCplH <= #1 rCplH - rCplHAmt;
            rCplD <= #1 rCplD - rCplDAmt;
        end
    end

endmodule
