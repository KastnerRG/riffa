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
// Filename:            tx_engine_selector.v
// Version:             1.00.a
// Verilog Standard:    Verilog-2001
// Description:         Searches for read and write requests.
//                      PCIe Endpoint core.
// Author:              Matt Jacobsen
// History:             @mattj: Version 2.0
// Additional Comments: 
//-----------------------------------------------------------------------------

`timescale 1ns/1ns
module tx_engine_selector 
    #(
      parameter C_NUM_CHNL = 4'd12
      )
    (
     input                  CLK,
     input                  RST,

     input [C_NUM_CHNL-1:0] REQ_ALL, // Write requests

     output                 REQ, // Write request
     output [3:0]           CHNL// Write channel
     );

    reg [3:0]               rReqChnl=0, _rReqChnl=0;
    reg [3:0]               rReqChnlNext=0, _rReqChnlNext=0;
    reg                     rReqChnlsSame=0, _rReqChnlsSame=0;
    reg [3:0]               rChnlNext=0, _rChnlNext=0;
    reg [3:0]               rChnlNextNext=0, _rChnlNextNext=0;
    reg                     rChnlNextDfrnt=0, _rChnlNextDfrnt=0;
    reg                     rChnlNextNextOn=0, _rChnlNextNextOn=0;
    wire                    wChnlNextNextOn;
    reg                     rReq=0, _rReq=0;
    wire                    wReq;// = (REQ_ALL>>(rReqChnl));
    reg                     rReqChnlNextUpdated=0, _rReqChnlNextUpdated=0;

    assign wReq = REQ_ALL[rReqChnl];
    assign wChnlNextNextOn = REQ_ALL[rChnlNextNext];
    assign REQ = rReq;
    assign CHNL = rReqChnl;


    // Search for the next request so that we can move onto it immediately after
    // the current channel has released its request.
    always @ (posedge CLK) begin
        rReq <= #1 (RST ? 1'd0 : _rReq);
        rReqChnl <= #1 (RST ? 4'd0 : _rReqChnl);
        rReqChnlNext <= #1 (RST ? 4'd0 : _rReqChnlNext);
        rChnlNext <= #1 (RST ? 4'd0 : _rChnlNext);
        rChnlNextNext <= #1 (RST ? 4'd0 : _rChnlNextNext);
        rChnlNextDfrnt <= #1 (RST ? 1'd0 : _rChnlNextDfrnt);
        rChnlNextNextOn <= #1 (RST ? 1'd0 : _rChnlNextNextOn);
        rReqChnlsSame <= #1 (RST ? 1'd0 : _rReqChnlsSame);
        rReqChnlNextUpdated <= #1 (RST ? 1'd1 : _rReqChnlNextUpdated);
    end

    always @ (*) begin
        // Go through each channel (RR), looking for requests
        _rChnlNextNextOn = wChnlNextNextOn;
        _rChnlNext = rChnlNextNext;
        _rChnlNextNext = (rChnlNextNext == C_NUM_CHNL - 1 ? 4'd0 : rChnlNextNext + 1'd1);
        _rChnlNextDfrnt = (rChnlNextNext != rReqChnl);
        _rReqChnlsSame = (rReqChnlNext == rReqChnl);

        // Save ready channel if it is not the same channel we're currently on
        if (rChnlNextNextOn & rChnlNextDfrnt & rReqChnlsSame & !rReqChnlNextUpdated) begin
            _rReqChnlNextUpdated = 1;
            _rReqChnlNext = rChnlNext;
        end
        else begin
            _rReqChnlNextUpdated = 0;
            _rReqChnlNext = rReqChnlNext;
        end
        
        // Assign the new channel
        _rReq = wReq;
        _rReqChnl = (!rReq ? rReqChnlNext : rReqChnl);
    end


endmodule
