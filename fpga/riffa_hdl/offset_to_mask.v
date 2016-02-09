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
`include "functions.vh"
module offset_to_mask
    #(parameter C_MASK_SWAP = 1,
      parameter C_MASK_WIDTH = 4)
    (
     input                            OFFSET_ENABLE,
     input [clog2s(C_MASK_WIDTH)-1:0] OFFSET,
     output [C_MASK_WIDTH-1:0]        MASK
     );

    reg [7:0]                         _rMask,_rMaskSwap; 
    wire [3:0]                        wSelect;
    assign wSelect = {OFFSET_ENABLE,{{(3-clog2s(C_MASK_WIDTH)){1'b0}},OFFSET}};
    assign MASK = (C_MASK_SWAP)? _rMaskSwap[7 -: C_MASK_WIDTH]: _rMask[C_MASK_WIDTH-1:0];
    always @(*) begin
        _rMask = 0;
        _rMaskSwap = 0;
        /* verilator lint_off CASEX */
        casex(wSelect)
            default: begin
                _rMask = 8'b1111_1111;
                _rMaskSwap = 8'b1111_1111;
            end
            4'b1000: begin
                _rMask = 8'b0000_0001;
                _rMaskSwap = 8'b1111_1111;
            end
            4'b1001: begin
                _rMask = 8'b0000_0011;
                _rMaskSwap = 8'b0111_1111;
            end
            4'b1010: begin
                _rMask = 8'b0000_0111;
                _rMaskSwap = 8'b0011_1111;
            end
            4'b1011: begin
                _rMask = 8'b0000_1111;
                _rMaskSwap = 8'b0001_1111;
            end
            4'b1100: begin
                _rMask = 8'b0001_1111;
                _rMaskSwap = 8'b0000_1111;
            end
            4'b1101: begin
                _rMask = 8'b0011_1111;
                _rMaskSwap = 8'b0000_0111;
            end
            4'b1110: begin
                _rMask = 8'b0111_1111;
                _rMaskSwap = 8'b0000_0011;
            end
            4'b1111: begin
                _rMask = 8'b1111_1111;
                _rMaskSwap = 8'b0000_0001;
            end
        endcase // casez ({OFFSET_MASK,OFFSET})

        /* verilator lint_on CASEX */
    end
endmodule
