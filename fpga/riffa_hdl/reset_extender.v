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
module reset_extender
    #(parameter C_RST_COUNT = 10)
    (input  CLK,
     input  RST_BUS,
     input  RST_LOGIC,
     output RST_OUT,
     output PENDING_RST);

    localparam C_CLOG2_RST_COUNT = clog2s(C_RST_COUNT);
    localparam C_CEIL2_RST_COUNT = 1 << C_CLOG2_RST_COUNT;
    localparam C_RST_SHIFTREG_DEPTH = 4;
    
    wire [C_CLOG2_RST_COUNT:0] wRstCount;
    wire [C_RST_SHIFTREG_DEPTH:0] wRstShiftReg;
    
    assign PENDING_RST = wRstShiftReg != 0;
    assign RST_OUT = wRstShiftReg[C_RST_SHIFTREG_DEPTH];
    
    counter
        #(// Parameters
          .C_MAX_VALUE                  (C_CEIL2_RST_COUNT),
          .C_SAT_VALUE                  (C_CEIL2_RST_COUNT),
          .C_RST_VALUE                  (C_CEIL2_RST_COUNT - C_RST_COUNT)
          /*AUTOINSTPARAM*/)
    rst_counter
        (// Outputs
         .VALUE                         (wRstCount),
         // Inputs
         .ENABLE                        (1'b1),
         .RST_IN                        (RST_BUS | RST_LOGIC),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));

    shiftreg
        #(// Parameters
          .C_DEPTH                      (C_RST_SHIFTREG_DEPTH),
          .C_WIDTH                      (1),
          .C_VALUE                      (0)
          /*AUTOINSTPARAM*/)
    rst_shiftreg
        (// Outputs
         .RD_DATA                       (wRstShiftReg),
         // Inputs
         .RST_IN                        (0),
         .WR_DATA                       (~wRstCount[C_CLOG2_RST_COUNT]),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK));
endmodule
