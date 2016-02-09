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
// Filename: tx_data_pipeline
// Version: 1.0
// Verilog Standard: Verilog-2001
//
// Description: The TX Data pipeline module takes arbitrarily 32-bit aligned data
// from the WR_TX_DATA interface and shifts the data so that it is 0-bit
// aligned. This data is presented on a set of N fifos, where N =
// (C_DATA_WIDTH/32). Each fifo provides it's own VALID signal and is
// controlled by a READY signal. Each fifo also provides an independent DATA bus
// and additional END_FLAG signal which inidicates that the dword provided in this
// fifo is the last dword in the current payload. The START_FLAG signal indicates
// that the dword at index N = 0 is the start of a new packet.
// 
// The TX Data Pipeline is built from two modules: tx_data_shift.v and
// tx_data_fifo.v. See these modules for more information.
// 
// Author: Dustin Richmond (@darichmond) 
//----------------------------------------------------------------------------
`include "trellis.vh" // Defines the user-facing signal widths.
module tx_data_pipeline
    #(parameter C_DATA_WIDTH = 128,
      parameter C_PIPELINE_INPUT = 1,
      parameter C_PIPELINE_OUTPUT = 1,
      parameter C_MAX_PAYLOAD_DWORDS = 256,
      parameter C_DEPTH_PACKETS = 10,
      parameter C_VENDOR = "ALTERA")
    (// Interface: Clocks
     input                               CLK,

     // Interface: Resets
     input                               RST_IN,

     // Interface: WR TX DATA
     input                               WR_TX_DATA_VALID,
     input [C_DATA_WIDTH-1:0]            WR_TX_DATA,
     input                               WR_TX_DATA_START_FLAG,
     input [clog2s(C_DATA_WIDTH/32)-1:0] WR_TX_DATA_START_OFFSET,
     input                               WR_TX_DATA_END_FLAG,
     input [clog2s(C_DATA_WIDTH/32)-1:0] WR_TX_DATA_END_OFFSET,
     output                              WR_TX_DATA_READY,

     // Interface: TX DATA FIFOS
     input [(C_DATA_WIDTH/32)-1:0]       RD_TX_DATA_WORD_READY,
     output [C_DATA_WIDTH-1:0]           RD_TX_DATA,
     output [(C_DATA_WIDTH/32)-1:0]      RD_TX_DATA_END_FLAGS,
     output                              RD_TX_DATA_START_FLAG,
     output [(C_DATA_WIDTH/32)-1:0]      RD_TX_DATA_WORD_VALID,
     output                              RD_TX_DATA_PACKET_VALID);
    
    wire                                 wRdTxDataValid;
    wire                                 wRdTxDataReady;
    wire                                 wRdTxDataStartFlag;
    wire [C_DATA_WIDTH-1:0]              wRdTxData;
    wire [(C_DATA_WIDTH/32)-1:0]         wRdTxDataEndFlags;
    wire [(C_DATA_WIDTH/32)-1:0]         wRdTxDataWordValid;

    /*AUTOWIRE*/
    /*AUTOINPUT*/
    /*AUTOOUTPUT*/

    tx_data_shift
        #(.C_PIPELINE_OUTPUT            (0),
          /*AUTOINSTPARAM*/
          // Parameters
          .C_PIPELINE_INPUT             (C_PIPELINE_INPUT),
          .C_DATA_WIDTH                 (C_DATA_WIDTH),
          .C_VENDOR                     (C_VENDOR))
    tx_shift_inst
        (// Outputs
         .RD_TX_DATA                    (wRdTxData),
         .RD_TX_DATA_VALID              (wRdTxDataValid),
         .RD_TX_DATA_START_FLAG         (wRdTxDataStartFlag),
         .RD_TX_DATA_WORD_VALID         (wRdTxDataWordValid),
         .RD_TX_DATA_END_FLAGS          (wRdTxDataEndFlags),
         // Inputs
         .RD_TX_DATA_READY              (wRdTxDataReady),
         /*AUTOINST*/
         // Outputs
         .WR_TX_DATA_READY              (WR_TX_DATA_READY),
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN),
         .WR_TX_DATA_VALID              (WR_TX_DATA_VALID),
         .WR_TX_DATA                    (WR_TX_DATA[C_DATA_WIDTH-1:0]),
         .WR_TX_DATA_START_FLAG         (WR_TX_DATA_START_FLAG),
         .WR_TX_DATA_START_OFFSET       (WR_TX_DATA_START_OFFSET[clog2s(C_DATA_WIDTH/32)-1:0]),
         .WR_TX_DATA_END_FLAG           (WR_TX_DATA_END_FLAG),
         .WR_TX_DATA_END_OFFSET         (WR_TX_DATA_END_OFFSET[clog2s(C_DATA_WIDTH/32)-1:0]));

    // TX Data Fifo
    tx_data_fifo
        #(// Parameters
          .C_PIPELINE_INPUT             (1),
          /*AUTOINSTPARAM*/
          // Parameters
          .C_DEPTH_PACKETS              (C_DEPTH_PACKETS),
          .C_DATA_WIDTH                 (C_DATA_WIDTH),
          .C_MAX_PAYLOAD_DWORDS         (C_MAX_PAYLOAD_DWORDS))
    txdf_inst
        (// Outputs
         .WR_TX_DATA_READY              (wRdTxDataReady),
         .RD_TX_DATA                    (RD_TX_DATA[C_DATA_WIDTH-1:0]),
         .RD_TX_DATA_START_FLAG         (RD_TX_DATA_START_FLAG),
         .RD_TX_DATA_WORD_VALID         (RD_TX_DATA_WORD_VALID[(C_DATA_WIDTH/32)-1:0]),
         .RD_TX_DATA_END_FLAGS          (RD_TX_DATA_END_FLAGS[(C_DATA_WIDTH/32)-1:0]),
         .RD_TX_DATA_PACKET_VALID       (RD_TX_DATA_PACKET_VALID),
         // Inputs
         .WR_TX_DATA                    (wRdTxData),
         .WR_TX_DATA_VALID              (wRdTxDataValid),
         .WR_TX_DATA_START_FLAG         (wRdTxDataStartFlag),
         .WR_TX_DATA_WORD_VALID         (wRdTxDataWordValid),
         .WR_TX_DATA_END_FLAGS          (wRdTxDataEndFlags),
         .RD_TX_DATA_WORD_READY         (RD_TX_DATA_WORD_READY),
         /*AUTOINST*/
         // Inputs
         .CLK                           (CLK),
         .RST_IN                        (RST_IN));

endmodule
// Local Variables:
// verilog-library-directories:("." "../../../common/")
// End:
