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
// Filename:            tlp.vh
// Version:             1.0
// Verilog Standard:    Verilog-2001
// Description:         The tlp.vh file is a header file that defines various
// TLP specific primitives for the Xilinx and Altera "Classic" endpoints
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`ifndef __TLP_VH
`define __TLP_VH 1
`include "widths.vh"
`include "types.vh"

`define TLP_MAXHDR_W 128

// Generic 1'st word TLP header widths & indices
`define TLP_LEN_I 0
`define TLP_LEN_W `LEN_W
`define TLP_LEN_R `TLP_LEN_I +: `TLP_LEN_W 

`define TLP_ADDRTYPE_I 10
`define TLP_ADDRTYPE_W 2
`define TLP_ADDRTYPE_R `TLP_ADDRTYPE_I +: `TLP_ADDRTYPE_W

`define TLP_ATTR0_I 12
`define TLP_ATTR0_W 2
`define TLP_ATTR0_R `TLP_ATTR0_I +: `TLP_ATTR0_W 

`define TLP_EP_I 14
`define TLP_EP_W `EP_W
`define TLP_EP_R `TLP_EP_I +: `TLP_EP_W 

`define TLP_TD_I 15
`define TLP_TD_W `TD_W
`define TLP_TD_R `TLP_TD_I +: `TLP_TD_W 

`define TLP_TH_I 16
`define TLP_TH_W 1
`define TLP_TH_R `TLP_TH_I +: `TLP_TH_W 

// Bit 17 is reserved
`define TLP_RSVD0_I 17
`define TLP_RSVD0_W 1
`define TLP_RSVD0_R `TLP_RSVD0_I +: `TLP_RSVD0_W
`define TLP_RSVD0_V `TLP_RSVD0_W'b0

// There are two attr fields
`define TLP_ATTR1_I 18
`define TLP_ATTR1_W 1
`define TLP_ATTR1_R `TLP_ATTR1_I +: `TLP_ATTR1_W 

`define TLP_ATTR_W (`TLP_ATTR0_W + `TLP_ATTR1_W)

// Bit 19 is reserved
`define TLP_RSVD1_I 19
`define TLP_RSVD1_W 1
`define TLP_RSVD1_R `TLP_RSVD1_I +: `TLP_RSVD1_W
`define TLP_RSVD1_V `TLP_RSVD1_W'b0

`define TLP_TC_I 20
`define TLP_TC_W `TC_W
`define TLP_TC_R `TLP_TC_I +: `TLP_TC_W 

// Bit 23 is reserved
`define TLP_RSVD2_I 23
`define TLP_RSVD2_W 1
`define TLP_RSVD2_R `TLP_RSVD2_I +: `TLP_RSVD2_W
`define TLP_RSVD2_V `TLP_RSVD2_W'b0

`define TLP_TYPE_I 24
`define TLP_TYPE_W `TYPE_W
`define TLP_TYPE_R `TLP_TYPE_I +: `TLP_TYPE_W 

`define TLP_FMT_I 29
`define TLP_FMT_W `FMT_W
`define TLP_FMT_R `TLP_FMT_I +: `TLP_FMT_W 

`define TLP_4DWHBIT_I 29
`define TLP_PAYBIT_I 30


// Request specific fields
`define TLP_REQMETADW0_I 0
`define TLP_REQMETADW1_I 32
`define TLP_REQADDRDW0_I 64
`define TLP_REQADDRDW1_I 96

`define TLP_REQFBE_I 32
`define TLP_REQFBE_W `FBE_W
`define TLP_REQFBE_R `TLP_REQFBE_I +: `TLP_REQFBE_W

`define TLP_REQLBE_I 36
`define TLP_REQLBE_W `LBE_W
`define TLP_REQLBE_R `TLP_REQLBE_I +: `TLP_REQLBE_W

`define TLP_REQTAG_I 40
`define TLP_REQTAG_W `TAG_W
`define TLP_REQTAG_R `TLP_REQTAG_I +: `TLP_REQTAG_W

`define TLP_REQREQID_I 48
`define TLP_REQREQID_W `REQID_W
`define TLP_REQREQID_R `TLP_REQREQID_I +: `TLP_REQREQID_W

`define TLP_REQADDRLO_I 64
`define TLP_REQADDRLO_W 32
`define TLP_REQADDRLO_R `TLP_REQADDRLO_I +: `TLP_REQADDRLO_W

`define TLP_REQADDRHI_I 96
`define TLP_REQADDRHI_W 32
`define TLP_REQADDRHI_R `TLP_REQADDRHI_I +: `TLP_REQADDRHI_W

`define TLP_REQADDR_W `ADDR_W

// Completion specific fields
`define TLP_CPLMETADW0_I 0
`define TLP_CPLMETADW1_I 32
`define TLP_CPLMETADW2_I 64
`define TLP_CPLPAYLOAD_I 96
`define TLP_CPLMDATA_I 95:0
`define TLP_CPLHDR_W 96

`define TLP_CPLBYTECNT_I 32
`define TLP_CPLBYTECNT_W `BYTECNT_W
`define TLP_CPLBYTECNT_R `TLP_CPLBYTECNT_I +: `TLP_CPLBYTECNT_W 

`define TLP_CPLBCM_I 44
`define TLP_CPLBCM_W 1
`define TLP_CPLBCM_R `TLP_CPLBCM_I +: `TLP_CPLBCM_W 

`define TLP_CPLSTAT_I 45
`define TLP_CPLSTAT_W `STAT_W
`define TLP_CPLSTAT_R `TLP_CPLSTAT_I +: `TLP_CPLSTAT_W

`define TLP_CPLLEN_I 0
`define TLP_CPLLEN_W `LEN_W
`define TLP_CPLLEN_R `TLP_CPLLEN_I +: `TLP_CPLLEN_W

`define TLP_CPLCPLID_I 48
`define TLP_CPLCPLID_W `CPLID_W
`define TLP_CPLCPLID_R `TLP_CPLCPLID_I +: `TLP_CPLCPLID_W

`define TLP_CPLADDR_I 64
`define TLP_CPLADDR_W `LOWADDR_W
`define TLP_CPLADDR_R `TLP_CPLADDR_I +: `TLP_CPLADDR_W

`define TLP_CPLRSVD0_I 71
`define TLP_CPLRSVD0_W 1
`define TLP_CPLRSVD0_R `TLP_CPLRSVD0_I +: `TLP_CPLRSVD0_W

`define TLP_CPLTAG_I 72
`define TLP_CPLTAG_W `TAG_W
`define TLP_CPLTAG_R `TLP_CPLTAG_I +: `TLP_CPLTAG_W

`define TLP_CPLREQID_I 80
`define TLP_CPLREQID_W `REQID_W
`define TLP_CPLREQID_R `TLP_CPLREQID_I +: `TLP_CPLREQID_W

`define TLP_CPLFBE_W `FBE_W
`define TLP_CPLLBE_W `LBE_W

// Values
`define TLP_4DWH_V 1'b1
`define TLP_3DWH_V (~`TLP_4DWH_V)
`define TLP_NODIGEST_V (`TLP_TD_W'b0)
`define TLP_CPL_SUCCESSFUL_V `TLP_CPLSTAT_W'd0;

// Decoding the format & Type field
`define TLP_TYPE_REQ 5'b00000
`define TLP_TYPE_CPL 5'b01010
`define TLP_TYPE_MSG 5'b10001

`define TLP_REQ_RD {{3'bx0x},`TLP_TYPE_REQ}
`define TLP_REQ_WR {{3'bx1x},`TLP_TYPE_REQ}
`define TLP_CPL_ND {{3'bx00},`TLP_TYPE_CPL}
`define TLP_CPL_WD {{3'bx10},`TLP_TYPE_CPL}
`define TLP_MSG_ND {{3'bx01},`TLP_TYPE_MSG}
`define TLP_MSG_WD {{3'bx11},`TLP_TYPE_MSG}

function [ `EXT_TYPE_W - 1 : 0 ] tlp_to_trellis_type;
    input [`FMT_W + `TYPE_W - 1 : 0 ] tlp_type;
    begin
        /* verilator lint_off CASEX */
        casex(tlp_type)
            `TLP_REQ_RD: tlp_to_trellis_type = `TRLS_REQ_RD;
            `TLP_REQ_WR: tlp_to_trellis_type = `TRLS_REQ_WR;
            `TLP_CPL_ND: tlp_to_trellis_type = `TRLS_CPL_ND;
            `TLP_CPL_WD: tlp_to_trellis_type = `TRLS_CPL_WD;
            `TLP_MSG_ND: tlp_to_trellis_type = `TRLS_MSG_ND; // We only use messages routed by address
            `TLP_MSG_WD: tlp_to_trellis_type = `TRLS_MSG_WD; // We only use messages routed by address
            default:     tlp_to_trellis_type = `TRLS_REQ_RD;
        endcase
        /* verilator lint_on CASEX */
   end
endfunction // if

function [`FMT_W + `TYPE_W - 1 : 0 ] trellis_to_tlp_type;
    input  [`EXT_TYPE_W - 1 : 0 ]    trellis_type;
    input                            hdrlen_wr;
    reg [`FMT_W + `TYPE_W - 1 : 0 ]  temp;
    begin
        /* verilator lint_off CASEX */
        casex(trellis_type)
            `TRLS_REQ_RD: temp = `TLP_REQ_RD;
            `TRLS_REQ_WR: temp = `TLP_REQ_WR;
            `TRLS_CPL_ND: temp = `TLP_CPL_ND;
            `TRLS_CPL_WD: temp = `TLP_CPL_WD;
            `TRLS_MSG_ND: temp = `TLP_MSG_ND; // We only use messages routed by address
            `TRLS_MSG_WD: temp = `TLP_MSG_WD; // We only use messages routed by address
            default:      temp = `TLP_REQ_RD;
        endcase
        /* verilator lint_on CASEX */
        trellis_to_tlp_type = {temp[`FMT_W +`TYPE_W - 1 : `TYPE_W + 1], hdrlen_wr, temp[`TYPE_W-1:0]};
    end
endfunction // if
`endif

