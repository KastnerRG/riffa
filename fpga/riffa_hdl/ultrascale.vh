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
// Filename:            ultrascale.vh
// Version:             1.0
// Verilog Standard:    Verilog-2001
// Description:         The ultrascale.vh file is a header file that defines
// various AXI packet primitives for the Xilinx Gen3 PCIE interface
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
// TODO: STANDARDIZE NAMES
// Ultrascale Request specific fields
`ifndef __ULTRASCALE_VH
`define __ULTRASCALE_VH 1
`include "widths.vh"
`include "types.vh"

`define SIG_CQ_TUSER_W 85
`define SIG_RC_TUSER_W 75
`define SIG_CC_TUSER_W 33
`define SIG_RQ_TUSER_W 60

`define UPKT_ATYPE_W 2
`define UPKT_ADDR_W 62
`define UPKT_LEN_W (`LEN_W + 1)
`define UPKT_TYPE_W 4
`define UPKT_IDEN_W 1
`define UPKT_TC_W `TC_W
`define UPKT_ATTR_W 3
`define UPKT_TARGET_FUNCTION_W 8
`define UPKT_BARID_W 3
`define UPKT_BARSIZE_W 6

`define UPKT_RXC_MAXHDR_W 96
`define UPKT_RXR_MAXHDR_W 128
`define UPKT_TXC_MAXHDR_W 96
`define UPKT_TXR_MAXHDR_W 128

// TXR Defines
`define UPKT_TXR_ATYPE_W `UPKT_ATYPE_W
`define UPKT_TXR_ATYPE_I 0
`define UPKT_TXR_ATYPE_R `UPKT_TXR_ATYPE_I +: `UPKT_TXR_ATYPE_W

`define UPKT_TXR_ADDR_W `UPKT_ADDR_W
`define UPKT_TXR_ADDR_I 2
`define UPKT_TXR_ADDR_R `UPKT_TXR_ADDR_I +: `UPKT_TXR_ADDR_W

`define UPKT_TXR_LENGTH_W `UPKT_LEN_W
`define UPKT_TXR_LENGTH_I 64
`define UPKT_TXR_LENGTH_R `UPKT_TXR_LENGTH_I +: `UPKT_TXR_LENGTH_W

`define UPKT_TXR_TYPE_W `UPKT_TYPE_W
`define UPKT_TXR_TYPE_I 75
`define UPKT_TXR_TYPE_R `UPKT_TXR_TYPE_I +: `UPKT_TXR_TYPE_W

`define UPKT_TXR_EP_W `EP_W 
`define UPKT_TXR_EP_I 79
`define UPKT_TXR_EP_R `UPKT_TXR_EP_I +: `UPKT_TXR_EP_W

`define BE_HACK 1
`ifndef BE_HACK

`define UPKT_TXR_REQID_W `REQID_W
`define UPKT_TXR_REQID_I 80
`define UPKT_TXR_REQID_R `UPKT_TXR_REQID_I +: `UPKT_TXR_REQID_W

`else 

`define UPKT_TXR_FBE_W `FBE_W
`define UPKT_TXR_FBE_I 80
`define UPKT_TXR_FBE_R `UPKT_TXR_FBE_I +: `UPKT_TXR_FBE_W

`define UPKT_TXR_LBE_W `LBE_W
`define UPKT_TXR_LBE_I 84
`define UPKT_TXR_LBE_R `UPKT_TXR_LBE_I +: `UPKT_TXR_LBE_W

`define UPKT_TXR_RSVD0_W 8
`define UPKT_TXR_RSVD0_I 88
`define UPKT_TXR_RSVD0_R `UPKT_TXR_RSVD0_I +: `UPKT_TXR_RSVD0_W

`endif

`define UPKT_TXR_TAG_W `TAG_W
`define UPKT_TXR_TAG_I 96
`define UPKT_TXR_TAG_R `UPKT_TXR_TAG_I +: `UPKT_TXR_TAG_W

`define UPKT_TXR_CPLID_W `CPLID_W
`define UPKT_TXR_CPLID_I 104
`define UPKT_TXR_CPLID_R `UPKT_TXR_CPLID_I +: `UPKT_TXR_CPLID_W

`define UPKT_TXR_REQIDEN_W 1
`define UPKT_TXR_REQIDEN_I 120
`define UPKT_TXR_REQIDEN_R `UPKT_TXR_REQIDEN_I +: `UPKT_TXR_REQIDEN_W

`define UPKT_TXR_TC_W `UPKT_TC_W
`define UPKT_TXR_TC_I 121
`define UPKT_TXR_TC_R `UPKT_TXR_TC_I +: `UPKT_TXR_TC_W

`define UPKT_TXR_ATTR_W `UPKT_ATTR_W
`define UPKT_TXR_ATTR_I 124
`define UPKT_TXR_ATTR_R `UPKT_TXR_ATTR_I +: `UPKT_TXR_ATTR_W

`define UPKT_TXR_TD_W `TD_W
`define UPKT_TXR_TD_I 127
`define UPKT_TXR_TD_R `UPKT_TXR_TD_I +: `UPKT_TXR_TD_W

// RXR Defines
`define UPKT_RXR_ATYPE_W `UPKT_ATYPE_W
`define UPKT_RXR_ATYPE_I 0
`define UPKT_RXR_ATYPE_R `UPKT_RXR_ATYPE_I +: `UPKT_RXR_ATYPE_W

`define UPKT_RXR_ADDR_W `UPKT_ADDR_W
`define UPKT_RXR_ADDR_I 2 
`define UPKT_RXR_ADDR_R `UPKT_RXR_ADDR_I +: `UPKT_RXR_ADDR_W

`define UPKT_RXR_LENGTH_W `UPKT_LEN_W
`define UPKT_RXR_LENGTH_I 64
`define UPKT_RXR_LENGTH_R `UPKT_RXR_LENGTH_I +: `UPKT_RXR_LENGTH_W

`define UPKT_RXR_TYPE_W `UPKT_TYPE_W
`define UPKT_RXR_TYPE_I 75
`define UPKT_RXR_TYPE_R `UPKT_RXR_TYPE_I +: `UPKT_RXR_TYPE_W

`define UPKT_RXR_EP_W `EP_W 
`define UPKT_RXR_EP_I 79
`define UPKT_RXR_EP_R `UPKT_RXR_EP_I +: `UPKT_RXR_EP_W

`define UPKT_RXR_REQID_W `REQID_W
`define UPKT_RXR_REQID_I 80
`define UPKT_RXR_REQID_R `UPKT_RXR_REQID_I +: `UPKT_RXR_REQID_W

`define UPKT_RXR_TAG_W `TAG_W
`define UPKT_RXR_TAG_I 96
`define UPKT_RXR_TAG_R `UPKT_RXR_TAG_I +: `UPKT_RXR_TAG_W

`define UPKT_RXR_TGTFN_W `UPKT_TARGET_FUNCTION_W
`define UPKT_RXR_TGTFN_I 104
`define UPKT_RXR_TGTFN_R `UPKT_RXR_TGTFN_I +: `UPKT_RXR_TGTFN_W

`define UPKT_RXR_BARID_W `UPKT_BARID_W
`define UPKT_RXR_BARID_I 112
`define UPKT_RXR_BARID_R `UPKT_RXR_BARID_I +: `UPKT_RXR_BARID_W

`define UPKT_RXR_BARSIZE_W `UPKT_BARSIZE_W
`define UPKT_RXR_BARSIZE_I 115
`define UPKT_RXR_BARSIZE_R `UPKT_RXR_BARSIZE_I +: `UPKT_RXR_BARSIZE_W

`define UPKT_RXR_TC_W `UPKT_TC_W
`define UPKT_RXR_TC_I 121
`define UPKT_RXR_TC_R `UPKT_RXR_TC_I +: `UPKT_RXR_TC_W

`define UPKT_RXR_ATTR_W `UPKT_ATTR_W
`define UPKT_RXR_ATTR_I 124
`define UPKT_RXR_ATTR_R `UPKT_RXR_ATTR_I +: `UPKT_RXR_ATTR_W

`define UPKT_RXR_TD_W `UPKT_TD_W
`define UPKT_RXR_TD_I 127
`define UPKT_RXR_TD_R `UPKT_RXR_TD_I +: `UPKT_RXR_TD_W

`define UPKT_RXR_ADDRDW0_I 0
`define UPKT_RXR_ADDRDW1_I 32
`define UPKT_RXR_METADW0_I 64
`define UPKT_RXR_METADW1_I 96
`define UPKT_RXR_PAYLOAD_I 128

// Indicies in M_AXIS_CQ_TUSER
`define UPKT_CQ_TUSER_SOP_I 40
`define UPKT_CQ_TUSER_SOP_W 1
`define UPKT_CQ_TUSER_SOP_R `UPKT_CQ_TUSER_SOP_I +: `UPKT_CQ_TUSER_SOP_W

`define UPKT_CQ_TUSER_BE_I 0
`define UPKT_CQ_TUSER_BE_W 8
`define UPKT_CQ_TUSER_BE_R `UPKT_CQ_TUSER_BE_I +: `UPKT_CQ_TUSER_BE_W

// RXC Fields
`define UPKT_RXC_ADDRLOW_W 12 // BYTE ADDRESS!!!
`define UPKT_RXC_ADDRLOW_I 0
`define UPKT_RXC_ADDRLOW_R `UPKT_RXC_ADDRLOW_I +: `UPKT_RXC_ADDRLOW_W

`define UPKT_RXC_ERRCODE_W 4
`define UPKT_RXC_ERRCODE_I 12
`define UPKT_RXC_ERRCODE_R `UPKT_RXC_ERRCODE_I +: `UPKT_RXC_ERRCODE_W

`define UPKT_RXC_BYTECNT_W 13
`define UPKT_RXC_BYTECNT_I 16
`define UPKT_RXC_BYTECNT_R `UPKT_RXC_BYTECNT_I +: `UPKT_RXC_BYTECNT_W

`define UPKT_RXC_LOCKED_W 1 // Same as Type[0]
`define UPKT_RXC_LOCKED_I 29
`define UPKT_RXC_LOCKED_R `UPKT_RXC_LOCKED_I +: `UPKT_RXC_LOCKED_W

`define UPKT_RXC_COMPLETE_W 1
`define UPKT_RXC_COMPLETE_I 30
`define UPKT_RXC_COMPLETE_R `UPKT_RXC_COMPLETE_I +: `UPKT_RXC_COMPLETE_W

`define UPKT_RXC_LENGTH_W 11
`define UPKT_RXC_LENGTH_I 32
`define UPKT_RXC_LENGTH_R `UPKT_RXC_LENGTH_I +: `UPKT_RXC_LENGTH_W

`define UPKT_RXC_STATUS_W 3
`define UPKT_RXC_STATUS_I 43
`define UPKT_RXC_STATUS_R `UPKT_RXC_STATUS_I +: `UPKT_RXC_STATUS_W

`define UPKT_RXC_EP_W 1
`define UPKT_RXC_EP_I 46
`define UPKT_RXC_EP_R `UPKT_RXC_EP_I +: `UPKT_RXC_EP_W

`define UPKT_RXC_REQID_W 16
`define UPKT_RXC_REQID_I 48
`define UPKT_RXC_REQID_R `UPKT_RXC_REQID_I +: `UPKT_RXC_REQID_W

`define UPKT_RXC_TAG_W 8
`define UPKT_RXC_TAG_I 64
`define UPKT_RXC_TAG_R `UPKT_RXC_TAG_I +: `UPKT_RXC_TAG_W

`define UPKT_RXC_CPLID_W 16
`define UPKT_RXC_CPLID_I 72
`define UPKT_RXC_CPLID_R `UPKT_RXC_CPLID_I +: `UPKT_RXC_CPLID_W

`define UPKT_RXC_TC_W `UPKT_TC_W
`define UPKT_RXC_TC_I 89
`define UPKT_RXC_TC_R `UPKT_RXC_TC_I +: `UPKT_RXC_TC_W

`define UPKT_RXC_ATTR_W 3
`define UPKT_RXC_ATTR_I 92
`define UPKT_RXC_ATTR_R `UPKT_RXC_ATTR_I +: `UPKT_RXC_ATTR_W

`define UPKT_RXC_METADW0_I 0
`define UPKT_RXC_METADW1_I 32
`define UPKT_RXC_METADW2_I 64
`define UPKT_RXC_PAYLOAD_I 96 // Payload DW0

// Indicies in M_AXIS_RC_TUSER
`define UPKT_RC_TUSER_SOP_I 32
`define UPKT_RC_TUSER_SOP_W 1
`define UPKT_RC_TUSER_SOP_R `UPKT_RC_TUSER_SOP_I +: `UPKT_RC_TUSER_SOP_W

// TXC Defines
`define UPKT_TXC_ADDRLOW_W 7
`define UPKT_TXC_ADDRLOW_I 0
`define UPKT_TXC_ADDRLOW_R `UPKT_TXC_ADDRLOW_I +: `UPKT_TXC_ADDRLOW_W

`define UPKT_TXC_RSVD0_W 1
`define UPKT_TXC_RSVD0_I 7
`define UPKT_TXC_RSVD0_R `UPKT_TXC_RSVD0_I +: `UPKT_TXC_RSVD0_W

`define UPKT_TXC_ATYPE_W 2
`define UPKT_TXC_ATYPE_I 8
`define UPKT_TXC_ATYPE_R `UPKT_TXC_ATYPE_I +: `UPKT_TXC_ATYPE_W

`define UPKT_TXC_RSVD1_W 6
`define UPKT_TXC_RSVD1_I 10
`define UPKT_TXC_RSVD1_R `UPKT_TXC_RSVD1_I +: `UPKT_TXC_RSVD1_W

`define UPKT_TXC_BYTECNT_W 13
`define UPKT_TXC_BYTECNT_I 16
`define UPKT_TXC_BYTECNT_R `UPKT_TXC_BYTECNT_I +: `UPKT_TXC_BYTECNT_W

`define UPKT_TXC_LOCKED_W 1
`define UPKT_TXC_LOCKED_I 29
`define UPKT_TXC_LOCKED_R `UPKT_TXC_LOCKED_I +: `UPKT_TXC_LOCKED_W

`define UPKT_TXC_RSVD2_W 2
`define UPKT_TXC_RSVD2_I 30
`define UPKT_TXC_RSVD2_R `UPKT_TXC_RSVD2_I +: `UPKT_TXC_RSVD2_W

`define UPKT_TXC_LENGTH_W 11
`define UPKT_TXC_LENGTH_I 32
`define UPKT_TXC_LENGTH_R `UPKT_TXC_LENGTH_I +: `UPKT_TXC_LENGTH_W

`define UPKT_TXC_STATUS_W 3
`define UPKT_TXC_STATUS_I 43
`define UPKT_TXC_STATUS_R `UPKT_TXC_STATUS_I +: `UPKT_TXC_STATUS_W

`define UPKT_TXC_EP_W 1
`define UPKT_TXC_EP_I 46
`define UPKT_TXC_EP_R `UPKT_TXC_EP_I +: `UPKT_TXC_EP_W

`define UPKT_TXC_RSVD3_W 1
`define UPKT_TXC_RSVD3_I 47
`define UPKT_TXC_RSVD3_R `UPKT_TXC_RSVD3_I +: `UPKT_TXC_RSVD3_W

`define UPKT_TXC_REQID_W 16
`define UPKT_TXC_REQID_I 48
`define UPKT_TXC_REQID_R `UPKT_TXC_REQID_I +: `UPKT_TXC_REQID_W

`define UPKT_TXC_TAG_W 8
`define UPKT_TXC_TAG_I 64
`define UPKT_TXC_TAG_R `UPKT_TXC_TAG_I +: `UPKT_TXC_TAG_W

`define UPKT_TXC_CPLID_W 16
`define UPKT_TXC_CPLID_I 72
`define UPKT_TXC_CPLID_R `UPKT_TXC_CPLID_I +: `UPKT_TXC_CPLID_W

`define UPKT_TXC_CPLIDEN_W 1
`define UPKT_TXC_CPLIDEN_I 88
`define UPKT_TXC_CPLIDEN_R `UPKT_TXC_CPLIDEN_I +: `UPKT_TXC_CPLIDEN_W

`define UPKT_TXC_TC_W `UPKT_TC_W
`define UPKT_TXC_TC_I 89
`define UPKT_TXC_TC_R `UPKT_TXC_TC_I +: `UPKT_TXC_TC_W

`define UPKT_TXC_ATTR_W 3
`define UPKT_TXC_ATTR_I 92
`define UPKT_TXC_ATTR_R `UPKT_TXC_ATTR_I +: `UPKT_RXC_ATTR_W

`define UPKT_TXC_TD_W 1
`define UPKT_TXC_TD_I 95
`define UPKT_TXC_TD_R `UPKT_TXC_TD_I +: `UPKT_TXC_TD_W
                         
// Decoding the type field
`define UPKT_REQ_RD 4'b0000
`define UPKT_REQ_WR 4'b0001
`define UPKT_MSG 4'b1100

function [ `EXT_TYPE_W - 1: 0 ] upkt_to_trellis_type;
    input [`UPKT_TYPE_W : 0 ] WR_UPKT_TYPE;
    begin
        /* verilator lint_off CASEX */
        casex(WR_UPKT_TYPE)
            {`UPKT_REQ_RD,1'bx}: upkt_to_trellis_type = `TRLS_REQ_RD;
            {`UPKT_REQ_WR,1'bx}: upkt_to_trellis_type = `TRLS_REQ_WR;
            {`UPKT_MSG   ,1'b0}: upkt_to_trellis_type = `TRLS_MSG_ND;
            {`UPKT_MSG   ,1'b1}: upkt_to_trellis_type = `TRLS_MSG_WD;
            default:             upkt_to_trellis_type = `TRLS_REQ_RD;
        endcase
        /* verilator lint_on CASEX */
    end
endfunction // if

function [`UPKT_TYPE_W - 1 : 0 ] trellis_to_upkt_type;
    input  [ `EXT_TYPE_W - 1 : 0 ] trellis_type;
    begin
        /* verilator lint_off CASEX */
        casex(trellis_type)
            `TRLS_REQ_RD : trellis_to_upkt_type = `UPKT_REQ_RD;
            `TRLS_REQ_WR : trellis_to_upkt_type = `UPKT_REQ_WR;
            `TRLS_MSG_ND : trellis_to_upkt_type = `UPKT_MSG; // We only use messages routed by address
            `TRLS_MSG_WD : trellis_to_upkt_type = `UPKT_MSG; // We only use messages routed by address
            default      : trellis_to_upkt_type = `UPKT_REQ_RD;
        endcase
        /* verilator lint_on CASEX */
    end
endfunction // if

`endif


