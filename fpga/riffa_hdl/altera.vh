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
// Filename:            altera.vh
// Version:             1.0
// Verilog Standard:    Verilog-2001
// Description:         The altera.vh file is a header file that defines
// various ALTERA-specific primitives.
// Author:              Dustin Richmond (@darichmond)
//-----------------------------------------------------------------------------
`ifndef __ALTERA_VH
`define __ALTERA_VH 1

`define TLCFG_DEVCTL_I `SIG_CFG_ADD_W'h0 
`define TLCFG_LNKCTL_I `SIG_CFG_ADD_W'h2
`define TLCFG_PRMCMD_I `SIG_CFG_ADD_W'h3
`define TLCFG_MSICSR_I `SIG_CFG_ADD_W'hD
`define TLCFG_BUSDEV_I `SIG_CFG_ADD_W'hF

`define TLSTS_LWIDTH_R 40:35
`define TLSTS_LRATE_R 34:31

`define TLCTL_MAXREQ_R 30:28
`define TLCTL_MAXPAY_R 23:21
`define TLCTL_RCB_R 19:19
`define TLCTL_BUSMSTR_R 10:10
`define TLCTL_MSIENABLE_R 0:0
`define TLCTL_BUSDEV_R 12:0

`define SIG_CFG_STS_W 53
`define SIG_CFG_CTL_W 32
`define SIG_CFG_ADD_W 4

`define SIG_KO_CPLD_W 12
`define SIG_KO_CPLH_W 8
`endif
