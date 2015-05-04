`ifndef __TYPES_VH
`define __TYPES_VH 1

`define TRLS_REQ_RD `EXT_TYPE_W'b000 
`define TRLS_REQ_WR `EXT_TYPE_W'b001 
`define TRLS_CPL_ND `EXT_TYPE_W'b010
`define TRLS_CPL_WD `EXT_TYPE_W'b011
`define TRLS_MSG_ND `EXT_TYPE_W'b100
`define TRLS_MSG_WD `EXT_TYPE_W'b101

`define TRLS_TYPE_PAY_I 0 // Payload Bit Index. If 1, packet has a payload, else 0
`define TRLS_TYPE_CPL_I 1 // Completion Bit Index. If 1, packet is a Completion
`define TRLS_TYPE_MSG_I 2 // Message Bit Index. If 1, packet is a message

`endif
