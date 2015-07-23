`define S_RC_IDLE 3'b001
`define S_RC_WAIT 3'b010
`define S_RC_ACTIVE 3'b100

`include "trellis.vh"
module reset_controller
    #(parameter C_RST_COUNT = 10)
    (
     input  CLK,
     input  RST_IN,

     output DONE_RST,
     output WAITING_RESET,
     output RST_OUT,
     
     input  SIGNAL_RST,
     input  WAIT_RST,
     input  NEXT_CYC_RST);

    localparam C_CLOG2_RST_COUNT = clog2s(C_RST_COUNT);
    localparam C_CEIL2_RST_COUNT = 1 << C_CLOG2_RST_COUNT;

    reg [2:0] _rState,rState;

    wire [C_CLOG2_RST_COUNT:0] wRstCount;

    assign DONE_RST = rState[0];
    assign WAITING_RESET = rState[1] & NEXT_CYC_RST;
    assign RST_OUT = rState[2];

    counter
        #(// Parameters
          .C_MAX_VALUE          (C_CEIL2_RST_COUNT),
          .C_SAT_VALUE          (C_CEIL2_RST_COUNT),
          .C_RST_VALUE          (C_CEIL2_RST_COUNT - C_RST_COUNT)
          /*AUTOINSTPARAM*/)
    rst_counter
        (// Outputs
         .VALUE                 (wRstCount),
         // Inputs
         .ENABLE                (1'b1),
         .RST_IN                (~rState[2] | RST_IN),
         /*AUTOINST*/
         // Inputs
         .CLK                   (CLK));

    always @(posedge CLK) begin
        if(RST_IN) begin
            rState <= `S_RC_ACTIVE;
        end else begin
            rState <= _rState;
        end
    end
    
    always @(*) begin
        _rState = rState;
        case(rState)
            `S_RC_IDLE:begin
                if(SIGNAL_RST & WAIT_RST) begin
                    _rState = `S_RC_WAIT;
                end else if(SIGNAL_RST) begin
                    _rState = `S_RC_ACTIVE;
                end
            end
            `S_RC_WAIT:begin
                if(NEXT_CYC_RST) begin
                    _rState = `S_RC_ACTIVE;
                end
            end
            `S_RC_ACTIVE:begin
                if(wRstCount[C_CLOG2_RST_COUNT] & ~SIGNAL_RST) begin
                    _rState = `S_RC_IDLE;
                end
            end
            default: _rState = rState;
        endcase
    end
endmodule
