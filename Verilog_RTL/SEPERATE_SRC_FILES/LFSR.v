//////////////////////////////////////////////////LFSR//////////////////////////////////////////////////////
module lfsr113(/*AUTOARG*/
 // Outputs
 lfsr113_prng ,
 // Inputs
 CLK, reset ,enable_p
 ) ;


    input CLK;
    input reset ,enable_p;
    output reg [31:0] lfsr113_prng;
    reg [31:0] z1, z2, z3,z4;

    reg state;
    reg next;

    parameter   CI_S0 = 1'b0,  CI_IDLE = 1'b1;

    always @(posedge CLK) begin
        if (!reset) begin
        state <= CI_IDLE;
        end else begin
        state <= next;
        end
        end

        //== Initialisation FSM one-hot encoding
        always @(*) begin
        next = CI_IDLE;
        case (state)

        CI_IDLE: begin
        if (enable_p==1'b0) begin
            next = CI_IDLE;
        end else begin
            next = CI_S0;
            end
        end
        CI_S0: begin
            next = CI_S0;
        end
        endcase
    end

    always @(posedge CLK) begin
        if (!reset) begin
        z1= 32'd987654321;
        z2= 32'd987654321;
        z3= 32'd987654321;
        z4= 32'd987654321;
        end else begin
        case (state)
        CI_IDLE : begin
            z1= z1; z2= z2;z3= z3;z4= z4;
        end
        CI_S0 : begin
            z1 = (((z1 & 32'd4294967294) << 18) ^ (((z1 << 6) ^ z1) >> 13));
            z2 = (((z2 & 32'd4294967288) << 2) ^ (((z2 << 2) ^ z2) >> 27));
            z3 = (((z3 & 32'd4294967280) << 7) ^ (((z3 << 13) ^ z3) >> 21));
            z4 = (((z4 & 32'd4294967168) << 13) ^ (((z4 << 3) ^ z4) >> 12));
        end
        endcase
        end
    end

    //assign lfsr113_prng = (state == CI_S0)? z1 ^ z2 ^ z3 ^ z4: 32'b0;
    always @(posedge CLK ) begin
    if (!reset) begin
    lfsr113_prng <= 32'b0;
    end else if (state==CI_S0) begin
    lfsr113_prng <= z1 ^ z2 ^ z3 ^ z4;
    end else begin
    lfsr113_prng <= lfsr113_prng;
    end
    end

endmodule

