

//////////////////////////////////////////////MUX 4X1 8BIT////////////////////////////////////////////////////////

module mux4x18 (d_in0, d_in1, d_in2, d_in3, d_out, select);
input [7:0]  d_in0, d_in1, d_in2, d_in3;
input [1:0]   select;
output [7:0] d_out;

reg [7:0] d_out;

always@ (*)
begin
    case (select)
        2'b00: begin d_out = d_in0; end
        2'b01: begin d_out = d_in1; end
        2'b10: begin d_out = d_in2; end
        2'b11: begin d_out = d_in3; end
    endcase
end
endmodule

//////////////////////////////////////////////MUX 2X1 32 BIT////////////////////////////////////////////////////////
module mux2x132 (d_in0, d_in1, d_out, select);
input [31:0]  d_in0, d_in1;
input   select;
output reg [31:0] d_out;

always@ (*)
begin
    case (select)
        1'b0: begin d_out = d_in0; end
        1'b1: begin d_out = d_in1; end
         endcase
end
endmodule




//////////////////////////////////////////////MUX 4X1 32 BIT////////////////////////////////////////////////////////
module mux4x1 (d_in0, d_in1, d_in2, d_in3, d_out, select);
input [31:0]  d_in0, d_in1, d_in2, d_in3;
input [1:0]   select;
output [31:0] d_out;

reg [31:0] d_out;

always@ (*)
begin
    case (select)
        2'b00: begin d_out = d_in0; end
        2'b01: begin d_out = d_in1; end
        2'b10: begin d_out = d_in2; end
        2'b11: begin d_out = d_in3; end
    endcase
end
endmodule
////////////////////////////////////////////////////////////////////////////////////////////////////
module mux2x1 (d_in0, d_in1, d_out, select);
input [7:0]  d_in0, d_in1;
input   select;
output reg [7:0] d_out;

always@ (*)
begin
    case (select)
        1'b0: begin d_out = d_in0; end
        1'b1: begin d_out = d_in1; end
         endcase
end
endmodule


//////////////////////////////////////////////////////////////////////////////////////////

module mux_8 (d_in0, d_in1, d_in2, d_in3, d_out, select);
    input [7:0]  d_in0, d_in1, d_in2, d_in3;
    input [1:0]   select;
    output [7:0] d_out;
    
    reg [7:0] d_out;
    
    always@ (*)
    begin
        case (select)
            2'b00: begin d_out = d_in0; end
            2'b01: begin d_out = d_in1; end
            2'b10: begin d_out = d_in2; end
            2'b11: begin d_out = d_in3; end
        endcase
    end
endmodule
