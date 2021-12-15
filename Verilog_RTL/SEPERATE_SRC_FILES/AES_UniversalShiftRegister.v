
/////////////////////////////////////////////USHR MODULE////////////////////////////////////////////////////////
 

module AES_USHR_CUSTOM(CLOCK , RESET,Serial_out_left,Serial_out_right, Serial_in_L, Serial_in_R, Parallel_in, Parallel_out,control_s, USHR_mode0,USHR_mode1,USHR_mode2,USHR_mode3);
input          CLOCK;
input          RESET;
input  [31:0]  Serial_in_L;
input  [31:0]  Serial_in_R;
input  [127:0] Parallel_in;
input  [1:0]   USHR_mode0,USHR_mode1,USHR_mode2,USHR_mode3;
output [127:0] Parallel_out;
output [31:0] Serial_out_left;

output [31:0] Serial_out_right;
reg [31:0] reg_0, reg_1, reg_2, reg_3;
input [15:0] control_s;
wire [31:0] d_out0, d_out1, d_out2, d_out3;
wire [31:0] td_out0, td_out1, td_out2, td_out3;

assign Serial_out_left = reg_0;
assign Serial_out_right = reg_3;

mux4x1 m40 (.d_in0(reg_0), .d_in1(Serial_in_R), .d_in2(reg_1), .d_in3(Parallel_in[127:96]), .d_out(td_out0), .select(USHR_mode0));
mux4x1 m41 (.d_in0(reg_1), .d_in1(reg_0), .d_in2(reg_2), .d_in3(Parallel_in[95:64]), .d_out(td_out1), .select(USHR_mode1));
mux4x1 m42 (.d_in0(reg_2), .d_in1(reg_1), .d_in2(reg_3), .d_in3(Parallel_in[63:32]), .d_out(td_out2), .select(USHR_mode2));
mux4x1 m43 (.d_in0(reg_3), .d_in1(reg_2), .d_in2(Serial_in_L), .d_in3(Parallel_in[31:0]), .d_out(td_out3), .select(USHR_mode3));

mux2x1 m200 (.d_in0( reg_0[31:24] ), .d_in1(td_out0[31:24]), .d_out(d_out0[31:24]), .select(control_s[0]));
mux2x1 m201 (.d_in0( reg_0[23:16] ), .d_in1(td_out0[23:16]), .d_out(d_out0[23:16]), .select(control_s[1]));
mux2x1 m202 (.d_in0( reg_0[15:8] ), .d_in1(td_out0[15:8]), .d_out(d_out0[15:8]),    .select(control_s[2]));
mux2x1 m203 (.d_in0( reg_0[7:0] ), .d_in1(td_out0[7:0]), .d_out(d_out0[7:0]),       .select(control_s[3]));

mux2x1 m210 (.d_in0( reg_1[31:24] ), .d_in1(td_out1[31:24]), .d_out(d_out1[31:24]), .select(control_s[4]));
mux2x1 m211 (.d_in0( reg_1[23:16] ), .d_in1(td_out1[23:16]), .d_out(d_out1[23:16]), .select(control_s[5]));
mux2x1 m212 (.d_in0( reg_1[15:8] ), .d_in1(td_out1[15:8]), .d_out(d_out1[15:8]),     .select(control_s[6]));
mux2x1 m213 (.d_in0( reg_1[7:0] ), .d_in1(td_out1[7:0]), .d_out(d_out1[7:0]),       .select(control_s[7]));

mux2x1 m220 (.d_in0( reg_2[31:24] ), .d_in1(td_out2[31:24]), .d_out(d_out2[31:24]), .select(control_s[8]));
mux2x1 m221 (.d_in0( reg_2[23:16] ), .d_in1(td_out2[23:16]), .d_out(d_out2[23:16]), .select(control_s[9]));
mux2x1 m222 (.d_in0( reg_2[15:8] ), .d_in1(td_out2[15:8]), .d_out(d_out2[15:8]),     .select(control_s[10]));
mux2x1 m223 (.d_in0( reg_2[7:0] ), .d_in1(td_out2[7:0]),    .d_out(d_out2[7:0]),       .select(control_s[11]));

mux2x1 m230 (.d_in0( reg_3[31:24] ), .d_in1(td_out3[31:24]), .d_out(d_out3[31:24]), .select(control_s[12]));
mux2x1 m231 (.d_in0( reg_3[23:16] ), .d_in1(td_out3[23:16]), .d_out(d_out3[23:16]), .select(control_s[13]));
mux2x1 m232 (.d_in0( reg_3[15:8] ), .d_in1(td_out3[15:8]),  .d_out(d_out3[15:8]),    .select(control_s[14]));
mux2x1 m233 (.d_in0( reg_3[7:0] ), .d_in1(td_out3[7:0]),    .d_out(d_out3[7:0]),       .select(control_s[15]));

always@ (posedge CLOCK or negedge RESET)
begin
    if(!RESET) begin reg_0 <= 32'b0; reg_1 <= 32'b0; reg_2 <= 32'b0; reg_3 <= 32'b0; end
    else begin 
         reg_0 <= d_out0; reg_1 <= d_out1; reg_2 <= d_out2; reg_3 <= d_out3; 
    end
end

assign Parallel_out = {reg_0, reg_1, reg_2, reg_3};

endmodule
