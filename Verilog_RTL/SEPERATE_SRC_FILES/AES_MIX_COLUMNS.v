/////////////////////////////////////MIX COLUMNS////////////////////////////////////////////////

module AES_MIX_COL (
 MC_IN,
 E_D,
 MC_COUNT_ROUND,
 MC_FINAL_ROUND_COUNT,
 MC_I_MIX_ACTIVE,
 MC_OUT);

 /////   OUTPUT //////////

output [31:0] MC_OUT;

////// INPUT /////////////

input [31:0] MC_IN;
input  E_D;
input  MC_I_MIX_ACTIVE;
input  [3:0] MC_COUNT_ROUND;
input  [3:0] MC_FINAL_ROUND_COUNT;

////  WIRES ///////////////

wire [31:0] wire_in;
wire [31:0] wire_out_mix;
wire [31:0] wire_out_inv_mix;
wire [31:0] wire_out;

                    /// CORE //////////



            //      ----> INPUT GATING    //
assign wire_in = MC_I_MIX_ACTIVE ? MC_IN : 32'bz;

            //      ----> OUTPUT LINE     //
assign MC_OUT = (MC_COUNT_ROUND==MC_FINAL_ROUND_COUNT)? MC_IN : E_D ?  wire_out_mix : wire_out_inv_mix;

            //     -----> E_D GATING    //
assign wire_out = E_D? 32'bz : wire_out_mix;

MixCol 		M1 	(.mix_in(wire_in), .mix_out(wire_out_mix));
InvMixCol 	IM1	(.s_in(wire_in), .w_in(wire_out), .out(wire_out_inv_mix));

endmodule



////////////////                     END OF TOP MODULE                          /////////////////////////


 
////////////////////////////////////     SUB-MODULES   ///////////////////////////////////////////


module MixCol( mix_out, mix_in);

input   [31:0] mix_in;
output  [31:0] mix_out;

wire    [7:0]  w0, w1, w2, w3;
wire    [7:0]  out0, out1, out2, out3;

XTime X0 (.in1(mix_in[31:24]), .in2(mix_in[23:16]), .XTout(out0), .w(w0));
XTime X1 (.in1(mix_in[23:16]), .in2(mix_in[15:8]), .XTout(out1), .w(w1));
XTime X2 (.in1(mix_in[15:8]), .in2(mix_in[7:0]), .XTout(out2), .w(w2));
XTime X3 (.in1(mix_in[7:0]), .in2(mix_in[31:24]), .XTout(out3), .w(w3));

assign mix_out[31:24]  = out0 ^ (mix_in[23:16] ^ w2);
assign mix_out[23:16]  = out1 ^ (mix_in[15:8]  ^ w3);
assign mix_out[15:8]   = out2 ^ (mix_in[7:0]   ^ w0);
assign mix_out[7:0]    = out3 ^ (mix_in[31:24] ^ w1);

endmodule

//////////////////////////////////////////////////////////////////////////////////


module XTime(in1, in2, XTout, w);

input   [7:0] in1, in2;
output  [7:0] XTout, w;

assign w = in1 ^ in2;
mixmulx2 m1 (.dout(XTout), .din(w));

endmodule
//////////////////////////////////////////////////////////////////////////////////


module mixmulx2 (dout,din);

input [7:0] din;
output [7:0] dout;

assign dout[0] = din[7];
xor x1(dout[1],din[0],din[7]);
assign dout[2] = din[1];
xor x2(dout[3],din[2],din[7]);
xor x3(dout[4],din[3],din[7]);
assign dout[5] = din[4];
assign dout[6] = din[5];
assign dout[7] = din[6];

endmodule

///////////////////////////////////////////////////////////////////////////////

module InvMixCol(s_in, w_in, out);

input 	[31:0] s_in, w_in; 

output 	[31:0] out;

wire 		[ 7:0] w0, w1, w_out, w;

X4Time X0 (.din(s_in[31:24] ^ s_in[15: 8]), .dout(w0));
X4Time X1 (.din(s_in[23:16] ^ s_in[ 7: 0]), .dout(w1));

XTime XT0 (.in1(w0), .in2(w1), .XTout(w_out), .w(w) );

assign out[31:24] = (w0 ^ w_out) ^ w_in[31:24];
assign out[23:16] = (w1 ^ w_out) ^ w_in[23:16];
assign out[15: 8] = (w0 ^ w_out) ^ w_in[15: 8];
assign out[ 7: 0] = (w1 ^ w_out) ^ w_in[ 7: 0];

endmodule

/////////////////////////////////////////////////////////////////////////////


module X4Time( input [7:0] din, output [7:0] dout);

assign dout[0] = din[6];

xor x1(dout[1],din[6],din[7]);
xor x2(dout[2],din[7],din[0]);
xor x3(dout[3],din[1],din[6]);
xor x4(dout[4],din[2],dout[1]);
xor x5(dout[5],din[7],din[3]);

assign dout[6] = din[4];
assign dout[7] = din[5];

endmodule

