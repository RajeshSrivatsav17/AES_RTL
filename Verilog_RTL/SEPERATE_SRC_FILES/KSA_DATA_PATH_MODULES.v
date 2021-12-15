

///////////////////////////////////////////////////////////////////////////////////////////

module Inverse_Mix_Columns(in, out);

    input		[31:0] in;
    //input				 E_D;
    
    
    output   [31:0] out;
    
    wire 		[31:0] wire_in , wire_out_mix, wire_out_inv_mix, wire_out;
    
    
    MixCol_KSA 		M1 	(.mix_in(in), .mix_out(wire_out_mix));
    InvMixCol_KSA 	IM1	(.s_in(in), .w_in(wire_out_mix), .out(out));

endmodule

///////////////////////////////////////////////////////////////////////////////////////////

module MixCol_KSA( mix_out, mix_in);

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

////////////////////////////////////////////////////////////////////////////////////////////

module InvMixCol_KSA(s_in, w_in, out);

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

////////////////////////////////////////////////////////////////////////////////////////////
////SBOX FOR KSA

module sbox(dataout,datain);

    input  [7:0] datain;
    output [7:0] dataout;
    wire [7:0] isoout;
    wire [3:0] sq,mulout1,cmulout,addout1,addout2,invout;
    wire [7:0] mulout2,invisoout;
    
    isomap i1(.datain(datain[7:0]),.dataout(isoout[7:0]));
    squarer s1(.datain(isoout[7:4]),.dataout(sq[3:0])); 
    adder a1(.dataout(addout1[3:0]),.d1(isoout[3:0]),.d2(isoout[7:4]));
    cmul c1(.dataout(cmulout[3:0]),.datain(sq[3:0]));
    mulinv4 m1(.dataout(mulout1[3:0]),.d1(addout1[3:0]),.d2(isoout[3:0]));
    adder a2(.dataout(addout2[3:0]),.d1(cmulout[3:0]),.d2(mulout1[3:0]));
    inv inv1(.dout(invout[3:0]),.din(addout2[3:0]));
    mulinv4 m2(.dataout(mulout2[7:4]),.d1(isoout[7:4]),.d2(invout[3:0]));
    mulinv4 m3(.dataout(mulout2[3:0]),.d1(addout1[3:0]),.d2(invout[3:0]));
    inviso iso1(.dout(invisoout[7:0]),.din(mulout2[7:0]));
    afftran af1(.dout(dataout[7:0]),.din(invisoout[7:0]));

endmodule

////////////////////////////////////////////////////////////////////////////////////////////
///SUB MODULES FOR SBOX 
////ADDER

module adder(dataout,d1,d2);
    input [3:0] d1,d2;
    output [3:0] dataout;
    
    xor x1(dataout[3],d1[3],d2[3]);
    xor x2(dataout[2],d1[2],d2[2]);
    xor x3(dataout[1],d1[1],d2[1]);
    xor x4(dataout[0],d1[0],d2[0]);
endmodule

////////////////////////////////////////////////////////////////////////////////////////////
////ADDER 8 BIT

module adder_8bit(dataout,d1,d2);
    input [7:0] d1,d2;
    output [7:0] dataout;
    
    xor x1(dataout[7],d1[7],d2[7]);
    xor x2(dataout[6],d1[6],d2[6]);
    xor x3(dataout[5],d1[5],d2[5]);
    xor x4(dataout[4],d1[4],d2[4]);
    xor x5(dataout[3],d1[3],d2[3]);
    xor x6(dataout[2],d1[2],d2[2]);
    xor x7(dataout[1],d1[1],d2[1]);
    xor x8(dataout[0],d1[0],d2[0]);
endmodule
////////////////////////////////////////////////////////////////////////////////////////////
////AFFTRAN


module afftran(dout,din);
    input [7:0] din;
    output [7:0] dout;
    
    xor x1(dout[7],din[7],din[6],din[5],din[4],din[3],1'b0);
    xor x2(dout[6],din[6],din[5],din[4],din[3],din[2],1'b1);
    xor x3(dout[5],din[5],din[4],din[3],din[2],din[1],1'b1);
    xor x4(dout[4],din[4],din[3],din[2],din[1],din[0],1'b0);
    xor x5(dout[3],din[7],din[3],din[2],din[1],din[0],1'b0);
    xor x6(dout[2],din[7],din[6],din[2],din[1],din[0],1'b0);
    xor x7(dout[1],din[7],din[6],din[5],din[1],din[0],1'b1);
    xor x8(dout[0],din[7],din[6],din[5],din[4],din[0],1'b1);


endmodule

////////////////////////////////////////////////////////////////////////////////////////////
////AFFTRAN NEW

module afftran_new(dout,din);
    input [7:0] din;
    output [7:0] dout;
    
    xor x1(dout[7],din[7],din[6],din[5],din[4],din[3]);
    xor x2(dout[6],din[6],din[5],din[4],din[3],din[2]);
    xor x3(dout[5],din[5],din[4],din[3],din[2],din[1]);
    xor x4(dout[4],din[4],din[3],din[2],din[1],din[0]);
    xor x5(dout[3],din[7],din[3],din[2],din[1],din[0]);
    xor x6(dout[2],din[7],din[6],din[2],din[1],din[0]);
    xor x7(dout[1],din[7],din[6],din[5],din[1],din[0]);
    xor x8(dout[0],din[7],din[6],din[5],din[4],din[0]);


endmodule
////////////////////////////////////////////////////////////////////////////////////////////
////CMUL
module cmul(dataout,datain);
    input [3:0] datain;
    output [3:0] dataout;
    
    xor x1(dataout[3],datain[2],datain[0]);
    xor x2(dataout[2],datain[3],datain[1],dataout[3]);
    assign dataout[1] = datain[3];
    assign dataout[0] = datain[2];

endmodule 

////////////////////////////////////////////////////////////////////////////////////////////
////INV

module inv(dout,din);
    input [3:0] din;
    output [3:0] dout;
    wire [3:0] w1,w2,w3,w4;
    squarer s1(.dataout(w1[3:0]),.datain(din[3:0]));
    squarer s2(.dataout(w2[3:0]),.datain(w1[3:0]));
    squarer s3(.dataout(w3[3:0]),.datain(w2[3:0]));
    
    mulinv4 m1(.dataout(w4[3:0]),.d1(w1[3:0]),.d2(w2[3:0]));
    mulinv4 m2(.dataout(dout[3:0]),.d1(w4[3:0]),.d2(w3[3:0]));

endmodule

////////////////////////////////////////////////////////////////////////////////////////////
///INVISO

module inviso(dout,din);
    input [7:0] din;
    output [7:0] dout;
    
    xor x1(dout[7],din[7],dout[5]);
    xor x2(dout[6],din[6],din[2]);
    xor x3(dout[5],din[6],din[5],din[1]);
    xor x4(dout[4],dout[5],din[4],din[2]);
    xor x5(dout[3],din[5],din[4],din[3],din[2],din[1]);
    xor x6(dout[2],din[7],din[4],din[3],din[2],din[1]);
    xor x7(dout[1],din[5],din[4]);
    xor x8(dout[0],din[6],din[5],din[4],din[2],din[0]);

endmodule
////////////////////////////////////////////////////////////////////////////////////////////
//////ISO MAP

module isomap(dataout,datain);
    input [7:0] datain;
    output [7:0] dataout;
    
    xor x1(dataout[7],datain[7],datain[5]);
    xor x2(dataout[6],datain[7],datain[4],datain[3],datain[2],datain[6],datain[1]);
    xor x3(dataout[5],datain[7],datain[5],datain[3],datain[2]);
    xor x4(dataout[4],datain[7],datain[5],datain[3],datain[2],datain[1]);
    xor x5(dataout[3],datain[7],datain[6],datain[2],datain[1]);
    xor x6(dataout[2],datain[7],datain[4],datain[3],datain[2],datain[1]);
    xor x7(dataout[1],datain[6],datain[4],datain[1]);
    xor x8(dataout[0],datain[6],datain[1],datain[0]);

endmodule
////////////////////////////////////////////////////////////////////////////////////////////
////MULLINV2

module mulinv2(dout,d1,d2);
    input [1:0] d1,d2;
    output [1:0] dout;
    
    xor x1(w1,d1[0],d1[1]);
    xor x2(w2,d2[0],d2[1]);
    and a1(w3,d1[1],d2[1]);
    and a2(w4,w1,w2);
    and a3(w5,d1[0],d2[0]);
    xor x3(dout[1],w4,w5);
    xor x4(dout[0],w3,w5);

endmodule

////////////////////////////////////////////////////////////////////////////////////////////
////MULLINV4
module mulinv4(dataout,d1,d2);
    input [3:0] d1,d2;
    output [3:0] dataout;
    wire [1:0] w1,w2,w3,w4,w5,w6;
    
    add2 a1(.dout(w1[1:0]),.d1(d1[1:0]),.d2(d1[3:2]));
    add2 a2(.dout(w2[1:0]),.d1(d2[1:0]),.d2(d2[3:2]));
    
    mulinv2 m1(.dout(w3[1:0]),.d1(d1[3:2]),.d2(d2[3:2]));
    mulinv2 m2(.dout(w4[1:0]),.d1(w1[1:0]),.d2(w2[1:0]));
    mulinv2 m3(.dout(w5[1:0]),.d1(d1[1:0]),.d2(d2[1:0]));
    
    smul s1(.dout(w6[1:0]),.din(w3[1:0]));
    add2 a3(.dout(dataout[3:2]),.d1(w4[1:0]),.d2(w5[1:0]));
    add2 a4(.dout(dataout[1:0]),.d1(w5[1:0]),.d2(w6[1:0]));

endmodule
////////////////////////////////////////////////////////////////////////////////////////////
////MULLINV4CORRECTED

module mulinv4_corrected(y0,y1,y2,y3,y4,y5,y6,y7,x1,x0);
    output [3:0]y0,y1,y2,y3,y4,y5,y6,y7;
    input [3:0]x0,x1;
    
    wire [3:0]x1,x0;
    
    assign y0[0]=(x0[3]&x0[2]&x0[1])^(x0[3]&x0[2]&x0[0])^(x0[3]&x0[1]&x0[0])^(x0[2]&x0[1]&x0[0])^(x0[3]&x0[1])^(x0[3]&x0[0])^(x0[2]&x0[1])^(x0[2])^(x0[1]);
    assign y1[0]=(x0[3]&x0[2]&x1[1])^(x0[3]&x0[2]&x1[0])^(x0[3]&x1[1]&x1[0])^(x0[2]&x1[1]&x1[0])^(x0[3]&x1[1])^(x0[3]&x1[0])^(x0[2]&x1[1])^(x1[1]);
    assign y2[0]=(x0[3]&x1[2]&x0[1])^(x0[3]&x1[2]&x1[0])^(x0[3]&x0[1]&x1[0])^(x1[2]&x0[1]&x1[0])^(x1[2]&x0[1])^(x1[2]);
    assign y3[0]=(x0[3]&x1[2]&x1[1])^(x0[3]&x1[2]&x0[0])^(x0[3]&x1[1]&x0[0])^(x1[2]&x1[1]&x0[0])^(x1[2]&x1[1]);
    assign y4[0]=(x1[3]&x0[2]&x0[1])^(x1[3]&x0[2]&x1[0])^(x1[3]&x0[1]&x1[0])^(x0[2]&x0[1]&x1[0])^(x1[3]&x0[1])^(x1[3]&x1[0]);
    assign y5[0]=(x1[3]&x0[2]&x1[1])^(x1[3]&x0[2]&x0[0])^(x1[3]&x1[1]&x0[0])^(x0[2]&x1[1]&x0[0])^(x1[3]&x1[1])^(x1[3]&x0[0]);
    assign y6[0]=(x1[3]&x1[2]&x0[1])^(x1[3]&x1[2]&x0[0])^(x1[3]&x0[1]&x0[0])^(x1[2]&x0[1]&x0[0])^(x0[0]);
    assign y7[0]=(x1[3]&x1[2]&x1[1])^(x1[3]&x1[2]&x1[0])^(x1[3]&x1[1]&x1[0])^(x1[2]&x1[1]&x1[0])^(x1[0]);
    
    assign y0[1]=(x0[3]&x0[2]&x0[1])^(x0[3]&x0[1]&x0[0])^(x0[2]&x0[0])^(x0[3])^(x0[2])^(x0[1]);
    assign y1[1]=(x0[3]&x0[2]&x1[1])^(x0[3]&x1[1]&x1[0])^(x0[2]&x1[0])^(x1[1]);
    assign y2[1]=(x0[3]&x1[2]&x0[1])^(x0[3]&x0[1]&x1[0])^(x1[2]&x1[0])^(x1[2]);
    assign y3[1]=(x0[3]&x1[2]&x1[1])^(x0[3]&x1[1]&x0[0])^(x1[2]&x0[0]);
    assign y4[1]=(x1[3]&x0[2]&x0[1])^(x1[3]&x0[1]&x0[0])^(x1[3]);
    assign y5[1]=(x1[3]&x0[2]&x1[1])^(x1[3]&x1[1]&x0[0]);
    assign y6[1]=(x1[3]&x1[2]&x0[1])^(x1[3]&x0[1]&x1[0]);
    assign y7[1]=(x1[3]&x1[2]&x1[1])^(x1[3]&x1[1]&x1[0]);
    
    assign y0[2]=(x0[3]&x0[2]&x0[0])^(x0[3]&x0[2]&x0[1])^(x0[3]&x0[0])^(x0[2]&x0[1])^(x0[2]);
    assign y1[2]=(x0[3]&x0[2]&x1[0])^(x0[3]&x0[2]&x1[1])^(x0[3]&x1[0])^(x0[2]&x1[1]);
    assign y2[2]=(x0[3]&x1[2]&x0[0])^(x0[3]&x1[2]&x0[1])^(x1[2]&x0[1]);
    assign y3[2]=(x0[3]&x1[2]&x1[0])^(x0[3]&x1[2]&x1[1])^(x1[2]&x1[1]);
    assign y4[2]=(x1[3]&x0[2]&x0[0])^(x1[3]&x0[2]&x0[1]);
    assign y5[2]=(x1[3]&x0[2]&x1[0])^(x1[3]&x0[2]&x1[1]);
    assign y6[2]=(x1[3]&x1[2]&x0[0])^(x1[3]&x1[2]&x0[1])^(x1[3]&x0[0]);
    assign y7[2]=(x1[3]&x1[2]&x1[0])^(x1[3]&x1[2]&x1[1])^(x1[3]&x1[0])^(x1[2]);
    
    assign y0[3]=(x0[3]&x0[2]&x0[1])^(x0[3]&x0[0]);
    assign y1[3]=(x0[3]&x0[2]&x1[1])^(x0[3]);
    assign y2[3]=(x0[3]&x1[2]&x0[1])^(x1[2]);
    assign y3[3]=(x0[3]&x1[2]&x1[1])^(x0[3]&x1[0]);
    assign y4[3]=(x1[3]&x0[2]&x0[1])^(x1[3]&x0[0]);
    assign y5[3]=(x1[3]&x0[2]&x1[1])^(x0[2]);
    assign y6[3]=(x1[3]&x1[2]&x0[1])^(x1[3]);
    assign y7[3]=(x1[3]&x1[2]&x1[1])^(x1[3]&x1[0]);


endmodule

////////////////////////////////////////////////////////////////////////////////////////////
/////MULT

module mult(am0,am1,bm0,bm1,cm0,cm1,cm2,cm3,r0,r1,r2);
    input [3:0]am0,am1,bm0,bm1;
    output [3:0]cm0,cm1,cm2,cm3;
    
    wire [3:0] multout0,multout1,multout2,multout3;
    input [3:0] r0,r1,r2;
    
    wire [3:0]r3,r4;
    
    
    mulinv4 M00(.dataout(multout0),.d1(am0),.d2(bm0));
    mulinv4 M01(.dataout(multout1),.d1(am0),.d2(bm1));
    mulinv4 M02(.dataout(multout2),.d1(am1),.d2(bm1));
    mulinv4 M03(.dataout(multout3),.d1(am1),.d2(bm0));
    
    
    adder A0(.dataout(cm0),.d1(r0),.d2(multout0));
    adder A1(.dataout(cm1),.d1(r1),.d2(multout1));
    adder A2(.dataout(cm2),.d1(r2),.d2(multout2));
    adder A3(.dataout(r3),.d1(r0),.d2(r1));
    adder A4(.dataout(r4),.d1(r3),.d2(r2));
    adder A5(.dataout(cm3),.d1(r4),.d2(multout3));
endmodule
////////////////////////////////////////////////////////////////////////////////////////////
////SMUL

module smul(dout,din);
    input [1:0] din;
    output [1:0] dout;
    
    xor x1(dout[1],din[1],din[0]);
    assign dout[0] = din[1];

endmodule
////////////////////////////////////////////////////////////////////////////////////////////
////SQUARE4

module square4(dataout,datain);
    input [3:0] datain;
    output [3:0] dataout;
    
    assign dataout[3] = datain[3];
    xor x1(dataout[2],datain[3],datain[2]);
    xor x2(dataout[1],datain[2],datain[1]);
    xor x3(dataout[0],datain[3],datain[1],datain[0]);

endmodule
////////////////////////////////////////////////////////////////////////////////////////////
////SQUARER

module squarer(dataout,datain);
    input [3:0] datain;
    output [3:0] dataout;
    
    assign dataout[3] = datain[3];
    xor x1(dataout[2],datain[3],datain[2]);
    xor x2(dataout[1],datain[2],datain[1]);
    xor x3(dataout[0],datain[3],datain[1],datain[0]);

endmodule
////////////////////////////////////////////////////////////////////////////////////////////
/////ADD2
module add2(dout,d1,d2);
    input [1:0] d1,d2;
    output [1:0] dout;
    
    xor x1(dout[1],d1[1],d2[1]);
    xor x2(dout[0],d1[0],d2[0]);

endmodule

