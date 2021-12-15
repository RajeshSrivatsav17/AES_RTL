
/////////////////////////////////////////////////////SBOX DATA PATH PIPELINED //////////////////////////////////////////////////

module AES_SBOX_PIPELINED(a,
b,
E_D,
s_out0,
s_out1,
clk,
random);

        ///////             OUTPUT           ////////////

output [7:0]s_out0;
output [7:0]s_out1;

      ///////////      INPUTS      //////////////
        
input E_D;
input clk;
input [7:0]a;
input [7:0]b;
input [27:0] random;

            //////////      WIRE        ///////////
        
wire [27:0]out1,out2,out3;
wire [7:0] dataout0,dataout1,dataout2,dataout3,dataout_0,dataout_1;
wire [3:0]addout1,addout2,addout3,addout4,addout5,addout6,addout7,addout8,addout9,addout10,addout11,addout12;
wire [3:0]sqout1,sqout2;
wire [3:0]cmulout1,cmulout2,c0,c1,c2,c3;
wire [3:0]invo0,invo1,invo2,invo3,invo4,invo5,invo6,invo7;
wire [7:0]cout0,cout1,cout2,cout3,add8out0,add8out1,isoout0,isoout1,invisoout0,invisoout1,invisoout2,invisoout3;
wire [7:0]a,b;

wire [7:0] wire_in_s_1; 
wire [7:0] wire_in_s_2; 
wire [7:0] wire_in_inv_s_1;
wire [7:0] wire_in_inv_s_2;
wire [7:0] wire_out_s_1;
wire [7:0] wire_out_s_2;
wire [7:0] wire_out_s_3;
wire [7:0] wire_out_s_4;
wire [7:0] wire_out_inv_s_1;
wire [7:0] wire_out_inv_s_2;
wire [7:0] wire_out_inv_s_3;
wire [7:0] wire_out_inv_s_4;

                //////////      REG         //////////
                
reg [3:0] reg0_0;
reg [3:0] reg0_1;
reg [3:0] reg0_2;
reg [3:0] reg0_3;
reg [3:0] reg0_4;
reg [3:0] reg0_5;
reg [3:0] reg0_6;
reg [3:0] reg0_7;
reg [3:0] reg1_0;
reg [3:0] reg1_1;
reg [3:0] reg1_2;
reg [3:0] reg1_3;
reg [3:0] reg1_4;
reg [3:0] reg1_5;
reg [3:0] reg1_6;
reg [3:0] reg1_7;
reg [3:0] reg1_8;
reg [3:0] reg1_9;
reg [3:0] reg1_10;
reg [3:0] reg1_11;


assign out1 = random; //[31:4];
assign out2 = random; //[31:4];
assign out3 = random; //[31:4];

assign wire_in_s_1 = E_D ? a : dataout_0[7:0];
assign wire_in_s_2 = E_D ? b : dataout_1[7:0];
assign wire_in_inv_s_1 = E_D ? 8'bzz : a;
assign wire_in_inv_s_2 = E_D ? 8'bzz : b;
  

invafftran iaf1(.dout(dataout_0[7:0]),.din(wire_in_inv_s_1));
invafftran_new iaf2(.dout(dataout_1[7:0]),.din(wire_in_inv_s_2));


isomap i1(.datain(wire_in_s_1),.dataout(isoout0[7:0]));
isomap i2(.datain(wire_in_s_2),.dataout(isoout1[7:0]));
adder R0(.dataout(addout1),.d1(isoout0[7:4]),.d2(isoout1[3:0]));
adder R1(.dataout(addout2),.d1(isoout0[3:0]),.d2(isoout1[7:4]));
square4 S0(.dataout(sqout1),.datain(isoout0[7:4]));
square4 S1(.dataout(sqout2),.datain(isoout1[7:4]));
cmul C1(.dataout(cmulout1),.datain(sqout1));
cmul C2(.dataout(cmulout2),.datain(sqout2));
mult M1(.r0(out3[3:0]),.r1(out3[7:4]),.r2(out3[11:8]),.am0(addout1),.am1(addout2),.bm0(isoout0[3:0]),.bm1(isoout1[3:0]),.cm0(c0),.cm1(c1),.cm2(c2),.cm3(c3));
adder R2(.dataout(addout3),.d1(c1),.d2(cmulout1));
adder R3(.dataout(addout4),.d1(c2),.d2(cmulout2));

always @ (posedge clk)
begin
reg0_0 <= isoout0[7:4];
reg0_1 <= addout1[3:0];
reg0_2 <= addout3;
reg0_3 <= addout4;
reg0_4 <= c3;
reg0_5 <= c0;
reg0_6 <= isoout1[7:4];
reg0_7 <= addout2[3:0];
end

always @ (posedge clk)
begin
reg1_0 <= reg0_0;
reg1_1 <= reg0_1;
reg1_2 <= reg0_6;
reg1_3 <= reg0_7;
reg1_4 <= invo0;
reg1_5 <= invo1;
reg1_6 <= invo2;
reg1_7 <= invo3;
reg1_8 <= invo4;
reg1_9 <= invo5;
reg1_10 <= invo6;
reg1_11 <= invo7;
end

adder R4(.dataout(addout5),.d1(reg0_5),.d2(reg0_2));
adder R5(.dataout(addout6),.d1(reg0_3),.d2(reg0_4));

mulinv4_corrected I(.x0(addout5),.x1(addout6),.y0(invo0),.y1(invo1),.y2(invo2),.y3(invo3),.y4(invo4),.y5(invo5),.y6(invo6),.y7(invo7));

adder R6(.dataout(addout7),.d1(reg1_4^out2[3:0]),.d2(reg1_5^out2[7:4]));
adder R7(.dataout(addout8),.d1(reg1_6^out2[11:8]),.d2(addout7));
adder R8(.dataout(addout9),.d1(reg1_7^out2[15:12]),.d2(addout8));

adder R9(.dataout(addout10),.d1(reg1_8^out2[19:16]),.d2(reg1_9^out2[27:24]));
adder R10(.dataout(addout11),.d1(reg1_10^out2[23:20]),.d2(addout10));
adder R11(.dataout(addout12),.d1(reg1_11^out2[3:0]^out2[7:4]^out2[11:8]^out2[15:12]^out2[19:16]^out2[27:24]^out2[23:20]),.d2(addout11));


mult M2(.r0(out1[3:0]),.r1(out1[7:4]),.r2(out1[11:8]),.am0(addout9),.am1(addout12),.bm0(reg1_0),.bm1(reg1_2),.cm0(cout0[7:4]),.cm1(cout1[7:4]),.cm2(cout2[7:4]),.cm3(cout3[7:4]));
mult M3(.r0(out1[15:12]),.r1(out1[19:16]),.r2(out1[23:20]),.am0(addout9),.am1(addout12),.bm0(reg1_1),.bm1(reg1_3),.cm0(cout0[3:0]),.cm1(cout1[3:0]),.cm2(cout2[3:0]),.cm3(cout3[3:0]));

inviso iso1(.dout(invisoout0[7:0]),.din(cout0[7:0]));
inviso iso2(.dout(invisoout1[7:0]),.din(cout1[7:0]));
inviso iso3(.dout(invisoout2[7:0]),.din(cout2[7:0]));
inviso iso4(.dout(invisoout3[7:0]),.din(cout3[7:0]));

assign wire_out_s_1 = E_D ? invisoout0[7:0] : 8'bzz;
assign wire_out_s_2 = E_D ? invisoout1[7:0] : 8'bzz;
assign wire_out_s_3 = E_D ? invisoout2[7:0] : 8'bzz;
assign wire_out_s_4 = E_D ? invisoout3[7:0] : 8'bzz;


afftran af11(.dout(dataout0[7:0]),.din(wire_out_s_1));
afftran_new af12(.dout(dataout1[7:0]),.din(wire_out_s_2));
afftran_new af13(.dout(dataout2[7:0]),.din(wire_out_s_3));
afftran_new af14(.dout(dataout3[7:0]),.din(wire_out_s_4));

assign wire_out_inv_s_1 = E_D ? dataout0[7:0] : invisoout0[7:0];
assign wire_out_inv_s_2 = E_D ? dataout1[7:0] : invisoout1[7:0];
assign wire_out_inv_s_3 = E_D ? dataout2[7:0] : invisoout2[7:0];
assign wire_out_inv_s_4 = E_D ? dataout3[7:0] : invisoout3[7:0];

adder_8bit R12(.dataout(s_out0),.d1(wire_out_inv_s_1),.d2(wire_out_inv_s_2));
adder_8bit R13(.dataout(s_out1),.d1(wire_out_inv_s_3),.d2(wire_out_inv_s_4));


endmodule

//////////////////////////////////////////////////////////////////////////////////
module invafftran(dout,din);
input [7:0] din;
output [7:0] dout;

xor x1(dout[7],din[6],din[4],din[1]);
xor x2(dout[6],din[5],din[3],din[0]);
xor x3(dout[5],din[7],din[4],din[2]);
xor x4(dout[4],din[6],din[3],din[1]);
xor x5(dout[3],din[5],din[2],din[0]);
xor x6(dout[2],din[7],din[4],din[1],1'b1);
xor x7(dout[1],din[6],din[3],din[0]);
xor x8(dout[0],din[7],din[5],din[2],1'b1);


endmodule



//////////////////////////////////////////////////////////////////////////////////

module invafftran_new(dout,din);
input [7:0] din;
output [7:0] dout;

xor x1(dout[7],din[6],din[4],din[1]);
xor x2(dout[6],din[5],din[3],din[0]);
xor x3(dout[5],din[7],din[4],din[2]);
xor x4(dout[4],din[6],din[3],din[1]);
xor x5(dout[3],din[5],din[2],din[0]);
xor x6(dout[2],din[7],din[4],din[1]);
xor x7(dout[1],din[6],din[3],din[0]);
xor x8(dout[0],din[7],din[5],din[2]);

endmodule


