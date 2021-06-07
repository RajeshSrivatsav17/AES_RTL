`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// This file contains all modules consolidated in a SINGLE FILE 
// *******************************************************************************
// Engineer: Rajesh Srivatsav Suresh
// Create Date: 05.05.2019 11:56:48
// Design: AES 32-bit iterative core design with Power analysis attack countermeasures (PACM)
// Target Devices: FPGA -- Artix 7 implemented and tested using Virtual Input Output 
// Revision v1.12 - Final Design
// *******************************************************************************

/////////////////////////////////////AES_PACM_TOP/////////////////////////////////////////////
// This module is used for Tapeout purpose where we require a smaller number of IO pins 
// 8-bit read write ports are present in this module with a Memory file which holds the inputs together and sends packets of 32 bits 
// to the core AES IP. 
module AES_PACM_TOP(    IO_DATAINOUT,
						IO_R_W,
						IO_OPCODE,
						IO_CLOCK,
						IO_RESET,
						IO_START,
						IO_ADDR,
						IO_DONE );
    
	inout           [7:0]               IO_DATAINOUT; /// INOUT port for and writing inputs reading outputs in 8-bit widths 
	input           [4:0]               IO_ADDR;
	input                               IO_R_W;
	input           [3:0]               IO_OPCODE;
	input                               IO_CLOCK;
	input                               IO_RESET;
	input                               IO_START;
  //	output          [7:0]               OUT_8; // if testing in a FPGA environment, please uncomment this line and change IO_DATAINOUT as input 
	output                              IO_DONE ;
	
	wire			[7:0]	            IN_8 ;
	wire                                R_W;
	wire            [1:0]               MODE;
	wire                                E_D;
	wire                                D_K;
	wire                                CLOCK;
	wire                                RESET;
	wire                                START_KSA_DP;
	wire            [4:0]               ADDR;      //ADDRESS FOR THE DATA/KEY LOADING OR OUTPUT READING
	wire            [7:0]               OUT_8;
	wire                                DONE;
	wire            [3:0]               OPCODE;                       
	
	
	
	wire [127:0] mem_128_1,mem_128_2,data_out_128;
	
	reg [7:0] MEM_I [31:0];
	
	assign {MODE[1],MODE[0],E_D,D_K}=OPCODE;
	
	assign mem_128_1 = { MEM_I[0],MEM_I[1],MEM_I[2],MEM_I[3],MEM_I[4],MEM_I[5],MEM_I[6],MEM_I[7],
	                    MEM_I[8],MEM_I[9],MEM_I[10],MEM_I[11],MEM_I[12],MEM_I[13],MEM_I[14],MEM_I[15] };
	
	
	assign mem_128_2 = { MEM_I[16],MEM_I[17],MEM_I[18],MEM_I[19],MEM_I[20],MEM_I[21],MEM_I[22],MEM_I[23],
	                    MEM_I[24],MEM_I[25],MEM_I[26],MEM_I[27],MEM_I[28],MEM_I[29],MEM_I[30],MEM_I[31] };
	
	
	AES_IP_v1  IP_v1(
					.IN_1_128 (mem_128_1),
					.IN_2_128 (mem_128_2),
					.D_K(D_K),
					.E_D(E_D),
					.DATA_OUT_128(data_out_128),
					.CLK(CLOCK),
					.RST(RESET),
					.MODE(MODE),
					.IP_START(START_KSA_DP),
					.DATA_DONE(DONE)
	    );
	
	
	always @ (posedge CLOCK)
	begin
	
	     
	// we have a 256 bit register and a 128 biit register which is used for input and oututs respectively
	//now if we have to give inputs , R_W = 0 , and we give the address from 0-31 and give the input 8bit data
	//for output reading  R_W = 1 , and we give the address from 0 - 15 adn we can read the data when the encryption or decryption is done
	     if(~RESET)
	     begin
	     MEM_I[0]<=32'd0;
	     MEM_I[1]<=32'd0;
	     MEM_I[2]<=32'd0;
	     MEM_I[3]<=32'd0;
	     MEM_I[4]<=32'd0;
	     MEM_I[5]<=32'd0;
	     MEM_I[6]<=32'd0;
	     MEM_I[7]<=32'd0;
	     MEM_I[8]<=32'd0;
	     MEM_I[9]<=32'd0;
	     MEM_I[10]<=32'd0;
	     MEM_I[11]<=32'd0;
	     MEM_I[12]<=32'd0;
	     MEM_I[13]<=32'd0;
	     MEM_I[14]<=32'd0;
	     MEM_I[15]<=32'd0;
	     MEM_I[16]<=32'd0;
	     MEM_I[17]<=32'd0;
	     MEM_I[18]<=32'd0;
	     MEM_I[19]<=32'd0;
	     MEM_I[20]<=32'd0;
	     MEM_I[21]<=32'd0;
	     MEM_I[22]<=32'd0;
	     MEM_I[23]<=32'd0;
	     MEM_I[24]<=32'd0;
	     MEM_I[25]<=32'd0;
	     MEM_I[26]<=32'd0;
	     MEM_I[27]<=32'd0;
	     MEM_I[28]<=32'd0;
	     MEM_I[29]<=32'd0;
	     MEM_I[30]<=32'd0;
	     MEM_I[31]<=32'd0;
	     
	    
	          end
	     
	     if (RESET  && ~R_W)
	     begin
	        MEM_I [ ADDR ] <= IN_8;
	        
	     end
	     
	end  
	
	
	
	 assign OUT_8 = (~ADDR[3]) & (~ADDR[2]) & (~ADDR[1]) & (~ADDR[0]) ? data_out_128  [127:120]   : 8'bz;
	 assign OUT_8 = (~ADDR[3]) & (~ADDR[2]) & (~ADDR[1]) & (ADDR[0])  ? data_out_128  [119:112]   : 8'bz;
	 assign OUT_8 = (~ADDR[3]) & (~ADDR[2]) & (ADDR[1]) & (~ADDR[0])  ? data_out_128  [111:104]   : 8'bz;
	 assign OUT_8 = (~ADDR[3]) & (~ADDR[2]) & (ADDR[1]) & (ADDR[0])   ? data_out_128  [103:96]    : 8'bz;
	 assign OUT_8 = (~ADDR[3]) & (ADDR[2]) & (~ADDR[1]) & (~ADDR[0])  ? data_out_128  [95:88]     : 8'bz;
	 assign OUT_8 = (~ADDR[3]) & (ADDR[2]) & (~ADDR[1]) & (ADDR[0])   ? data_out_128  [87:80]     : 8'bz;
	 assign OUT_8 = (~ADDR[3]) & (ADDR[2]) & (ADDR[1]) & (~ADDR[0])   ? data_out_128  [79:72]     : 8'bz;
	 assign OUT_8 = (~ADDR[3]) & (ADDR[2]) & (ADDR[1]) & (ADDR[0])    ? data_out_128  [71:64]     : 8'bz;
	 assign OUT_8 = (ADDR[3]) & (~ADDR[2]) & (~ADDR[1]) & (~ADDR[0])  ? data_out_128  [63:56]     : 8'bz;
	 assign OUT_8 = (ADDR[3]) & (~ADDR[2]) & (~ADDR[1]) & (ADDR[0])   ? data_out_128  [55:48]     : 8'bz;
	 assign OUT_8 = (ADDR[3]) & (~ADDR[2]) & (ADDR[1]) & (~ADDR[0])   ? data_out_128  [47:40]     : 8'bz;
	 assign OUT_8 = (ADDR[3]) & (~ADDR[2]) & (ADDR[1]) & (ADDR[0])    ? data_out_128  [39:32]     : 8'bz;
	 assign OUT_8 = (ADDR[3]) & (ADDR[2]) & (~ADDR[1]) & (~ADDR[0])   ? data_out_128  [31:24]     : 8'bz;
	 assign OUT_8 = (ADDR[3]) & (ADDR[2]) & (~ADDR[1]) & (ADDR[0])    ? data_out_128  [23:16]     : 8'bz;
	 assign OUT_8 = (ADDR[3]) & (ADDR[2]) & (ADDR[1]) & (~ADDR[0])    ? data_out_128  [15:8]      : 8'bz;
	 assign OUT_8 = (ADDR[3]) & (ADDR[2]) & (ADDR[1]) & (ADDR[0])     ? data_out_128  [7:0]       : 8'bz;
   
                        
endmodule

/////////////////////////////////////////////AES IP  //////////////////////////////////////////////////
/// this module was instantiated to covert the AES core into an IP for creating an API using ZYNQ 7000 SoC

module AES_IP_v1(
IN_1_128,IN_2_128,D_K,E_D,DATA_OUT_128,CLK,RST,MODE,IP_START,DATA_DONE
    );
	input [127:0] IN_1_128,IN_2_128;
	input D_K,E_D;
	input CLK,RST,IP_START;
	input [1:0] MODE;
	output DATA_DONE;
	output [127:0] DATA_OUT_128;
	wire [127:0] PARALLEL_OUT_DATA;
	wire [127:0] PARALLEL_OUT_RANDOM;
	wire [31:0] DATAIN;
	reg   IO_DATALATCH    ,IO_DATA_RW;
	wire  s2,s1,s0;
	reg START;
	wire [31:0] DOUT_1,DOUT_2;
	reg [1:0] state;
	reg [3:0] count_io_ip;
	reg [2:0] s;
	reg ula;
	wire W_DATA_DONE;
	assign DATA_OUT_128 =PARALLEL_OUT_DATA ^ PARALLEL_OUT_RANDOM;
	assign DATA_DONE = W_DATA_DONE;
	always @ (posedge CLK)
	begin
	
	if (~RST)
	    begin
	         IO_DATALATCH <=0;
	         IO_DATA_RW <= 0; 
	         s <= 0;
	         state <= 0;
	            ula <= 0;
	            START <= 0;
	    end
	    
	else 
	    begin
	        if( IP_START && ~ula)
	          begin
	              
	        
	            case (state)
	                2'b00:
	                begin
	                IO_DATALATCH <=1;
	                state <=1; 
	                START <= 0;
	                end
	                
	                2'b01:
	                begin
	                state <= 2;
	                count_io_ip <= count_io_ip -1 ;
	                end
	                
	                2'b10:
	                begin 
	                IO_DATALATCH <= 0;
	                if(count_io_ip >0)
	                begin
	                state <= 0;
	                s <= s-1;
	                end
	                else 
	                state <= 3;
	                end
	                
	                2'b11:
	                begin
	                IO_DATALATCH <= 0;
	                
	                ula <= 1;

	
	                if (D_K)
	                      START <= 1;
	                end   
	          endcase
	      end 
	    
	    if ( ~IP_START )
	        begin
	        ula <= 0;
	        if (W_DATA_DONE)
	            begin
	             START <=0;
	             //state <= 0;
	             end
	            state <= 0;
	            
	            
	        if ( ~D_K)
	                   begin
	                   if(MODE ==0  ||  MODE==1)
	                   begin
	                   s <= 3;
	                   count_io_ip <= 4;
	                   end
	                   else if (MODE ==2 )
	                   begin
	                   s <= 5;
	                   count_io_ip <= 6;
	                   end
	                   else if (MODE == 3)
	                   begin
	                   s <= 7;
	                   count_io_ip <= 8;
	                   end
	                  end
	                  
	        else if (D_K)
	        begin
	                  count_io_ip <= 4;
	                    s <= 3;
	        end
	        end
	    end
	
	end
	
	assign {s2,s1,s0}= s;
	
	AES_PIPELINED_v1   AES_v1(
	.E_D(E_D),
	.D_K(D_K),
	.MODE(MODE),
	.CLK(CLK),
	.RST(RST),
	.DATAIN(DATAIN),
	.IO_DATALATCH(IO_DATALATCH),
	.IO_DATA_RW(IO_DATA_RW),
	.START(START),
	.DATA_DONE(W_DATA_DONE),
	.PARALLEL_OUT_DATA(PARALLEL_OUT_DATA),   ////EXTRA SIGNAL ADDED FOR VIO -- FPGA output verification 
	.PARALLEL_OUT_RANDOM(PARALLEL_OUT_RANDOM)
	    );
	
	mux4x1  M1(.d_in0(IN_1_128[127:96]), .d_in1(IN_1_128[95:64]), .d_in2(IN_1_128[63:32]), 
	        .d_in3(IN_1_128[31:0]), .d_out(DOUT_1), .select({s1,s0}));
	
	mux4x1 M2 (.d_in0(IN_2_128[127:96]), .d_in1(IN_2_128[95:64]), .d_in2(IN_2_128[63:32]), 
	        .d_in3(IN_2_128[31:0]), .d_out(DOUT_2 ), .select({s1,s0}));
	
	mux2x132 mm (.d_in0(DOUT_1), .d_in1(DOUT_2), .d_out(DATAIN), .select(s2));
endmodule

/////////////////////////////////////// AES TOP MODULE /////////////////////////////////////////////////////////
// AES Core Module is connected in AES_PIPELINED_v1 module


 module AES_PIPELINED_v1(
                            E_D,
                            D_K,
                            MODE,
                            CLK,
                            RST,
                            DATAIN,
                            IO_DATALATCH,
                            IO_DATA_RW,
                            START,
                            KEY_DONE,
                            DATA_DONE,
                            DATA_OUT,
                            PARALLEL_OUT_DATA,   ////EXTRA SIGNAL ADDED FOR VIO
                            PARALLEL_OUT_RANDOM,
                            key_wire,
                            KEY_ADDR
                                );
                //////////      OUTPUTS      //////////////

    output KEY_DONE;
    output DATA_DONE;
    output [31:0] DATA_OUT;
    
                  ///////////       INOUT          ///////////////
                      
    input [31:0] DATAIN;    
    
                   ///////////      INPUTS      ///////////////
    
    input E_D;
    input D_K;
    input [1:0] MODE;
    input CLK;
    input RST;
    input IO_DATALATCH;
    input IO_DATA_RW;
    input START;
        
                      ///////////// WIRE //////////////////
    
    wire DATA_LD_WAIT;                 
    wire [3:0] COUNT_IO;
    wire KEY_LD_WAIT;
    wire SHFT_ACTIVE;
    wire SBOX_ACTIVE;
    wire MIX_ACTIVE;
    wire [2:0] MIX_ADDR;
    output wire [5:0] KEY_ADDR;
    wire [3:0] ADDR_WORD;
    wire LD;
    wire ST;
    wire KSA_KEYSTART;
    wire [31:0] MUX_C_OUT;
    wire [3:0] COUNT_ROUND;
    wire [3:0] FINAL_ROUND;
    wire       ENABLE_RANDOM_NUMBER;
    
    wire [31:0] MC_OUT;
    wire [31:0] ADD_RK_IN;
    wire [31:0] KEY_IN;
    wire [31:0] ADD_RK_OUT;
    
    wire [7:0] MUX_S_8_DATA_OUT;
    wire [7:0] MUX_S_8_RANDOM_OUT;
    wire [7:0] SBOX_DATA_OUT;
    wire [7:0] SBOX_RANDOM_OUT;
    
    
    wire [31:0] RANDOM_OUT;
    output wire [127:0] PARALLEL_OUT_DATA;
    output wire [127:0] PARALLEL_OUT_RANDOM;
    wire R1;
    wire R0;
    wire [31:0] KSA_SERIAL_IN;
    wire [31:0] DATAOUT_32;
    output wire [31:0] key_wire ;
    
    assign ENABLE_RANDOM_NUMBER = 1'b1;
    //////// CORE /////////////////
    
    AES_CONTROL_UNIT ACU_x_1(
    .ACU_I_CLK(CLK),
    .ACU_I_RST(RST),
    .ACU_I_E_D(E_D),
    .ACU_I_D_K(D_K),
    .ACU_I_AES_MODE(MODE),
    .ACU_I_START(START),
    .ACU_I_KSA_DONE(KEY_DONE),
    .ACU_I_DATA_LD_WAIT(DATA_LD_WAIT),
    .ACU_I_COUNT_IO(COUNT_IO),
    .ACU_I_KEY_LD_WAIT(KEY_LD_WAIT),
    .ACU_O_DATA_DONE(DATA_DONE),
    .ACU_O_SHFT_ACTIVE(SHFT_ACTIVE),
    .ACU_O_SBOX_ACTIVE(SBOX_ACTIVE),
    .ACU_O_MIX_ACTIVE(MIX_ACTIVE),
    .ACU_O_ADDR_WORD(ADDR_WORD),
    .ACU_O_MIX_ADDR(MIX_ADDR),
    .ACU_O_KEY_ADDR(KEY_ADDR),
    .ACU_O_FINAL_ROUND(FINAL_ROUND),
    .ACU_O_ST(ST),
    .ACU_O_LD(LD),
    .ACU_O_COUNT_ROUND(COUNT_ROUND));
    
    
     AES_MIX_COL MC_x_1(
     .MC_IN(MUX_C_OUT),
     .E_D(E_D),
     .MC_COUNT_ROUND(COUNT_ROUND),
     .MC_FINAL_ROUND_COUNT(FINAL_ROUND),
     .MC_I_MIX_ACTIVE(MIX_ACTIVE),
     .MC_OUT(MC_OUT));
    
    
     AES_ADD_RK A_x_1(
    .ARK_I_DATAIN(MC_OUT),
    .ARK_I_MIX_ACTIVE(MIX_ACTIVE),
    .ARK_I_KEY_IN(key_wire),
    .ARK_KEY_OUT(ADD_RK_OUT),
    .ARK_I_B0(MIX_ADDR[2]));
    
    AES_SBOX_PIPELINED sb_x_1(
    .a(MUX_S_8_DATA_OUT),
    .b(MUX_S_8_RANDOM_OUT),
    .E_D(E_D),
    .s_out0(SBOX_DATA_OUT),
    .s_out1(SBOX_RANDOM_OUT),
    .clk(CLK),
    .random(RANDOM_OUT[27:0]));
    
    
    
    
    AES_USHR_CONTROL a_x_1(
    .DATA_IN_IO(DATAIN),
    .CLOCK(CLK),
    .RESET(RST),
    .E_D(E_D),
    .D_K(D_K),
    .b(MIX_ADDR[2]),
    .R0(R0),
    .R1(R1),
    .SBOX_ACTIVE(SBOX_ACTIVE),
    .IO_DATALATCH(IO_DATALATCH),  
    .IO_DATA_RW(IO_DATA_RW),
    .DATA_DONE(DATA_DONE),
    .DATA_REG_TO_KSA_MEM(KSA_SERIAL_IN),
    .DATA_OUT(DATA_OUT),
    .COUNT(COUNT_IO),
    .PARALLEL_OUT_DATA(PARALLEL_OUT_DATA),
    .PARALLEL_OUT_RANDOM(PARALLEL_OUT_RANDOM),
    .LD(LD),
    .ST(ST),
    .MIX_ACTIVE(MIX_ACTIVE),
    .SHIFT_ACTIVE(SHFT_ACTIVE),
    .RANDOM_8_IN(SBOX_RANDOM_OUT),
    .DATA_8_IN(SBOX_DATA_OUT),
    .ROUND_DATA_32_IN(ADD_RK_OUT),
    .RANDOM_8_OUT(MUX_S_8_RANDOM_OUT),
    .DATA_8_OUT(MUX_S_8_DATA_OUT),
    .ROUND_DATA_32(MUX_C_OUT),
    .KEYLOADED_CP(KEY_LD_WAIT),       
    .KEY_MODE(MODE),
    .S0(ADDR_WORD[0]),
    .S1(ADDR_WORD[1]),
    .KEY_IN(key_wire),
    .RANDOM_NUMBER(RANDOM_OUT),
    .LOAD_WAIT(DATA_LD_WAIT),
    .KSA_KEYSTART(KSA_KEYSTART)
    );
    
    
    KSA_v1  KSA_v1(
                   .CLK(CLK),
                   .RST(RST),
                   .key_start(KSA_KEYSTART),
                   .key_size(MODE),
                   .wk(KSA_SERIAL_IN),
                   .E_D(E_D),
                   .address_from_top(KEY_ADDR),
                   .key_done(KEY_DONE),
                   .MEM_DATAOUT(key_wire)
                   );
    
    
    lfsr113 RANDOM_NUM_GEN(
     // Outputs
     .lfsr113_prng(RANDOM_OUT) ,
     // Inputs
     .CLK(CLK), .reset(RST) ,.enable_p(ENABLE_RANDOM_NUMBER)
     ) ;
    
    assign R1 = MIX_ACTIVE? MIX_ADDR[1]:ADDR_WORD[3];
    assign R0 = MIX_ACTIVE? MIX_ADDR[0]:ADDR_WORD[2];    

endmodule

				 

///////////////////////////////////////////AES CONTROL PATH/////////////////////////////////////////////////////////
module AES_CONTROL_UNIT(
                        ACU_I_CLK,
                        ACU_I_RST,
                        ACU_I_E_D,
                        ACU_I_D_K,
                        ACU_I_AES_MODE,
                        ACU_I_START,
                        ACU_I_KSA_DONE,
                        ACU_I_DATA_LD_WAIT,
                        ACU_I_COUNT_IO,
                        ACU_I_KEY_LD_WAIT,
                        ACU_O_DATA_DONE,
                        ACU_O_SHFT_ACTIVE,
                        ACU_O_SBOX_ACTIVE,
                        ACU_O_MIX_ACTIVE,
                        ACU_O_ADDR_WORD,
                        ACU_O_MIX_ADDR,
                        ACU_O_KEY_ADDR,
                        ACU_O_FINAL_ROUND,
                        ACU_O_ST,
                        ACU_O_LD,
                        ACU_O_COUNT_ROUND
                        );

    //      OUTPUTS       //

    output reg ACU_O_SHFT_ACTIVE;
    output reg ACU_O_SBOX_ACTIVE;
    output reg ACU_O_MIX_ACTIVE;
    output reg [3:0] ACU_O_ADDR_WORD;
    output reg [2:0] ACU_O_MIX_ADDR;
    output reg [5:0] ACU_O_KEY_ADDR;
    output reg ACU_O_LD;
    output reg ACU_O_ST;
    output reg ACU_O_DATA_DONE;
     output reg [3:0] ACU_O_COUNT_ROUND;
    
     output reg [3:0] ACU_O_FINAL_ROUND;
    
    //      INPUTS      //
    
    input ACU_I_E_D;
    input ACU_I_D_K;
    input ACU_I_START;
    input ACU_I_KSA_DONE;
    input ACU_I_CLK;
    input ACU_I_RST;
    input [1:0] ACU_I_AES_MODE;
    input ACU_I_DATA_LD_WAIT;
    input ACU_I_KEY_LD_WAIT;
    input [3:0] ACU_I_COUNT_IO;
    
    //          REG         //
    
    reg [3:0] STATE;
    reg [2:0] LOCAL_COUNT;
    //reg ula;
    
    //       STATE VARIABLE NAMES        //
    
    parameter KEY_STATE = 4'b0000;
    parameter IO_WAIT = 4'b0001;
    parameter DATA_STATE = 4'b0010;
    parameter START_STATE = 4'b011;
    parameter SBOX_STATE = 4'b0100;
    parameter MIX_ADDRK_STATE = 4'b0101;
    parameter OP_STATE = 4'b0110;
    parameter STREAM = 4'b0111;
    
    
    
    /////       REG STATE        //////
            
    always @ (posedge ACU_I_CLK)
    begin
        if(!ACU_I_RST)
        begin
        
             STATE <= KEY_STATE;
             {ACU_O_LD,ACU_O_ST}<={1'b1,1'b1};
             ACU_O_ADDR_WORD<={4{1'b0}}; 
             ACU_O_MIX_ADDR <= 3'b000;
             ACU_O_SHFT_ACTIVE <= 0;
             ACU_O_SBOX_ACTIVE <= 0;
             ACU_O_MIX_ACTIVE <= 0;
             ACU_O_DATA_DONE <= 0;
             STATE <= KEY_STATE;
             LOCAL_COUNT <= 0;
             ACU_O_COUNT_ROUND <= 0;   
    
        end
        
    else
    begin    
        
        case(STATE)
            
    KEY_STATE:
           begin
           if (!ACU_I_D_K)
             begin
                {ACU_O_LD,ACU_O_ST}<={1'b1,1'b0};
                
                if(ACU_I_AES_MODE==2'b00 || ACU_I_AES_MODE==2'b01) //////// AES-128
                begin
                    if(ACU_I_COUNT_IO==4)
                        begin
                            STATE <= IO_WAIT;
                            ACU_O_FINAL_ROUND <= 10;
                        end
                            
                    else
                        STATE <= KEY_STATE;        
                end       
                
                
                
                if(ACU_I_AES_MODE==2'b10)                       ////////// AES-192
                begin
                    if(ACU_I_COUNT_IO==6)
                    begin    
                        STATE <= IO_WAIT;
                        ACU_O_FINAL_ROUND <= 12;
                    end    
                        
                    else
                        STATE <= KEY_STATE;            
                end                
                
                
                
                if(ACU_I_AES_MODE==2'b11)                       ////////// AES-256
                begin
                    if(ACU_I_COUNT_IO==8)
                    begin
                        STATE <= IO_WAIT;
                        ACU_O_FINAL_ROUND <= 14;
                    end
                        
                    else
                        STATE <= KEY_STATE;    
                end                       
          
              end                     
           
           end       
                           
                
     
    IO_WAIT:
                                        // I_O WAIT STATE KEY FIRST/ DATA NEXT
           begin       
                 {ACU_O_LD,ACU_O_ST}<={1'b1,1'b1};
                 if(!ACU_I_KEY_LD_WAIT)
                    STATE <= DATA_STATE;
          
                 else
                    STATE <= IO_WAIT;
           end         
           
          
    DATA_STATE:
           begin
           
           
          
           
             if(ACU_I_D_K)
             begin
                if(!ACU_I_DATA_LD_WAIT)
                    begin
    
                        if(ACU_I_E_D)
                        
                           ACU_O_KEY_ADDR <= 3;
                        
                        else if (!ACU_I_E_D)
                        begin
                            case(ACU_I_AES_MODE)
                                2'b00: ACU_O_KEY_ADDR <= 43;
                                2'b01: ACU_O_KEY_ADDR <= 43;
                                2'b10: ACU_O_KEY_ADDR <= 51;
                                2'b11: ACU_O_KEY_ADDR <= 59;
                            endcase
                        end
                        
                        STATE<= START_STATE;
                        
                        end
                   
                else
                begin
                {ACU_O_LD,ACU_O_ST}<={1'b1,1'b0};
      
                if(!ACU_I_E_D)
                begin
                    case(ACU_I_AES_MODE)
                        2'b00: ACU_O_KEY_ADDR <= ACU_I_COUNT_IO +39;
                        2'b01: ACU_O_KEY_ADDR <= ACU_I_COUNT_IO +39;
                        2'b10: ACU_O_KEY_ADDR <= ACU_I_COUNT_IO +47;
                        2'b11: ACU_O_KEY_ADDR <= ACU_I_COUNT_IO +55;////count-1 + the base value for each key depending on the key mode
                    endcase
                end
                
                else                       
                 ACU_O_KEY_ADDR <= ACU_I_COUNT_IO-1;   
                end      
             end       
           
           end     
           
    START_STATE:
           begin
                 if(ACU_I_START && ACU_I_KSA_DONE )
                 begin
                            {ACU_O_LD,ACU_O_ST}<={1'b0,1'b1};
                            ACU_O_ADDR_WORD <= 0;
                            ACU_O_SHFT_ACTIVE <= 1;
                            ACU_O_SBOX_ACTIVE <= 0;
                            ACU_O_MIX_ACTIVE <= 0;
                            
                            ACU_O_COUNT_ROUND <= ACU_O_COUNT_ROUND+4'b0001;
                            
                           if(ACU_I_E_D)
                                ACU_O_KEY_ADDR <= ACU_O_KEY_ADDR+1;
                           else
                                ACU_O_KEY_ADDR <= ACU_O_KEY_ADDR-7;
                                
                           STATE <=SBOX_STATE;      
                end
                
                else
                begin
                    {ACU_O_LD,ACU_O_ST}<={1'b1,1'b1};
                    ACU_O_COUNT_ROUND <= ACU_O_COUNT_ROUND;
                    STATE <= START_STATE;
                    ACU_O_KEY_ADDR <= ACU_O_KEY_ADDR;       //operation_stopped_or_paused <= 1;
                end
           
           end
           
    SBOX_STATE:
        begin 
            if(ACU_O_SBOX_ACTIVE)
            begin
                
                if(LOCAL_COUNT<3)
                begin
                    {ACU_O_LD,ACU_O_ST}<={1'b0,1'b0};
                    LOCAL_COUNT <= LOCAL_COUNT+1;
                end
                    
                if(LOCAL_COUNT==3)
                begin
                    {ACU_O_LD,ACU_O_ST}<={1'b0,1'b1};
                    LOCAL_COUNT <= LOCAL_COUNT+1;
                end
                    
                if(LOCAL_COUNT==4)
                begin    
                    if(ACU_O_ADDR_WORD==4'b1111)
                    begin
                        LOCAL_COUNT <= 0;
                        ACU_O_SBOX_ACTIVE <= 0;
                        ACU_O_ADDR_WORD <= 0;
                        {ACU_O_LD,ACU_O_ST}<={1'b1,1'b1};   ///// IVE CHANGED
                        STATE <= MIX_ADDRK_STATE;
                    end
                    
                    else
                    begin
                        ACU_O_ADDR_WORD <= ACU_O_ADDR_WORD+1;
                        LOCAL_COUNT <= 0;
                    end      
                    {ACU_O_LD,ACU_O_ST}<={1'b1,1'b1};
                end
            end
            
           else
           begin     
                ACU_O_SBOX_ACTIVE <= 1; //// SBOX ACTIVE TO MODULES         
                ACU_O_SHFT_ACTIVE <= 0;
                {ACU_O_LD,ACU_O_ST}<={1'b1,1'b1};
           end
           
           end
                         
    MIX_ADDRK_STATE:
        begin
           if(ACU_O_MIX_ACTIVE)
           begin
              
                if(LOCAL_COUNT==0)
                begin
                      {ACU_O_LD,ACU_O_ST}<={1'b0,1'b0};
                      LOCAL_COUNT <= LOCAL_COUNT+1;
                end
                
                if(LOCAL_COUNT==1)
                begin
                      {ACU_O_LD,ACU_O_ST}<={1'b0,1'b1};
                      LOCAL_COUNT <= LOCAL_COUNT+1;
                end
                            
                
                if(LOCAL_COUNT==2)     
                begin 
                     {ACU_O_LD,ACU_O_ST}<={1'b0,1'b0};  
                     ACU_O_MIX_ADDR <= ACU_O_MIX_ADDR+4;
                     LOCAL_COUNT <= LOCAL_COUNT+1;
                end
           
                if(LOCAL_COUNT==3)
                begin
                     {ACU_O_LD,ACU_O_ST}<={1'b0,1'b1};
                     LOCAL_COUNT <= LOCAL_COUNT+1;
                end
                                 
                if(LOCAL_COUNT==4)
                begin
                    {ACU_O_LD,ACU_O_ST}<={1'b1,1'b1}; // NO-OP/ADDRESS CHANGE STATE
                    
                    if(ACU_O_MIX_ADDR==3'b111)
                    begin
                        LOCAL_COUNT <= 0;
                        ACU_O_MIX_ACTIVE <= 0;
                        ACU_O_MIX_ADDR <= 0;
                        if(ACU_O_COUNT_ROUND==ACU_O_FINAL_ROUND)
                            STATE <= OP_STATE;
                        else
                            STATE <= START_STATE;
                    end
                
                    else                
                    begin
                        ACU_O_MIX_ADDR <= ACU_O_MIX_ADDR-3;
                        LOCAL_COUNT <= 0;
                        ACU_O_KEY_ADDR <= ACU_O_KEY_ADDR+1;
                    end
                end             
           end
           
           else        
                ACU_O_MIX_ACTIVE <= 1;
                
     end
            
    OP_STATE:
            begin
                ACU_O_DATA_DONE <= 1;
                {ACU_O_LD,ACU_O_ST}<={1'b1,1'b1};   
                STATE <= STREAM;
                
            end    
            
            
    STREAM:
        begin
            
            if(ACU_I_START && ACU_O_DATA_DONE)
            begin
            STATE <= STREAM;    ///wait until data is taken by the memory unit
            end
            else
            begin
            STATE <= DATA_STATE;
            ACU_O_DATA_DONE <= 0;
             ACU_O_SHFT_ACTIVE <= 0;      
             ACU_O_SBOX_ACTIVE <= 0;
             ACU_O_MIX_ACTIVE <= 0;
             LOCAL_COUNT <= 0;
             ACU_O_COUNT_ROUND <= 0;
                     
            end 
        
        
        end        
            
     endcase
     
    end       
    end

endmodule



//////////////////////////////////////////////USHR CONTROL MODULE////////////////////////////////////////////////////////
module AES_USHR_CONTROL(
                         DATA_IN_IO,
                         CLOCK,
                         RESET,
                         E_D,
                         b,
                         R0,
                         R1,
                         SBOX_ACTIVE,
                         IO_DATALATCH,  
                         IO_DATA_RW ,
                         DATA_DONE ,
                         DATA_REG_TO_KSA_MEM,
                         DATA_OUT,
                         COUNT,
                         PARALLEL_OUT_DATA,
                         PARALLEL_OUT_RANDOM,
                         LD,
                         ST,
                         MIX_ACTIVE,
                         SHIFT_ACTIVE,
                         D_K,
                         RANDOM_8_IN,
                         DATA_8_IN,
                         ROUND_DATA_32_IN,
                         RANDOM_8_OUT,
                         DATA_8_OUT,
                         ROUND_DATA_32,
                         KEYLOADED_CP,       
                         KEY_MODE
                         ,S0,S1
                        ,KEY_IN
                        ,RANDOM_NUMBER
                        ,LOAD_WAIT
                        ,KSA_KEYSTART);
                    
    input       [31:0]      DATA_IN_IO;
    input                   RESET;
    input                   CLOCK;
    input                   E_D;
    input       [31:0]      ROUND_DATA_32_IN;
    input                   b;
    input                   R1;
    input                   R0;
    input                   S0;
    input                   S1;
    input                   SBOX_ACTIVE;
    input                   LD;
    input                   ST;
    input                   MIX_ACTIVE;
    input                   SHIFT_ACTIVE;
    input                   IO_DATALATCH;
    input                   D_K;
    input                   IO_DATA_RW ;
    input                   DATA_DONE ;
    input       [7:0]       RANDOM_8_IN,DATA_8_IN;
    input       [1:0]       KEY_MODE;
    input       [31:0]      RANDOM_NUMBER;
    input       [31:0]      KEY_IN;
    
    output                  KSA_KEYSTART;
    output      [31:0]      DATA_REG_TO_KSA_MEM;
    output      [31:0]      DATA_OUT;
    output      [7:0]       RANDOM_8_OUT,DATA_8_OUT;
    output      [31:0]      ROUND_DATA_32 ;
    output      [127:0]     PARALLEL_OUT_DATA;
    output      [127:0]     PARALLEL_OUT_RANDOM;
    output reg  [3:0]       COUNT;
    output reg              LOAD_WAIT;
    output reg              KEYLOADED_CP;
    
    wire        [1:0]       USHR_mode0_data;
    wire        [1:0]       USHR_mode1_data;
    wire        [1:0]       USHR_mode2_data;
    wire        [1:0]       USHR_mode3_data;
    wire        [1:0]       USHR_mode0_random;
    wire        [1:0]       USHR_mode1_random;
    wire        [1:0]       USHR_mode2_random;
    wire        [1:0]       USHR_mode3_random;
     
    wire        [15:0]      CONTROL_2X1_MUX_SBOX_RANDOM;
    wire        [15:0]      CONTROL_2X1_MUX_SBOX_DATA;
    
    wire        [31:0]      SERIAL_RIGHT_DATA_OUT;
    wire        [31:0]      SERIAL_LEFT_DATA_OUT ;
    wire        [31:0]      SERIAL_RIGHT_RANDOM_OUT;
    wire        [31:0]      SERIAL_LEFT_RANDOM_OUT;
    wire        [31:0]      SERIAL_LEFT_DATA_MUX_SELECTED;
    wire        [31:0]      SERIAL_LEFT_DATA_IN;
    wire        [31:0]      SERIAL_RIGHT_RANDOM_IN;
    wire        [31:0]      data_32_out;
    wire        [31:0]      RANDOM_32_out;
    wire        [31:0]      round_data_load;
    wire        [31:0]      round_RANDOM_load;
    
    wire        [127:0]     RANDOM_WIRE_128;
    wire        [127:0]     SHIFT_ROWS_OR_INVERSE_SHIFT_ROWS_OUT_DATA;
    wire        [127:0]     SHIFT_ROWS_OR_INVERSE_SHIFT_ROWS_OUT_RANDOM;
    wire        [127:0]     DATA_WIRE_128;
    wire        [127:0]     SHIFT_ROWS_DATA_OUT;
    wire        [127:0]     INV_SHIFT_ROWS_DATA_OUT;
    wire        [127:0]     SHIFT_ROWS_RANDOM_OUT;
    wire        [127:0]     INV_SHIFT_ROWS_RANDOM_OUT;
    wire        [127:0]     Parallel_in_RANDOM;
    wire        [127:0]     Parallel_in_data;
    reg                     ula;
    reg                     KEY_WAIT_LOCAL;
    reg                     STATE_FOR_D_K_REG_LOAD;
    reg                     STATE_FOR_KSA_MEM_LOAD;
    
    reg         [3:0]       COUNT_value;                
    

    assign round_data_load           =  SBOX_ACTIVE ? {DATA_8_IN,DATA_8_IN,DATA_8_IN,DATA_8_IN} : ROUND_DATA_32_IN   ;
    assign round_RANDOM_load         =  SBOX_ACTIVE ? {RANDOM_8_IN,RANDOM_8_IN,RANDOM_8_IN,RANDOM_8_IN} : ROUND_DATA_32_IN;
    assign DATA_WIRE_128             =  {round_data_load,round_data_load,round_data_load,round_data_load} ;
    assign RANDOM_WIRE_128           =  {round_RANDOM_load,round_RANDOM_load,round_RANDOM_load,round_RANDOM_load};
    assign Parallel_in_data          =  SHIFT_ACTIVE ? SHIFT_ROWS_OR_INVERSE_SHIFT_ROWS_OUT_DATA : DATA_WIRE_128;
    assign  Parallel_in_RANDOM       =  SHIFT_ACTIVE ?  SHIFT_ROWS_OR_INVERSE_SHIFT_ROWS_OUT_RANDOM : RANDOM_WIRE_128;
    
    assign KSA_KEYSTART = KEY_WAIT_LOCAL;
    
    
    always @(posedge CLOCK)
    begin
        if(~RESET)
        begin
    STATE_FOR_D_K_REG_LOAD <= 0;
    ula <= 0;
    STATE_FOR_KSA_MEM_LOAD <= 0;
    COUNT <= 0;
    KEY_WAIT_LOCAL <= 0;
    LOAD_WAIT <= 1;
    KEYLOADED_CP <= 1;
     if (KEY_MODE==2)
    COUNT_value <= 6;
    
    else if (KEY_MODE==3)
    COUNT_value <= 8;
    
    else //(KEY_MODE==1 || KEY_MODE==0)
    COUNT_value <= 4;
    
        end
        
        else if (RESET)
        begin
        if (LD==1 && ST==0 && IO_DATALATCH && ~STATE_FOR_D_K_REG_LOAD && ~KEY_WAIT_LOCAL )
                       begin
                                        if(ula)
                                        begin
                                                   STATE_FOR_D_K_REG_LOAD <= 1;
                                                   if(~D_K && KEYLOADED_CP)///COUNT value ust be unchanged even aftr n CLOCK cycles after key loading  
                                                       COUNT <= COUNT+1;
                                                    else if(D_K )
                                                         begin
                                                       COUNT <= COUNT-1;
                                                         end
                                                    else 
                                                       COUNT <= COUNT     ; 
                                                       end
                                        else 
                                        ula <= 1;
                                                                  
                         end
    
         else if (DATA_DONE && IO_DATA_RW && IO_DATALATCH && ~STATE_FOR_D_K_REG_LOAD)
               begin
                              STATE_FOR_D_K_REG_LOAD <= 1;
                end
        
          ////now the conditions are to reset the control signals ....STATE_FOR_D_K_REG_LOAD   and    parallel_load_state
        if(~IO_DATALATCH)
        begin
                STATE_FOR_D_K_REG_LOAD <= 0;
                ula <= 0;
        end
                         
        if(COUNT==COUNT_value  &&   ~D_K  &&  ~KEY_WAIT_LOCAL && KEYLOADED_CP)
             begin
                COUNT <= 0;
                KEY_WAIT_LOCAL <= 1;
            end
        
        if(KEY_WAIT_LOCAL)
             begin
                if(STATE_FOR_KSA_MEM_LOAD)
                COUNT <= COUNT+1;
        
                if(COUNT==COUNT_value-1  && STATE_FOR_KSA_MEM_LOAD)
                    begin
                        KEY_WAIT_LOCAL <= 0;
                        COUNT <= 4;   
                        KEYLOADED_CP <= 0;
                    end
                    
                 STATE_FOR_KSA_MEM_LOAD <= 0;               
             end
        
        
        if(D_K && COUNT==0)
               LOAD_WAIT <= 0;
               
               
        if ( ~STATE_FOR_KSA_MEM_LOAD && KEY_WAIT_LOCAL)
                STATE_FOR_KSA_MEM_LOAD <= 1;       
               
         if (DATA_DONE )
            begin
            LOAD_WAIT <= 1;
            COUNT <= 4;
            end
            
      end  //// reset ends here
    end    ////always@ (posedge clock) ends here
      
           
     assign USHR_mode0_data=             
     (LD) && (~ST) && (~SHIFT_ACTIVE) && (~MIX_ACTIVE) && (~SBOX_ACTIVE) && (IO_DATALATCH) && (~STATE_FOR_D_K_REG_LOAD) && ~IO_DATA_RW && ula? ///initial data load state right shift 
       ~KEY_WAIT_LOCAL &&   (D_K && ~KEYLOADED_CP || ~D_K && KEYLOADED_CP)  ? 1 : KEY_WAIT_LOCAL ? 2 : 0 : ///if key has been loaded and we are trying to load again ushr mode will be 0 ....else ushr mode will be 1
      ~LD && ST && SHIFT_ACTIVE && ~MIX_ACTIVE && ~SBOX_ACTIVE    ? 3 : ////parallel load if shift load is high
       ~LD && ST  && SBOX_ACTIVE ? 3:                ///////if subactive is high then all ushr modes must be 3 and the control goes to the 2x1 muxes 
      ~LD && ST  ? {(~b) && (~R1) && (~R0), (~b) && (~R1) && (~R0)} : /////this logic is used to select the round data loading ushr register
      ~SHIFT_ACTIVE && ~MIX_ACTIVE && ~SBOX_ACTIVE && IO_DATALATCH && DATA_DONE && IO_DATA_RW   || (KEY_WAIT_LOCAL && STATE_FOR_KSA_MEM_LOAD) ? 2 : 0;
      ////IF DATA DONE IS HIGH ALL THE VALUES WILL BE SHFTD OUT IF I DATA LATCH IS HIGH
      ////THIS HAS TO BE CHANGED ....DIDNT EXOR RANDOM AND DATA IN THE LAST ROUND 
      
      
      
      
      assign USHR_mode1_data=             
      (LD) && (~ST) && (~SHIFT_ACTIVE) && (~MIX_ACTIVE) && (~SBOX_ACTIVE) && (IO_DATALATCH) && (~STATE_FOR_D_K_REG_LOAD) && ~IO_DATA_RW && ula? 
       ~KEY_WAIT_LOCAL && (D_K && ~KEYLOADED_CP || ~D_K && KEYLOADED_CP)  ? 1 : KEY_WAIT_LOCAL ? 2 : 0 :
       ~LD && ST && SHIFT_ACTIVE && ~MIX_ACTIVE && ~SBOX_ACTIVE    ? 3 :
        ~LD && ST  && SBOX_ACTIVE ? 3:
       ~LD && ST  ? {(~b) && (~R1) && (R0), (~b) && (~R1) && (R0)} :
       ~SHIFT_ACTIVE && ~MIX_ACTIVE && ~SBOX_ACTIVE && IO_DATALATCH && DATA_DONE && IO_DATA_RW  || (KEY_WAIT_LOCAL && STATE_FOR_KSA_MEM_LOAD)  ? 2 : 0;
       
       
      assign USHR_mode2_data=             
       (LD) && (~ST) && (~SHIFT_ACTIVE) && (~MIX_ACTIVE) && (~SBOX_ACTIVE) && (IO_DATALATCH) && (~STATE_FOR_D_K_REG_LOAD) && ~IO_DATA_RW && ula? 
       ~KEY_WAIT_LOCAL && (D_K && ~KEYLOADED_CP || ~D_K && KEYLOADED_CP) ? 1 : KEY_WAIT_LOCAL ? 2  : 0 :
        ~LD && ST && SHIFT_ACTIVE && ~MIX_ACTIVE && ~SBOX_ACTIVE    ? 3 :
         ~LD && ST  && SBOX_ACTIVE ? 3:
        ~LD && ST  ? {(~b) && (R1) && (~R0), (~b) && (R1) && (~R0)} :
        ~SHIFT_ACTIVE && ~MIX_ACTIVE && ~SBOX_ACTIVE && IO_DATALATCH && DATA_DONE && IO_DATA_RW  || (KEY_WAIT_LOCAL && STATE_FOR_KSA_MEM_LOAD) ? 2 : 0;
        
        
       assign USHR_mode3_data=             
        (LD) && (~ST) && (~SHIFT_ACTIVE) && (~MIX_ACTIVE) && (~SBOX_ACTIVE) && (IO_DATALATCH) && (~STATE_FOR_D_K_REG_LOAD) && ~IO_DATA_RW && ula? 
         ~KEY_WAIT_LOCAL && (D_K && ~KEYLOADED_CP || ~D_K && KEYLOADED_CP)  ? 1 : KEY_WAIT_LOCAL ? 2 : 0 :
         ~LD && ST && SHIFT_ACTIVE && ~MIX_ACTIVE && ~SBOX_ACTIVE    ? 3 :
          ~LD && ST  && SBOX_ACTIVE ? 3:
         ~LD && ST  ? {(~b) && (R1) && (R0) , (~b) && (R1) && (R0) } :
         ~SHIFT_ACTIVE && ~MIX_ACTIVE && ~SBOX_ACTIVE && IO_DATALATCH && DATA_DONE && IO_DATA_RW  || (KEY_WAIT_LOCAL && STATE_FOR_KSA_MEM_LOAD) ? 2 : 0;
         
         
       assign USHR_mode0_random=             
          (LD) && (~ST) && (~SHIFT_ACTIVE) && (~MIX_ACTIVE) && (~SBOX_ACTIVE) && (IO_DATALATCH) && (~STATE_FOR_D_K_REG_LOAD) && ~IO_DATA_RW && ula? 
       ~KEY_WAIT_LOCAL && (D_K && ~KEYLOADED_CP || ~D_K && KEYLOADED_CP) ? 1 : KEY_WAIT_LOCAL ? 2 : 0 :
           ~LD && ST && SHIFT_ACTIVE && ~MIX_ACTIVE && ~SBOX_ACTIVE    ? 3 :
            ~LD && ST  && SBOX_ACTIVE ? 3:
           ~LD && ST  ? {(b) && (~R1) && (~R0), (b) && (~R1) && (~R0)}  :
           ~SHIFT_ACTIVE && ~MIX_ACTIVE && ~SBOX_ACTIVE && IO_DATALATCH && DATA_DONE && IO_DATA_RW  || (KEY_WAIT_LOCAL && STATE_FOR_KSA_MEM_LOAD) ? 2 : 0;
           
           
        assign USHR_mode1_random=             
           (LD) && (~ST) && (~SHIFT_ACTIVE) && (~MIX_ACTIVE) && (~SBOX_ACTIVE) && (IO_DATALATCH) && (~STATE_FOR_D_K_REG_LOAD) && ~IO_DATA_RW && ula? 
        ~KEY_WAIT_LOCAL &&  (D_K && ~KEYLOADED_CP || ~D_K && KEYLOADED_CP)  ? 1 : KEY_WAIT_LOCAL ? 2 : 0 :
            ~LD && ST && SHIFT_ACTIVE && ~MIX_ACTIVE && ~SBOX_ACTIVE    ? 3 :
             ~LD && ST  && SBOX_ACTIVE ? 3:
            ~LD && ST  ? {(b) && (~R1) && (R0), (b) && (~R1) && (R0)} :
            ~SHIFT_ACTIVE && ~MIX_ACTIVE && ~SBOX_ACTIVE && IO_DATALATCH && DATA_DONE && IO_DATA_RW  || (KEY_WAIT_LOCAL && STATE_FOR_KSA_MEM_LOAD) ? 2 : 0;
            
            
        assign USHR_mode2_random=             
            (LD) && (~ST) && (~SHIFT_ACTIVE) && (~MIX_ACTIVE) && (~SBOX_ACTIVE) && (IO_DATALATCH) && (~STATE_FOR_D_K_REG_LOAD) && ~IO_DATA_RW && ula? 
          ~KEY_WAIT_LOCAL &&  (D_K && ~KEYLOADED_CP || ~D_K && KEYLOADED_CP)? 1 : KEY_WAIT_LOCAL ? 2 : 0 :
             ~LD && ST && SHIFT_ACTIVE && ~MIX_ACTIVE && ~SBOX_ACTIVE    ? 3 :
              ~LD && ST  && SBOX_ACTIVE ? 3:
             ~LD && ST  ? {(b) && (R1) && (~R0), (b) && (R1) && ( ~R0)} :
             ~SHIFT_ACTIVE && ~MIX_ACTIVE && ~SBOX_ACTIVE && IO_DATALATCH && DATA_DONE && IO_DATA_RW  || (KEY_WAIT_LOCAL && STATE_FOR_KSA_MEM_LOAD)  ? 2 : 0;
             
             
        assign USHR_mode3_random=             
             (LD) && (~ST) && (~SHIFT_ACTIVE) && (~MIX_ACTIVE) && (~SBOX_ACTIVE) && (IO_DATALATCH) && (~STATE_FOR_D_K_REG_LOAD) && ~IO_DATA_RW && ula? 
           ~KEY_WAIT_LOCAL &&   (D_K && ~KEYLOADED_CP || ~D_K && KEYLOADED_CP) ? 1 : KEY_WAIT_LOCAL ? 2 : 0 :
              ~LD && ST && SHIFT_ACTIVE && ~MIX_ACTIVE && ~SBOX_ACTIVE    ? 3 :
               ~LD && ST  && SBOX_ACTIVE ? 3:
              ~LD && ST  ? {(b) && (R1) && (R0) , (b) && (R1) && (R0) } :
              ~SHIFT_ACTIVE && ~MIX_ACTIVE && ~SBOX_ACTIVE && IO_DATALATCH && DATA_DONE && IO_DATA_RW  || (KEY_WAIT_LOCAL && STATE_FOR_KSA_MEM_LOAD)  ? 2 : 0;
    
    
    
    assign DATA_REG_TO_KSA_MEM               =          SERIAL_LEFT_DATA_OUT;
    assign DATA_OUT                          =          SERIAL_LEFT_DATA_OUT ^ SERIAL_LEFT_RANDOM_OUT ;
    assign SERIAL_LEFT_DATA_IN               = D_K?     DATA_IN_IO  ^ KEY_IN  ^  RANDOM_NUMBER : DATA_IN_IO;
    assign SERIAL_LEFT_DATA_MUX_SELECTED     =          SERIAL_LEFT_RANDOM_OUT;
    assign SERIAL_RIGHT_RANDOM_IN            = D_K ?    RANDOM_NUMBER : SERIAL_RIGHT_DATA_OUT;
    
    AES_USHR_CUSTOM DATA_x_1 (.CLOCK(CLOCK ), .RESET( RESET), .Serial_in_L ( SERIAL_LEFT_DATA_MUX_SELECTED ), .Serial_in_R( SERIAL_LEFT_DATA_IN ), 
                                                .Serial_out_left(SERIAL_LEFT_DATA_OUT),
                                                .Serial_out_right(SERIAL_RIGHT_DATA_OUT),
                                                .Parallel_in( Parallel_in_data ), 
                                                .Parallel_out( PARALLEL_OUT_DATA ), 
                                                .USHR_mode0( USHR_mode0_data),
                                                .USHR_mode1(USHR_mode1_data ),
                                                .USHR_mode2(USHR_mode2_data ),
                                                .USHR_mode3(USHR_mode3_data ),
                                                .control_s(CONTROL_2X1_MUX_SBOX_DATA));
                                                
    AES_USHR_CUSTOM RANDOM_x_1 (.CLOCK(CLOCK ), .RESET( RESET), .Serial_in_L ( 32'd0 ),
                                               .Serial_out_left(SERIAL_LEFT_RANDOM_OUT),
                                                .Serial_out_right(SERIAL_RIGHT_RANDOM_OUT), 
                                               .Serial_in_R( SERIAL_RIGHT_RANDOM_IN  ), 
                                               .Parallel_in( Parallel_in_RANDOM  ),                 
                                               .Parallel_out( PARALLEL_OUT_RANDOM ),
                                               .USHR_mode0( USHR_mode0_random),
                                               .USHR_mode1(USHR_mode1_random ),
                                               .USHR_mode2(USHR_mode2_random ),
                                               .USHR_mode3(USHR_mode3_random )
                                               ,.control_s(CONTROL_2X1_MUX_SBOX_RANDOM));                                            
    
    
    shiftrows s_1 (.in(PARALLEL_OUT_DATA),.out(SHIFT_ROWS_DATA_OUT));
    inverseshiftrows is_1(.in(PARALLEL_OUT_DATA),.out(INV_SHIFT_ROWS_DATA_OUT));
    shiftrows s_2 (.in(PARALLEL_OUT_RANDOM),.out(SHIFT_ROWS_RANDOM_OUT));
    inverseshiftrows is_2(.in(PARALLEL_OUT_RANDOM),.out(INV_SHIFT_ROWS_RANDOM_OUT));
    
    assign SHIFT_ROWS_OR_INVERSE_SHIFT_ROWS_OUT_DATA= E_D?SHIFT_ROWS_DATA_OUT:INV_SHIFT_ROWS_DATA_OUT;
    assign  SHIFT_ROWS_OR_INVERSE_SHIFT_ROWS_OUT_RANDOM= E_D?SHIFT_ROWS_RANDOM_OUT:INV_SHIFT_ROWS_RANDOM_OUT;
    
    /////multiplexers used to select the round data: #########################################################
    /////#####################################################################################################
    
    mux4x1 M0 (.d_in0(PARALLEL_OUT_DATA[127:96]), .d_in1(PARALLEL_OUT_DATA[95:64]), .d_in2(PARALLEL_OUT_DATA[63:32]), .d_in3(PARALLEL_OUT_DATA[31:0]),
                             .d_out(data_32_out), .select({R1,R0}));
                             
    mux4x1 M1 (.d_in0(PARALLEL_OUT_RANDOM[127:96]), .d_in1(PARALLEL_OUT_RANDOM[95:64]), .d_in2(PARALLEL_OUT_RANDOM[63:32]), .d_in3(PARALLEL_OUT_RANDOM[31:0]),
                              .d_out(RANDOM_32_out), .select({R1,R0}));
    
    
    
    mux4x18 M2 (.d_in0(RANDOM_32_out[31:24]), .d_in1(RANDOM_32_out[23:16]), .d_in2(RANDOM_32_out[15:8]), .d_in3(RANDOM_32_out[7:0]),
                              .d_out(RANDOM_8_OUT), .select({S1,S0}));
    
    
    mux4x18 M3 (.d_in0(data_32_out[31:24]), .d_in1(data_32_out[23:16]), .d_in2(data_32_out[15:8]), .d_in3(data_32_out[7:0]),
                              .d_out(DATA_8_OUT), .select({S1,S0}));
    
    mux2x132 mselect (.d_in0(data_32_out), .d_in1(RANDOM_32_out), .d_out(ROUND_DATA_32), .select(b));
    
    
    
    assign CONTROL_2X1_MUX_SBOX_DATA[0] =   (   (~R1)  &&  (~R0)  &&  (~S1)  &&  (~S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_DATA[1] =   (   (~R1)  &&  (~R0)  &&  (~S1)  &&  (S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_DATA[2] =   (   (~R1)  &&  (~R0)  &&  (S1)  &&  (~S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_DATA[3] =   (   (~R1)  &&  (~R0)  &&  (S1)  &&  (S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_DATA[4] =   (   (~R1)  &&  (R0)   &&   (~S1)  &&  (~S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_DATA[5] =   (   (~R1)  &&  (R0)   &&   (~S1)  &&  (S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_DATA[6] =   (   (~R1)  &&  (R0)   &&   (S1)  &&  (~S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_DATA[7] =   (   (~R1)  &&  (R0)   &&   (S1)  &&  (S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_DATA[8] =   (   (R1)   &&  (~R0)  &&  (~S1)  &&  (~S0) ) ||  (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_DATA[9] =   (   (R1)   &&  (~R0)  &&  (~S1)  &&  (S0) ) ||  ( ~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_DATA[10]=   (   (R1)   &&  (~R0)  &&  (S1)  &&  (~S0) ) ||  (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_DATA[11]=   (   (R1)   &&  (~R0)  &&  (S1)  &&  (S0) ) ||   (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_DATA[12]=   (   (R1)   &&  (R0)   &&   (~S1)  &&  (~S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_DATA[13]=   (   (R1)   &&  (R0)   &&   (~S1)  &&  (S0) ) ||  (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_DATA[14]=   (   (R1)   &&  (R0)   &&   (S1)  &&  (~S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_DATA[15]=   (   (R1)   &&  (R0)   &&   (S1)  &&  (S0) ) || (~SBOX_ACTIVE); 
    
    
    
    assign CONTROL_2X1_MUX_SBOX_RANDOM[0]=    (   (~R1)  &&  (~R0)  &&  (~S1)  &&  (~S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_RANDOM[1]=    (   (~R1)  &&  (~R0)  &&  (~S1)  &&  (S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_RANDOM[2]=    (   (~R1)  &&  (~R0)  &&  (S1)  &&  (~S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_RANDOM[3]=    (   (~R1)  &&  (~R0)  &&  (S1)  &&  (S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_RANDOM[4]=    (   (~R1)  &&  (R0)  &&   (~S1)  &&  (~S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_RANDOM[5]=    (   (~R1)  &&  (R0)  &&   (~S1)  &&  (S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_RANDOM[6]=    (   (~R1)  &&  (R0)  &&   (S1)  &&  (~S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_RANDOM[7]=    (   (~R1)  &&  (R0)  &&   (S1)  &&  (S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_RANDOM[8]=    (   (R1)  &&  (~R0)  &&   (~S1)  &&  (~S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_RANDOM[9]=    (   (R1)  &&  (~R0)  &&   (~S1)  &&  (S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_RANDOM[10]=   (   (R1)  &&  (~R0)  &&   (S1)  &&  (~S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_RANDOM[11]=   (   (R1)  &&  (~R0)  &&   (S1)  &&  (S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_RANDOM[12]=   (   (R1)  &&  (R0)  &&    (~S1)  &&  (~S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_RANDOM[13]=   (   (R1)  &&  (R0)  &&    (~S1)  &&  (S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_RANDOM[14]=   (   (R1)  &&  (R0)  &&    (S1)  &&  (~S0) ) || (~SBOX_ACTIVE);
    assign CONTROL_2X1_MUX_SBOX_RANDOM[15]=   (   (R1)  &&  (R0)  &&   (S1)  &&  (S0) ) || (~SBOX_ACTIVE); 



endmodule



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

///////////////////////////////////////////////////////SHIFT ROWS//////////////////////////////////////////////////

module shiftrows(in,out);
input [127:0] in;
output [127:0] out;

assign out= {in[127:120],in[87:80],in[47:40],in[7:0],in[95:88],in[55:48],in[15:8],in[103:96],in[63:56],in[23:16],
					in[111:104],in[71:64],in[31:24],in[119:112],in[79:72],in[39:32]};


endmodule


/////////////////////////////////////////////////////INVERSE SHIFT ROWS//////////////////////////////////////////////////

module inverseshiftrows(in,out);
 input [127:0]in;  
 output [127:0]out;
 
 assign out={in[127:120],in[23:16],in[47:40],in[71:64],in[95:88],in[119:112],in[15:8],in[39:32],
				in[63:56],in[87:80],in[111:104],in[7:0],in[31:24],in[55:48],in[79:72],in[103:96]};

endmodule


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

/////////////////////////////////////////////////////////////////////////////
////ADD RK

module AES_ADD_RK(
ARK_I_DATAIN,
ARK_I_MIX_ACTIVE,
ARK_I_KEY_IN,
ARK_KEY_OUT,
ARK_I_B0);

/////////       OUTPUTS      //////////////

output [31:0] ARK_KEY_OUT;

/////////   INPUTS    ///////////////

input [31:0] ARK_I_DATAIN;
input [31:0] ARK_I_KEY_IN;
input ARK_I_B0;
input ARK_I_MIX_ACTIVE;

wire [31:0] ARK_SELECT_IN;

///////     CORE     ///////////////////

assign ARK_SELECT_IN    =  ARK_I_MIX_ACTIVE? ARK_I_DATAIN: 32'bz;
assign ARK_KEY_OUT      =  ARK_I_B0? ARK_SELECT_IN : ARK_SELECT_IN ^ ARK_I_KEY_IN;

endmodule

//////////////////////////////////// KSA //////////////////////////////////////////////

module KSA_v1(
               input CLK,
               input RST,
               input key_start,
               input [1:0]key_size,
               input [31:0]wk,
               input E_D,
               input [5:0] address_from_top,
               output key_done,
               output [31:0] MEM_DATAOUT
               );

    //wire [3:0] i;
    wire out_x_en, isX, mem_r_wbar;
    wire [1:0]  ushr_mode;
    wire [7:0]  rcon;
    wire [31:0] data_to_mem;
    wire [31:0] data_from_mem;
    wire [31:0] input_to_inv_mix;
    wire [31:0] data_from_inv_mix;
    wire [31:0] data_to_mem_w;
    wire [5:0]  address_from_KSA;
    wire [5:0] address;
    wire [5:0] round_count;
    MEMORY_32x64 m(
    		.clock(CLK), 
    		.MEM_DATAIN(data_to_mem), 
    		.MEM_DATAOUT(data_from_mem), 
    		.MEM_Addr(address), 
    		.MEM_rw(mem_r_wbar)
                  );
    
    KSA_DP	   d  (
    		.KD_key_out(data_to_mem_w),
    		.KD_key_in(data_from_mem),
    		.KD_USHR_mode(ushr_mode),
    		.KD_dout_x_en(out_x_en),
    		.KD_rcon(rcon),
    		.KD_c(~isX),
    		.KD_clk(CLK),
    		.KD_rst(RST),
    		.KD_active_rot(isX)
    	      );
    
    
    
    KSA_CP     k (
                    .length(key_size),
                	.key_start(key_start),
                	.clk(CLK),
                	.rst(RST),
                	.keydone(key_done),
                	.round_count_total(round_count),
                	.ushr_mode(ushr_mode),
                	.isX(isX),
                	.out_x_en(out_x_en),
                	.rcon(rcon),
                	.address(address_from_KSA),
                	.mem_r_wbar(mem_r_wbar)
                  );	
    
    assign input_to_inv_mix = E_D ? 32'bz : data_from_mem;
    
    Inverse_Mix_Columns M1(
    			.in(input_to_inv_mix),
    			.out(data_from_inv_mix)
    	      	      );
    
    assign MEM_DATAOUT = E_D ? data_from_mem : (address_from_top < 4) | (address_from_top >(round_count-4)) ? data_from_mem : data_from_inv_mix;
    assign data_to_mem = key_start ? wk : data_to_mem_w;
    assign address = key_done ?  address_from_top : address_from_KSA;

endmodule

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module MEMORY_32x64(clock, MEM_DATAIN, MEM_DATAOUT, MEM_Addr, MEM_rw);
    input         clock;
    input  [31:0] MEM_DATAIN;   //Memory Data Input Bus
    input  [5:0]  MEM_Addr;     //Memory Address Bus
    input         MEM_rw;       //Memory read/write Control Signal
    output [31:0] MEM_DATAOUT;  //Memory Data Output Bus
    
    reg    [31:0] MEMORY [63:0];
    reg    [31:0] MEM_DATAOUT;
    
    always@(posedge clock)
    begin
        if(!MEM_rw) MEMORY[MEM_Addr] <= MEM_DATAIN;
        MEM_DATAOUT <= MEMORY[MEM_Addr];
    end

endmodule

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module KSA_DP	(
		output [31:0] KD_key_out,
		input  [31:0] KD_key_in,
		input  [1:0]  KD_USHR_mode,
		input         KD_dout_x_en,
		input  [7:0]  KD_rcon,
		input         KD_c,
		input         KD_clk,
		input         KD_rst,
		input	      KD_active_rot
		);

reg 	[31:0] dout_x;
wire 	[7:0] sbox_out,sbox_in;
wire	[31:0] ushr_out,rot_out,rcon_out,c_mux_out;

Universal_SHR_32 u1(
		.clock(KD_clk),
		.reset(KD_rst), 
		.Serial_in_R(sbox_out), 
		.Serial_in_L(8'd0), 
		.Parallel_in(KD_key_in), 
		.Parallel_out(ushr_out),
		.USHR_mode(KD_USHR_mode),
		.serial_out_R(sbox_in)
		);

    RotWord r1	(
    		.in(ushr_out & {KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot,KD_active_rot}),
    		.out(rot_out)
    		);
    
    sbox s1		(
    		.dataout(sbox_out),
    		.datain(sbox_in)
    		);
    
    assign rcon_out={rot_out[31:24]^KD_rcon,rot_out[23:0]};
    
    assign c_mux_out = KD_c?ushr_out:rcon_out;
    
    assign KD_key_out = c_mux_out^dout_x;
    
    always@(posedge KD_clk or negedge KD_rst)
    begin
    	if(!KD_rst)
    		dout_x <= 0;
    	else if(KD_dout_x_en)
    		dout_x <= KD_key_in;
    end

endmodule
module RotWord	( 
		input [31:0] in,
		output [31:0] out
		);

assign out={in[23:16],in[15:8],in[7:0],in[31:24]};


endmodule
module Universal_SHR_32(clock, reset, Serial_in_L, Serial_in_R, Parallel_in, Parallel_out, USHR_mode,serial_out_R);
    input          clock;
    input          reset;
    input  [7:0]  Serial_in_L;
    input  [7:0]  Serial_in_R;
    input  [31:0] Parallel_in;
    input  [1:0]   USHR_mode;
    output [31:0] Parallel_out;
    output [7:0]  serial_out_R;
    reg [7:0] reg_0, reg_1, reg_2, reg_3;
    
    wire [7:0] d_out0, d_out1, d_out2, d_out3;
    
    mux_8 m0 (.d_in0(reg_0), 
    	  .d_in1(Serial_in_R), 
    	  .d_in2(reg_1), 
    	  .d_in3(Parallel_in[31:24]), 
    	  .d_out(d_out0), 
    	  .select(USHR_mode)
    	 );
    
    mux_8 m1 (.d_in0(reg_1), 
    	  .d_in1(reg_0), 
    	  .d_in2(reg_2), 
    	  .d_in3(Parallel_in[23:16]), 
    	  .d_out(d_out1), 
    	  .select(USHR_mode)
    	 );
    
    mux_8 m2 (.d_in0(reg_2), 
    	  .d_in1(reg_1), 
    	  .d_in2(reg_3), 
    	  .d_in3(Parallel_in[15:8]), 
    	  .d_out(d_out2), 
    	  .select(USHR_mode)
    	 );
    
    mux_8 m3 (.d_in0(reg_3), 
    	  .d_in1(reg_2),
    	  .d_in2(Serial_in_L),
    	  .d_in3(Parallel_in[7:0]),
    	  .d_out(d_out3),
    	  .select(USHR_mode)
    	 );
    
    
    always@ (posedge clock or negedge reset)
    begin
        if(!reset) 
        begin 
    	reg_0 <= 8'b0; 
    	reg_1 <= 8'b0; 
    	reg_2 <= 8'b0; 
    	reg_3 <= 8'b0; 
        end
        
        else 
        begin 
           reg_0 <= d_out0; 
    	   reg_1 <= d_out1; 
    	   reg_2 <= d_out2; 
    	   reg_3 <= d_out3; 
        end
    end
    
    assign Parallel_out = {reg_0, reg_1, reg_2, reg_3};
    assign serial_out_R = reg_3;

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

///////////////////////////////////////////////////////////////////////////////////////////

module KSA_CP (
            	input [1:0] length,
       	    	input key_start,
            	input clk,
		input rst,
            
		//output to main module
            	output reg keydone,
            	output [5:0] round_count_total,
            	
		//outputs to datapath
            	output reg [1:0]ushr_mode,
            	output reg isX,
            	output reg out_x_en,
            	output reg [7:0]rcon,
            
		//outputs to memory
            	output reg [5:0]address,
            	output reg mem_r_wbar
              );


    parameter hold       = 3'b000;
    parameter store      = 3'b001;
    parameter fetch  	 = 3'b010;
    parameter fetchX     = 3'b011;
    parameter s_box_path = 3'b100;
    parameter write  	 = 3'b101;
    parameter no_op      = 3'b110;
    
    
    reg  [2:0] state;
    reg  [5:0] count;
    reg  [3:0] rcon_addr;
    wire [3:0] i;
    
    
    //output to keytop
    
     reg [7:0] rcon_reg [0:9]; 
    reg [2:0] s_box_count;
    reg 	  activate;
    reg key_loaded;
    
    assign i = (~length[1]) ? 4'd4 : (length==2'b10) ? 4'd6 : 4'd8; 
    
    assign round_count_total = (i == 4'd4) ? 6'd43 : (i==4'd6) ? 6'd51 : 6'd59;
    
    
    always @ (posedge clk )
    begin
    	
    	if(!rst)
    	begin
    		keydone <= 1'b0;
    		rcon_addr <= 4'b0000;
    		rcon <= rcon_reg[0];
    		s_box_count <= 3'b000;
    		mem_r_wbar <= 1'b1;
    		state <= hold;
    		count <= 6'b000000;
    		address <= 6'b000000;
    		ushr_mode <= 2'b00;
    		out_x_en <= 1'b0;
    		isX <= 1'b0;
    		key_loaded <= 1'b0;
    		rcon_reg [0]<= 8'h01;
    		rcon_reg [1]<= 8'h02;
    		rcon_reg [2]<= 8'h04;
    		rcon_reg [3]<= 8'h08;
    		rcon_reg [4]<= 8'h10;
    		rcon_reg [5]<= 8'h20;
    		rcon_reg [6]<= 8'h40;
    		rcon_reg [7]<= 8'h80;
    		rcon_reg [8]<= 8'h1b;
    		rcon_reg [9]<= 8'h36;
    	end	
    	
    	else 
    	begin
    	
    		case (state)
    		
    		hold:
        	begin
        	   address <= count;			          
    			if(!key_start && !keydone && count >= (i-1))
    			begin 
    				count <= count + 1;
    				mem_r_wbar <= 1'b1;
    				state <= fetch;
    			end
    			else if (key_start && count < i && !key_loaded)
    			begin
                    count <= count + 1;			
    				mem_r_wbar <= 1'b0;
    				state <= store;
    			end
    			else if (keydone)
    			begin
    				mem_r_wbar <= 1'b1;
    				state <= hold;
    			end
    			else
    				state <= hold;
    		end
    		
    		store:
    		begin
                		if (count == i)
                		begin
                		    mem_r_wbar <= 1'b1;
                		    count <= count - 1;
                		    key_loaded <= 1;  
                		end
    	    		address <= count;
    	    		state <= hold;
            	end
                    
    		fetch:
    		begin
    			ushr_mode <= 2'b11;
    			out_x_en  <= 1'b0;
    			address <= address-(i-1);
    			isX <= (~length[1] && ((count) % 4==0)) || ((length==2) && ((count) % 6==0)) || ((length==3) && ((count) % 8==0));
    			mem_r_wbar<= 1'b1;		
    			state <= fetchX;
    		end
    	
    		fetchX:
    		begin
    			ushr_mode <= 2'b00;
    			out_x_en <= 1'b1;
    			address <= address + i;           
    			s_box_count <= 1'b0;
    			if (isX || ((length==3) && ((count-4) % 8==0)))
    			     state <= s_box_path;
    			else
    			     state <= write;
    		end
    	
    		s_box_path:
    		begin
    			ushr_mode <= 2'b01;
    			rcon <= rcon_reg[rcon_addr];
    			out_x_en <= 0;
    			mem_r_wbar <= 1;
    			s_box_count <= s_box_count+1;
    			if(s_box_count < 3)
    				state <= s_box_path;
    			else
    				state <= write;
    		end
    	
    		write:
    		begin
    			ushr_mode <= 2'b00;
    			  //write
    			out_x_en <= 1'b0;
             
    			if(count>=i && isX)	
    				rcon_addr <= rcon_addr+1;            	
             
    			if(count < round_count_total+1)
    			begin
    			     mem_r_wbar <= 1'b0;
    				address <= count;
    				state <= hold;
    			end
    			else
    			begin
    			     	
    			     	state <= hold;
    			     	keydone<= 1;
    			     	mem_r_wbar <= 1;
    			end
    		end
    		default:
    			state <= store;
    		endcase
    	end
    end

endmodule

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
