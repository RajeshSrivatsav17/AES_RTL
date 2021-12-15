`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// -------- THIS FILE CONTAINS ALL MODULES CONSOLIDATED IN A SINGLE FILE --------- 
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
module AES_PACM_TOP(    			IO_DATAINOUT,
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
	
	//MODE[1:0] decoding ::
	//MODE[1:0] == 2'b00 || 2'b01 --> 128 bit encryption/decryption
	//MODE[1:0] == 2'b10          --> 192 bit encryption/decryption
	//MODE[1:0] == 2'b11          --> 256 bit encryption/decryption
	
	//D_K mentions if the incoming MEM_I represents Data/ Whitening Key

	//E_D mentions if the operation to be performed is encryption/ decryption
	
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

