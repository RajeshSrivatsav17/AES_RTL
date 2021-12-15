
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


