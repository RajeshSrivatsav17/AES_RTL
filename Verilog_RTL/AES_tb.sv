
//---------------------------------------------------------------------------------
// Author           : Rajesh Srivatsav
// Language         : System Verilog
// File Description : This file contains testbench for checking the functioanality 
//                    of AES module
//---------------------------------------------------------------------------------


//---------------------------------------------------------------------------------
// TestBench Description:
//		 The testbench has following components,
//			--> Transaction class - Contains Input data packets i.e. opcode, ciphertext/plaintext for DUT (AES)
//			--> Generator class   - Generates above transactions 
//			--> Driver class      - Drives the input to DUT
//			--> Env class         - Contains the Generator and Driver Classes
//---------------------------------------------------------------------------------
`timescale 1ns / 1ps 
class transactions;
	bit [255:0] keyMODE
	bit [3:0] opcode; //  MODE[1:0], D_K, E_D
	bit [127:0] cptext;
	integer set_key;
endclass

class generator;
	mailbox drv_mbx;
	event drv_done;
	transactions pkt;

	task run();

// 128 bit encryption and decryption
		

		pkt = new;
		pkt.key = 256'h00010203_04050607_08090a0b_0c0d0e0f_00000000_00000000_00000000_00000000;
		pkt.opcode = 4'b0010;
		pkt.set_key = 1;
		drv_mbx.put(pkt);
		@(drv_done);
		
		pkt = new;
		pkt.cptext = 128'h00112233_44556677_8899aabb_ccddeeff;
		pkt.opcode = 4'b0011;
		pkt.set_key = 0;
		drv_mbx.put(pkt);
		@(drv_done);
		
		pkt = new;
		pkt.cptext = 128'h69c4e0d8_6a7b0430_d8cdb780_70b4c55a;
		pkt.opcode = 4'b0001;
		pkt.set_key = 0;
		drv_mbx.put(pkt);
		@(drv_done);

		pkt = new;
		pkt.set_key=2;
		pkt.opcode = 4'b1010;
		drv_mbx.put(pkt);
		@(drv_done);
//192 bit encryption and decryption	
		pkt = new;
		pkt.key = 256'h00010203_04050607_08090a0b_0c0d0e0f_10111213_14151617_00000000_00000000;
		pkt.opcode = 4'b1010;
		pkt.set_key = 1;
		drv_mbx.put(pkt);
		@(drv_done);
		
		pkt = new;
		pkt.cptext = 128'h00112233_44556677_8899aabb_ccddeeff;
		pkt.opcode = 4'b1011;
		pkt.set_key = 0;
		drv_mbx.put(pkt);
		@(drv_done);
		
		pkt = new;
		pkt.cptext = 128'hdda97ca4_864cdfe0_6eaf70a0_ec0d7191;
		pkt.opcode = 4'b1001;
		pkt.set_key = 0;
		drv_mbx.put(pkt);
		@(drv_done);

		pkt = new;
		pkt.set_key=2;
		pkt.opcode = 4'b1110;
		drv_mbx.put(pkt);
		@(drv_done);
	
//256 bit encryption and decryption		
		pkt = new;
		pkt.key = 256'h00010203_04050607_08090a0b_0c0d0e0f_10111213_14151617_18191a1b_1c1d1e1f;
 		pkt.opcode = 4'b1110;
		pkt.set_key = 1;
		drv_mbx.put(pkt);
		@(drv_done);
		
		pkt = new;
		pkt.cptext = 128'h00112233_44556677_8899aabb_ccddeeff;
		pkt.opcode = 4'b1111;
		pkt.set_key = 0;
		drv_mbx.put(pkt);
		@(drv_done);
		
		pkt = new;
		pkt.cptext = 128'h8ea2b7ca_516745bf_eafc4990_4b496089;
		pkt.opcode = 4'b1101;
		pkt.set_key = 0;
		drv_mbx.put(pkt);
		@(drv_done);

		pkt = new;
		pkt.set_key=2;
		drv_mbx.put(pkt);
		@(drv_done);
	endtask
endclass

class driver;
	mailbox drv_mbx;
	event drv_done;
	virtual aes_if dvif;
	int n;
	int delay = 4;
	bit [4:0] addr;
	
	task run();
		int i;
		@(posedge dvif.clk)
		forever 
		begin
			transactions pkt;
			drv_mbx.get(pkt);
			if(pkt.set_key == 1) begin 
				
				dvif.opcd = pkt.opcode;
				n=32; i=32; 
				addr = 5'b0;
				dvif.srt = 0;
				dvif.io_rw=0;
				
				repeat(n)
				begin 
					@(posedge dvif.clk) 
					#delay
					$display("Input data %h",pkt.key[(8*i)-1 -: 8]);
					dvif.datain <= pkt.key[(8*i)-1 -: 8];
					dvif.io_addr <= addr;
					dvif.io_rw <= 1;
					addr = addr + 1'b1;
					i = i - 1;
				end
				dvif.io_rw = 0;
				dvif.srt = 1;
				#10000;
				dvif.srt = 0;
				->drv_done;
			end
			else if (pkt.set_key ==0 )begin 
				
			dvif.opcd = pkt.opcode;
				n=16; i=16; 
				addr = 5'b0;
				dvif.srt = 0;
				dvif.io_rw=0;
				
				repeat(n)
				begin 
					@(posedge dvif.clk) 
					#delay
					$display("Input data %h",pkt.cptext[(8*i)-1 -: 8]);
					dvif.datain <= pkt.cptext[(8*i)-1 -: 8];
					dvif.io_addr <= addr;
					dvif.io_rw <= 1;
					addr = addr + 1'b1;
					i = i - 1;
				end
				@(posedge dvif.clk) #delay
				dvif.io_addr = 5'b0;
				dvif.datain=0;
				dvif.io_rw = 0;
				dvif.srt = 1;
				@(posedge dvif.done) ; 
				dvif.srt=0;
				addr = 5'b0;
				repeat(n)
				begin
					@(posedge dvif.clk)
					#delay
					dvif.io_addr <= addr;
					dvif.io_rw <= 0;
					addr = addr + 1'b1;
				end
				->drv_done;
			end
			else begin
				dvif.rstn=0;
			 	dvif.opcd = pkt.opcode;
				#200;
				dvif.rstn=1;
				dvif.opcd = pkt.opcode;
				->drv_done;
			     end
			
		end
	endtask	
endclass

class env;
	driver d0;
//	monitor m0;
	generator g0;
//	scoreboard s0;
	
	mailbox drv_mbx;
//	mailbox scb_mbx;
	event drv_done;
	
	virtual aes_if vif;
	
	function new();
		d0 = new;
//		m0 = new;
		g0 = new;
//		s0 = new;
		drv_mbx = new;
		
		d0.drv_mbx = drv_mbx;
		g0.drv_mbx = drv_mbx;
		
		d0.drv_done = drv_done;
		g0.drv_done = drv_done;
	endfunction
	
	virtual task run();
		d0.dvif = vif;
//		m0.vif = vif;
		
		fork
			d0.run();
//			m0.run();
			g0.run();
//			s0.run();
		join_any
	endtask
endclass

class test;
	env e0;
	
	function new();
		e0 = new();
	endfunction
	
	virtual task run();
		e0.run();
	endtask

endclass

interface aes_if (input bit if_clk);
	logic       clk;
	logic       rstn;
	logic       io_rw;
	logic [7:0] datain;
	tri   [7:0] io_datainout;
	logic [4:0] io_addr;
	logic [3:0] opcd;
	logic       srt;
	logic       done;
	assign clk = if_clk;
endinterface

module testbench();
	reg clk;
	
	always #10 clk = ~clk;
	
	aes_if _if (clk);

	AES_PACM_TOP DUT (.IO_CLOCK(clk), 
			  .IO_RESET(_if.rstn), 
                          .IO_R_W(~_if.io_rw), 
                          .IO_DATAINOUT(_if.io_datainout), 
                          .IO_ADDR(_if.io_addr), 
                          .IO_DONE(_if.done), 
                          .IO_START(_if.srt), 
                          .IO_OPCODE(_if.opcd));
	
	assign _if.io_datainout = _if.io_rw ? _if.datain: 8'bz;
				  
	initial begin
		test t0;
		
		clk <= 0;
		_if.rstn <= 0;
		#40 _if.rstn <= 1;
		
		t0 = new;
		t0.e0.vif = _if;
		t0.run();
		
  		#200 $finish;
	end
	
endmodule
