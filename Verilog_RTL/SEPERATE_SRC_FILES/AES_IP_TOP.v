
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

