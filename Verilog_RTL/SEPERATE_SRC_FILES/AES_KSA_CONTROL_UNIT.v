

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
