
				 

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

