
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

