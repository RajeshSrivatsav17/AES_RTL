
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
