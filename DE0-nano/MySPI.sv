//=======================================================

module MySPI (
	input  logic		 theClock, theReset,
	input  logic       MySPI_clk, MySPI_cs, MySPI_sdi,
	output logic  		 MySPI_sdo,
	output logic [16:0] Config,
	input  logic [16:0] Status,
	output logic [16:0] Led70,
	input  logic [16:0] IO_A_Data_In,    IO_B_Data_In,    IO_C_Data_In,    IO_D_Data_In,    IO_E_Data_In,    IO_F_Data_In,    IO_G_Data_In,    IO_H_Data_In,    IO_I_Data_In,    IO_J_Data_In,    IO_K_Data_In,    IO_L_Data_In,    IO_M_Data_In,		IO_N_Data_In,		IO_O_Data_In,		IO_P_Data_In,			IO_Q_Data_In,			IO_R_Data_In,
	output logic [16:0] IO_A_Data_Out,   IO_B_Data_Out,   IO_C_Data_Out,   IO_D_Data_Out,   IO_E_Data_Out,   IO_F_Data_Out,   IO_G_Data_Out,   IO_H_Data_Out,   IO_I_Data_Out,   IO_J_Data_Out,   IO_K_Data_Out,   IO_L_Data_Out,   IO_M_Data_Out,		IO_N_Data_Out,		IO_O_Data_Out,		IO_P_Data_Out,			IO_Q_Data_Out,			IO_R_Data_Out,
	output logic [16:0] IO_A_Enable_Out, IO_B_Enable_Out, IO_C_Enable_Out, IO_D_Enable_Out, IO_E_Enable_Out, IO_F_Enable_Out, IO_G_Enable_Out, IO_H_Enable_Out, IO_I_Enable_Out, IO_J_Enable_Out, IO_K_Enable_Out, IO_L_Enable_Out, IO_M_Enable_Out, 	IO_N_Enable_Out,	IO_O_Enable_Out, 	IO_P_Enable_Out,		IO_Q_Enable_Out,		IO_R_Enable_Out);

//--- Registers Address ---------------------------------

parameter A_Config     			= 15'h00;
parameter A_Status     			= 15'h01;
parameter A_Led70      			= 15'h02;
parameter A_IO_A_Data  			= 15'h10;
parameter A_IO_A_Enable_Out	= 15'h11;
parameter A_IO_B_Data  			= 15'h12;
parameter A_IO_B_Enable_Out	= 15'h13;
parameter A_IO_C_Data  			= 15'h14;
parameter A_IO_C_Enable_Out	= 15'h15;
parameter A_IO_D_Data  			= 15'h16;
parameter A_IO_D_Enable_Out	= 15'h17;
parameter A_IO_E_Data  			= 15'h18;
parameter A_IO_E_Enable_Out	= 15'h19;
parameter A_IO_F_Data  			= 15'h20;
parameter A_IO_F_Enable_Out	= 15'h21;
parameter A_IO_G_Data  			= 15'h22;
parameter A_IO_G_Enable_Out	= 15'h23;
parameter A_IO_H_Data  			= 15'h24;
parameter A_IO_H_Enable_Out	= 15'h25;
parameter A_IO_I_Data  			= 15'h26;
parameter A_IO_I_Enable_Out	= 15'h27;
parameter A_IO_J_Data  			= 15'h28;
parameter A_IO_J_Enable_Out	= 15'h29;
parameter A_IO_K_Data  			= 15'h30;
parameter A_IO_K_Enable_Out	= 15'h31;
parameter A_IO_L_Data  			= 15'h32;
parameter A_IO_L_Enable_Out	= 15'h33;
parameter A_IO_M_Data  			= 15'h34;
parameter A_IO_M_Enable_Out	= 15'h35;
parameter A_IO_N_Data  			= 15'h36;
parameter A_IO_N_Enable_Out	= 15'h37;
parameter A_IO_O_Data  			= 15'h38;
parameter A_IO_O_Enable_Out	= 15'h39;
parameter A_IO_P_Data  			= 15'h3a;
parameter A_IO_P_Enable_Out	= 15'h3b;
parameter A_IO_Q_Data  			= 15'h40;
parameter A_IO_Q_Enable_Out	= 15'h41;
parameter A_IO_R_Data  			= 15'h42;
parameter A_IO_R_Enable_Out	= 15'h43;

//--- FSM States ----------------------------------------

typedef enum logic [3:0] {
	S_Wait, 
	S_Addr, S_Addr_00, S_Addr_01, S_Addr_11,
	S_Data, S_Data_00, S_Data_01, S_Data_11,
	S_End} statetype;

//--- Declarations --------------------------------------

statetype	SPI_state, SPI_nextstate;
logic			SPI_CLK0, SPI_CLK;
logic			SPI_CS0, SPI_CS;
logic [3:0] SPI_counter; // ???
logic			SPI_counter_reset, SPI_counter_inc;	 
logic [15:0] SPI_address, SPI_data;
logic			SPI_address_shift;
logic			SPI_data_shift, SPI_data_load, SPI_data_update;

//--- SPI Output ----------------------------------------

assign MySPI_sdo = SPI_data[15];

//--- SPI Double Synchronization ------------------------

always @ (posedge theClock)
begin
	SPI_CLK0 <= MySPI_clk;	SPI_CLK  <= SPI_CLK0;
	SPI_CS0  <= MySPI_cs;	SPI_CS   <= SPI_CS0;
end


//--- SPI FSM -------------------------------------------

always_ff @ (posedge theClock)
	SPI_state <= SPI_nextstate;
	
always_comb
begin
	SPI_nextstate = SPI_state;
	case (SPI_state)
		S_Wait	 : if (SPI_CS) SPI_nextstate = S_Wait;
							else SPI_nextstate = S_Addr;
		S_Addr	 : SPI_nextstate = S_Addr_00;
		S_Addr_00 : if (SPI_CLK) SPI_nextstate = S_Addr_01;
		S_Addr_01 : SPI_nextstate = S_Addr_11;
		S_Addr_11 : if (SPI_CLK) SPI_nextstate = S_Addr_11;
							else if (SPI_counter == 4'b0000) SPI_nextstate = S_Data;
								else SPI_nextstate = S_Addr_00;
		S_Data	 : SPI_nextstate = S_Data_00;
		S_Data_00 : if (SPI_CLK) SPI_nextstate = S_Data_01;
		S_Data_01 : SPI_nextstate = S_Data_11;
		S_Data_11 : if (SPI_CLK) SPI_nextstate = S_Data_11;
							else if (SPI_counter == 4'b0000) SPI_nextstate = S_End;
								else SPI_nextstate = S_Data_00;
		S_End     : SPI_nextstate = S_Wait;
	endcase
	if (SPI_CS) SPI_nextstate = S_Wait;
end

assign SPI_counter_reset = ((SPI_state == S_Addr)    | (SPI_state == S_Data));
assign SPI_counter_inc   = ((SPI_state == S_Addr_01) | (SPI_state == S_Data_01));
assign SPI_address_shift = (SPI_state == S_Addr_01);
assign SPI_data_shift	 = (SPI_state == S_Data_01);
assign SPI_data_load		 = (SPI_state == S_Data);
assign SPI_data_update   = ((SPI_state == S_End) & SPI_address[7]);

//--- On the positive edge of the clock -----------------

always_ff @ (posedge theClock)
begin
	
	if (SPI_counter_reset) SPI_counter <= 4'b0000;
		else if (SPI_counter_inc) SPI_counter <= SPI_counter + 1;
		
	if (SPI_address_shift) SPI_address <= { SPI_address[14:0], MySPI_sdi };
	
	if (SPI_data_shift) SPI_data <= { SPI_data[14:0], MySPI_sdi };
		else if (SPI_data_load)
			case (SPI_address[14:0])
				A_Config    		: SPI_data <= Config;
				A_Status    		: SPI_data <= Status;
				A_Led70     		: SPI_data <= Led70;
				A_IO_A_Data   		: SPI_data <= IO_A_Data_In;
				A_IO_A_Enable_Out : SPI_data <= IO_A_Enable_Out;
				A_IO_B_Data 		: SPI_data <= IO_B_Data_In;
				A_IO_B_Enable_Out : SPI_data <= IO_B_Enable_Out;
				A_IO_C_Data 		: SPI_data <= IO_C_Data_In;
				A_IO_C_Enable_Out : SPI_data <= IO_C_Enable_Out;
				A_IO_D_Data 		: SPI_data <= IO_D_Data_In;
				A_IO_D_Enable_Out : SPI_data <= IO_D_Enable_Out;
				A_IO_E_Data 		: SPI_data <= IO_E_Data_In;
				A_IO_E_Enable_Out : SPI_data <= IO_E_Enable_Out;
				A_IO_F_Data 		: SPI_data <= IO_F_Data_In;
				A_IO_F_Enable_Out : SPI_data <= IO_F_Enable_Out;
				A_IO_G_Data 		: SPI_data <= IO_G_Data_In;
				A_IO_G_Enable_Out : SPI_data <= IO_G_Enable_Out;
				A_IO_H_Data 		: SPI_data <= IO_H_Data_In;
				A_IO_H_Enable_Out : SPI_data <= IO_H_Enable_Out;
				A_IO_I_Data 		: SPI_data <= IO_I_Data_In;
				A_IO_I_Enable_Out : SPI_data <= IO_I_Enable_Out;
				A_IO_J_Data 		: SPI_data <= IO_J_Data_In;
				A_IO_J_Enable_Out : SPI_data <= IO_J_Enable_Out;
				A_IO_K_Data 		: SPI_data <= IO_K_Data_In;
				A_IO_K_Enable_Out : SPI_data <= IO_K_Enable_Out;
				A_IO_L_Data 		: SPI_data <= IO_L_Data_In;
				A_IO_L_Enable_Out : SPI_data <= IO_L_Enable_Out;
				A_IO_M_Data 		: SPI_data <= IO_M_Data_In;
				A_IO_M_Enable_Out : SPI_data <= IO_M_Enable_Out;
				A_IO_N_Data 		: SPI_data <= IO_N_Data_In;
				A_IO_N_Enable_Out : SPI_data <= IO_N_Enable_Out;
				A_IO_O_Data 		: SPI_data <= IO_O_Data_In;
				A_IO_O_Enable_Out : SPI_data <= IO_O_Enable_Out;
				A_IO_P_Data 		: SPI_data <= IO_P_Data_In;
				A_IO_P_Enable_Out : SPI_data <= IO_P_Enable_Out;
				A_IO_Q_Data 		: SPI_data <= IO_Q_Data_In;
				A_IO_Q_Enable_Out : SPI_data <= IO_Q_Enable_Out;
				A_IO_R_Data 		: SPI_data <= IO_R_Data_In;
				A_IO_R_Enable_Out : SPI_data <= IO_R_Enable_Out;
			endcase
		
	if (theReset) Config <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_Config)) Config <= SPI_data;
		
	if (theReset) Led70 <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_Led70)) Led70 <= SPI_data;	

	if (theReset) IO_A_Enable_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_A_Enable_Out)) IO_A_Enable_Out <= SPI_data;	
	if (theReset) IO_B_Enable_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_B_Enable_Out)) IO_B_Enable_Out <= SPI_data;
	if (theReset) IO_C_Enable_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_C_Enable_Out)) IO_C_Enable_Out <= SPI_data;
	if (theReset) IO_D_Enable_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_D_Enable_Out)) IO_D_Enable_Out <= SPI_data;
	if (theReset) IO_E_Enable_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_E_Enable_Out)) IO_E_Enable_Out <= SPI_data;
	if (theReset) IO_F_Enable_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_F_Enable_Out)) IO_F_Enable_Out <= SPI_data;
	if (theReset) IO_G_Enable_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_G_Enable_Out)) IO_G_Enable_Out <= SPI_data;
	if (theReset) IO_H_Enable_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_H_Enable_Out)) IO_H_Enable_Out <= SPI_data;
	if (theReset) IO_I_Enable_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_I_Enable_Out)) IO_I_Enable_Out <= SPI_data;
	if (theReset) IO_J_Enable_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_J_Enable_Out)) IO_J_Enable_Out <= SPI_data;
	if (theReset) IO_K_Enable_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_K_Enable_Out)) IO_K_Enable_Out <= SPI_data;
	if (theReset) IO_L_Enable_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_L_Enable_Out)) IO_L_Enable_Out <= SPI_data;
	if (theReset) IO_M_Enable_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_M_Enable_Out)) IO_M_Enable_Out <= SPI_data;
	if (theReset) IO_N_Enable_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_N_Enable_Out)) IO_N_Enable_Out <= SPI_data;
	if (theReset) IO_O_Enable_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_O_Enable_Out)) IO_O_Enable_Out <= SPI_data;
	if (theReset) IO_P_Enable_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_P_Enable_Out)) IO_P_Enable_Out <= SPI_data;
	if (theReset) IO_Q_Enable_Out <= 16'h00;
	else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_Q_Enable_Out)) IO_Q_Enable_Out <= SPI_data;
	if (theReset) IO_R_Enable_Out <= 16'h00;
	else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_R_Enable_Out)) IO_R_Enable_Out <= SPI_data;
		
	if (theReset) IO_A_Data_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_A_Data)) IO_A_Data_Out <= SPI_data;	
	if (theReset) IO_B_Data_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_B_Data)) IO_B_Data_Out <= SPI_data;
	if (theReset) IO_C_Data_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_C_Data)) IO_C_Data_Out <= SPI_data;
	if (theReset) IO_D_Data_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_D_Data)) IO_D_Data_Out <= SPI_data;
	if (theReset) IO_E_Data_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_E_Data)) IO_E_Data_Out <= SPI_data;
	if (theReset) IO_F_Data_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_F_Data)) IO_F_Data_Out <= SPI_data;
	if (theReset) IO_G_Data_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_G_Data)) IO_G_Data_Out <= SPI_data;
	if (theReset) IO_H_Data_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_H_Data)) IO_H_Data_Out <= SPI_data;
	if (theReset) IO_I_Data_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_I_Data)) IO_I_Data_Out <= SPI_data;
	if (theReset) IO_J_Data_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_J_Data)) IO_J_Data_Out <= SPI_data;
	if (theReset) IO_K_Data_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_K_Data)) IO_K_Data_Out <= SPI_data;
	if (theReset) IO_L_Data_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_L_Data)) IO_L_Data_Out <= SPI_data;
	if (theReset) IO_M_Data_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_M_Data)) IO_M_Data_Out <= SPI_data;
	if (theReset) IO_N_Data_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_N_Data)) IO_N_Data_Out <= SPI_data;
	if (theReset) IO_O_Data_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_O_Data)) IO_O_Data_Out <= SPI_data;
	if (theReset) IO_P_Data_Out <= 16'h00;
		else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_P_Data)) IO_P_Data_Out <= SPI_data;
	if (theReset) IO_Q_Data_Out <= 16'h00;
	else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_Q_Data)) IO_Q_Data_Out <= SPI_data;
	if (theReset) IO_R_Data_Out <= 16'h00;
	else if ((SPI_data_update) & (SPI_address[14:0] == A_IO_R_Data)) IO_R_Data_Out <= SPI_data;
		
end

endmodule

//=======================================================