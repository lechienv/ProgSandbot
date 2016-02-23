//=======================================================

module MySPI (
	input  logic		 theClock, theReset,
	input  logic       MySPI_clk, MySPI_cs, MySPI_sdi,
	output logic  		 MySPI_sdo,
	output logic [8:0] Config,
	input  logic [8:0] Status,
	output logic [8:0] Led70,
	input  logic [8:0] IO_A_Data_In, IO_B_Data_In, IO_C_Data_In, IO_D_Data_In,
	output logic [8:0] IO_A_Data_Out, IO_B_Data_Out, IO_C_Data_Out, IO_D_Data_Out,
	output logic [8:0] IO_A_Enable_Out, IO_B_Enable_Out, IO_C_Enable_Out, IO_D_Enable_Out
);

//--- Registers Address ---------------------------------

parameter A_Config     			= 7'h00;
parameter A_Status     			= 7'h01;
parameter A_Led70      			= 7'h02;
parameter A_IO_A_Data  			= 7'h10;
parameter A_IO_A_Enable_Out	= 7'h11;
parameter A_IO_B_Data  			= 7'h20;
parameter A_IO_B_Enable_Out	= 7'h21;
parameter A_IO_C_Data  			= 7'h30;
parameter A_IO_C_Enable_Out	= 7'h31;
parameter A_IO_D_Data  			= 7'h40;
parameter A_IO_D_Enable_Out	= 7'h41;

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
logic [2:0] SPI_counter;
logic			SPI_counter_reset, SPI_counter_inc;	 
logic [7:0] SPI_address, SPI_data;
logic			SPI_address_shift;
logic			SPI_data_shift, SPI_data_load, SPI_data_update;

//--- SPI Output ----------------------------------------

assign MySPI_sdo = SPI_data[7];

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
							else if (SPI_counter == 3'b000) SPI_nextstate = S_Data;
								else SPI_nextstate = S_Addr_00;
		S_Data	 : SPI_nextstate = S_Data_00;
		S_Data_00 : if (SPI_CLK) SPI_nextstate = S_Data_01;
		S_Data_01 : SPI_nextstate = S_Data_11;
		S_Data_11 : if (SPI_CLK) SPI_nextstate = S_Data_11;
							else if (SPI_counter == 3'b000) SPI_nextstate = S_End;
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
	
	if (SPI_counter_reset) SPI_counter <= 3'b000;
		else if (SPI_counter_inc) SPI_counter <= SPI_counter + 1;
		
	if (SPI_address_shift) SPI_address <= { SPI_address[6:0], MySPI_sdi };
	
	if (SPI_data_shift) SPI_data <= { SPI_data[6:0], MySPI_sdi };
		else if (SPI_data_load)
			case (SPI_address[6:0])
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
			endcase
		
	if (theReset) Config <= 8'h00;
		else if ((SPI_data_update) & (SPI_address[6:0] == A_Config)) Config <= SPI_data;
		
	if (theReset) Led70 <= 8'h00;
		else if ((SPI_data_update) & (SPI_address[6:0] == A_Led70)) Led70 <= SPI_data;	

	if (theReset) IO_A_Enable_Out <= 8'h00;
		else if ((SPI_data_update) & (SPI_address[6:0] == A_IO_A_Enable_Out)) IO_A_Enable_Out <= SPI_data;	
	if (theReset) IO_B_Enable_Out <= 8'h00;
		else if ((SPI_data_update) & (SPI_address[6:0] == A_IO_B_Enable_Out)) IO_B_Enable_Out <= SPI_data;
	if (theReset) IO_C_Enable_Out <= 8'h00;
		else if ((SPI_data_update) & (SPI_address[6:0] == A_IO_C_Enable_Out)) IO_C_Enable_Out <= SPI_data;
	if (theReset) IO_D_Enable_Out <= 8'h00;
		else if ((SPI_data_update) & (SPI_address[6:0] == A_IO_D_Enable_Out)) IO_D_Enable_Out <= SPI_data;

	if (theReset) IO_A_Data_Out <= 8'h00;
		else if ((SPI_data_update) & (SPI_address[6:0] == A_IO_A_Data)) IO_A_Data_Out <= SPI_data;	
	if (theReset) IO_B_Data_Out <= 8'h00;
		else if ((SPI_data_update) & (SPI_address[6:0] == A_IO_B_Data)) IO_B_Data_Out <= SPI_data;
	if (theReset) IO_C_Data_Out <= 8'h00;
		else if ((SPI_data_update) & (SPI_address[6:0] == A_IO_C_Data)) IO_C_Data_Out <= SPI_data;
	if (theReset) IO_D_Data_Out <= 8'h00;
		else if ((SPI_data_update) & (SPI_address[6:0] == A_IO_D_Data)) IO_D_Data_Out <= SPI_data;
	
end

endmodule

//=======================================================