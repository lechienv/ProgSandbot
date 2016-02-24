`timescale 1ns/1ps

//=======================================================
//
// MYCycloneMips
//
//-------------------------------------------------------

module DE0_NANO(
//////////// CLOCK //////////
input logic		     	CLOCK_50,
//////////// LED //////////
output logic	[7:0]	LED,
//////////// KEY //////////
input logic		[1:0]	KEY,
//////////// SW //////////
input logic		[3:0]	SW,
//////////// 2x13 GPIO Header //////////
inout logic	   [12:0] GPIO_2,
input logic		[2:0]	 GPIO_2_IN,
//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
inout wire		[33:0] GPIO_0,
input logic		[1:0]	 GPIO_0_IN,
//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
inout wire		[33:0] GPIO_1,
input wire		[1:0]	 GPIO_1_IN);


//=======================================================
//   PIC32 Interface
//=======================================================

//--- Declarations --------------------------------------


logic	PIC32_SDO1A, PIC32_SDI1A, PIC32_SCK1A, PIC32_CS_FPGA;
logic	PIC32_INT1, PIC32_INT2;
logic	PIC32_C1TX, PIC32_C1RX;
logic	PIC32_SCL3A, PIC32_SDA3A;
logic	PIC32_RESET;

logic [15:0] Config;
logic [15:0] Status;
logic [15:0] Led70;
logic [15:0] IO_A_Data_In, IO_A_Data_Out, IO_A_Enable_Out;
logic [15:0] IO_B_Data_In, IO_B_Data_Out, IO_B_Enable_Out;
logic [15:0] IO_C_Data_In, IO_C_Data_Out, IO_C_Enable_Out;
logic [15:0] IO_D_Data_In, IO_D_Data_Out, IO_D_Enable_Out;
logic [15:0] IO_E_Data_In, IO_E_Data_Out, IO_E_Enable_Out;
logic [15:0] IO_F_Data_In, IO_F_Data_Out, IO_F_Enable_Out;
logic [15:0] IO_G_Data_In, IO_G_Data_Out, IO_G_Enable_Out;
logic [15:0] IO_H_Data_In, IO_H_Data_Out, IO_H_Enable_Out;
logic [15:0] IO_I_Data_In, IO_I_Data_Out, IO_I_Enable_Out;
logic [15:0] IO_J_Data_In, IO_J_Data_Out, IO_J_Enable_Out;

genvar i;

//---- Assign GPIO_2 Header (connected to PIC32) --------

assign PIC32_SDO1A	= GPIO_2[0];
assign GPIO_2[1]		= PIC32_CS_FPGA ? 1'bz : PIC32_SDI1A;
assign PIC32_SCK1A	= GPIO_2[2];
assign PIC32_CS_FPGA	= GPIO_2[3];

assign GPIO_2[4]     = PIC32_INT1;
assign GPIO_2[5]     = PIC32_INT2;

assign PIC32_C1TX		= GPIO_2[6];
assign GPIO_2[7]		= Config[6] ? PIC32_C1RX : 1'bz;

assign PIC32_SCL3A	= GPIO_2[8];
assign PIC32_SDA3A	= GPIO_2[9];

assign PIC32_RESET	= GPIO_2[10];
//assign PIC32_RESET	= ~KEY[0];


//--- Assign GPIO_1 Header -----------------------------

assign GPIO_1[0]     = Config[4];
assign GPIO_1[33]    = Config[5];


assign PIC32_C1RX    = GPIO_1[29];
assign GPIO_1[30]    = PIC32_C1TX;
assign GPIO_1[31]    = PIC32_SCL3A;
assign GPIO_1[32]    = PIC32_SDA3A;

//--- Assign Status, INT ---------------------------------

assign PIC32_INT1 = Config[0] ? KEY[0] : 1'b1;
assign PIC32_INT2 = Config[1] ? KEY[1] : 1'b1;

always @ (posedge CLOCK_50)
	Status = {SW, 2'b00, KEY};

//--- Assign signals FPGA -> PIC -------------------------

logic LaserSign, LaserSignSansGlitch, LaserSync, LaserCodeurA, LaserCodeurB;
logic PropLeftCodeurA, PropLeftCodeurB, PropRightCodeurA, PropRightCodeurB;
logic UartTx, UartRx, UartDir;

assign LaserSync = GPIO_1[17];
assign LaserSign = GPIO_1[18];
assign LaserCodeurA = GPIO_1[5];
assign LaserCodeurB = GPIO_1[6];

assign PropLeftCodeurA = GPIO_1[1];
assign PropLeftCodeurB = GPIO_1[2];
assign PropRightCodeurA = GPIO_1[3];
assign PropRightCodeurB = GPIO_1[4];
/*
assign UartTx = GPIO_1[24];
assign UartRx = GPIO_1[25];
assign UartDir = GPIO_1[26];
*/

//--- Gestion LED ---------------------------------------

assign LED = {~LaserSign, LaserSync, LaserCodeurA, LaserCodeurB, PropLeftCodeurA, PropLeftCodeurB, PropRightCodeurA, PropRightCodeurB}; //


//--- Gestion Glitch et Compteur Codeur -----------------

logic [15:0] CompteurLaser, CompteurLaserBis;
reg   [15:0] CompteurMax, DebutReception, FinReception, CompteTour;
reg   [31:0] CompteurTest;
reg 			 positiveSpeedL, positiveSpeedR;
logic Send, NoSign, TempSign; 

counter #(16) CompteurTourelle(LaserCodeurA, LaserSync, CompteurLaser);

GlitchHandler GlitchSignLaser(CLOCK_50, PIC32_RESET,  ~LaserSign, LaserSync, CompteurLaser, DebutReception, FinReception, Send, LaserCodeurA);

always_ff @(posedge LaserCodeurA)
		CompteurLaserBis <= CompteurLaser;
		
always_ff @(posedge LaserSync)
	CompteurMax <= CompteurLaserBis;
	
always_ff @(posedge LaserSync, posedge LaserSign) 
begin
	if(LaserSync) TempSign <= 1'b1;
	//if(LaserSign) TempSign <= 1'b0; 
	else TempSign <= 0;
end

always_ff @(posedge LaserSync) 
begin
	NoSign <= TempSign;
end
	
// --- Vitesse des roues --------------------------------

logic [15:0]  CompteurCodeurLeft, CompteurCodeurRight, CompteurCodeurLeftB, CompteurCodeurRightB;
logic [31:0]  CompteurVitesse;
reg   [15:0] VitesseLeft, VitesseRight;
logic ResetCompteurVitesse; 

counter #(32) CompteurTemps(CLOCK_50,  ResetCompteurVitesse, CompteurVitesse);
counter #(16) ComptCodLeft (PropLeftCodeurA,  ResetCompteurVitesse, CompteurCodeurLeft);
counter #(16) ComptCodRight(PropRightCodeurA, ResetCompteurVitesse, CompteurCodeurRight);
counter #(16) ComptCodLeftB (PropLeftCodeurB,  ResetCompteurVitesse, CompteurCodeurLeftB);
counter #(16) ComptCodRightB(PropRightCodeurB, ResetCompteurVitesse, CompteurCodeurRightB);
counter #(32) CompteurTesting(PropLeftCodeurA, ~KEY[1], CompteurTest);
counter #(16) CompteTourTourelle(LaserSync, PIC32_RESET, CompteTour);

always_ff@(posedge PropLeftCodeurA)
begin 
	if(PropLeftCodeurB)
		positiveSpeedL <= 'b1;
	else
		positiveSpeedL <= 'b0;
	
end
always_ff@(posedge PropRightCodeurA)
begin 
	if(PropRightCodeurB)
		positiveSpeedR <= 'b0;
	else
		positiveSpeedR <= 'b1;
	
end
	
always_ff@(posedge CLOCK_50)
begin
	if(CompteurVitesse == 32'd500000) 
		begin 
			VitesseLeft <= (CompteurCodeurLeft + CompteurCodeurLeftB)/2; 
			VitesseRight <= (CompteurCodeurRight + CompteurCodeurRightB)/2; 
		end
	else if (CompteurVitesse == 32'd500001) 
			ResetCompteurVitesse <= 1'b1; 
	else 
			ResetCompteurVitesse <= 1'b0;
end


//--- Envoi data ----------------------------------------

always @(posedge CLOCK_50)
begin 
	IO_A_Data_In <= CompteurMax[15:0];//CompteurMax[15:8];
	IO_B_Data_In <= (NoSign)? 16'b00000000 : DebutReception[15:0];//CompteurMax[7:0];
	IO_C_Data_In <= (NoSign)? 16'b00000000 : FinReception[15:0];//(NoSign)? 8'b00000000 : DebutReception[15:8]; 
	IO_D_Data_In <= VitesseRight[15:0];//(NoSign)? 8'b00000000:DebutReception[7:0]; 
	IO_E_Data_In <= VitesseLeft[15:0];//(NoSign)? 8'b00000000:FinReception[15:8]; 
	IO_F_Data_In <= CompteurTest[15:0];
	IO_G_Data_In <= CompteurTest[31:16];
	IO_H_Data_In <= {14'b0, positiveSpeedR, positiveSpeedL};
	IO_I_Data_In <= CompteTour[15:0];
	/*IO_F_Data_In <=(NoSign)? 8'b00000000:FinReception[7:0]; 
	IO_G_Data_In <= VitesseRight[15:8]; 
	IO_H_Data_In <= VitesseRight[7:0]; 
	IO_I_Data_In <= VitesseLeft[15:8]; 
	IO_J_Data_In <= VitesseLeft[7:0]; 
	*/
end


//--- Clock Test ----------------------------------------

logic clk_Test, unused;
	
ClockTest	ClockTest_inst (
	.areset ( PIC32_RESET ),
	.inclk0 ( CLOCK_50 ),
	.c0 ( clk_Test ),
	.locked ( unused )
	);


//--- SPI Interface -------------------------------------

MySPI MySPI_instance (
	.theClock(CLOCK_50), .theReset(PIC32_RESET),
	.MySPI_clk(PIC32_SCK1A), .MySPI_cs(PIC32_CS_FPGA), .MySPI_sdi(PIC32_SDO1A), .MySPI_sdo(PIC32_SDI1A),
	.Config(Config),
	.Status(Status),
	.Led70(Led70),
	.IO_A_Data_In(IO_A_Data_In), 			.IO_B_Data_In(IO_B_Data_In), 			.IO_C_Data_In(IO_C_Data_In), 			.IO_D_Data_In(IO_D_Data_In),			.IO_E_Data_In(IO_E_Data_In),			.IO_F_Data_In(IO_F_Data_In),			.IO_G_Data_In(IO_G_Data_In),			.IO_H_Data_In(IO_H_Data_In),			.IO_I_Data_In(IO_I_Data_In),			.IO_J_Data_In(IO_J_Data_In),
	.IO_A_Data_Out(IO_A_Data_Out), 		.IO_B_Data_Out(IO_B_Data_Out), 		.IO_C_Data_Out(IO_C_Data_Out), 		.IO_D_Data_Out(IO_D_Data_Out),		.IO_E_Data_Out(IO_E_Data_Out),		.IO_F_Data_Out(IO_F_Data_Out),		.IO_G_Data_Out(IO_G_Data_Out),		.IO_H_Data_Out(IO_H_Data_Out),		.IO_I_Data_Out(IO_I_Data_Out),		.IO_J_Data_Out(IO_J_Data_Out),
	.IO_A_Enable_Out(IO_A_Enable_Out), 	.IO_B_Enable_Out(IO_B_Enable_Out), 	.IO_C_Enable_Out(IO_C_Enable_Out), 	.IO_D_Enable_Out(IO_D_Enable_Out), 	.IO_E_Enable_Out(IO_E_Enable_Out), 	.IO_F_Enable_Out(IO_F_Enable_Out), 	.IO_G_Enable_Out(IO_G_Enable_Out), 	.IO_H_Enable_Out(IO_H_Enable_Out), 	.IO_I_Enable_Out(IO_I_Enable_Out), 	.IO_J_Enable_Out(IO_J_Enable_Out));


endmodule
  
/*
=======================================================
=====================  Compteur  ======================
=======================================================
*/

module counter #(	parameter bits = 32	)
					 ( input 	 	logic clk,
					   input			logic	reset,
						output reg	[bits-1:0]	count	);

always_ff @ (posedge clk, posedge reset)
	begin
		if(reset | (count === 'x)) count	<=	'b0;
		else count <= count + 'b1;
	end	    
	
endmodule  

/*
=======================================================
=====================  Compteur 6clk ==================
=======================================================
*/

module counter20Clk ( input logic clk,
			input logic reset,
			output logic flag);
logic [5:0] count;
always_ff @ (posedge clk, posedge reset)
	begin
		if(reset | (count === 'x)) begin count <= 6'b0; flag <= 1'b1; end 
		else if(count >= 6'd20 & flag == 1'b1 & count < 6'd22) begin count <= 6'd23; flag <= 1'b0; end
		else if((count === 6'b0) | (count === 6'd1) | (count === 6'd2) | (count === 6'd3) | (count === 6'd4) | (count === 6'd5) | (count === 6'd6) | (count === 6'd7) | (count === 6'd8) | (count === 6'd9) | (count === 6'd10) | (count === 6'd11) | (count === 6'd12) | (count === 6'd13) | (count === 6'd14) | (count === 6'd15) | (count === 6'd16) | (count === 6'd17) | (count === 6'd18) | (count === 6'd19)) begin count <= count + 6'd1; flag <= 1'b1; end
		else count <= 6'd23;
	end	    
	
endmodule  

/*
=======================================================
==================  GlitchHandler  ====================
=======================================================
*/

module GlitchHandler(input logic clk,
                     input logic reset,
                     input logic Sign,
							input logic Sync,
							input logic [15:0] NbrCodeur,
							output reg [15:0] DebutReception,
							output reg [15:0] FinReception,
							output logic Send,
							input logic LaserCodeurA);

logic [15:0] Buffer;
logic Flag, resetCount;
							
typedef enum logic [1:0] {Wait, Reception, Glitch} statetype;
statetype state, nextstate;

always_ff @(posedge clk)
	if (reset) state <= Wait;
	else state <= nextstate;

always
	begin
		case(state)
			Wait: begin 
						if(Sign)  nextstate = Reception;
						else nextstate = Wait;
						Send = 1'b0;
						resetCount <= 1'b0;
			      end
					
			Reception: begin
								if(Sign == 1'b0) begin resetCount <= 1'b1; nextstate <= Glitch; end
								else nextstate = Reception;
								Send = 1'b0; 
						   end 
						  
			Glitch: begin
						if(Flag == 1'b0) begin nextstate = Wait; Send =1'b1; end
						else if (Sign == 1'b1) nextstate = Reception;
						else nextstate = Glitch;
						resetCount <= 1'b0;
					  end
		endcase
	end

always//_ff @(posedge clk, posedge Sign)
begin
	if((Sign == 1'b1) & (state == Wait)) DebutReception <= NbrCodeur;
	else if((Sign == 1'b0) & (state == Reception)) Buffer <= NbrCodeur;
	else if((Flag == 1'b0) & (state == Glitch)) FinReception <= Buffer;
end

counter20Clk Compteur(LaserCodeurA, resetCount, Flag);
endmodule
