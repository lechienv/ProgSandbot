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
//////////// SDRAM //////////
output		    [12:0]		DRAM_ADDR,
output		     [1:0]		DRAM_BA,
output		          		DRAM_CAS_N,
output		          		DRAM_CKE,
output		          		DRAM_CLK,
output		          		DRAM_CS_N,
inout 		    [15:0]		DRAM_DQ,
output		     [1:0]		DRAM_DQM,
output		          		DRAM_RAS_N,
output		          		DRAM_WE_N,
//////////// EPCS //////////
output		          		EPCS_ASDO,
input 		          		EPCS_DATA0,
output		          		EPCS_DCLK,
output		          		EPCS_NCSO,
//////////// Accelerometer and EEPROM //////////
output		          		G_SENSOR_CS_N,
input 		          		G_SENSOR_INT,
output							I2C_SCLK,
inout 		          		I2C_SDAT,
//////////// ADC //////////
output		          		ADC_CS_N,
output		          		ADC_SADDR,
output		          		ADC_SCLK,
input 		          		ADC_SDAT,
//////////// 2x13 GPIO Header //////////
inout logic	   [12:0] GPIO_2,
input logic		[2:0]	 GPIO_2_IN,
//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
inout wire		[33:0] GPIO_0,
input logic		[1:0]	 GPIO_0_IN,
//////////// GPIO_1 to LT24 //////////
output		          		LT24_ADC_CS_N,
output		          		LT24_ADC_DCLK,
output		          		LT24_ADC_DIN,
input		          		   LT24_ADC_BUSY,
input		          		   LT24_ADC_DOUT,
input		          		   LT24_ADC_PENIRQ_N,
output		    [15:0]		LT24_D,
output		          		LT24_WR_N,
output		          		LT24_RD_N,
output		         		LT24_CS_N,
output		         		LT24_RESET_N,
output		         		LT24_RS,
output		         		LT24_LCD_ON
);

//=======================================================
//   TL24 Interface
//=======================================================

wire				[7:0]			ResponseFromLT24;

wire		          			LT24_ADC_CS_N_bus;
wire		          			LT24_ADC_DCLK_bus;
wire		          			LT24_ADC_DIN_bus;
wire		          			LT24_ADC_BUSY_bus;
wire	          		   	LT24_ADC_DOUT_bus;
wire	          		   	LT24_ADC_PENIRQ_N_bus;
wire		    	[15:0]		LT24_D_bus;
wire		          			LT24_WR_N_bus;
wire		          			LT24_RD_N_bus;
wire		         			LT24_CS_N_bus;
wire		         			LT24_RESET_N_bus;
wire		         			LT24_RS_bus;
wire  	         			LT24_LCD_ON_bus;


wire [11:0] pic_mem_s2_address;
wire        pic_mem_s2_chipselect; 
wire        pic_mem_s2_clken;     
wire        pic_mem_s2_write;          
wire [15:0] pic_mem_s2_readdata;    
wire [15:0] pic_mem_s2_writedata;           
wire [1:0]  pic_mem_s2_byteenable;

wire [12:0] background_mem_s2_address;                   
wire        background_mem_s2_chipselect;               
wire        background_mem_s2_clken;                     
wire        background_mem_s2_write;                    
wire [15:0] background_mem_s2_readdata;                 
wire [15:0] background_mem_s2_writedata;                
wire [1:0]  background_mem_s2_byteenable; 

wire 			lt24_buffer_flag;           
wire RST_N;
wire [7:0] illegalLED;

assign RST_N       = KEY[1];
assign LT24_LCD_ON = 1'b1; //default on

DE0_LT24_SOPC DE0_LT24_SOPC_inst(
		.clk_clk(CLOCK_50),          							//        clk.clk
		.reset_reset_n(RST_N),    								//      reset.reset_n
		
		// SDRAM
		.sdram_wire_addr(DRAM_ADDR),  						// sdram_wire.addr
		.sdram_wire_ba(DRAM_BA),    							//           .ba
		.sdram_wire_cas_n(DRAM_CAS_N), 						//           .cas_n
		.sdram_wire_cke(DRAM_CKE),  							//           .cke
		.sdram_wire_cs_n(DRAM_CS_N),  						//           .cs_n
		.sdram_wire_dq(DRAM_DQ),    							//           .dq
		.sdram_wire_dqm(DRAM_DQM),   							//           .dqm
		.sdram_wire_ras_n(DRAM_RAS_N),						//           .ras_n
		.sdram_wire_we_n(DRAM_WE_N),  						//           .we_n
		
		// KEY
		.from_key_export(KEY[0]),  							//   from_key.export
		
		// LEDS
		.to_led_export(illegalLED),                 				//   to_led.export
		
		// LT24 - LCD
		.lt24_conduit_cs(LT24_CS_N_bus),      					//  lt24_conduit.cs
		.lt24_conduit_rs(LT24_RS_bus),     						//              .rs
		.lt24_conduit_rd(LT24_RD_N_bus),      					//              .rd
		.lt24_conduit_wr(LT24_WR_N_bus),      					//              .wr
		.lt24_conduit_data(LT24_D_bus),    	   				//              .data
		.lt24_lcd_rstn_export(LT24_RESET_N_bus),				//       lt24_lcd_rstn.export
		
		// LT24 - TOUCH
		.lt24_touch_spi_MISO(LT24_ADC_DOUT_bus),        		//      lt24_touch_spi.MISO
		.lt24_touch_spi_MOSI(LT24_ADC_DIN_bus),        		//                    .MOSI
		.lt24_touch_spi_SCLK(LT24_ADC_DCLK_bus),        		//                    .SCLK
		.lt24_touch_spi_SS_n(LT24_ADC_CS_N_bus),       		//                    .SS_n
		.lt24_touch_penirq_n_export(LT24_ADC_PENIRQ_N_bus), // lt24_touch_penirq_n.export
		.lt24_touch_busy_export(LT24_ADC_BUSY_bus),         //           lt24_touch_busy.export
		
		// PLL
		.alt_pll_c4_conduit_export(),        				//        alt_pll_c4_conduit.export
		.alt_pll_c3_conduit_export(),        				//        alt_pll_c3_conduit.export
		.alt_pll_areset_conduit_export(),    				//    alt_pll_areset_conduit.export
		.alt_pll_locked_conduit_export(),    				//    alt_pll_locked_conduit.export
		.alt_pll_phasedone_conduit_export(),  				// alt_pll_phasedone_conduit.export
		.alt_pll_c1_clk(DRAM_CLK),                      //                alt_pll_c1.clk  
		
		.pic_mem_s2_address(pic_mem_s2_address),               //                pic_mem_s2.address
		.pic_mem_s2_chipselect(pic_mem_s2_chipselect),            //                          .chipselect
		.pic_mem_s2_clken(pic_mem_s2_clken),                 //                          .clken
		.pic_mem_s2_write(pic_mem_s2_write),                 //                          .write
		.pic_mem_s2_readdata(pic_mem_s2_readdata),              //                          .readdata
		.pic_mem_s2_writedata(pic_mem_s2_writedata),             //                          .writedata
		.pic_mem_s2_byteenable(pic_mem_s2_byteenable),            //                          .byteenable
		
		.lt24_buffer_flag_external_connection_export(lt24_buffer_flag),
		
		.background_mem_s2_address(background_mem_s2_address),                   
		.background_mem_s2_chipselect(background_mem_s2_chipselect),                
		.background_mem_s2_clken(background_mem_s2_clken),                     
		.background_mem_s2_write(background_mem_s2_write),                     
		.background_mem_s2_readdata(background_mem_s2_readdata),                  
		.background_mem_s2_writedata(background_mem_s2_writedata),                 
		.background_mem_s2_byteenable(background_mem_s2_byteenable),
		
		.responsetl24_external_connection_export     (ResponseFromLT24)      //     responsetl24_external_connection.export
	);
	
	
	LT24_buffer lt24_buf(
		.clk(CLOCK_50),          							
		.rst_n(RST_N),
		.LT24_ADC_BUSY_bus(LT24_ADC_BUSY_bus),
		.LT24_ADC_CS_N_bus(LT24_ADC_CS_N_bus),
		.LT24_ADC_DCLK_bus(LT24_ADC_DCLK_bus),
		.LT24_ADC_DIN_bus(LT24_ADC_DIN_bus),
		.LT24_ADC_DOUT_bus(LT24_ADC_DOUT_bus),
		.LT24_ADC_PENIRQ_N_bus(LT24_ADC_PENIRQ_N_bus),
		.LT24_D_bus(LT24_D_bus),
		.LT24_WR_N_bus(LT24_WR_N_bus),
		.LT24_RD_N_bus(LT24_RD_N_bus),
		.LT24_CS_N_bus(LT24_CS_N_bus),
		.LT24_RESET_N_bus(LT24_RESET_N_bus),
		.LT24_RS_bus(LT24_RS_bus),
		
		.LT24_ADC_BUSY_screen(LT24_ADC_BUSY),
		.LT24_ADC_CS_N_screen(LT24_ADC_CS_N),
		.LT24_ADC_DCLK_screen(LT24_ADC_DCLK),
		.LT24_ADC_DIN_screen(LT24_ADC_DIN),
		.LT24_ADC_DOUT_screen(LT24_ADC_DOUT),
		.LT24_ADC_PENIRQ_N_screen(LT24_ADC_PENIRQ_N),
		.LT24_D_screen(LT24_D),
		.LT24_WR_N_screen(LT24_WR_N),
		.LT24_RD_N_screen(LT24_RD_N),
		.LT24_CS_N_screen(LT24_CS_N),
		.LT24_RESET_N_screen(LT24_RESET_N),
		.LT24_RS_screen(LT24_RS),
		
		.pic_mem_s2_address(pic_mem_s2_address),               //                pic_mem_s2.address
		.pic_mem_s2_chipselect(pic_mem_s2_chipselect),            //                          .chipselect
		.pic_mem_s2_clken(pic_mem_s2_clken),                 //                          .clken
		.pic_mem_s2_write(pic_mem_s2_write),                 //                          .write
		.pic_mem_s2_readdata(pic_mem_s2_readdata),              //                          .readdata
		.pic_mem_s2_writedata(pic_mem_s2_writedata),             //                          .writedata
		.pic_mem_s2_byteenable(pic_mem_s2_byteenable),            //                          .byteenable
		
		.lt24_buffer_flag(lt24_buffer_flag),
		
		.background_mem_s2_address(background_mem_s2_address),                   
		.background_mem_s2_chipselect(background_mem_s2_chipselect),                
		.background_mem_s2_clken(background_mem_s2_clken),                     
		.background_mem_s2_write(background_mem_s2_write),                     
		.background_mem_s2_readdata(background_mem_s2_readdata),                  
		.background_mem_s2_writedata(background_mem_s2_writedata),                 
		.background_mem_s2_byteenable(background_mem_s2_byteenable) 
	);




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
logic [15:0] IO_K_Data_In, IO_K_Data_Out, IO_K_Enable_Out;
logic [15:0] IO_L_Data_In, IO_L_Data_Out, IO_L_Enable_Out;
logic [15:0] IO_M_Data_In, IO_M_Data_Out, IO_M_Enable_Out;
logic [15:0] IO_N_Data_In, IO_N_Data_Out, IO_N_Enable_Out;
logic [15:0] IO_O_Data_In, IO_O_Data_Out, IO_O_Enable_Out;
logic [15:0] IO_P_Data_In, IO_P_Data_Out, IO_P_Enable_Out;


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


//--- Assign GPIO_0 Header -----------------------------
assign GPIO_0[0]     = Config[4];

//assign GPIO_0[33]    = Config[5]; 
assign GPIO_0[28]		= Config[5]; // !!!! C'était la pin 33 pas 28

assign PIC32_C1RX    = GPIO_0[29];
assign GPIO_0[30]    = PIC32_C1TX;
assign GPIO_0[31]    = PIC32_SCL3A;
assign GPIO_0[32]    = PIC32_SDA3A;

//--- Assign Status, INT ---------------------------------

assign PIC32_INT1 = Config[0] ? KEY[0] : 1'b1;
assign PIC32_INT2 = Config[1] ? KEY[1] : 1'b1;

always @ (posedge CLOCK_50)
	Status = {SW, 2'b00, KEY};

//--- Assign signals FPGA -> PIC -------------------------

logic LaserSign, LaserSignSansGlitch, LaserSync, LaserCodeurA, LaserCodeurB;
logic PropLeftCodeurA, PropLeftCodeurB, PropRightCodeurA, PropRightCodeurB;
logic OdoLeftCodeurA, OdoLeftCodeurB, OdoRightCodeurA, OdoRightCodeurB;
logic PinceCodeurA, PinceCodeurB;
logic RateauLeftCodeurA, RateauLeftCodeurB, RateauRightCodeurA, RateauRightCodeurB;
logic uSwitchLeft, uSwitchRight, uSwitchPince, uSwitchRateauLeft, uSwitchRateauRight;
logic Start;
logic ID;
logic UartTx, UartRx, UartDir;
logic dynEnRead,dynEnWrite,dynDebug;
logic [2:0] dynSelector;
logic [31:0] dynDataWrite,dynDataRead;
logic UARTTX,UARTRX,UARTDIR;



assign PropRightCodeurA 	= GPIO_0[1];
assign PropRightCodeurB 	= GPIO_0[2];
assign PropLeftCodeurA 		= GPIO_0[3];
assign PropLeftCodeurB 		= GPIO_0[4];
assign LaserCodeurA 			= GPIO_0[5];
assign LaserCodeurB 			= GPIO_0[6];
assign RateauLeftCodeurA 	= GPIO_0[7];
assign RateauLeftCodeurB 	= GPIO_0[8];
assign RateauRightCodeurA 	= GPIO_0[9];
assign RateauRightCodeurB 	= GPIO_0[10];
assign PinceCodeurA 			= GPIO_0[11];
assign PinceCodeurB 			= GPIO_0[12];
assign OdoLeftCodeurA 		= GPIO_0[13];
assign OdoLeftCodeurB 		= GPIO_0[14];
assign OdoRightCodeurA 		= GPIO_0[15];
assign OdoRightCodeurB 		= GPIO_0[16];
assign LaserSync 				= GPIO_0[17];
assign LaserSign 				= GPIO_0[18];
assign uSwitchLeft 			= GPIO_0[19];
assign uSwitchRight 			= GPIO_0[20];
assign uSwitchPince 			= GPIO_0[21];
assign uSwitchRateauLeft 	= GPIO_0[22];
assign uSwitchRateauRight 	= GPIO_0[23];
assign UartTx 					= GPIO_0[24];
assign UartRx 					= GPIO_0[25];
assign UartDir 				= GPIO_0[26];
assign Start = GPIO_0[33];

assign ID						= SW[0];

/*assign LaserSync = GPIO_0[7];
assign LaserSign = GPIO_0[8];
assign LaserCodeurA = GPIO_0[4];
assign LaserCodeurB = GPIO_0[5];

assign PropLeftCodeurA = GPIO_0[1];
assign PropLeftCodeurB = GPIO_0[2];
assign PropRightCodeurA = GPIO_0_IN[0];
assign PropRightCodeurB = GPIO_0_IN[1];

assign UartTx = GPIO_0[26];
assign UartRx = GPIO_0[24];
assign UartDir = GPIO_0[22];*/

//--- Gestion LED ---------------------------------------
logic [31:0] TestCodeurs;
counter #(32) CompteurTest(PinceCodeurA,PIC32_RESET,TestCodeurs);

assign LED = {~LaserSign, LaserSync, LaserCodeurA, LaserCodeurB, PropLeftCodeurA, PropLeftCodeurB, PropRightCodeurA, PropRightCodeurB}; //
//assign LED = IO_N_Data_Out[7:0];
//assign LED = {Start, Start, Start, Start, Start, Start, Start, Start};
//--- Gestion Glitch et Compteur Codeur -----------------

logic [15:0] CompteurLaser;
reg   [15:0] DebutReception, FinReception;
logic NewTowerTurn;

counter #(16) CompteurTourelle(LaserCodeurA, LaserSync, CompteurLaser);

GlitchHandler GlitchSignLaser(CLOCK_50, PIC32_RESET,  LaserSign, LaserSync, CompteurLaser, DebutReception, FinReception, LaserCodeurA);

always_ff@(posedge LaserSync, posedge PIC32_RESET) begin
	if(PIC32_RESET || NewTowerTurn === 'x) NewTowerTurn <= 1'b0;
	else NewTowerTurn <= ~NewTowerTurn;
end


// --- Vitesse des roues --------------------------------

logic [15:0] CompteurCodeurLeft, CompteurCodeurRight, CompteurCodeurLeftB, CompteurCodeurRightB;
logic [15:0] CompteurOdoLeftA, CompteurOdoRightA, CompteurOdoLeftB, CompteurOdoRightB;
logic [15:0] CompteurPinceA, CompteurPinceB;
logic [15:0] CompteurRateauLeftA, CompteurRateauRightA, CompteurRateauLeftB, CompteurRateauRightB;

logic [31:0] CompteurVitesse;

reg   [15:0] VitesseLeft, VitesseRight;
reg 			 positiveSpeedL, positiveSpeedR;
reg	[15:0] SpeedOdoL, SpeedOdoR;
reg	 		 positiveSpeedOdoL, positiveSpeedOdoR;
reg 	[15:0] SpeedPince;
reg			 positiveSpeedPince;
reg 	[15:0] SpeedRateauL,SpeedRateauR;
reg			 positiveSpeedRateauL, positiveSpeedRateauR;

logic ResetCompteurVitesse; 

counter #(32) CompteurTemps(CLOCK_50,  ResetCompteurVitesse, CompteurVitesse);

counter #(16) ComptCodLeftA (PropLeftCodeurA,  ResetCompteurVitesse, CompteurCodeurLeft);
counter #(16) ComptCodRightA(PropRightCodeurA, ResetCompteurVitesse, CompteurCodeurRight);
counter #(16) ComptCodLeftB (PropLeftCodeurB,  ResetCompteurVitesse, CompteurCodeurLeftB);
counter #(16) ComptCodRightB(PropRightCodeurB, ResetCompteurVitesse, CompteurCodeurRightB);

counter #(16) ComptCodOdoLeftA (OdoLeftCodeurA,  ResetCompteurVitesse, CompteurOdoLeftA);
counter #(16) ComptCodOdoRightA(OdoRightCodeurA, ResetCompteurVitesse, CompteurOdoRightA);
counter #(16) ComptCodOdoLeftB (OdoLeftCodeurB,  ResetCompteurVitesse, CompteurOdoLeftB);
counter #(16) ComptCodOdoRightB(OdoRightCodeurB, ResetCompteurVitesse, CompteurOdoRightB);

counter #(16) ComptCodPinceA (PinceCodeurA,  ResetCompteurVitesse, CompteurPinceA);
counter #(16) ComptCodPinceB (PinceCodeurB,  ResetCompteurVitesse, CompteurPinceB);

counter #(16) ComptCodRateauLeftA (RateauLeftCodeurA,  ResetCompteurVitesse, CompteurRateauLeftA);
counter #(16) ComptCodRateauRightA(RateauRightCodeurA, ResetCompteurVitesse, CompteurRateauRightA);
counter #(16) ComptCodRateauLeftB (RateauLeftCodeurB,  ResetCompteurVitesse, CompteurRateauLeftB);
counter #(16) ComptCodRateauRightB(RateauRightCodeurB, ResetCompteurVitesse, CompteurRateauRightB);

// SpeedDirection
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
always_ff@(posedge OdoLeftCodeurA)
begin 
	if(OdoLeftCodeurB)
		positiveSpeedOdoL <= 'b0;
	else
		positiveSpeedOdoL <= 'b1;
end
always_ff@(posedge OdoRightCodeurA)
begin 
	if(OdoRightCodeurB)
		positiveSpeedOdoR <= 'b1;
	else
		positiveSpeedOdoR <= 'b0;
end
always_ff@(posedge PinceCodeurA)
begin 
	if(PinceCodeurB)
		positiveSpeedPince <= 'b1;
	else
		positiveSpeedPince <= 'b0;
end
always_ff@(posedge RateauLeftCodeurA)
begin 
	if(RateauLeftCodeurB)
		positiveSpeedRateauL <= 'b1;
	else
		positiveSpeedRateauL <= 'b0;
end
always_ff@(posedge RateauLeftCodeurB)
begin 
	if(RateauLeftCodeurB)
		positiveSpeedRateauR <= 'b1;
	else
		positiveSpeedRateauR <= 'b0;
end
	
always_ff@(posedge CLOCK_50)
begin
	if(CompteurVitesse == 32'd500000) 
		begin 
			VitesseLeft <= (CompteurCodeurLeft + CompteurCodeurLeftB)/2; 
			VitesseRight <= (CompteurCodeurRight + CompteurCodeurRightB)/2; 
			SpeedOdoL <= (CompteurOdoLeftA + CompteurOdoLeftB)/2;
			SpeedOdoR <= (CompteurOdoRightA + CompteurOdoRightB)/2;
			SpeedRateauL <= (CompteurRateauLeftA + CompteurRateauLeftB)/2;
			SpeedRateauR <= (CompteurRateauRightA + CompteurRateauRightB)/2;
			SpeedPince <= (CompteurPinceA + CompteurPinceB)/2;
		end
	else if (CompteurVitesse == 32'd500001) 
			ResetCompteurVitesse <= 1'b1; 
	else 
			ResetCompteurVitesse <= 1'b0;
end

assign GPIO_0[26] = UARTTX;
assign UARTRX = GPIO_0[24];
assign GPIO_0[22] = UARTDIR;

//--- Envoi data ----------------------------------------

always @(posedge CLOCK_50)
begin 
	IO_A_Data_In <= {	NewTowerTurn,1'b0, Start,
							uSwitchRateauRight, uSwitchRateauLeft, uSwitchPince, uSwitchRight, uSwitchLeft,
							positiveSpeedOdoR, positiveSpeedOdoL,
							positiveSpeedPince,
							positiveSpeedRateauR, positiveSpeedRateauL,
							positiveSpeedR, positiveSpeedL,
							ID};
	IO_B_Data_In <= VitesseRight[15:0];
	IO_C_Data_In <= VitesseLeft[15:0];
	IO_D_Data_In <= SpeedOdoL[15:0];
	IO_E_Data_In <= SpeedOdoR[15:0];
	IO_F_Data_In <= SpeedPince[15:0];
	IO_G_Data_In <= SpeedRateauL[15:0];
	IO_H_Data_In <= SpeedRateauR[15:0];
	IO_I_Data_In <= TestCodeurs[31:16];
	IO_J_Data_In <= TestCodeurs[15:0];
	IO_K_Data_In <= DebutReception[15:0];
	IO_L_Data_In <= FinReception[15:0];
	IO_M_Data_In <= 16'b0; //CompteurMax[15:0];
	IO_N_Data_In <= {8'b0,ResponseFromLT24};
	
	dynSelector 		<= IO_N_Data_Out[2:0];
	dynEnRead			<= IO_N_Data_Out[3];
	dynEnWrite			<= IO_N_Data_Out[4];
	
	dynDataWrite[7:0]		<= IO_N_Data_Out[15:8];
	dynDataWrite[15:8]	<= IO_O_Data_Out[7:0];
	dynDataWrite[23:16]	<= IO_O_Data_Out[15:8];
	dynDataWrite[31:24]	<= IO_P_Data_Out[7:0];
	
	//IO_E_Data_In <= dynDataRead[7:0];
	//IO_F_Data_In <= dynDataRead[15:8];
	//IO_G_Data_In <= dynDataRead[23:16];
	//IO_H_Data_In <= dynDataRead[31:24];
	
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
	.IO_A_Data_In(IO_A_Data_In), 			.IO_B_Data_In(IO_B_Data_In), 			.IO_C_Data_In(IO_C_Data_In), 			.IO_D_Data_In(IO_D_Data_In),			.IO_E_Data_In(IO_E_Data_In),			.IO_F_Data_In(IO_F_Data_In),			.IO_G_Data_In(IO_G_Data_In),			.IO_H_Data_In(IO_H_Data_In),			.IO_I_Data_In(IO_I_Data_In),			.IO_J_Data_In(IO_J_Data_In),			.IO_K_Data_In(IO_K_Data_In),			.IO_L_Data_In(IO_L_Data_In),			.IO_M_Data_In(IO_M_Data_In), 				.IO_N_Data_In(IO_N_Data_In),				.IO_O_Data_In(IO_O_Data_In),				.IO_P_Data_In(IO_P_Data_In),
	.IO_A_Data_Out(IO_A_Data_Out), 		.IO_B_Data_Out(IO_B_Data_Out), 		.IO_C_Data_Out(IO_C_Data_Out), 		.IO_D_Data_Out(IO_D_Data_Out),		.IO_E_Data_Out(IO_E_Data_Out),		.IO_F_Data_Out(IO_F_Data_Out),		.IO_G_Data_Out(IO_G_Data_Out),		.IO_H_Data_Out(IO_H_Data_Out),		.IO_I_Data_Out(IO_I_Data_Out),		.IO_J_Data_Out(IO_J_Data_Out),		.IO_K_Data_Out(IO_K_Data_Out),		.IO_L_Data_Out(IO_L_Data_Out),		.IO_M_Data_Out(IO_M_Data_Out),			.IO_N_Data_Out(IO_N_Data_Out),			.IO_O_Data_Out(IO_O_Data_Out),			.IO_P_Data_Out(IO_P_Data_Out),
	.IO_A_Enable_Out(IO_A_Enable_Out), 	.IO_B_Enable_Out(IO_B_Enable_Out), 	.IO_C_Enable_Out(IO_C_Enable_Out), 	.IO_D_Enable_Out(IO_D_Enable_Out), 	.IO_E_Enable_Out(IO_E_Enable_Out), 	.IO_F_Enable_Out(IO_F_Enable_Out), 	.IO_G_Enable_Out(IO_G_Enable_Out), 	.IO_H_Enable_Out(IO_H_Enable_Out), 	.IO_I_Enable_Out(IO_I_Enable_Out), 	.IO_J_Enable_Out(IO_J_Enable_Out), 	.IO_K_Enable_Out(IO_K_Enable_Out), 	.IO_L_Enable_Out(IO_L_Enable_Out), 	.IO_M_Enable_Out(IO_M_Enable_Out),		.IO_N_Enable_Out(IO_N_Enable_Out),		.IO_O_Enable_Out(IO_N_Enable_Out),		.IO_P_Enable_Out(IO_P_Enable_Out));


	
	UART_Dynamixel myDyn(
	// NIOS signals
	.clk(CLOCK_50),
	.reset(PIC32_RESET),
	.write_en(dynEnWrite),
	.read_en(dynEnRead),
	.rw_ad(dynSelector),
	.write_data(dynDataWrite),
	.data_useful(dynDataRead),
	// exported signals
	.RXD(UARTRX),
	.TXD(UARTTX), 
	.UART_DIR(~UARTDIR),
	.debug(dynDebug)
);
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
							input logic LaserCodeurA); 
//							output logic [15:0] BeauSignal);

logic [15:0] Buffer;
logic [15:0] DebutReceptionBuffer;
logic Flag, resetCount;
//reg [15:0] SignalSansGlitch;
typedef enum logic [1:0] {Wait, Reception, Glitch} statetype;
statetype state, nextstate;

always_ff @(posedge clk, posedge reset)
	if (reset) begin state <= Wait; end
	else begin  state <= nextstate; end
	
always
	begin
		case(state)
			Wait: begin 
						if(Sign == 1'b1) begin nextstate = Reception; end
						else begin nextstate = Wait; end
						resetCount <= 1'b1;
						if(~Flag & ~Sign) nextstate = Wait;
			      end
					
			Reception: begin
								if(Sign == 1'b0) begin resetCount = 1'b1; nextstate = Glitch; end
								//if (Sign == 1'b0 && Flag == 1'b0) nextstate = Wait;
								else nextstate = Reception;
								if(~Flag & ~Sign) nextstate = Wait;
						   end 
						  
			Glitch: begin
						if(Flag == 1'b0) begin nextstate = Wait; end
						else if (Sign == 1'b1) begin nextstate = Reception; end
						else begin nextstate = Glitch; end
						resetCount = 1'b0;
						if(~Flag & ~Sign) nextstate = Wait;
					  end
		endcase
	end

always//_ff @(posedge clk, posedge Sign)
begin
	if((nextstate == Reception) & (state == Wait)) DebutReceptionBuffer <= NbrCodeur;
	else if((Sign == 1'b0) & (state == Reception)) Buffer <= NbrCodeur;
	if ((nextstate == Wait) & ( (state == Glitch)))begin DebutReception <= DebutReceptionBuffer; FinReception <= Buffer; end//(state == Reception) |
	//else if((Flag == 1'b0) & (state == Glitch)) FinReception <= Buffer;
end

counter20Clk Compteur(LaserCodeurA, resetCount, Flag);

endmodule
