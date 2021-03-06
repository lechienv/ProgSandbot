	DE0_LT24_SOPC u0 (
		.alt_pll_areset_conduit_export               (<connected-to-alt_pll_areset_conduit_export>),               //               alt_pll_areset_conduit.export
		.alt_pll_c1_clk                              (<connected-to-alt_pll_c1_clk>),                              //                           alt_pll_c1.clk
		.alt_pll_c3_clk                              (<connected-to-alt_pll_c3_clk>),                              //                           alt_pll_c3.clk
		.alt_pll_locked_conduit_export               (<connected-to-alt_pll_locked_conduit_export>),               //               alt_pll_locked_conduit.export
		.alt_pll_phasedone_conduit_export            (<connected-to-alt_pll_phasedone_conduit_export>),            //            alt_pll_phasedone_conduit.export
		.background_mem_s2_address                   (<connected-to-background_mem_s2_address>),                   //                    background_mem_s2.address
		.background_mem_s2_chipselect                (<connected-to-background_mem_s2_chipselect>),                //                                     .chipselect
		.background_mem_s2_clken                     (<connected-to-background_mem_s2_clken>),                     //                                     .clken
		.background_mem_s2_write                     (<connected-to-background_mem_s2_write>),                     //                                     .write
		.background_mem_s2_readdata                  (<connected-to-background_mem_s2_readdata>),                  //                                     .readdata
		.background_mem_s2_writedata                 (<connected-to-background_mem_s2_writedata>),                 //                                     .writedata
		.background_mem_s2_byteenable                (<connected-to-background_mem_s2_byteenable>),                //                                     .byteenable
		.clk_clk                                     (<connected-to-clk_clk>),                                     //                                  clk.clk
		.from_key_export                             (<connected-to-from_key_export>),                             //                             from_key.export
		.lt24_buffer_flag_external_connection_export (<connected-to-lt24_buffer_flag_external_connection_export>), // lt24_buffer_flag_external_connection.export
		.lt24_conduit_cs                             (<connected-to-lt24_conduit_cs>),                             //                         lt24_conduit.cs
		.lt24_conduit_rs                             (<connected-to-lt24_conduit_rs>),                             //                                     .rs
		.lt24_conduit_rd                             (<connected-to-lt24_conduit_rd>),                             //                                     .rd
		.lt24_conduit_wr                             (<connected-to-lt24_conduit_wr>),                             //                                     .wr
		.lt24_conduit_data                           (<connected-to-lt24_conduit_data>),                           //                                     .data
		.lt24_lcd_rstn_export                        (<connected-to-lt24_lcd_rstn_export>),                        //                        lt24_lcd_rstn.export
		.lt24_touch_busy_export                      (<connected-to-lt24_touch_busy_export>),                      //                      lt24_touch_busy.export
		.lt24_touch_penirq_n_export                  (<connected-to-lt24_touch_penirq_n_export>),                  //                  lt24_touch_penirq_n.export
		.lt24_touch_spi_MISO                         (<connected-to-lt24_touch_spi_MISO>),                         //                       lt24_touch_spi.MISO
		.lt24_touch_spi_MOSI                         (<connected-to-lt24_touch_spi_MOSI>),                         //                                     .MOSI
		.lt24_touch_spi_SCLK                         (<connected-to-lt24_touch_spi_SCLK>),                         //                                     .SCLK
		.lt24_touch_spi_SS_n                         (<connected-to-lt24_touch_spi_SS_n>),                         //                                     .SS_n
		.pic_mem_s2_address                          (<connected-to-pic_mem_s2_address>),                          //                           pic_mem_s2.address
		.pic_mem_s2_chipselect                       (<connected-to-pic_mem_s2_chipselect>),                       //                                     .chipselect
		.pic_mem_s2_clken                            (<connected-to-pic_mem_s2_clken>),                            //                                     .clken
		.pic_mem_s2_write                            (<connected-to-pic_mem_s2_write>),                            //                                     .write
		.pic_mem_s2_readdata                         (<connected-to-pic_mem_s2_readdata>),                         //                                     .readdata
		.pic_mem_s2_writedata                        (<connected-to-pic_mem_s2_writedata>),                        //                                     .writedata
		.pic_mem_s2_byteenable                       (<connected-to-pic_mem_s2_byteenable>),                       //                                     .byteenable
		.reset_reset_n                               (<connected-to-reset_reset_n>),                               //                                reset.reset_n
		.sdram_wire_addr                             (<connected-to-sdram_wire_addr>),                             //                           sdram_wire.addr
		.sdram_wire_ba                               (<connected-to-sdram_wire_ba>),                               //                                     .ba
		.sdram_wire_cas_n                            (<connected-to-sdram_wire_cas_n>),                            //                                     .cas_n
		.sdram_wire_cke                              (<connected-to-sdram_wire_cke>),                              //                                     .cke
		.sdram_wire_cs_n                             (<connected-to-sdram_wire_cs_n>),                             //                                     .cs_n
		.sdram_wire_dq                               (<connected-to-sdram_wire_dq>),                               //                                     .dq
		.sdram_wire_dqm                              (<connected-to-sdram_wire_dqm>),                              //                                     .dqm
		.sdram_wire_ras_n                            (<connected-to-sdram_wire_ras_n>),                            //                                     .ras_n
		.sdram_wire_we_n                             (<connected-to-sdram_wire_we_n>),                             //                                     .we_n
		.to_led_export                               (<connected-to-to_led_export>),                               //                               to_led.export
		.responsetl24_external_connection_export     (<connected-to-responsetl24_external_connection_export>)      //     responsetl24_external_connection.export
	);

