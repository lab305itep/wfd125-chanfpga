`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   19:33:54 10/03/2014
// Design Name:   spi_wbmaster
// Module Name:   /home/igor/proj/wfd125/wfd125-chanfpga/test_wbmaster.v
// Project Name:  fpga_chan
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: spi_wbmaster
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module test_wbmaster;

	// Inputs
	reg CLK = 0;
	reg SPICLK;
	reg SPIFR;
	reg [1:0] STADDR;
	reg [15:0] wb_dat_i;
	reg wb_ack_i;
	reg wb_err_i;
	reg wb_rty_i;

	// Outputs
	wire [12:0] wb_adr_o;
	wire [15:0] wb_dat_o;
	wire [3:0] wb_sel_o;
	wire wb_we_o;
	wire wb_cyc_o;
	wire wb_stb_o;
	wire [2:0] wb_cti_o;
	wire [1:0] wb_bte_o;

	// Bidirs
	wire SPIDAT;

	// Instantiate the Unit Under Test (UUT)
	spi_wbmaster uut (
		.CLK(CLK), 
		.SPICLK(SPICLK), 
		.SPIDAT(SPIDAT), 
		.SPIFR(SPIFR), 
		.STADDR(STADDR), 
		.wb_adr_o(wb_adr_o), 
		.wb_dat_o(wb_dat_o), 
		.wb_sel_o(wb_sel_o), 
		.wb_we_o(wb_we_o), 
		.wb_cyc_o(wb_cyc_o), 
		.wb_stb_o(wb_stb_o), 
		.wb_cti_o(wb_cti_o), 
		.wb_bte_o(wb_bte_o), 
		.wb_dat_i(wb_dat_i), 
		.wb_ack_i(wb_ack_i), 
		.wb_err_i(wb_err_i), 
		.wb_rty_i(wb_rty_i)
	);

	assign SPIDAT = 0;
	
	initial begin
		// Initialize Inputs
		SPICLK = 1;
		SPIFR = 1;
		STADDR = 0;
		wb_dat_i = 0;
		wb_ack_i = 0;
		wb_err_i = 0;
		wb_rty_i = 0;

		// Wait 100 ns for global reset to finish
		#100;
        
		// Add stimulus here
		
		SPIFR = 0; #100;
		
		repeat (32) begin
      	#100; SPICLK = 0;
			#100; SPICLK = 1;
		end
		
		 #100; SPIFR = 1;
   end
      
	always begin
		# 4 CLK <= !CLK;
	end
			
endmodule

