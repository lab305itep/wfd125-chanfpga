`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: ITEP
// Engineer: SvirLex
//
// Create Date:   14:42:41 06/17/2015
// Design Name:   normmult
// Module Name:   /home/igor/proj/wfd125/wfd125-chanfpga/testmult.v
// Project Name:  wfd125-chanfpga
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: normmult
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module testmult;

	// Inputs
	reg [15:0] din;
	reg clk;
	reg [15:0] coef;

	reg [2:0] d2sum_waddr;
	reg [2:0] d2sum_raddr;
	reg ADCCLK;
	reg d2sum_arst;
	reg d2sum_arst_d;

	// Outputs
	wire [15:0] dout;

	// Instantiate the Unit Under Test (UUT)
	normmult uut (
		.din(din), 
		.dout(dout), 
		.clk(clk), 
		.coef(coef)
	);

	initial begin
		// Initialize Inputs
		din = 0;
		clk = 0;
		coef = 16'h8000;

		d2sum_waddr = 0;
		d2sum_raddr = 0;
		ADCCLK = 0;
		d2sum_arst = 0;
		d2sum_arst_d = 0;

		// Wait 100 ns for global reset to finish
		#100;
        
		// Add stimulus here

	end
      
	always @ (*) #4 clk <= !clk;
	always @ (*) #3.9 ADCCLK <= !ADCCLK;
	
	always @(posedge clk) din <= -2 * din + 1;
	
	//		to total sum -- resync adc data to clk
	// fill buffer at ADFCCLK
	always @ (posedge ADCCLK) begin
		// send zero if masked or raw data requested
		d2sum_waddr <= d2sum_waddr + 1;
		d2sum_arst <= (d2sum_waddr == 0) ? 1 : 0;
	end
	// read buffer at clk
	always @ (posedge clk) begin
		d2sum_arst_d <= d2sum_arst;
		d2sum_raddr <= (d2sum_arst_d) ? 0 : d2sum_raddr + 1;
	end
	
endmodule

