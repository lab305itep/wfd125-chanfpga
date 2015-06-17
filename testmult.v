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

		// Wait 100 ns for global reset to finish
		#100;
        
		// Add stimulus here

	end
      
	always @ (*) #4 clk <= !clk;
	
	always @(posedge clk) din <= -2 * din + 1;
	
endmodule

