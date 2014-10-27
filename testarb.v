`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   00:37:42 10/28/2014
// Design Name:   arbitter
// Module Name:   /home/igor/proj/wfd250/firmware/wfd125-chanfpga/testarb.v
// Project Name:  fpga_chan
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: arbitter
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module testarb;

	// Inputs
	reg clk;
	reg [255:0] data;
	reg trigger;
	reg [15:0] req;
	
	wire [15:0] dout;
	wire [15:0] ack;
	wire kchar;

	// Instantiate the Unit Under Test (UUT)
	arbitter uut (
		.clk(clk), 
		.data(data),
		.dout(dout),
		.kchar(kchar),
		.trigger(trigger),
		.req(req),
		.ack(ack)
	);

	initial begin
		// Initialize Inputs
		clk = 0;
		req = 0;
		trigger = 0;
		data = 0;
		data[47:32] = 16'h1234;
		
		// Wait 100 ns for global reset to finish
		#102;
		// Add stimulus here
		forever begin
			#320; req[2] <= 1;
			#200; req[2] <= 0;
      end  

	end
      
	always @ (*) #4 clk <= !clk;
		
endmodule

