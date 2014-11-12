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
	wire [15:0] req;
	reg [10:0] rfaddr;
	reg [10:0] ffaddr;
	reg [10:0] fffaddr;
	
	wire [15:0] dout;
	wire [15:0] ack;
	wire kchar;
	integer i;
	reg [15:0] fifo [2047:0];

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
		trigger = 0;
		data = 0;
		rfaddr = 0;
		ffaddr = 0;
		fffaddr = 0;
		
		for (i=0; i<2048; i = i + 1) fifo[i] = i;
		
		// Wait 100 ns for global reset to finish
		#200;
		// Add stimulus here
		ffaddr = 10;


		#120;
		trigger = 1;
		#8;
		trigger = 0;

	end
      
	always @ (*) #4 clk <= !clk;

	assign req[1:0] = 0;
	assign req[15:3] = 0;
   assign req[2] = rfaddr != fffaddr;

//		Fifo to arbitter
	always @ (posedge clk) begin
		data[47:32] <= fifo[rfaddr];
		fffaddr <= ffaddr;
		if (ack[2]) rfaddr <= rfaddr + 1;
	end
		
endmodule

