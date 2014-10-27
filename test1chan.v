`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   19:25:45 10/27/2014
// Design Name:   prc1chan
// Module Name:   /home/igor/proj/wfd125/wfd125-chanfpga/test1chan.v
// Project Name:  fpga_chan
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: prc1chan
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module test1chan;

	// Inputs
	reg clk;
	reg [11:0] data;
	reg [11:0] zthr;
	reg [11:0] sthr;
	reg [11:0] cped;
	reg [15:0] prescale;
	reg [9:0] winbeg;
	reg [9:0] swinbeg;
	reg [7:0] winlen;
	reg [15:0] trigger;
	reg [5:0] num;
	reg ack;
	reg smask;
	reg tmask;
	reg stmask;

	// Outputs
	wire [11:0] d2sum;
	wire [11:0] ped;
	wire [15:0] dout;
	wire req;

	reg [7:0] cnt = 0;

	// Instantiate the Unit Under Test (UUT)
	prc1chan uut (
		.clk(clk), 
		.data(data), 
		.d2sum(d2sum), 
		.ped(ped), 
		.zthr(zthr), 
		.sthr(sthr), 
		.cped(cped),
		.prescale(prescale), 
		.winbeg(winbeg), 
		.swinbeg(swinbeg), 
		.winlen(winlen), 
		.trigger(trigger), 
		.dout(dout), 
		.num(num), 
		.req(req), 
		.ack(ack), 
		.smask(smask), 
		.tmask(tmask), 
		.stmask(stmask)
	);

	initial begin
		// Initialize Inputs
		clk = 0;
		data = 0;
		zthr = 10;
		sthr = 10;
		prescale = 10;
		winbeg = 50;
		swinbeg = 40;
		winlen = 200;
		cped = 50;
		trigger = 0;
		num = 6'h2A;
		ack = 0;
		smask = 0;
		tmask = 0;
		stmask = 0;

		// Wait 100 ns for global reset to finish
		#10000;
		trigger <= 16'h8555;
		#20;
		trigger <= 0;
        
		// Add stimulus here

	end
   
	always @ (*) #4 clk <= !clk;
	
	always @ (posedge clk) begin 
		cnt <= cnt + 1;
		case (cnt) 
		20:	data <= 15;
		21:	data <= 20;
		22:	data <= 40;
		23:	data <= 100;
		24:	data <= 120;
		25:	data <= 110;
		26:	data <= 90;
		27: 	data <= 50;
		28:	data <= 20;
		29:	data <= 15;
		default	data <= 10;
		endcase
		
		ack <= #2 req;
	end	
		
	
		
endmodule

