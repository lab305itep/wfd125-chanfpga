`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   17:30:54 06/10/2015
// Design Name:   snd_arb
// Module Name:   /home/igor/proj/wfd125/wfd125-chanfpga/testsarb.v
// Project Name:  fpga_chan
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: snd_arb
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module testsarb;

	// Inputs
	reg clk;
	reg [16:0] fifo_have;
	reg [271:0] datain;
	reg trig;

	// Outputs
	wire [16:0] arb_want;
	reg [16:0] arb_old;
	wire [4:0] debug;
	wire [15:0] dataout;
	wire kchar;
	reg [8:0] cnt;
	wire flag;
	
	// Instantiate the Unit Under Test (UUT)
	snd_arb uut (
		.clk(clk), 
		.arb_want(arb_want), 
		.fifo_have(fifo_have), 
		.datain(datain), 
		.trig(trig), 
		.debug(debug), 
		.dataout(dataout), 
		.kchar(kchar)
	);

/*	genvar i;
	generate 
		begin: ggg
			for (i=0; i<17; i=i+1) begin: fggg
				assign ddd[16*i +: 16] = {flag, i[5:0], (flag) ? 9'h003 : cnt};
			end
		end
   endgenerate */
	integer i;
	always @(posedge clk) begin
		if (|arb_want) arb_old <= arb_want;
		if (flag & |arb_want) cnt <= 0; else cnt <= cnt + 1;
		fifo_have <= (cnt == 9'h003 && ~flag) ? 0 : arb_want;
		for (i=0; i<17; i=i+1) begin: fggg
			datain[16*i +: 16] <= {flag, i[5:0], (flagf
	assign flag = arb_want != arb_old;

	initial begin
		// Initialize Inputs
		clk = 0;
		trig = 0;
		cnt = 0;
		arb_old = 0;

		// Wait 100 ns for global reset to finish
		#100;
        
		// Add stimulus here

	end
	
	always @ (*) #4 clk <= !clk;
	
      
endmodule

