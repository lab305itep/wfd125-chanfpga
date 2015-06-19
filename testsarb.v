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
	wire [16:0] fifo_have;
	wire [271:0] datain;
	reg trig;

	// Outputs
	wire [16:0] arb_want;
	reg [16:0] arb_old;
	wire [15:0] dataout;
	wire kchar;
	reg [8:0] cnt;
	wire flag;
	reg [8:0] trcnt = 0;
	
	// Instantiate the Unit Under Test (UUT)
	snd_arb uut (
		.clk(clk), 
		.arb_want(arb_want), 
		.fifo_have(fifo_have), 
		.datain(datain), 
		.trig(trig), 
		.dataout(dataout), 
		.kchar(kchar)
	);

	localparam blen = 4;	// number of words to treansmit -1
	localparam [16:0] fsense = 17'h101;

	assign flag = ((arb_want & fsense) != arb_old) & |(arb_want & fsense);
	assign fifo_have = ((|cnt) | flag) ? (arb_want & fsense) : 0;

	
	genvar i;
	generate 
		begin: ggg
		for (i=0; i<17; i=i+1) begin: fggg
			assign datain[16*i +: 16] = {flag, i[5:0], (flag) ? blen[8:0] : cnt};
		end
		end
   endgenerate


	always @(posedge clk) begin
		if (|(arb_want & fsense)) arb_old <= arb_want & fsense;
		if (flag) begin
			cnt <= blen; 
		end else if (|cnt & |(arb_want & fsense)) begin
			cnt <= cnt - 1;
		end
		// trig
		trig <= 0;
		if (|trcnt) begin
			trcnt <= trcnt - 1;
		end else begin
			trig <= 1;
			trcnt <= blen;
		end
		
	end
	
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

