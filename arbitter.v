`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    00:17:59 10/28/2014 
// Design Name: 
// Module Name:    arbitter 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//		Arbitter
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//		Send k-char meaning trigger on trigger request - out of band
//		Send blocks of data on requests
//		Send commas in between
//////////////////////////////////////////////////////////////////////////////////
module arbitter(
		input clk,
		input [255:0] data,
		output reg [15:0] dout,
		output reg kchar,
		input trigger,
		input [15:0] req,
		output reg [15:0] ack
   );

	localparam	CH_COMMA = 16'h00BC;		// comma K28.5
	localparam  CH_TRIG  = 16'h801C;		// K-character K28.0

	reg [3:0] sel = 4'h0;
	reg    active = 0;
	reg [15:0] dmux;
	reg [15:0] amux;
	reg        rmux;
	
	genvar i;
	generate
		for (i=0; i<16; i=i+1) begin: GMUX
			always @ (*) begin
				if (sel == i) begin
					dmux = data[16*i+15:16*i];
					amux = 1 << i;
					rmux = req[i];
				end
			end
		end
	endgenerate

	always @ (posedge clk) begin
		ack <= 0;
		kchar <= 1;
		dout <= CH_COMMA;
		if (trigger) begin
			dout <= CH_TRIG;
		end else if (active) begin 
			if (rmux) begin
				dout <= dmux;
				kchar <= 0;
				ack <= amux;
			end else begin
				active <= 0;
			end
		end else if (rmux) begin
			active <= 1;
		end else begin
			sel <= sel + 1;
		end
	end

endmodule
