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
		output [15:0] ack
   );

	localparam	CH_COMMA = 16'h00BC;		// comma K28.5
	localparam  CH_TRIG  = 16'h801C;		// K-character K28.0

	reg [3:0] sel = 4'h0;
	wire [15:0] amux;
	wire       rmux;
	reg trigger_t = 0;
	reg dvalid = 0;
	
	wire [15:0] data_r [15:0];
	assign amux = 1 << sel;
	assign rmux = | (req & amux);
	assign ack = (!trigger && rmux) ? amux : 0;
	
	genvar i;
	generate
		for (i=0; i<16; i=i+1) begin: GMUX
			assign data_r[i] = data[16*i+15:16*i];
		end
	endgenerate
	
	always @ (posedge clk) begin
		trigger_t <= trigger;
		kchar <= 1;
		dout <= CH_COMMA;
		dvalid <= rmux;
		if (trigger_t) begin
			dout <= CH_TRIG;
		end else	if (dvalid) begin
			dout <= data_r[sel];
			kchar <= 0;
		end else if (!rmux) begin
			sel <= sel + 1;
		end
	end

endmodule
