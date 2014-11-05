`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 	ITEP
// Engineer: 	SvirLex
// 
// Create Date:    23:33:17 11/05/2014 
// Design Name: 
// Module Name:    checkseq 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//		Make sequence for ADC input test
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//		Counting time = 2**(16 + 2*cntmax) clk periods
//////////////////////////////////////////////////////////////////////////////////
module checkseq(
		input clk,					// system clock
		input start,				// start
		input [2:0] cntmax,		// 
		output reg reset,
		output reg enable,
		output reg ready
	);

	reg start_d = 0;
	reg [31:0] cnt = 0;
	reg [1:0] state = 0;
	
	always @ (posedge clk) begin
		start_d <= start;
		
		case (state)
		0 : begin
				enable <= 0;
				ready <= 1;
				if (start & (!start_d)) begin
					state <= 1;
					reset <= 1;
					ready <= 0;
				end
			 end
		1 : begin
				cnt <= 1 << (16 + 2 * cntmax);
				reset <= 0;
				enable <= 1;
				state <= 2;
			 end
		2 : if (| cnt) begin
				cnt <= cnt - 1;
			 end else begin
				state <= 0;
			 end
		endcase
	end

endmodule
