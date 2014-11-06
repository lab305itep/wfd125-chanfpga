`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    15:49:33 11/06/2014 
// Design Name: 
// Module Name:    iodelaypulse 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module iodelaypulse(
		input clk,
		input reset,
		input pulse,
		input cal,
		output reg del_ce,
		output reg del_rst,
		output reg del_cal
   );

	reg reset_d = 0;
	reg reset_dd = 0;
	reg ce_d = 0;
	reg ce_dd = 0;
	reg cal_d = 0;
	reg cal_dd = 0;

	always @ (posedge clk) begin
		del_cal <= 0;
		del_rst <= 0;
		del_ce <= 0;
		reset_d <= reset;
		reset_dd <= reset_d;
		ce_d <= pulse;
		ce_dd <= ce_d;
		cal_d <= cal;
		cal_dd <= cal_d;
		if (cal_d & (!cal_dd)) begin
			del_cal <= 1'b1;
		end 
		if (reset_d & (!reset_dd)) begin
			del_rst <= 1'b1;
		end 
		if (ce_d & (!ce_dd)) begin
			del_ce <= 1'b1;
		end
	end

endmodule
