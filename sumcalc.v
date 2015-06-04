`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    14:04:00 10/28/2014 
// Design Name: 
// Module Name:    sumcalc 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//		Calculated local summ of 16 channels and total sum.
//		Issue request for master trigger
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module sumcalc(
		input clk,						// master clock
		input [255:0] data,			// input data
		input [47:0] sumdata,		// sums from 3 other xilinxes
		input [2:0]  xcomma,			// commas from other xilinxes
		output reg [15:0] sumres,	// 16-channel sum
		output reg sumcomma,			// comma / data
		input [15:0] s16thr,			// 16-channel sum threshold
		input [15:0] s64thr,			// 64-channel sum threshold
		output reg trigout			// 64-channel trigger
   );

	parameter XDELAY = 5;
	localparam	CH_COMMA = 16'h00BC;		// comma K28.5

//		Delay local result	
	reg [17*XDELAY-1:0] xdelay;
	always @ (posedge clk) xdelay <= {xdelay[17*XDELAY-18:0], {sumcomma, sumres}};
	
//		Calculate local sum
	reg [15:0] sum4 [3:0];
	reg [15:0] sum16;
	always @ (posedge clk) begin
		sum4[0] <= data[0+:16] + data[16+:16] + data[32+:16] + data[48+:16];
		sum4[1] <= data[64+:16] + data[80+:16] + data[96+:16] + data[112+:16];
		sum4[2] <= data[128+:16] + data[144+:16] + data[160+:16] + data[176+:16];
		sum4[3] <= data[192+:16] + data[208+:16] + data[224+:16] + data[240+:16];
		sum16 <= sum4[0] + sum4[1] + sum4[2] + sum4[3];
		if (sum16 > s16thr) begin
			sumres <= sum16;
			sumcomma <= 0;
		end else begin
			sumcomma <= 1;
			sumres <= CH_COMMA;
		end
	end

//		MAster trigger
	reg [17:0] sum64;
	reg trigout_s;
	always @ (posedge clk) begin
		sum64 <= ((!xcomma[0]) ? sumdata[15:0] : 0) + ((!xcomma[1]) ? sumdata[31:16] : 0) + 
			((!xcomma[2]) ? sumdata[47:32] : 0) + ((!xdelay[17*XDELAY-1]) ? xdelay[17*XDELAY-2:17*XDELAY-17] : 0);
		trigout <= 0;
		if (sum64 > {2'b00, s64thr}) begin
			trigout_s <= 1;
			if (!trigout_s) trigout <= 1;
		end else begin
			trigout_s <= 0;
		end
	end

endmodule
