`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:			 ITEP 
// Engineer: 		 SvirLex
// 
// Create Date:    14:04:00 10/28/2014 
// Design Name: 	 chanfpga
// Module Name:    sumcalc 
// Project Name: 	 wfd125
// Description: 
//		Calculats local summ of 16 channels and sends it to to other X's, sends comma instead of zero value
//		Delays local sum to be added with data from other X's to make total sum.
//		Issue request for master trigger if total sum exceeds threshold
//		Forms and sends trigger formation history block
// Revision 0.01 - File Created
// Revision 0.02 - modified for signed data and programmable delay
//
//////////////////////////////////////////////////////////////////////////////////
module sumcalc # (
		parameter		DBITS		= 4,	// number of bits in delay line addr
		parameter		CBITS		= 10,	// number of bits in the circular buffer addr
		parameter		FBITS		= 11	// number of bits in output fifo addr
	)
	(
		input 				clk,			// master clock
		input [255:0] 		data,			// input data from local channels clk timed
		// communication to other X's
		input [47:0] 		xdata,		// sums from 3 other xilinxes
		input [2:0]  		xcomma,		// commas from other xilinxes
		output reg [15:0]	sumres,		// 16-channel sum to other X's
		output reg 			sumcomma,	// comma / data to other X's
		// programmable parameters
		input [15:0] 		s64thr,		// 64-channel sum threshold
		input [DBITS-1:0]	xdelay,		// delay for local sum to be added to other X's		
		input	[CBITS-1:0]	winbeg,		// trigger history window begin
		input [8:0]			winlen,		// trigger history window length
		// communication to sending arbitter
		input					give,			// arbitter wants history data
		output				have,			// history data ready
		output [15:0]		dout,			// history data to arbitter
		output reg 			trigout		// 64-channel trigger
   );

	localparam	CH_COMMA = 16'h00BC;		// comma K28.5

	// local sum
	reg signed [15:0] 	sum4 [3:0];		// 4-channel sums
	reg signed [15:0] 	sum16;			// full local sum
	
	// circular buffer for summing delay
	reg [15:0] 				dbuf [2**DBITS-1:0];	// buffer itself
	reg [15:0]				db_data = 0;			// buffer output data
	reg [DBITS-1:0]		db_waddr = 0;			// write address	
	reg [DBITS-1:0]		db_raddr = 0;			// read address	
		
	// master trigger 
	reg signed [17:0]		sum64;					// sum of 4 X's
	reg 						trigout_s;				// sum above threshold

	// circular buffer for trigger history
	reg [14:0] 				cbuf [2**CBITS-1:0];	// buffer itself
	reg [14:0]				cb_data = 0;			// buffer output data
	reg [CBITS-1:0]		cb_waddr = 0;			// write address	
	reg [CBITS-1:0]		cb_raddr = 0;			// read address	


	
//		Calculate local sum and send it
	always @ (posedge clk) begin
		sum4[0] <= $signed(data[0+:16]) + 	$signed(data[16+:16]) + 	$signed(data[32+:16]) + 	$signed(data[48+:16]);
		sum4[1] <= $signed(data[64+:16]) + 	$signed(data[80+:16]) + 	$signed(data[96+:16]) + 	$signed(data[112+:16]);
		sum4[2] <= $signed(data[128+:16]) + $signed(data[144+:16]) + 	$signed(data[160+:16]) + 	$signed(data[176+:16]);
		sum4[3] <= $signed(data[192+:16]) + $signed(data[208+:16]) + 	$signed(data[224+:16]) + 	$signed(data[240+:16]);
		sum16 <=   sum4[0] +		sum4[1] +	sum4[2] +	sum4[3];
		// sum16 suuposed to be noisy around zero
		if (sum16 != 0) begin
			sumres <= sum16;
			sumcomma <= 0;
		end else begin
			// we send comma instead of zero value
			sumcomma <= 1;
			sumres <= CH_COMMA;
		end
	end

//		Delay local result	
	always @ (posedge clk) begin
		dbuf[db_waddr] <= sum16;
		db_waddr <= db_waddr + 1;
		db_raddr <= db_waddr - xdelay;	// raddr is less then waddr at least by 1
		db_data <= dbuf[db_raddr];
	end

//		Master trigger
	always @ (posedge clk) begin
		sum64 <= $signed((~xcomma[0]) ? xdata[15:0] : 0) + 
					$signed((~xcomma[1]) ? xdata[31:16] : 0) + 
					$signed((~xcomma[2]) ? xdata[47:32] : 0) + 
					$signed(db_data);
		trigout <= 0;		// default
		if (sum64 > $signed({2'b00, s64thr})) begin
			// generate trigger when sum exceeds threshold
			trigout_s <= 1;
			if (!trigout_s) begin
				trigout <= 1;				// one clk pulse
				tr_addr <= cb_waddr;		// memorise buffer address
			end
		end else if (sum64 <= $signed({3'b000, s64thr[14:0]})) begin
			// ready for new trigger when sum below half threshold
			trigout_s <= 0;
		end
	end
	
//		circular buffer for trigger history
	always @ (posedge clk) begin
		cbuf[cb_waddr] <= sum64[15:1];
		cb_waddr <= cb_waddr + 1;
		cb_data <= cbuf[cb_raddr];
	end

// 

endmodule
