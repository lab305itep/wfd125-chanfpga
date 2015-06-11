`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 		ITEP
// Engineer: 		SvirLex
// 
// Create Date:    00:17:59 06/04/2015 
// Design Name: 	 fpga_chan
// Module Name:    snd_arb 
// Project Name: 	 wfd125
// Target Devices: s6
//
// Additional Comments: 
//		Sends k-char meaning trigger on trigger request - out of band
//		Polls channel fifo's in a round-robbin manner and sends one block of data
//		from each fifo if data is availiable
//		Sends commas in between
//////////////////////////////////////////////////////////////////////////////////

module snd_arb #(
		parameter				NFIFO =17
		) (
		input						clk,
		// fifo control and data
		output reg [NFIFO-1:0]	arb_want,
		input [NFIFO-1:0]		fifo_have,
		input	[NFIFO*16-1:0]			datain,
		// trigger from summing to be sent to main
		input						trig,
output [4:0] debug,
		// GTP data for sending
		output reg [15:0]		dataout,
		output reg				kchar,
		input [8:0]				winlen
		);

	localparam	CH_COMMA = 16'h00BC;		// comma K28.5
	localparam  CH_TRIG  = 16'h801C;		// K-character K28.0

	reg [4:0]			rr_cnt = 0;			// counter for Round Robbin arbitration
	wire 					fifohave;			// OR of dvalids from fifos, actually have from currently selected fifo
	reg [8:0]			towrite = 0;		// number of words in block to write
	wire					nextf;				// force increment of RR counter (after block is fully read)
	wire [15:0]		   datamux [NFIFO-1:0];

	genvar i;
   generate
      for (i=0; i<NFIFO; i=i+1) 
      begin: gwant
//         assign arb_want[i] = ((rr_cnt == i) & ~trig);
			assign datamux[i] = datain[16*i +:16];
      end
   endgenerate

	// RR arbitration and intermediate fifo
	assign	fifohave = |fifo_have;
	assign	nextf = (towrite == 1);

assign debug = 0;

	always @ (posedge clk) begin
		if (trig) begin
			// send trigger out of band
			dataout <= CH_TRIG;
			kchar <= 1;
		end else begin
			// advance round robbin counter
			if (~fifohave | nextf) begin
//			if (~fifohave) begin
				if (rr_cnt == NFIFO-1) begin
					rr_cnt <= 0;
					arb_want <= 1;
				end else begin
					rr_cnt <= rr_cnt + 1;
					arb_want <= {arb_want[NFIFO-2:0], 1'b0};
				end
				towrite <= winlen + 3;
			end
			// send data or comma
			if (fifohave) begin
				// send data if we have any
				dataout <= datamux[rr_cnt];
				kchar <= 0;
				if (|towrite) towrite <= towrite - 1;
				// check block structure
			end else begin
				// send comma if no data
				dataout <= CH_COMMA;
				kchar <= 1;
			end
		end
	end

endmodule
