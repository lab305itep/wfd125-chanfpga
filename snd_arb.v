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
		input [NFIFO-1:0]			fifo_have,
		input	[NFIFO*16-1:0]		datain,
		// trigger from summing to be sent to main
		input							trig,
		output reg [4:0] 			debug,
		// GTP data for sending
		output reg [15:0]			dataout,
		output reg					kchar
		);

	localparam	CH_COMMA = 16'h00BC;		// comma K28.5
	localparam  CH_TRIG  = 16'h801C;		// K-character K28.0

	reg [4:0]			rr_cnt = 0;			// counter for Round Robbin arbitration
	wire 					fifohave;			// OR of dvalids from fifos, actually have from currently selected fifo
	reg [8:0]			towrite = 0;		// number of words in block to write
	wire [15:0]		   datamux [NFIFO-1:0];
	reg					trig_d = 0;			// delayed trigger
	reg [1:0]			state = 0;
	localparam ST_NEXT = 0;
	localparam ST_WAIT = 1;
	localparam ST_CW = 2;
	localparam ST_COPY = 3;
	
	genvar i;
   generate
      for (i=0; i<NFIFO; i=i+1) 
      begin: gwant
			assign datamux[i] = datain[16*i +:16];
      end
   endgenerate

	// RR arbitration and intermediate fifo
	assign	fifohave = |fifo_have;


	always @ (posedge clk) begin
		debug <= {kchar, dataout[15], fifohave, |towrite, rr_cnt == 0};
	//	comma is the default
		kchar <= 1;
		dataout <= CH_COMMA;
	// send data if we have it
		if (fifohave) begin
			kchar <= 0;
			dataout <= datamux[rr_cnt];
		end 
		if (trig_d) begin
	// send trigger when we paused the data - one clock delay
			kchar <= 1;
			dataout <= CH_TRIG;
		end
		trig_d <= trig;
		
		arb_want <= 0;
		if (trig) begin
			if (state == ST_WAIT) begin
				state <= ST_CW;
			end
		end else begin
			case (state)
				ST_NEXT: begin
					if (rr_cnt == NFIFO-1) begin
						rr_cnt <= 0;
						arb_want <= 1;
					end else begin
						rr_cnt <= rr_cnt + 1;
						arb_want[rr_cnt + 1] <= 1;
					end
					state <= ST_WAIT;
				end
				ST_WAIT: begin
					state <= ST_CW;
				end
				ST_CW: begin
					if (fifohave) begin 
						arb_want[rr_cnt + 1] <= 1;
						if (datamux[rr_cnt][15]) begin
							towrite <= datamux[rr_cnt][8:0];
							state <= ST_COPY;
						end else begin
							state <= ST_WAIT;
						end
					end else begin
						state <= ST_NEXT;
					end
				end
				ST_COPY: begin
					if (towrite == 1) begin
						arb_want[rr_cnt + 1] <= 1;
						towrite <= towrite - 1;
					end else begin
						state <= ST_NEXT;
						arb_want[rr_cnt + 1] <= 1;
					end
				end
			endcase
		end

	end

endmodule
