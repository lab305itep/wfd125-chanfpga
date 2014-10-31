`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    23:52:53 10/26/2014 
// Design Name: 
// Module Name:    prc1chan 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//		Process single channel.
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//	- module calculates and subtracks pedestall
// - do self trigger
// - do zero suppression
// - produce master trigger block
//		Self trigger block:
//	10NN NNNN LLLL LLLL - N - channel number, L - data length 
//	0000 XXXX XXXX XXXX - L words of ADC 12-bit data - only these words are counted by L
//		MAster trigger block:
//	11NN NNNN LLLL LLLL - N - channel number, L - data length 
//	1TTT TTTT TTTT TTTT - T - trigger information
// 0000 XXXX XXXX XXXX - L words of ADC 12-bit data - only these words are counted by L
//////////////////////////////////////////////////////////////////////////////////
module prc1chan(
		input clk,					// 125MHz clock
		input [11:0] data,		// ADC raw data
		output reg [11:0] d2sum,	// ADC - pedestal
		output reg [11:0] ped = 0,	// pedestal
		input [15:0] cped,		// common pedestal (12 bits)
		input [15:0] zthr,		// zero suppression threshold (12 bits)
		input [15:0] sthr,		// self trigger threshold (12 bits)
		input [15:0] prescale,	// prescale for self trigger (16 bits)
		input [15:0] winbeg,		// window begin relative to the master trigger (10 bits)
		input [15:0] swinbeg,		// self trigger window begin (10 bits)
		input [15:0] winlen,		// window length (8 bits)
		input [15:0] trigger,	// master trigger information
		output reg [15:0] dout = 0,	// data to arbitter
		input [5:0] num,			// ADC number
		output reg req,			// request to arbitter
		input ack,					// acknowledge from arbitter
		input smask,				// 1 bit mask for sum
		input tmask,				// 1 bit mask for trigger
		input stmask				// 1 bit mask for self trigger
   );

	localparam PBITS = 16;
	reg [PBITS+11:0] pedsum = 0;
	reg [PBITS-1:0] pedcnt = 0;
	reg [11:0] pdata = 0;
	reg [9:0] waddr = 0;
	reg [9:0] raddr = 0;
	reg [11:0] mem [1023:0];
	reg [11:0] rdata;
	reg [15:0] trg_data;
	reg [10:0] wfaddr = 0;
	reg [10:0] rfaddr = 0;
	reg [10:0] swfaddr = 0;
	reg [10:0] ffaddr = 0;		// the word after the full block
	reg [10:0] fffaddr = 0;		// the word after the full block clk negedge
	reg [15:0] fifo [2047:0];
	reg [7:0] copied = 0;
	
//		to total sum
	always @ (posedge clk) begin
		if (data + cped[11:0] > ped) begin
			pdata <= data - ped + cped[11:0];
		end else begin
			pdata <= 0;
		end
		d2sum <= ((!smask) && (data > ped)) ? data - ped : 0;		
	end

//		pedestal calculation
	always @ (posedge clk) begin
		if (&pedcnt) begin
			pedcnt <= 0;
			ped <= pedsum[PBITS+11:PBITS];
			pedsum <= data;
		end else begin
			pedcnt <= pedcnt + 1;
			pedsum <= pedsum + data;
		end
	end

//		circle memory buffer
	always @ (posedge clk) begin
		mem[waddr] <= pdata;
		rdata <= mem[raddr];
		waddr <= waddr + 1;
	end

//		self trigger & prescale 
	reg [15:0] presc_cnt = 0;
	reg strig = 0;
	reg strig_d = 0;
	
	always @ (posedge clk) begin
		strig <= 0;
		if (pdata > (sthr[11:0] + cped[11:0]) && !strig_d) begin
			strig_d <= 1;
			if (presc_cnt == prescale) begin 
				if (!stmask) strig <= 1;
				presc_cnt <= 0;
			end else begin
				presc_cnt <= presc_cnt + 1;
			end
		end
		if (pdata < (sthr[11:0] + cped[11:0])) strig_d <= 0;
	end

//		state mathine definitions
	localparam [4:0] ST_IDLE   = 5'b0_0001;
	localparam [4:0] ST_STCOPY = 5'b0_0010;
	localparam [4:0] ST_MTRIG  = 5'b0_0100;
	localparam [4:0] ST_MTNUM  = 5'b0_1000;
	localparam [4:0] ST_MTCOPY = 5'b1_0000;
	reg [4:0] trg_state = ST_IDLE;
	reg zthr_flag = 0;

//		trigger processing
	reg [15:0] tofifo;
	always @ (posedge clk) begin
		case (trg_state) 
		ST_IDLE: begin
				if (trigger[15] && !tmask) begin
					trg_state <= ST_MTRIG;
					trg_data <= trigger;
					swfaddr <= wfaddr;			// save write fifo address if we will have to reject due to zero suppression
				end else if (strig) begin
					trg_state <= ST_STCOPY;
					swfaddr <= wfaddr;			// save write fifo address if we will have to abort on real trigger
					raddr <= waddr - swinbeg[9:0];
					tofifo = {2'b10, num, winlen[7:0]};	// 2'b10 - self trigger signature
					wfaddr <= wfaddr + 1;
					copied <= 0;
				end
			end
		ST_STCOPY: begin
				if (trigger[15] && !tmask) begin
					trg_state <= ST_MTRIG;
					trg_data <= trigger;
					wfaddr <= swfaddr;
				end else if (copied == winlen[7:0]) begin
					trg_state <= ST_IDLE;
					ffaddr <= wfaddr;
				end else begin
					tofifo = copied; // {4'h0, rdata};
					raddr <= raddr + 1;
					wfaddr <= wfaddr + 1;
					copied <= copied + 1;
				end
			end
		ST_MTRIG: begin
				tofifo = {2'b11, num, winlen[7:0]};	// 2'b11 - master trigger signature
				wfaddr <= wfaddr + 1;
				raddr <= waddr - winbeg[9:0];
				trg_state <= ST_MTNUM;
				zthr_flag <= 0;
			end
		ST_MTNUM: begin
				tofifo = trg_data;
				wfaddr <= wfaddr + 1;
				trg_state <= ST_MTCOPY;
				copied <= 0;
			end
		ST_MTCOPY: begin
				if (copied == winlen[7:0]) begin
					trg_state <= ST_IDLE;
					if (zthr_flag) begin
						ffaddr <= wfaddr;
					end else begin
						wfaddr <= swfaddr;
					end
				end else begin
					tofifo = {4'h0, rdata};
					raddr <= raddr + 1;
					wfaddr <= wfaddr + 1;
					copied <= copied + 1;
					if (rdata > (zthr[11:0] + cped[11:0])) zthr_flag <= 1;
				end
			end
		default: trg_state <= ST_IDLE;
		endcase
		fifo[wfaddr] <= tofifo;
	end

//		Fifo to arbitter
	always @ (posedge clk) begin
		dout <= fifo[rfaddr];
		fffaddr <= ffaddr;
		if (rfaddr != fffaddr) begin
			if (ack) rfaddr <= rfaddr + 1;
			req <= 1;
		end else begin
			req <= 0;
		end
	end

endmodule