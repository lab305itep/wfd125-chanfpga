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
//	1NNN NNNL LLLL LLLL - N - channel number, L - data length in 16-bit words not including CW 
// 0ttt nnnn nnnn nnnn - ttt - trigger type (1 - master, 0 - self)
// 		n - token or selftrigger seq number
//	0000 XXXX XXXX XXXX - L-1 words of ADC 12-bit data
//////////////////////////////////////////////////////////////////////////////////
module prc1chan(
		input clk,					// 125MHz clock
		input ADCCLK,				// ADC data clock
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
		output req,					// request to arbitter
		input ack,					// acknowledge from arbitter
		input smask,				// 1 bit mask for sum
		input tmask,				// 1 bit mask for trigger
		input stmask,				// 1 bit mask for self trigger
		output fifo_full,			// 1 bit fifo full signature
		input raw					// test mode: no selftrigger, zero for summing, raw data on master trigger
   );

	localparam PBITS = 16;
	reg [PBITS+11:0] pedsum = 0;
	reg [PBITS-1:0] pedcnt = 0;
	reg [11:0] pdata = 0;
	reg [9:0] waddr = 0;
	reg [9:0] raddr = 0;
	reg [9:0] wwaddr = 0;
	reg [11:0] mem [1023:0];
	reg [11:0] rdata;
	reg [15:0] trg_data;
	reg [10:0] wfaddr = 0;
	reg [10:0] rfaddr = 0;
	reg [10:0] swfaddr = 0;
	reg [10:0] ffaddr = 0;		// the word after the full block
	reg [10:0] fffaddr = 0;		// the word after the full block clk negedge
	reg [15:0] fifo [2047:0];
	reg [7:0]  copied = 0;
	reg [11:0] ped_s = 0;
	reg ped_pulse = 0;
	reg [1:0] ped_pulse_d = 0;
	reg [1:0] mtrig = 0;
	reg wcopy = 0;
	reg wcopy_d = 0;
	reg [15:0] trigger_s = 0;
	reg [11:0] d2sumfifo [3:0];
	reg [1:0] d2sum_waddr = 0;
	reg [1:0] d2sum_raddr = 2;
	reg d2sum_arst = 0;
	reg d2sum_arst_d = 0;
	wire [13:0] accum;
	
	assign accum = {2'b00, data} - {2'b00, ped_s} + cped[13:0];
	
//		to total sum
	always @ (posedge ADCCLK) begin
		if (raw) begin
			pdata <= data;
		end else if (accum[13]) begin
			pdata <= 0;		//	negative
		end else if (accum[12]) begin
			pdata <= 12'hFFF;	// maximum
		end else begin
			pdata <= accum[11:0];
		end
		d2sumfifo[d2sum_waddr] <= ((!smask) && (data > ped_s) && (!raw)) ? data - ped_s : 0;
		d2sum_waddr <= d2sum_waddr + 1;
		d2sum_arst <= (d2sum_waddr == 0) ? 1 : 0;
	end
	
	always @ (posedge clk) begin
		d2sum_arst_d <= d2sum_arst;
		d2sum <= d2sumfifo[d2sum_raddr];
		d2sum_raddr <= (d2sum_arst_d) ? 0 : d2sum_raddr + 1;
	end

//		pedestal calculation
	always @ (posedge ADCCLK) begin
		if (&pedcnt) begin
			pedcnt <= 0;
			ped_s <= pedsum[PBITS+11:PBITS];
			pedsum <= data;
		end else begin
			pedcnt <= pedcnt + 1;
			pedsum <= pedsum + data;
		end
		ped_pulse = (pedcnt < 3) ? 1 : 0;
	end
	
//		do safe pedestal output
	always @ (posedge clk) begin
		ped_pulse_d <= {ped_pulse_d[0], ped_pulse};
		if (ped_pulse_d == 2'b01) ped <= ped_s;
	end

//		circle memory buffer
	always @ (posedge ADCCLK) begin
		mem[waddr] <= pdata;
		waddr <= waddr + 1;
	end

	always @ (posedge clk) begin
		rdata <= mem[raddr];
	end

//		self trigger & prescale 
	reg [15:0] presc_cnt = 0;
	reg strig = 0;
	reg [1:0] strig_cnt = 0;
	reg strig_d = 0;
	
	always @ (posedge ADCCLK) begin
		strig <= (| strig_cnt) & (!raw);
		if (| strig_cnt) strig_cnt <= strig_cnt - 1;
		if (pdata > (sthr[11:0] + cped[11:0]) && !strig_d) begin
			strig_d <= 1;
			if (presc_cnt >= prescale) begin 
				if (!stmask) begin 
					strig_cnt <= 3;
					wwaddr <= waddr;
				end
				presc_cnt <= 0;
			end else begin
				presc_cnt <= presc_cnt + 1;
			end
		end
		if (pdata < (sthr[11:0] + cped[11:0])) strig_d <= 0;
		wcopy_d <= wcopy;
		if (wcopy_d) wwaddr <= waddr;
	end

//		state mathine definitions
	localparam [4:0] ST_IDLE   = 5'b0_0001;
	localparam [4:0] ST_STCOPY = 5'b0_0010;
	localparam [4:0] ST_MTRIG  = 5'b0_0100;
	localparam [4:0] ST_MTNUM  = 5'b0_1000;
	localparam [4:0] ST_MTCOPY = 5'b1_0000;
	reg [4:0] trg_state = ST_IDLE;
	reg zthr_flag = 0;
	wire [10:0] fifo_free;
	
	assign fifo_free = rfaddr - ffaddr;
	assign fifo_full = (fifo_free < winlen[7:0] + 2) && (| fifo_free);

//		trigger processing
	reg [15:0] tofifo;
	always @ (posedge clk) begin
//		master trigger start
		if (trigger[15]) trigger_s <= trigger;
		mtrig <= {mtrig[0], trigger[15]};
		wcopy <= trigger[15] | mtrig[0];
//		state machine
		case (trg_state) 
		ST_IDLE: if (!fifo_full) begin
				if (mtrig[1] && !tmask) begin
					trg_state <= ST_MTRIG;
					trg_data <= trigger_s;
					swfaddr <= wfaddr;			// save write fifo address if we will have to reject due to zero suppression
				end else if (strig) begin
					trg_state <= ST_STCOPY;
					swfaddr <= wfaddr;			// save write fifo address if we will have to abort on real trigger
					raddr <= wwaddr - swinbeg[9:0];
					tofifo[15] = 1;
					tofifo[14:9] = num;
					tofifo[8:0] = winlen[8:0];
					wfaddr <= wfaddr + 1;
					copied <= 0;
				end
			end
		ST_STCOPY: begin
				if (mtrig[1] && !tmask) begin
					trg_state <= ST_MTRIG;
					trg_data <= trigger_s;
					wfaddr <= swfaddr;
				end else if (copied == winlen[7:0]) begin
					trg_state <= ST_IDLE;
					ffaddr <= wfaddr;
				end else begin
					tofifo = {4'h0, rdata};
					raddr <= raddr + 1;
					wfaddr <= wfaddr + 1;
					copied <= copied + 1;
				end
			end
		ST_MTRIG: begin
				tofifo[15] = 1;
				tofifo[14:9] = num;
				tofifo[8:0] = winlen[8:0] + 1;
				wfaddr <= wfaddr + 1;
				raddr <= wwaddr - winbeg[9:0];
				trg_state <= ST_MTNUM;
				zthr_flag <= 0;
			end
		ST_MTNUM: begin
				tofifo = {4'b0001 ,trg_data[11:0]};
				wfaddr <= wfaddr + 1;
				trg_state <= ST_MTCOPY;
				copied <= 0;
			end
		ST_MTCOPY: begin
				if (copied == winlen[7:0]) begin
					trg_state <= ST_IDLE;
					if (zthr_flag | raw) begin
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
	assign req = rfaddr != fffaddr;
	always @ (posedge clk) begin
		dout <= fifo[rfaddr];
		fffaddr <= ffaddr;
		if (ack) rfaddr <= rfaddr + 1;
	end

endmodule
