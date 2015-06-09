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
//	- module calculates and subtracks pedestal
// - does self trigger
// - does zero suppression
// - produce master trigger block
//		Blocks sent to arbitter 
//		0	1CCC CCCL LLLL LLLL - CCCCCC - 6-bit channel number which produced the block ; 
//										 LLLLLLLLL - 9-bit block length in 16-bit words not including CW, L = WinLen + 2
// 	1	0ttt penn nnnn nnnn - ttt - trigger block type (0 - self, 1 - master, 3 - raw ADC data on master trigger) ;
//										 n - master : 10-bit trigger token from gtp, e - token error (as recieved by main FPGA)
//											- self : 10-bit sequential selftrigger number, as counted after prescale, e = 0;
//										 p - sent block sequential number LSB, independently on master/self
//		2	0000 0000 0000 0DDD - master : high resolution relative trigger time 0-5 or timing error if 7
//			or
//		2	0000 DDDD DDDD DDDD - self : baseline absolute value in ADC units
//		3	0XXX XXXX XXXX XXXX - L-2 words of ADC signed data after pedestal subtraction or ADC raw data
//////////////////////////////////////////////////////////////////////////////////
module prc1chan # (
		parameter ABITS = 12,			// width of ADC data
		parameter CBITS = 10,			// number of bits in circular buffer memory
		parameter FBITS = 11				// number of bits in output fifo memory
		)
		(
		input 				clk,			// 125MHz GTP and output data clock
		input [5:0] 		num,			// ADC number
		// ADC data from its reciever
		input 				ADCCLK,		// ADC data clock
		input [ABITS-1:0]	ADCDAT,		// ADC raw data
		// data processing programmable parameters
		input [ABITS-1:0]	zthr,			// zero suppression threshold (12 bits)
		input [ABITS-1:0]	sthr,			// self trigger threshold (12 bits)
		input [15:0] 		prescale,	// prescale for self trigger (16 bits)
		input [CBITS-1:0]	mwinbeg,		// window begin relative to the master trigger (10 bits)
		input [CBITS-1:0]	swinbeg,		// self trigger window begin relative to sthr crossing (10 bits)
		input [8:0] 		winlen,		// window length (9 bits, but not greater than 509)
		input 				smask,		// 1 bit mask for sum
		input 				tmask,		// 1 bit mask for trigger
		input 				stmask,		// 1 bit mask for self trigger
		input					invert,		// change waveform sign
		input 				raw,			// test mode: no selftrigger, zero for summing, raw data on master trigger
		// calculated baseline value for readout
		output reg [ABITS-1:0] ped = 0,	// pedestal (baseline)
		// trigger token
		input [15:0] 		token,		// master trigger token as recieved by main FPGA and transmitted 
												// through GTP: 0000 0enn nnnn nnnn
		input					tok_vld,		// token valid strobe
		// trigger pulse and time
		input					adc_trig,	// master trigger as recieved by this ADC
		input	[2:0]			trig_time,	// high resolution master trigger time
		// inhibit
		input					inhibit,		// inhibits ONLY self trigger production
		// arbitter interface for data output
		input 				give,			// request from arbitter
		output 				have,			// acknowledge to arbitter, immediate with give
		output [15:0] 		dout,			// tristate data to arbitter
		output reg			missed,		// 1 clk pulse when fifo cannot accept data because of full
		
		// ???
		output [4:0] debug,
		
		// to sumtrig
		output reg [15:0] d2sum			// (ADC - pedestal) signed to trigger summation
   );

	assign debug = {trg_clr, tok_got, mtrig, tok_vld, adc_trig};

	// pedestal calculations
		localparam 					PBITS = 16;			// number of bits in pedestal window counter
		reg [PBITS+ABITS-1:0] 	pedsum = 0;			// sum for average
		reg [PBITS-1:0] 			pedcnt = 0;			// ped window counter
		reg [ABITS-1:0] 			ped_s = 0;			// averaged value, ADCCLK timed
		reg 							ped_pulse = 0;		// ped ready
		reg [1:0] 					ped_pulse_d = 0;	// for CLK sync

	//	ADC data after pedestal subtraction and inversion, signed, ADCCLK timed
		reg signed [15:0]			pdata = 0;

	//	circular buffer for keeping prehistory and resynching ADC data to clk 
		reg [15:0] 				cbuf [2**CBITS-1:0];	// buffer itself
		reg [15:0]				cb_data = 0;			// buffer output data
		reg [CBITS-1:0]		cb_waddr = 0;			// write address, ADCCLK timed
		reg [CBITS-1:0]		cb_raddr = 0;			// read address, clk timed
		reg [CBITS-1:0] 		str_addr = 0;			// write address at self trigger start, ADCCLK timed
		reg [CBITS-1:0] 		mtr_addr = 0;			// write address at master trigger start, ADCCLK timed

	// self trigger & prescale 
		reg 						discr = 0;				// signal above selftrigger threshold
		reg 						strig = 0;				// self trigger
		reg [9:0]				strig_cnt = 0;			// self trigger counter after prescale
		reg [15:0] 				presc_cnt = 0;			// selftrigger prescale counter
	
	// master trigger
		reg						mtrig = 0;				// master trigger
		reg [2:0]				tr_time = 0;			// high resoultion trigger time	
		reg						tok_got = 0;			// token accepted for this mtrig
		reg [10:0]				tr_tok = 0;				// memorized token with error
		
	// output fifo
		reg [15:0] 				fifo [2**FBITS-1:0];	// fifo itself
		reg [15:0]				tofifo;					// variable to store data for writes to fifo
		reg [15:0]				f_data;					// fifo output data
		reg [FBITS-1:0] 		f_waddr = 0;			// fifo current write address
		reg [FBITS-1:0] 		f_waddr_s = 0;			// temprary holder for waddr for token out of order writing
		reg [FBITS-1:0] 		f_raddr = 0;			// fifo current read address
		wire [FBITS-1:0] 		graddr;					// fifo read address for arbitter output
		reg [FBITS-1:0] 		f_blkend = 0;			// memorized address of block end or start of currently written block
		wire [10:0] 			fifo_free;				// number of words availiable for the next block
		wire						fifo_full;				// fifo cannot accept next block

	//		state mathine definitions
		localparam ST_IDLE   = 0;
		localparam ST_MTRIG  = 1;
		localparam ST_MTIME  = 2;
		localparam ST_MTCOPY = 3;
		localparam ST_MTOK	= 4;
		localparam ST_STRIG  = 5;
		localparam ST_STPED	= 6;
		localparam ST_STCOPY = 7;
		localparam ST_TRGFIN = 8;
		localparam ST_TRGCLR	= 9;

		reg [3:0] 				trg_state = ST_IDLE;	// state
		reg [8:0] 				to_copy = 0;			// number of words from CB left for copying
		reg [8:0] 				blklen;					// block length derived from winlen
		reg 						zflag = 0;				// flag to apply zero suppression to the current block
		reg						blkpar = 0;				// sequential parity of any sent block
		reg						trg_clr;					// flag to indicate end of trigger block writing

	// 4 word circular buffer to resync ADC data (ped subtracted) to clk for trigger sum calculations
		reg [15:0] 			d2sumfifo [3:0];			//	buffer itself
		reg [1:0] 			d2sum_waddr = 0;			// write address
		reg [1:0] 			d2sum_raddr = 2;			// read address
		reg 					d2sum_arst = 0;			// sync as generated at ADCCLK
		reg 					d2sum_arst_d = 0;			// sync as felt at clk
	
//		pedestal calculation
	always @ (posedge ADCCLK) begin
		if (&pedcnt) begin
			pedcnt <= 0;
			ped_s <= pedsum[PBITS+11:PBITS];
			pedsum <= ADCDAT;
		end else begin
			pedcnt <= pedcnt + 1;
			pedsum <= pedsum + ADCDAT;
		end
		ped_pulse = (pedcnt < 3) ? 1 : 0;
	end
	//	do safe pedestal output
	always @ (posedge clk) begin
		ped_pulse_d <= {ped_pulse_d[0], ped_pulse};
		if (ped_pulse_d == 2'b01) ped <= ped_s;
	end

// 	pedestal subtraction and inversion
	always @ (posedge ADCCLK) begin
		if (raw) begin
			pdata <= {{(16-ABITS){1'b0}} ,ADCDAT};
		end else if (invert) begin
			pdata <= ped_s - ADCDAT;
		end else begin
			pdata <= ADCDAT - ped_s;
		end
	end

//		circular memory buffer
	// write at ADCCLK
	always @ (posedge ADCCLK) begin
		cbuf[cb_waddr] <= pdata;
		cb_waddr <= cb_waddr + 1;
	end
	// read at clk
	always @ (posedge clk) begin
		cb_data <= cbuf[cb_raddr];
	end

//		self trigger & prescale 
	always @ (posedge ADCCLK) begin
		if (~stmask & ~raw & ~inhibit) begin
			if (pdata > $signed({1'b0,sthr})) begin
				if (~discr) begin
					// crossing threshold (for the first time)
					discr <= 1;
					// prescale threshold crossings
					if (|presc_cnt) begin
						presc_cnt <= presc_cnt - 1;
					end else begin
						presc_cnt <= prescale;
						// produce self trigger and memorize current position in circular buffer
						strig <= 1;
						strig_cnt <= strig_cnt + 1;	// count self triggers after prescale independently of transmission
						str_addr <= cb_waddr;
					end
				end
			end else if (pdata <= $signed({1'b0,sthr[ABITS-1:1]})) begin
				// HALF threshold crossed back (noise reduction)
				discr <= 0;
				// finish with trigger on command from state machine
				if (trg_clr) strig <= 0;
			end 
		end else begin
			strig <= 0;
		end
	end

//		master trigger	and token
	always @ (posedge ADCCLK) begin
		if (adc_trig & ~mtrig & ~tmask) begin
			// catch the first adc_trig and corresponding trigger time
			mtrig <= 1;
			mtr_addr <= cb_waddr;
			tr_time <= trig_time;
		end else if (trg_clr) begin
			// finish with trigger on command from state machine after token is accepted
			mtrig <= 0;
		end
	end
	// process token from GTP (appears later than all adc_trig's)
	always @ (posedge clk) begin
		if (mtrig) begin
			// catch token from GTP
			if (tok_vld) begin
				tok_got <= 1;
				tr_tok <= token[10:0];
			end
		end else begin
			// clear token flag on trigger end
			tok_got <= 0;
		end
	end

//		block writing on triggers with state machine
	assign 	fifo_free = f_raddr - f_blkend;
	assign 	fifo_full = (fifo_free < (winlen + 3)) & (|fifo_free);

	// state machine
	always @ (posedge clk) begin
		trg_clr <= 0;		// default
		missed <= 0;		// default
		blklen <= winlen + 2;	// relatch for better timing
//		state machine
		case (trg_state) 
		ST_IDLE: begin 
			if (mtrig | strig) begin
				if (~fifo_full) begin
					// write nothing on zero winlen
					if (~|winlen) begin
						trg_state <= ST_TRGCLR;
					end else begin
					// we can write to fifo, write CW
						tofifo <= {1'b1, num, blklen};
//						f_waddr <= f_waddr + 1;
						to_copy <= winlen;
						if (mtrig) begin		// master trigger has priority
							trg_state <= ST_MTRIG;
						end else	begin // strig
							trg_state <= ST_STRIG;
						end
					end
				end else begin
					// we can't write to fifo -- just finish the trigger
					missed <= 1;
					trg_state <= ST_TRGCLR;
				end
			end
		end
		ST_MTRIG: begin
			// just skip one word, we cannot write token here
			f_waddr <= f_waddr + 1;
			cb_raddr <= mtr_addr - mwinbeg;	// prepare for reading from circular buffer
			trg_state <= ST_MTIME;
		end
		ST_MTIME: begin
			// write high resolution time
			tofifo <= {13'h0000, tr_time};
			f_waddr <= f_waddr + 1;
			cb_raddr <= cb_raddr + 1;			// preincrement circular buffer read address
			zflag <= ~raw;							// set zero suppression flag (no zero suppression in raw mode)
			trg_state <= ST_MTCOPY;
		end
		ST_MTCOPY: begin
			// stream data from circular buffer to fifo
			tofifo <= {1'b0, cb_data[14:0]};
			f_waddr <= f_waddr + 1;
			cb_raddr <= cb_raddr + 1;
			to_copy <= to_copy - 1;
			if ($signed(cb_data) > $signed({1'b0, zthr})) zflag <= 0;	// remove ZS flag if signal is above threshold
			if (to_copy == 1)	begin
//				f_waddr <= f_blkend + 1;			// prepare waddr for token writing
//				f_waddr_s <= f_waddr + 1;			// save next waddr for further restoration
				f_waddr_s <= f_waddr + 2;			// save last waddr for further restoration
				trg_state <= ST_MTOK;
			end
		end
		ST_MTOK: begin
			if (zflag) begin
				// if zero suppression happens, restore write pointer to the beg of block and finish with trigger
				f_waddr <= f_blkend;
				trg_state <= ST_TRGCLR;
			end else if (tok_got) begin
				// no ZS -- wait for token, write it to proper place and update block end pointer
				tofifo <= {2'b00, raw, 1'b1, blkpar, tr_tok};
//				f_waddr <= f_waddr_s;			// restore waddr to the first empty word
//				f_blkend <= f_waddr_s;			// f_blkend now points to the end of the newly written block
//				blkpar <= ~blkpar;
//				trg_state <= ST_TRGCLR;
				f_waddr <= f_blkend + 1;
				trg_state <= ST_TRGFIN;
			end
		end
		ST_STRIG: begin
			if (mtrig) begin
			// enforce master trigger priority
				f_waddr <= f_blkend;
				trg_state <= ST_IDLE;
			end else begin
			// write sequential self trigger number
				tofifo <= {4'h0, blkpar, 1'b0, strig_cnt};
//				f_waddr <= f_waddr + 1;
				cb_raddr <= str_addr - swinbeg;	// prepare for reading from circular buffer
				trg_state <= ST_STPED;
			end
		end
		ST_STPED: begin
			if (mtrig) begin
			// enforce master trigger priority
				f_waddr <= f_blkend;
				trg_state <= ST_IDLE;
			end else begin
			// write pedestal value
				tofifo <= {{(16-ABITS){1'b0}}, ped};
				f_waddr <= f_waddr + 1;
				cb_raddr <= cb_raddr + 1;			// preincrement circular buffer read address
				trg_state <= ST_STCOPY;
			end
		end
		ST_STCOPY: begin
			if (mtrig) begin
			// enforce master trigger priority
				f_waddr <= f_blkend;
				trg_state <= ST_IDLE;
			end else begin
				// stream data from circular buffer to fifo
				tofifo <= {1'b0, cb_data[14:0]};
				f_waddr <= f_waddr + 1;
				cb_raddr <= cb_raddr + 1;
				to_copy <= to_copy - 1;
				if (to_copy == 1)	begin
//					f_blkend <= f_waddr;		// f_blkend now points to the end of the newly written block
//					blkpar <= ~blkpar;
//					trg_state <= ST_TRGCLR;
					f_waddr_s <= f_waddr + 2;
					trg_state <= ST_TRGFIN;
				end
			end
		end
		ST_TRGFIN: begin
			f_waddr <= f_waddr_s;			// restore waddr to the first empty word
			f_blkend <= f_waddr_s;			// f_blkend now points to the end of the newly written block
			blkpar <= ~blkpar;
			trg_state <= ST_TRGCLR;
		end
		ST_TRGCLR: begin
			trg_clr <= 1;
			if (~mtrig & ~strig)
				trg_state <= ST_IDLE;
		end
		default: trg_state <= ST_IDLE;
		endcase
		// fifo
		// write fifo
		fifo[f_waddr] <= tofifo;
		// read fifo
		f_data <= fifo[graddr];
		// increment raddr on data outputs
		if (have) begin
			f_raddr <= f_raddr + 1;
		end	
	end

//		Fifo and it's connection to arbitter
	assign graddr = (have) ? (f_raddr + 1) : f_raddr;	
	assign have = give & (f_raddr != f_blkend);
	assign dout = (have) ? f_data : 16'hZZZZ;

//		to total sum -- resync adc data to clk
	// fill buffer at ADFCCLK
	always @ (posedge ADCCLK) begin
		// send zero if masked or raw data requested
		d2sumfifo[d2sum_waddr] <= ((~smask) & (~raw)) ? pdata : 0;
		d2sum_waddr <= d2sum_waddr + 1;
		d2sum_arst <= (d2sum_waddr == 0) ? 1 : 0;
	end
	// read buffer at clk
	always @ (posedge clk) begin
		d2sum_arst_d <= d2sum_arst;
		d2sum <= d2sumfifo[d2sum_raddr];
		d2sum_raddr <= (d2sum_arst_d) ? 0 : d2sum_raddr + 1;
	end

endmodule
