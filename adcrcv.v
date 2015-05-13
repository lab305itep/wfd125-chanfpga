`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:			 ITEP 
// Engineer: 		 SvirLex
// 
// Create Date:    18:35:17 28/04/2015 
// Design Name: 	 fpga_chan
// Module Name:    adcrcv 
// Project Name: 	 wfd125
// Target Devices: s6
// Revision 0.01 - File Created
// Additional Comments: 
//
//		Gets data from one ADC: 4 channels by two differential bit lines
//		ADC output: 12 bit DDR, two-lane, x1 FRAME, bytewise mode
//		Allows individual bit delay adjustment, individual and coherent bitslip adjustment
//		Performs stability checks on bit level and test data checks on channel level
//		
//		Registers:
//	
//		0		(RW)	CSR:
//			8-0	(RW)	bit mask, frame and 8 bit lines, for IODELAY increment commands
//			9		(WC)	IODELAY increment, autoclear
//			10		(WC)	IODELAY reset, autoclear, resets all bit line delays to minimal possible value
//			11		(WC)	SERDES asyncronous reset, autoclear
//			12		(RW)	allow bitslip on individual bit lines
//			13		(RW)	allow coherent bitslip on all bit lines according to frame
//			14		(RW)	IODELAY calibrate, autoclear
//			15		(RW)	unused
//		1		(R)	ADC frequency counter
//		2-5	(R)	ADC 0-3 test data error counters
//		6		(R)	bitslip sticky indicators, any write to this reg clears all indicators
//		7		(R)	frame instability counter
//		8-15	(R)	bit lines 0-7 instability counters
//
//////////////////////////////////////////////////////////////////////////////////
module adcrcv(
		// inputs from ADC
		input  [1:0]  		CLKIN,	// input clock from ADC (375 MHz)
		input  [15:0] 		DIN,		// Input data from ADC
		input  [1:0]  		FR,		// Input frame from ADC 
		// outputs to further processing
		output        		CLK,		// data clock derived from ADC clock
		output [47:0] 		DOUT,		// output data (CLK clocked)
		//	WB interface
		input 				wb_clk,
		input 				wb_cyc,
		input 				wb_stb,
		input 				wb_we,
		input [3:0] 		wb_adr,
		input [31:0] 		wb_dat_i,
		output reg 			wb_ack,
		output [31:0] 		wb_dat_o,
		// checking signals
		input [3:0]			chk_type,	// test pattern number to check (from main CSR)
		input 				chk_rst,		// reset error counters	(from checkseq)
		input 				chk_enb		// enable checking (checking interval, from checkseq)
	);
	 
	wire 	[1:0]		CLKIN_s;		// clocks from input buffer (375 MHz)
	wire 	[1:0]		CLKIN_2;		// clocks from BUFIO2_2CLK (375 MHz)
	wire 	[5:0]		FR_r;			// frame data
	reg	[5:0]		FR_d;			// frame data delayed 1 CLK
	wire       		IOCE;			// SERDESSTROBE
	wire       		DIVCLK;		// data clocks from BUFIO2_2CLK (125 MHz)
	wire 	[8:0]		BS;			// individual data bitslips and frame bitslip = master coherent bitslip
	reg 	[8:0]		dinc;			// CLK timed iodelay increment signal
	reg 				drst;			// CLK timed iodelay reset signal
	reg	[47:0]	DOUT_d;		// ADC data delayed 1 CLK
	
	// registers
	reg 	[15:0]	csr;					// commands and status reg
	reg 	[31:0]	frq_cnt;				// ADC frequency counter
	wire 	[15:0]	err_cnt	[3:0];	// test data error counters
	reg	[8:0]		bs_cap;				// bitslip capture
	reg 	[7:0]		ins_cnt	[8:0];	// instability counters
	reg	[15:0]	rdat;					// WB read register
	reg   [8:0]    delay_inc;			// increment delay
	reg   [1:0]    delay_pulse;		// make pulse on csr[9] leading edge
	reg   [3:0]    autoclr;				// for autoclear in 2 wb_clk
	 

	// Recieving and processing clocks
	
   IBUFGDS_DIFF_OUT #(
      .DIFF_TERM	("TRUE"),   	// Differential Termination, "TRUE"/"FALSE" 
      .IOSTANDARD	("LVDS_25") 	// Specify the input I/O standard
   ) IBUFGDS_DIFF_OUT_inst (
      .O			(CLKIN_s[0]), 		// Buffer diff_p output
      .OB		(CLKIN_s[1]),		// Buffer diff_n output
      .I			(CLKIN[0]), 		// Diff_p buffer input (connect directly to top-level port)
      .IB		(CLKIN[1])  		// Diff_n buffer input (connect directly to top-level port)
   );

   BUFIO2_2CLK #(
      .DIVIDE	(6)  					// DIVCLK divider (3-8)
   )
   BUFIO2_2CLK_p (
      .DIVCLK	(DIVCLK),        	// 1-bit output: Divided clock output
      .IOCLK	(CLKIN_2[0]),    	// 1-bit output: I/O output clock
      .SERDESSTROBE	(IOCE), 		// 1-bit output: Output SERDES strobe (connect to ISERDES2/OSERDES2)
      .I			(CLKIN_s[0]),     // 1-bit input: Clock input (connect to IBUFG)
      .IB		(CLKIN_s[1])      // 1-bit input: Secondary clock input
   );

   BUFIO2_2CLK #(
      .DIVIDE	(6)  					// DIVCLK divider (3-8)
   )
   BUFIO2_2CLK_n (
      .DIVCLK	(),             	// 1-bit output: Divided clock output
      .IOCLK	(CLKIN_2[1]),     // 1-bit output: I/O output clock
      .SERDESSTROBE	(), 			// 1-bit output: Output SERDES strobe (connect to ISERDES2/OSERDES2)
      .I			(CLKIN_s[1]),     // 1-bit input: Clock input (connect to IBUFG)
      .IB		(CLKIN_s[0])      // 1-bit input: Secondary clock input
   );

/*
wire CLKIN_b;

   IBUFGDS #(
      .DIFF_TERM("TRUE"), // Differential Termination
      .IOSTANDARD("LVDS_25") // Specifies the I/O standard for this buffer
   ) IBUFGDS_inst (
      .O(CLKIN_b),  // Clock buffer output
      .I(CLKIN[0]),  // Diff_p clock buffer input
      .IB(CLKIN[1]) // Diff_n clock buffer input
   );

   BUFIO2 #(
      .DIVIDE(6),             // DIVCLK divider (1,3-8)
      .DIVIDE_BYPASS("FALSE"), // Bypass the divider circuitry (TRUE/FALSE)
      .I_INVERT("FALSE"),     // Invert clock (TRUE/FALSE)
      .USE_DOUBLER("TRUE")   // Use doubler circuitry (TRUE/FALSE)
   )
   BUFIO2_p (
      .DIVCLK(DIVCLK),             // 1-bit output: Divided clock output
      .IOCLK(CLKIN_2[0]),               // 1-bit output: I/O output clock
      .SERDESSTROBE(IOCE), // 1-bit output: Output SERDES strobe (connect to ISERDES2/OSERDES2)
      .I(CLKIN_b)                        // 1-bit input: Clock input (connect to IBUFG)
   );

   BUFIO2 #(
      .DIVIDE(6),             // DIVCLK divider (1,3-8)
      .DIVIDE_BYPASS("FALSE"), // Bypass the divider circuitry (TRUE/FALSE)
      .I_INVERT("TRUE"),     // Invert clock (TRUE/FALSE)
      .USE_DOUBLER("FALSE")   // Use doubler circuitry (TRUE/FALSE)
   )
   BUFIO2_n (
      .DIVCLK(),             // 1-bit output: Divided clock output
      .IOCLK(CLKIN_2[1]),               // 1-bit output: I/O output clock
      .SERDESSTROBE(), // 1-bit output: Output SERDES strobe (connect to ISERDES2/OSERDES2)
      .I(CLKIN_b)                        // 1-bit input: Clock input (connect to IBUFG)
   );
*/

	BUFG BUFG_inst (
      .O			(CLK), 				// 1-bit output: Clock buffer output
      .I			(DIVCLK)  			// 1-bit input: Clock buffer input
   );

//		Recieving frame
	adc1rcvd FR_rcv(
		.CLK		(CLK),					// clock for SERDES bitslip and IODELAY inc/rst timing
		.CLKIN	(CLKIN_2),				// clocks from ADC
		.DIN		(FR),						// one line data from ADC
		.DOUT		(FR_r),					// deserialized data to FPGA
		.IOCE		(IOCE),					// SERDESSTROBE
		.BS		(BS[8]),					// bitslip pulse - master bs
		.SRST		(csr[11]),				// asyncronous reset to ISERDES
		.DINC		(delay_inc[8]),   	// increment command to IODELAY
		.DCAL		(csr[14]),				// CAL command to IODELAY
		.DRST		(csr[10])				// reset command to IODELAY
   );

	bitslip FR_bs(
    .CLK			(CLK),		// ADC clock divided
    .DATA		(FR_r),		// ADC one line 6-bit data, CLK timed
	 .BSENB		(csr[13]),	// allow master bitslip
    .BS			(BS[8])		// master coherent bitslip
    );

//	Recieving data
	genvar i;
   generate
      for (i=0; i < 8; i=i+1) 
      begin: DIN_RCV 
			adc1rcvd DATA_rcv(
				.CLK		(CLK),					// clock for SERDES bitslip and IODELAY inc/rst timing
				.CLKIN	(CLKIN_2),				// clocks from ADC
				.DIN		(DIN[2*i+1:2*i]),		// one line data from ADC
				.DOUT		(DOUT[6*i+5:6*i]),	// deserialized data to FPGA
				.IOCE		(IOCE),					// SERDESSTROBE
				.BS		(BS[8] | BS[i]),		// bitslip pulse - this or master bs
				.SRST		(csr[11]),				// asyncronous reset to ISERDES
				.DINC		(delay_inc[i]),	   // increment command to IODELAY
				.DCAL		(csr[14]),				// CAL command to IODELAY
				.DRST		(csr[10])				// reset command to IODELAY
			);
			
			bitslip DATA_bs(
				.CLK		(CLK),					// ADC clock divided
				.DATA		(DOUT[6*i+5:6*i]),	// ADC one line 6-bit data, CLK timed
				.BSENB	(csr[12]),				// allow master bitslip
				.BS		(BS[i])					// master coherent bitslip
			);
      end
   endgenerate

	// count ADC frequncy
	always @ (posedge CLK or posedge chk_rst) begin
		if	(chk_rst) begin
			// asyncronous reset
			frq_cnt <= 0;
		end else if (chk_enb) begin
			// count freq transitions when check enabled
			frq_cnt <= frq_cnt + 1;
		end
	end 
	
	// check ADC data
   generate
      for (i=0; i < 4; i=i+1) 
      begin: UADCCHK 
			adccheck ADC_CHK(
				.data		(DOUT[12*i+11:12*i]),	// ADC data received
				.clk		(CLK),						// ADC data clock
				.cnt		(err_cnt[i]),				//	Error counter
				.count	(chk_enb),					// Count errors enable
				.reset	(chk_rst),					// Reset error counter
				.type		(chk_type)					// Pattern type
			);
      end
   endgenerate

	// capture bitslips
   generate
      for (i=0; i < 9; i=i+1) 
      begin: BS_CAP 
			always @ (posedge wb_clk or posedge BS[i]) begin
				if	(BS[i]) begin
					// asyncronous set = capture
					bs_cap[i] <= 1;
				end else if (wb_cyc & wb_stb & wb_we & (wb_adr == 6)) begin
				// syncronous clear on WB write
					bs_cap[i] <= 0;
				end
			end 
      end
   endgenerate

	// frame instability
	always @ (posedge CLK) begin
		FR_d <= FR_r;
	end
	always @ (posedge CLK or posedge chk_rst) begin
		if	(chk_rst) begin
			// asyncronous reset
			ins_cnt[0] <= 0;
		end else if (chk_enb & (FR_r != FR_d) & ~(&ins_cnt[0])) begin
			// increment when previous data is not equal to current untill full
			ins_cnt[0] <= ins_cnt[0] + 1;
		end
	end 

	// data instability
	always @ (posedge CLK) begin
		DOUT_d <= DOUT;
	end
   generate
      for (i=0; i < 8; i=i+1) 
      begin: INS_CNT 
			always @ (posedge CLK or posedge chk_rst) begin
				if	(chk_rst) begin
				// asyncronous reset
					ins_cnt[i+1] <= 0;
				end else if (chk_enb & (DOUT_d[6*i+5:6*i] != DOUT[6*i+5:6*i]) & ~(&ins_cnt[i+1])) begin
				// increment when previous data is not equal to current untill full
					ins_cnt[i+1] <= ins_cnt[i+1] + 1;
				end
			end 
		end
	endgenerate
	
	//	make IODEALY inc pulses
	always @ (posedge CLK) begin
		delay_pulse <= {delay_pulse[0], csr[9]};
		delay_inc <= 0;
		if (delay_pulse == 2'b01) begin
			delay_inc <= csr[8:0];
		end
	end
	
	// Wishbone interface
	assign wb_dat_o = (wb_ack) ? {16'h0000, rdat} : {32'hZZZZZZZZ};
	always @ (posedge wb_clk) begin
		wb_ack <= 0;		// default
		if (wb_cyc & wb_stb & ~wb_we) begin
			// read regs
			wb_ack <= 1;
			case (wb_adr)
				0:	 rdat <= csr;										// csr
				1:  rdat <= frq_cnt[31:16];							// high bits of frequency counter
				2:  rdat <= err_cnt[0];
				3:  rdat <= err_cnt[1];
				4:  rdat <= err_cnt[2];
				5:  rdat <= err_cnt[3];
				6:  rdat <= {7'h00, bs_cap};							// bitslip capture
				7:  rdat <= {8'h00, ins_cnt[0]};
				8:  rdat <= {8'h00, ins_cnt[1]};
				9:  rdat <= {8'h00, ins_cnt[2]};
				10: rdat <= {8'h00, ins_cnt[3]};
				11: rdat <= {8'h00, ins_cnt[4]};
				12: rdat <= {8'h00, ins_cnt[5]};
				13: rdat <= {8'h00, ins_cnt[6]};
				14: rdat <= {8'h00, ins_cnt[7]};
				15: rdat <= {8'h00, ins_cnt[8]};
			endcase
		end else if (wb_cyc & wb_stb) begin
			wb_ack <= 1;
			if (~(|wb_adr)) begin
				// write to csr
				csr <= wb_dat_i[15:0];
			end
		end
		// autoclear iodelay inc and reset commands and serdes reset after 2 clk
		if (autoclr[0]) csr[9] <= 0;
		if (autoclr[1]) csr[10] <= 0;
		if (autoclr[2]) csr[11] <= 0;
		if (autoclr[3]) csr[14] <= 0;
		autoclr <= {csr[14], csr[11:9]};
	end

endmodule
