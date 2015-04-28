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
//			8-0	(RW)	bit mask, frame and 8 bit lines, for IODELAY increment and reset commands
//			9		(WC)	IODELAY increment, autoclear
//			10		(WC)	IODELAY reset, autoclear, resets all bit line delays to minimal possible
//			11		(WC)	SERDES reset, autoclear
//			12		(RW)	allow bitslip on individual bit lines
//			13		(RW)	allow coherent bitslip on all bit lines according to frame
//		1		(R)	ADC frequency counter
//		2-5	(R)	ADC 0-3 test data error counters
//		6		(R)	bitslip sticky indicators
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
		output        		CLK,		// data clock
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
		input [3:0]			chk_type,	// test pattern number to check 
		input 				chk_rst,		// reset error counters
		input 				chk_enb,		// enable checking (checking interval)
	);
	 
	wire [1:0] 	CLKIN_s;		// clocks from input buffer (375 MHz)
	wire [1:0] 	CLKIN_2;		// clocks from BUFIO2_2CLK (375 MHz)
	wire [5:0] 	FR_r;			// frame data
	wire       	IOCE;			// SERDESSTROBE
	wire       	DIVCLK;		// data clocks from BUFIO2_2CLK (125 MHz)
	 

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

	BUFG BUFG_inst (
      .O			(CLK), 				// 1-bit output: Clock buffer output
      .I			(DIVCLK)  			// 1-bit input: Clock buffer input
   );

//		Recieving frame
	adc1rcvd FR_rcv(
		.CLK		(CLK),				// clock for SERDES bitslip and IODELAY inc/rst timing
		.CLKIN	(CLKIN_2),			// clocks from ADC
		.DIN		(FR),					// one line data from ADC
		.DOUT		(FR_r),				// deserialized data to FPGA
		.IOCE		(IOCE),				// SERDESSTROBE
		.BS		(MBS),				// bitslip enable
		.DINC		(csr[8] & dinc),	// increment command to IODELAY
		.DRST		(csr[8] & drst),	// reset command to IODELAY
		.SRST		(srst)				// asyncronous reset to ISERDES
   );


	adc1rcv FR_rcv(
		.CLK(CLK),
		.CLKIN(CLKIN_2),
		.DIN(FR),
		.DOUT(FR_r),
		.BS(BS),
		.IOCE(IOCE),
		.reset(reset),
		.debug(debug[1])
	);


	 

	always @ (posedge CLK) begin


		if (bs_reset) begin
			bs_cnt <= 0;
		end else if (bs_cntenb && BS && bs_cnt < 16'hFFFF) begin
			bs_cnt <= bs_cnt + 1;
		end
	end 

//		Data
	genvar i;
   generate
      for (i=0; i < 8; i=i+1) 
      begin: DIN_RCV
			adc1rcv DATA_rcv(
			.CLK(CLK),
			.CLKIN(CLKIN_2),
			.DIN(DIN[2*i+1:2*i]),
			.DOUT(DOUT[6*i+5:6*i]),
			.BS(BS),
			.IOCE(IOCE),
			.reset(reset),
			.debug()
		);
      end
   endgenerate

endmodule
