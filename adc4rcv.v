`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    18:35:17 10/08/2014 
// Design Name: 
// Module Name:    adc4rcv 
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
//	ADC output: 12 bit DDR, two-lane, x1 FRAME, bytewise mode
//
//////////////////////////////////////////////////////////////////////////////////
module adc4rcv(
		output        	CLK,		// data clock
		input  [1:0]  	CLKIN,	// input clock from ADC (375 MHz)
		input  [15:0] 	DIN,		// Input data from ADC
		input  [1:0]  	FR,		// Input frame from ADC 
		output [47:0] 	DOUT,		// output data (CLK clocked)
		input         	BSENABLE,
		input 			reset,
		output reg [15:0] bs_cnt,
		input 			bs_reset,
		input 			bs_cntenb,
		output [9:0]	debug
	);
	 
	 parameter [5:0] FRAME = 6'b111000;
	 wire [1:0] CLKIN_s;
	 wire [1:0] CLKIN_2;
	 wire [5:0] FR_r;
	 reg        BS;
	 reg  [3:0] BSCNT;
	 wire       IOCE;
	 wire       DIVCLK;
	 
	 assign debug[0] = BS;

   IBUFGDS_DIFF_OUT #(
      .DIFF_TERM("TRUE"),   // Differential Termination, "TRUE"/"FALSE" 
      .IOSTANDARD("LVDS_25") // Specify the input I/O standard
   ) IBUFGDS_DIFF_OUT_inst (
      .O(CLKIN_s[0]),   // Buffer diff_p output
      .OB(CLKIN_s[1]), // Buffer diff_n output
      .I(CLKIN[0]),   // Diff_p buffer input (connect directly to top-level port)
      .IB(CLKIN[1])  // Diff_n buffer input (connect directly to top-level port)
   );

   BUFIO2_2CLK #(
      .DIVIDE(6)  // DIVCLK divider (3-8)
   )
   BUFIO2_2CLK_p (
      .DIVCLK(DIVCLK),        // 1-bit output: Divided clock output
      .IOCLK(CLKIN_2[0]),     // 1-bit output: I/O output clock
      .SERDESSTROBE(IOCE), 	// 1-bit output: Output SERDES strobe (connect to ISERDES2/OSERDES2)
      .I(CLKIN_s[0]),         // 1-bit input: Clock input (connect to IBUFG)
      .IB(CLKIN_s[1])         // 1-bit input: Secondary clock input
   );

   BUFIO2_2CLK #(
      .DIVIDE(6)  // DIVCLK divider (3-8)
   )
   BUFIO2_2CLK_n (
      .DIVCLK(),             	// 1-bit output: Divided clock output
      .IOCLK(CLKIN_2[1]),     // 1-bit output: I/O output clock
      .SERDESSTROBE(), 	// 1-bit output: Output SERDES strobe (connect to ISERDES2/OSERDES2)
      .I(CLKIN_s[1]),         // 1-bit input: Clock input (connect to IBUFG)
      .IB(CLKIN_s[0])         // 1-bit input: Secondary clock input
   );

	BUFG BUFG_inst (
      .O(CLK), 	// 1-bit output: Clock buffer output
      .I(DIVCLK)  // 1-bit input: Clock buffer input
   );

//		Frame
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
	 
	assign debug[9:2] = {2'b0, FR_r};
	
	always @ (posedge CLK) begin
		if (BSENABLE && (!BSCNT) && FR_r != FRAME) begin
			BS <= 1'b1;
			BSCNT <= 4'b1111;
		end else begin
			BS <= 1'b0;
		end
		if (BSCNT) BSCNT <= BSCNT - 1;
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
