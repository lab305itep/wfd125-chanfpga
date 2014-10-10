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
    input         CLK,		// global clock
    input  [1:0]  CLKIN,	// input clock from ADC (375 MHz)
    input  [15:0] DIN,		// Input data from ADC
    input  [1:0]  FR,		// Input frame from ADC 
    output [47:0] DOUT,		// output data (CLK clocked)
	 output 			debug
    );
	 
	 parameter [5:0] FRAME = 6'b111000;
	 wire [1:0] CLKIN_s;
	 wire [1:0] CLKIN_2;
	 wire [5:0] FR_r;
	 reg        BS;
	 wire       IOCE;
	 
	 assign debug = BS;

   IBUFGDS_DIFF_OUT #(
      .DIFF_TERM("TRUE"),   // Differential Termination, "TRUE"/"FALSE" 
      .IOSTANDARD("LVDS_25") // Specify the input I/O standard
   ) IBUFGDS_DIFF_OUT_inst (
      .O(CLKIN_s[0]),   // Buffer diff_p output
      .OB(CLKIN_s[1]), // Buffer diff_n output
      .I(CLKIN[0]),   // Diff_p buffer input (connect directly to top-level port)
      .IB(CLKIN[1])  // Diff_n buffer input (connect directly to top-level port)
   );

   BUFIO2 #(
      .DIVIDE(6),             // DIVCLK divider (1,3-8)
      .DIVIDE_BYPASS("FALSE"),// Bypass the divider circuitry (TRUE/FALSE)
      .I_INVERT("FALSE"),     // Invert clock (TRUE/FALSE)
      .USE_DOUBLER("TRUE")    // Use doubler circuitry (TRUE/FALSE)
   )
   BUFIO2_P (
      .DIVCLK(),      			 // 1-bit output: Divided clock output
      .IOCLK(CLKIN_2[0]),       	 // 1-bit output: I/O output clock
      .SERDESSTROBE(IOCE), 	 // 1-bit output: Output SERDES strobe (connect to ISERDES2/OSERDES2)
      .I(CLKIN_s[0])              // 1-bit input: Clock input (connect to IBUFG)
   );

   BUFIO2 #(
      .DIVIDE(6),             // DIVCLK divider (1,3-8)
      .DIVIDE_BYPASS("FALSE"),// Bypass the divider circuitry (TRUE/FALSE)
      .I_INVERT("FALSE"),     // Invert clock (TRUE/FALSE)
      .USE_DOUBLER("TRUE")    // Use doubler circuitry (TRUE/FALSE)
   )
   BUFIO2_N (
      .DIVCLK(),      			 // 1-bit output: Divided clock output
      .IOCLK(CLKIN_2[1]),       	 // 1-bit output: I/O output clock
      .SERDESSTROBE(), 	 // 1-bit output: Output SERDES strobe (connect to ISERDES2/OSERDES2)
      .I(CLKIN_s[1])              // 1-bit input: Clock input (connect to IBUFG)
   );

//		Frame
	adc1rcv FR_rcv(
    .CLK(CLK),
    .CLKIN(CLKIN_2),
    .DIN(FR),
    .DOUT(FR_r),
	 .BS(BS),
	 .IOCE(IOCE)
    );

	always @ (posedge CLK) begin
		if (!BS && FR_r != FRAME) begin
			BS <= 1'b1;
		end else begin
			BS <= 1'b0;
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
			.IOCE(IOCE)
		);
      end
   endgenerate

endmodule
