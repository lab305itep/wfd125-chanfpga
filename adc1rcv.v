`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    18:47:04 10/08/2014 
// Design Name: 
// Module Name:    adc1rcv 
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
//////////////////////////////////////////////////////////////////////////////////
module adc1rcv(
		input CLK,
		input  [1:0] CLKIN,
		input  [1:0] DIN,
		output [5:0] DOUT,
		input BS,
		input IOCE,
		input del_ce,
		input del_rst,
		input del_cal,
		output debug
   );

	wire DIN_s;
	wire DIN_ss;
	wire M2S;

   IBUFDS #(
      .DIFF_TERM("TRUE"),   // Differential Termination
      .IOSTANDARD("LVDS_25") // Specify the input I/O standard
   ) IBUFDS_inst (
      .O(DIN_s),  // Buffer output
      .I(DIN[0]),  // Diff_p buffer input (connect directly to top-level port)
      .IB(DIN[1]) // Diff_n buffer input (connect directly to top-level port)
   );

	assign DIN_ss = DIN_s;

/*
  IODELAY2 #(
      .COUNTER_WRAPAROUND("WRAPAROUND"), // "STAY_AT_LIMIT" or "WRAPAROUND" 
      .DATA_RATE("DDR"),                 // "SDR" or "DDR" 
      .DELAY_SRC("IDATAIN"),             // "IO", "ODATAIN" or "IDATAIN" 
      .IDELAY2_VALUE(0),                 // Delay value when IDELAY_MODE="PCI" (0-255)
      .IDELAY_MODE("NORMAL"),            // "NORMAL" or "PCI" 
      .IDELAY_TYPE("VARIABLE_FROM_ZERO"),  // "FIXED", "DEFAULT", "VARIABLE_FROM_ZERO", "VARIABLE_FROM_HALF_MAX" 
                                         // or "DIFF_PHASE_DETECTOR" 
      .IDELAY_VALUE(0),                  // Amount of taps for fixed input delay (0-255)
      .ODELAY_VALUE(0),                  // Amount of taps fixed output delay (0-255)
      .SERDES_MODE("NONE"),              // "NONE", "MASTER" or "SLAVE" 
      .SIM_TAPDELAY_VALUE(75)            // Per tap delay used for simulation in ps
   )
   IODELAY2_inst (
      .BUSY(),         		// 1-bit output: Busy output after CAL
      .DATAOUT(DIN_ss),    // 1-bit output: Delayed data output to ISERDES/input register
      .DATAOUT2(), 			// 1-bit output: Delayed data output to general FPGA fabric
      .DOUT(),         		// 1-bit output: Delayed data output
      .TOUT(),         		// 1-bit output: Delayed 3-state output
      .CAL(del_cal),       // 1-bit input: Initiate calibration input
      .CE(del_ce),         // 1-bit input: Enable INC input
      .CLK(CLK),       		// 1-bit input: Clock input
      .IDATAIN(DIN_s),     // 1-bit input: Data input (connect to top-level port or I/O buffer)
      .INC(del_ce),        // 1-bit input: Increment / decrement input
      .IOCLK0(CLKIN[0]),   // 1-bit input: Input from the I/O clock network
      .IOCLK1(CLKIN[1]),   // 1-bit input: Input from the I/O clock network
      .ODATAIN(),   			// 1-bit input: Output data input from output register or OSERDES2.
      .RST(del_rst),       // 1-bit input: Reset to zero or 1/2 of total delay period
      .T(1'b1)             // 1-bit input: 3-state input signal
   );
*/

  ISERDES2 #(
      .BITSLIP_ENABLE("TRUE"),      // Enable Bitslip Functionality (TRUE/FALSE)
      .DATA_RATE("DDR"),             // Data-rate ("SDR" or "DDR")
      .DATA_WIDTH(6),                // Parallel data width selection (2-8)
      .INTERFACE_TYPE("RETIMED"), // "NETWORKING", "NETWORKING_PIPELINED" or "RETIMED" 
      .SERDES_MODE("MASTER")           // "NONE", "MASTER" or "SLAVE" 
   )
   ISERDES2_master (
      .CFB0(),           // 1-bit output: Clock feed-through route output
      .CFB1(),           // 1-bit output: Clock feed-through route output
      .DFB(),             // 1-bit output: Feed-through clock output
      .FABRICOUT(debug), // 1-bit output: Unsynchrnonized data output
      .INCDEC(),       // 1-bit output: Phase detector output
      // Q1 - Q4: 1-bit (each) output: Registered outputs to FPGA logic
      .Q1(DOUT[3]),
      .Q2(DOUT[2]),
      .Q3(DOUT[1]),
      .Q4(DOUT[0]),
      .SHIFTOUT(M2S),   // 1-bit output: Cascade output signal for master/slave I/O
      .VALID(),         // 1-bit output: Output status of the phase detector
      .BITSLIP(BS),     // 1-bit input: Bitslip enable input
      .CE0(1'b1),       // 1-bit input: Clock enable input
      .CLK0(CLKIN[0]),  // 1-bit input: I/O clock network input
      .CLK1(CLKIN[1]),  // 1-bit input: Secondary I/O clock network input
      .CLKDIV(CLK),     // 1-bit input: FPGA logic domain clock input
      .D(DIN_ss),       // 1-bit input: Input data
      .IOCE(IOCE),      // 1-bit input: Data strobe input
      .RST(del_rst),    // 1-bit input: Asynchronous reset input
      .SHIFTIN(1'b0)    // 1-bit input: Cascade input signal for master/slave I/O
   );
	
  ISERDES2 #(
      .BITSLIP_ENABLE("TRUE"),      // Enable Bitslip Functionality (TRUE/FALSE)
      .DATA_RATE("DDR"),             // Data-rate ("SDR" or "DDR")
      .DATA_WIDTH(6),                // Parallel data width selection (2-8)
      .INTERFACE_TYPE("RETIMED"), // "NETWORKING", "NETWORKING_PIPELINED" or "RETIMED" 
      .SERDES_MODE("SLAVE")           // "NONE", "MASTER" or "SLAVE" 
   )
   ISERDES2_slave (
      .CFB0(),           // 1-bit output: Clock feed-through route output
      .CFB1(),           // 1-bit output: Clock feed-through route output
      .DFB(),            // 1-bit output: Feed-through clock output
      .FABRICOUT(), // 1-bit output: Unsynchrnonized data output
      .INCDEC(),       // 1-bit output: Phase detector output
      // Q1 - Q4: 1-bit (each) output: Registered outputs to FPGA logic
      .Q1(),
      .Q2(),
      .Q3(DOUT[5]),
      .Q4(DOUT[4]),
      .SHIFTOUT(),      // 1-bit output: Cascade output signal for master/slave I/O
      .VALID(),         // 1-bit output: Output status of the phase detector
      .BITSLIP(BS),     // 1-bit input: Bitslip enable input
      .CE0(1'b1),       // 1-bit input: Clock enable input
      .CLK0(CLKIN[0]),  // 1-bit input: I/O clock network input
      .CLK1(CLKIN[1]),  // 1-bit input: Secondary I/O clock network input
      .CLKDIV(CLK),     // 1-bit input: FPGA logic domain clock input
      .D(0),        		// 1-bit input: Input data
      .IOCE(IOCE),      // 1-bit input: Data strobe input
      .RST(del_rst),    // 1-bit input: Asynchronous reset input
      .SHIFTIN(M2S)     // 1-bit input: Cascade input signal for master/slave I/O
   );
endmodule
