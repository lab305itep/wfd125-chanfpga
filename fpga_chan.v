`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    14:05:44 09/30/2014 
// Design Name: 
// Module Name:    fpga_chan 
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
module fpga_chan(
    // ADC A
	 input [15:0] ADA,
    input [1:0] ACA,
    input [1:0] AFA,
	 // ADC B
    input [15:0] ADB,
    input [1:0] ACB,
    input [1:0] AFB,
	 // ADC C
    input [15:0] ADC,
    input [1:0] ACC,
    input [1:0] AFC,
	 // ADC D
    input [15:0] ADD,
    input [1:0] ACD,
    input [1:0] AFD,
	 // I2C to clock buffer
    inout I2CCLK,
    inout I2CDAT,
	 // SPI to ADCs
    output SPICLK,
    inout SPIDAT,
    output [3:0] SPISEL,
	 // Analog input control
    output [3:0] ACNTR,
	// Fast serial connections and CLK
	// Main Clock
    input [1:0] RCLK,
	// Recievers
    input [1:0] RX0,
    input [1:0] RX1,
    input [1:0] RX2,
    input [1:0] RX3,
	// Transmitters
    output [1:0] TX0,
    output [1:0] TX1,
    output [1:0] TX2,
    output [1:0] TX3,
	 // Stone number
	 input [1:0] CHN,
	 // Common lines to other FPGA
    output [15:0] ICX,
	 // Configuration pins
    input INIT,
    input DOUT,
    input CCLK,
    input DIN,
	 // Test points
    output [5:1] TP
    );

	wire [15:0] GTP_DAT_A;
	wire [1:0] GTP_CHARISK_A;
	wire [1:0] GTP_CHARISC_A;
	wire [1:0] GTP_DISPERR_A;
	wire [1:0] GTP_NOTINTAB_A;
	wire 		  GTP_ALIGNED_A;
	wire [1:0] GTPCLKOUT;

	assign SPISEL = 4'bzzzz;
	assign SPICLK = 1'bz;
	assign SPIDAT = 1'bz;
	assign ACNTR = 4'bzzzz;
	assign TP = 4'bzzzz;
	assign I2CCLK = 1'bz;
	assign I2CDAT = 1'bz;

    //--------------------------- The GTP Wrapper -----------------------------
    s6_gtpwizard_v1_11 #
    (
        .WRAPPER_SIM_GTPRESET_SPEEDUP   (0),      // Set this to 1 for simulation
        .WRAPPER_SIMULATION             (0)       // Set this to 1 for simulation
    )
    s6_gtpwizard_v1_11_i
    (
        //_____________________________________________________________________
        //TILE0  (X0_Y0)
        //---------------------- Loopback and Powerdown Ports ----------------------
        .TILE0_LOOPBACK0_IN             (3'b000),
        .TILE0_LOOPBACK1_IN             (3'b000),
        //------------------------------- PLL Ports --------------------------------
        .TILE0_CLK00_IN                 (tile0_gtp0_refclk_i),
        .TILE0_CLK01_IN                 (tile0_gtp0_refclk_i),
        .TILE0_GTPRESET0_IN             (),
        .TILE0_GTPRESET1_IN             (),
        .TILE0_PLLLKDET0_OUT            (tile0_plllkdet_o),
        .TILE0_PLLLKDET1_OUT            (),
        .TILE0_RESETDONE0_OUT           (),
        .TILE0_RESETDONE1_OUT           (),
        //--------------------- Receive Ports - 8b10b Decoder ----------------------
        .TILE0_RXCHARISCOMMA0_OUT       (GTP_CHARISC_A),
        .TILE0_RXCHARISCOMMA1_OUT       (),
        .TILE0_RXCHARISK0_OUT           (GTP_CHARISK_A),
        .TILE0_RXCHARISK1_OUT           (),
        .TILE0_RXDISPERR0_OUT           (GTP_DISPERR_A),
        .TILE0_RXDISPERR1_OUT           (),
        .TILE0_RXNOTINTABLE0_OUT        (GTP_NOTINTAB_A),
        .TILE0_RXNOTINTABLE1_OUT        (),
        //------------- Receive Ports - Comma Detection and Alignment --------------
        .TILE0_RXBYTEISALIGNED0_OUT     (GTP_ALIGNED_A),
        .TILE0_RXBYTEISALIGNED1_OUT     (),
        .TILE0_RXCOMMADET0_OUT          (),
        .TILE0_RXCOMMADET1_OUT          (),
        .TILE0_RXENMCOMMAALIGN0_IN      (1'b1),
        .TILE0_RXENMCOMMAALIGN1_IN      (1'b1),
        .TILE0_RXENPCOMMAALIGN0_IN      (1'b1),
        .TILE0_RXENPCOMMAALIGN1_IN      (1'b1),
        //----------------- Receive Ports - RX Data Path interface -----------------
        .TILE0_RXDATA0_OUT              (GTP_DAT_A),
        .TILE0_RXDATA1_OUT              (),
        .TILE0_RXUSRCLK0_IN             (CLK250),
        .TILE0_RXUSRCLK1_IN             (CLK250),
        .TILE0_RXUSRCLK20_IN            (CLK125),
        .TILE0_RXUSRCLK21_IN            (CLK125),
        //----- Receive Ports - RX Driver,OOB signalling,Coupling and Eq.,CDR ------
        .TILE0_RXN0_IN                  (RX0[1]),
        .TILE0_RXN1_IN                  (RX1[1]),
        .TILE0_RXP0_IN                  (RX0[0]),
        .TILE0_RXP1_IN                  (RX1[0]),
        //------------- Receive Ports - RX Loss-of-sync State Machine --------------
        .TILE0_RXLOSSOFSYNC0_OUT        (),
        .TILE0_RXLOSSOFSYNC1_OUT        (),
        //-------------------------- TX/RX Datapath Ports --------------------------
        .TILE0_GTPCLKOUT0_OUT           (GTPCLKOUT),
        .TILE0_GTPCLKOUT1_OUT           (),
        //----------------- Transmit Ports - 8b10b Encoder Control -----------------
        .TILE0_TXCHARISK0_IN            (GTP_CHARISK_A),
        .TILE0_TXCHARISK1_IN            (),
        //---------------- Transmit Ports - TX Data Path interface -----------------
        .TILE0_TXDATA0_IN               (GTP_DAT_A),
        .TILE0_TXDATA1_IN               (),
        .TILE0_TXUSRCLK0_IN             (CLK250),
        .TILE0_TXUSRCLK1_IN             (CLK250),
        .TILE0_TXUSRCLK20_IN            (CLK125),
        .TILE0_TXUSRCLK21_IN            (CLK125),
        //------------- Transmit Ports - TX Driver and OOB signalling --------------
        .TILE0_TXN0_OUT                 (TX0[1]),
        .TILE0_TXN1_OUT                 (TX1[1]),
        .TILE0_TXP0_OUT                 (TX0[0]),
        .TILE0_TXP1_OUT                 (TX1[0]),
    
        //_____________________________________________________________________
        //_____________________________________________________________________
        //TILE1  (X1_Y0)
 
        //---------------------- Loopback and Powerdown Ports ----------------------
        .TILE1_LOOPBACK0_IN             (),
        .TILE1_LOOPBACK1_IN             (),
        //------------------------------- PLL Ports --------------------------------
        .TILE1_CLK00_IN                 (tile0_gtp0_refclk_i),
        .TILE1_CLK01_IN                 (tile0_gtp0_refclk_i),
        .TILE1_GTPRESET0_IN             (),
        .TILE1_GTPRESET1_IN             (),
        .TILE1_PLLLKDET0_OUT            (),
        .TILE1_PLLLKDET1_OUT            (),
        .TILE1_RESETDONE0_OUT           (),
        .TILE1_RESETDONE1_OUT           (),
        //--------------------- Receive Ports - 8b10b Decoder ----------------------
        .TILE1_RXCHARISCOMMA0_OUT       (),
        .TILE1_RXCHARISCOMMA1_OUT       (),
        .TILE1_RXCHARISK0_OUT           (),
        .TILE1_RXCHARISK1_OUT           (),
        .TILE1_RXDISPERR0_OUT           (),
        .TILE1_RXDISPERR1_OUT           (),
        .TILE1_RXNOTINTABLE0_OUT        (),
        .TILE1_RXNOTINTABLE1_OUT        (),
        //------------- Receive Ports - Comma Detection and Alignment --------------
        .TILE1_RXBYTEISALIGNED0_OUT     (),
        .TILE1_RXBYTEISALIGNED1_OUT     (),
        .TILE1_RXCOMMADET0_OUT          (),
        .TILE1_RXCOMMADET1_OUT          (),
        .TILE1_RXENMCOMMAALIGN0_IN      (1'b1),
        .TILE1_RXENMCOMMAALIGN1_IN      (1'b1),
        .TILE1_RXENPCOMMAALIGN0_IN      (1'b1),
        .TILE1_RXENPCOMMAALIGN1_IN      (1'b1),
        //----------------- Receive Ports - RX Data Path interface -----------------
        .TILE1_RXDATA0_OUT              (),
        .TILE1_RXDATA1_OUT              (),
        .TILE1_RXUSRCLK0_IN             (CLK250),
        .TILE1_RXUSRCLK1_IN             (CLK250),
        .TILE1_RXUSRCLK20_IN            (CLK125),
        .TILE1_RXUSRCLK21_IN            (CLK125),
        //----- Receive Ports - RX Driver,OOB signalling,Coupling and Eq.,CDR ------
        .TILE1_RXN0_IN                  (RX2[1]),
        .TILE1_RXN1_IN                  (RX3[1]),
        .TILE1_RXP0_IN                  (RX2[0]),
        .TILE1_RXP1_IN                  (RX3[0]),
        //------------- Receive Ports - RX Loss-of-sync State Machine --------------
        .TILE1_RXLOSSOFSYNC0_OUT        (),
        .TILE1_RXLOSSOFSYNC1_OUT        (),
        //-------------------------- TX/RX Datapath Ports --------------------------
        .TILE1_GTPCLKOUT0_OUT           (),
        .TILE1_GTPCLKOUT1_OUT           (),
        //----------------- Transmit Ports - 8b10b Encoder Control -----------------
        .TILE1_TXCHARISK0_IN            (),
        .TILE1_TXCHARISK1_IN            (),
        //---------------- Transmit Ports - TX Data Path interface -----------------
        .TILE1_TXDATA0_IN               (),
        .TILE1_TXDATA1_IN               (),
        .TILE1_TXUSRCLK0_IN             (CLK250),
        .TILE1_TXUSRCLK1_IN             (CLK250),
        .TILE1_TXUSRCLK20_IN            (CLK125),
        .TILE1_TXUSRCLK21_IN            (CLK125),
        //------------- Transmit Ports - TX Driver and OOB signalling --------------
        .TILE1_TXN0_OUT                 (TX2[1]),
        .TILE1_TXN1_OUT                 (TX3[1]),
        .TILE1_TXP0_OUT                 (TX2[0]),
        .TILE1_TXP1_OUT                 (TX3[0])
    );
    
    IBUFDS tile0_gtp0_refclk_ibufds_i
    (
        .O                              (tile0_gtp0_refclk_i),
        .I                              (RCLK[0]),  // Connect to package pin A10
        .IB                             (RCLK[1])   // Connect to package pin B10
    );
   // BUFIO2: I/O Clock Buffer
   //         Spartan-6
   // Xilinx HDL Language Template, version 14.6

   BUFIO2 #(
      .DIVIDE(1),             // DIVCLK divider (1,3-8)
      .DIVIDE_BYPASS("TRUE"), // Bypass the divider circuitry (TRUE/FALSE)
      .I_INVERT("FALSE"),     // Invert clock (TRUE/FALSE)
      .USE_DOUBLER("FALSE")   // Use doubler circuitry (TRUE/FALSE)
   )
   BUFIO2_inst (
      .DIVCLK(CLKBUFIO),           // 1-bit output: Divided clock output
      .IOCLK(),           		// 1-bit output: I/O output clock
      .SERDESSTROBE(), 			// 1-bit output: Output SERDES strobe (connect to ISERDES2/OSERDES2)
      .I(GTPCLKOUT[0])        // 1-bit input: Clock input (connect to IBUFG)
   );

   BUFG BUFG_inst (
      .O(CLK125_i), // 1-bit output: Clock buffer output
      .I(CLKBUFIO)  // 1-bit input: Clock buffer input
   );

   PLL_BASE #(
      .BANDWIDTH("OPTIMIZED"),             // "HIGH", "LOW" or "OPTIMIZED" 
      .CLKFBOUT_MULT(4),                   // Multiply value for all CLKOUT clock outputs (1-64)
      .CLKFBOUT_PHASE(0.0),                // Phase offset in degrees of the clock feedback output (0.0-360.0).
      .CLKIN_PERIOD(8.0),                  // Input clock period in ns to ps resolution (i.e. 33.333 is 30
                                           // MHz).
      // CLKOUT0_DIVIDE - CLKOUT5_DIVIDE: Divide amount for CLKOUT# clock output (1-128)
      .CLKOUT0_DIVIDE(4),
      .CLKOUT1_DIVIDE(2),
      .CLKOUT2_DIVIDE(5),
      .CLKOUT3_DIVIDE(1),
      .CLKOUT4_DIVIDE(1),
      .CLKOUT5_DIVIDE(1),
      // CLKOUT0_DUTY_CYCLE - CLKOUT5_DUTY_CYCLE: Duty cycle for CLKOUT# clock output (0.01-0.99).
      .CLKOUT0_DUTY_CYCLE(0.5),
      .CLKOUT1_DUTY_CYCLE(0.5),
      .CLKOUT2_DUTY_CYCLE(0.5),
      .CLKOUT3_DUTY_CYCLE(0.5),
      .CLKOUT4_DUTY_CYCLE(0.5),
      .CLKOUT5_DUTY_CYCLE(0.5),
      // CLKOUT0_PHASE - CLKOUT5_PHASE: Output phase relationship for CLKOUT# clock output (-360.0-360.0).
      .CLKOUT0_PHASE(0.0),
      .CLKOUT1_PHASE(0.0),
      .CLKOUT2_PHASE(0.0),
      .CLKOUT3_PHASE(0.0),
      .CLKOUT4_PHASE(0.0),
      .CLKOUT5_PHASE(0.0),
      .CLK_FEEDBACK("CLKFBOUT"),           // Clock source to drive CLKFBIN ("CLKFBOUT" or "CLKOUT0")
      .COMPENSATION("SYSTEM_SYNCHRONOUS"), // "SYSTEM_SYNCHRONOUS", "SOURCE_SYNCHRONOUS", "EXTERNAL" 
      .DIVCLK_DIVIDE(1),                   // Division value for all output clocks (1-52)
      .REF_JITTER(0.1),                    // Reference Clock Jitter in UI (0.000-0.999).
      .RESET_ON_LOSS_OF_LOCK("FALSE")      // Must be set to FALSE
   )
   PLL_BASE_inst (
      .CLKFBOUT(CLKPLLFB), // 1-bit output: PLL_BASE feedback output
      // CLKOUT0 - CLKOUT5: 1-bit (each) output: Clock outputs
      .CLKOUT0(CLK125_o),
      .CLKOUT1(CLK250_o),
      .CLKOUT2(CLK100_o),
      .CLKOUT3(),
      .CLKOUT4(),
      .CLKOUT5(),
      .LOCKED(),     // 1-bit output: PLL_BASE lock status output
      .CLKFBIN(CLKPLLFB),   // 1-bit input: Feedback clock input
      .CLKIN(CLK125_i),       // 1-bit input: Clock input
      .RST(~tile0_plllkdet_o)            // 1-bit input: Reset input
   );

   BUFG BUFG_inst250 (
      .O(CLK250), // 1-bit output: Clock buffer output
      .I(CLK250_o)  // 1-bit input: Clock buffer input
   );
   BUFG BUFG_inst125 (
      .O(CLK125), // 1-bit output: Clock buffer output
      .I(CLK125_o)  // 1-bit input: Clock buffer input
   );
   BUFG BUFG_inst100 (
      .O(CLK), // 1-bit output: Clock buffer output
      .I(CLK100_o)  // 1-bit input: Clock buffer input
   );

	genvar i;
   generate
      for (i=0; i < 4; i=i+1) 
      begin: icxout
         assign ICX[i*4] = (i == CHN) ? GTP_DISPERR_A[0]: 1'bz;
         assign ICX[i*4+1] = (i == CHN) ? GTP_NOTINTAB_A[0]: 1'bz;
         assign ICX[i*4+2] = (i == CHN) ? GTP_ALIGNED_A: 1'bz;
         assign ICX[i*4+3] = (i == CHN) ? GTP_CHARISC_A[0]: 1'bz;
      end
   endgenerate

endmodule
