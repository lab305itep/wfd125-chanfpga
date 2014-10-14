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
//  ICX usage:
//  2  - XSPI frame
//  3  - XSPI data
//  4  - XSPI clock
//  5  - WB reset
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
    inout [15:0] ICX,
	 // Configuration pins
    input INIT,
    input DOUT,
    input CCLK,
    input DIN,
	 // Test points
    output [5:1] TP
    );

	wire wb_clk;
	reg wb_rst;
`include "wb_intercon.vh"

	wire [15:0] GTP_DAT_A;
	wire [1:0] GTP_CHARISK_A;
	wire [1:0] GTP_CHARISC_A;
	wire [1:0] GTP_DISPERR_A;
	wire [1:0] GTP_NOTINTAB_A;
	wire 		  GTP_ALIGNED_A;
	wire [1:0] GTPCLKOUT;
	wire CLK125;
	wire [191:0] D_s;		// data from ADC
	reg [15:0] D2GTP;	// data to GTP
	wire [31:0] CSR;	
	reg [1:0] txcomma = 2'b00;
	reg [9:0] commacnt = 0;
	
	always @ (posedge CLK125) begin
		if (CSR[4]) begin
			D2GTP <= {10'h000, debug[7:2]};
		end else begin
			case (CSR[3:0]) 
			4'h0:	D2GTP <= {4'h0, D_s[11:0]};
			4'h1:	D2GTP <= {4'h1, D_s[23:12]};
			4'h2:	D2GTP <= {4'h2, D_s[35:24]};
			4'h3:	D2GTP <= {4'h3, D_s[47:36]};
			4'h4:	D2GTP <= {4'h4, D_s[59:48]};
			4'h5:	D2GTP <= {4'h5, D_s[71:60]};
			4'h6:	D2GTP <= {4'h6, D_s[83:72]};
			4'h7:	D2GTP <= {4'h7, D_s[95:84]};
			4'h8:	D2GTP <= {4'h8, D_s[107:96]};
			4'h9:	D2GTP <= {4'h9, D_s[119:108]};
			4'hA:	D2GTP <= {4'hA, D_s[131:120]};
			4'hB:	D2GTP <= {4'hB, D_s[143:132]};
			4'hC:	D2GTP <= {4'hC, D_s[155:144]};
			4'hD:	D2GTP <= {4'hD, D_s[167:156]};
			4'hE:	D2GTP <= {4'hE, D_s[179:168]};
			4'hF:	D2GTP <= {4'hF, D_s[191:180]};
			endcase
		end
		txcomma <= 2'b00;
		if (commacnt < 10) begin
			txcomma <= 2'b01;
			D2GTP <= {commacnt[7:0], 8'hBC};
		end
		commacnt <= commacnt + 1;
	end
	assign ACNTR = 4'bzzzz;
	assign wb_clk = CLK125;
	
	wire I2CCLK_o;
	wire I2CCLK_en;
	wire I2CDAT_o;
	wire I2CDAT_en;

	always @ (posedge wb_clk) wb_rst <= ICX[5];
	

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
        .TILE0_GTPRESET0_IN             (1'b0),
        .TILE0_GTPRESET1_IN             (1'b0),
        .TILE0_PLLLKDET0_OUT            (tile0_plllkdet_o),
        .TILE0_PLLLKDET1_OUT            (),
        .TILE0_RESETDONE0_OUT           (),
        .TILE0_RESETDONE1_OUT           (),
        //--------------------- Receive Ports - 8b10b Decoder ----------------------
        .TILE0_RXCHARISCOMMA0_OUT       (),
        .TILE0_RXCHARISCOMMA1_OUT       (),
        .TILE0_RXCHARISK0_OUT           (GTP_CHARISK_A),
        .TILE0_RXCHARISK1_OUT           (),
        .TILE0_RXDISPERR0_OUT           (),
        .TILE0_RXDISPERR1_OUT           (),
        .TILE0_RXNOTINTABLE0_OUT        (),
        .TILE0_RXNOTINTABLE1_OUT        (),
        //------------- Receive Ports - Comma Detection and Alignment --------------
        .TILE0_RXBYTEISALIGNED0_OUT     (),
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
        .TILE0_TXCHARISK0_IN            (txcomma),
        .TILE0_TXCHARISK1_IN            (),
        //---------------- Transmit Ports - TX Data Path interface -----------------
        .TILE0_TXDATA0_IN               (D2GTP),
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
      .CLKOUT2(),
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

	wire [3:0] empty_spi_cs;

xspi_master  #(
	.CLK_DIV (49),
	.CLK_POL (1'b0)
) adc_spi (
	 .wb_rst    (wb_rst),
    .wb_clk    (wb_clk),
    .wb_we     (wb_m2s_adc_spi_we),
    .wb_dat_i  (wb_m2s_adc_spi_dat[15:0]),
    .wb_dat_o  (wb_s2m_adc_spi_dat[15:0]),
    .wb_cyc		(wb_m2s_adc_spi_cyc),
    .wb_stb		(wb_m2s_adc_spi_stb),
    .wb_ack		(wb_s2m_adc_spi_ack),
    .spi_dat   (SPIDAT),
    .spi_clk   (SPICLK),
    .spi_cs    ({empty_spi_cs, SPISEL}),
    .wb_adr		(wb_m2s_adc_spi_adr[2])
);
	assign wb_s2m_adc_spi_err = 0;
	assign wb_s2m_adc_spi_rty = 0;	
	assign wb_s2m_adc_spi_dat[31:16] = 0;

	assign wb_s2m_i2c_clk_err = 0;
	assign wb_s2m_i2c_clk_rty = 0;

  i2c_master_slave i2c_clk (
		.wb_clk_i  (wb_clk), 
		.wb_rst_i  (wb_rst),		// active high 
		.arst_i    (1'b0), 		// active high
		.wb_adr_i  (wb_m2s_i2c_clk_adr[4:2]), 
		.wb_dat_i  (wb_m2s_i2c_clk_dat), 
		.wb_dat_o  (wb_s2m_i2c_clk_dat),
		.wb_we_i   (wb_m2s_i2c_clk_we),
		.wb_stb_i  (wb_m2s_i2c_clk_stb),
		.wb_cyc_i  (wb_m2s_i2c_clk_cyc), 
		.wb_ack_o  (wb_s2m_i2c_clk_ack), 
		.wb_inta_o (),
		.scl_pad_i (I2CCLK), 
		.scl_pad_o (I2CCLK_o), 
		.scl_padoen_o (I2CCLK_en), 	// active low ?
		.sda_pad_i (I2CDAT), 
		.sda_pad_o (I2CDAT_o), 
		.sda_padoen_o (I2CDAT_en)		// active low ?
	);

   assign I2CCLK = (!I2CCLK_en) ? (I2CCLK_o) : 1'bz;
   assign I2CDAT = (!I2CDAT_en) ? (I2CDAT_o) : 1'bz;

spi_wbmaster spi_master(
    .CLK 		(wb_clk),
    .SPICLK 	(ICX[4]),
    .SPIDAT		(ICX[3]),
    .SPIFR		(ICX[2]),
    .STADDR		(CHN),
    .wb_adr_o  (wb_m2s_spi_master_adr[14:2]),
    .wb_dat_o  (wb_m2s_spi_master_dat[15:0]),
    .wb_sel_o  (wb_m2s_spi_master_sel),
    .wb_we_o	(wb_m2s_spi_master_we),
    .wb_cyc_o  (wb_m2s_spi_master_cyc),
    .wb_stb_o  (wb_m2s_spi_master_stb),
    .wb_cti_o  (wb_m2s_spi_master_cti),
    .wb_bte_o  (wb_m2s_spi_master_bte),
    .wb_dat_i  (wb_s2m_spi_master_dat[15:0]),
    .wb_ack_i  (wb_s2m_spi_master_ack),
    .wb_err_i  (wb_s2m_spi_master_err),
    .wb_rty_i  (wb_s2m_spi_master_rty)
    );

	assign wb_m2s_spi_master_adr[1:0] = 2'b00;
	assign wb_m2s_spi_master_adr[31:15] = 17'h00000;
	assign wb_m2s_spi_master_dat[31:16] = 16'h0000;
	assign ICX[15:4] = 12'hZZZ;
	assign ICX[2:0] = 3'bZZZ;

  inoutreg reg_csr (
		.wb_clk    (wb_clk), 
		.wb_adr    (wb_m2s_reg_csr_adr[2]), 
		.wb_dat_i  (wb_m2s_reg_csr_dat), 
		.wb_dat_o  (wb_s2m_reg_csr_dat),
		.wb_we     (wb_m2s_reg_csr_we),
		.wb_stb    (wb_m2s_reg_csr_stb),
		.wb_cyc    (wb_m2s_reg_csr_cyc), 
		.wb_ack    (wb_s2m_reg_csr_ack), 
		.reg_i	  (CSR),
		.reg_o	  (CSR)
	);
	assign wb_s2m_reg_csr_err = 0;
	assign wb_s2m_reg_csr_rty = 0;
	
	wire [9:0] debug;

//	ADC receiver
	adc4rcv DINA_rcv (
    .CLK    (CLK125),	// global clock
    .CLKIN  (ACA),		// input clock from ADC (375 MHz)
    .DIN    (ADA),		// Input data from ADC
    .FR	   (AFA),		// Input frame from ADC 
    .DOUT	(D_s[47:0]),	// output data (CLK clocked)
	 .BSENABLE (!CSR[5]),
	 .debug  (debug)
    );
	adc4rcv DINB_rcv (
    .CLK    (CLK125),	// global clock
    .CLKIN  (ACB),		// input clock from ADC (375 MHz)
    .DIN    (ADB),		// Input data from ADC
    .FR	   (AFB),		// Input frame from ADC 
    .DOUT	(D_s[95:48]),// output data (CLK clocked)
	 .BSENABLE (!CSR[5]),
	 .debug  ()
    );
	adc4rcv DINC_rcv (
    .CLK    (CLK125),	// global clock
    .CLKIN  (ACC),		// input clock from ADC (375 MHz)
    .DIN    (ADC),		// Input data from ADC
    .FR	   (AFC),		// Input frame from ADC 
    .DOUT	(D_s[143:96]),	// output data (CLK clocked)
	 .BSENABLE (!CSR[5]),
	 .debug  ()
    );
	adc4rcv DIND_rcv (
    .CLK    (CLK125),	// global clock
    .CLKIN  (ACD),		// input clock from ADC (375 MHz)
    .DIN    (ADD),		// Input data from ADC
    .FR	   (AFD),		// Input frame from ADC 
    .DOUT	(D_s[191:144]),	// output data (CLK clocked)
	 .BSENABLE (!CSR[5]),
	 .debug  ()	 
    );
	 
	 assign TP = debug[4:0];
	 
endmodule
