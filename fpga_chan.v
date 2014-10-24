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

	wire CLK125;
	wire [191:0] D_s;		// data from ADC
	reg [15:0] D2GTP;	// data to GTP
	wire [31:0] CSR;	
	reg txcomma = 1'b0;
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
		txcomma <= 1'b0;
		if (commacnt < 10) begin
			txcomma <= 1'b1;
			D2GTP <= {commacnt[7:0], 8'hBC};
		end
		commacnt <= commacnt + 1;
	end
	assign ACNTR = 4'bzzzz;
	
	wire I2CCLK_o;
	wire I2CCLK_en;
	wire I2CDAT_o;
	wire I2CDAT_en;

	always @ (posedge wb_clk) wb_rst <= ICX[5];
	
	gtprcv4 # (.WB_DIVIDE(5), .WB_MULTIPLY(5))
	UGTP (
		.rxpin	({RX3, RX2, RX1, RX0}),	// input data pins
		.txpin	({TX3, TX2, TX1, TX0}),	// output data pins
		.clkpin	(RCLK),						// input clock pins - tile0 package pins A10/B10
		.clkout	(CLK125),					// output 125 MHz clock
		.clkwb   (wb_clk),					// output clock for wishbone
		.data_o		(),						// output data 4x16bit
		.charisk_o	(), 						// output char is K-char signature
		.data_i     ({48'h00BC00BC00BC00BC, D2GTP}),
		.charisk_i  ({3'b111, txcomma}),
		.locked  ()
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
