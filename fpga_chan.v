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
//  	ICX usage:
//  2  - XSPI frame
//  3  - XSPI data
//  4  - XSPI clock
//  5  - WB reset
//		GTP channel usage
//		Output:
//	 0 - sending data blocks, commas as spacer, K-char (not comma) if total sum is above the trigger threshold. 
//      This character interrupts data blocks.
//	 3:1 - send summ of all channels, comma if the summ is too small.
//    Input :
//	 0 - trigger from the communication Xilinx
//  3:1 - summs from the other Xilinxes.
//		CSR bits:
//	3:0 - pattern for ADC receiver check
//	6:4 - counter max = 2**(16 + 2*CSR[6:4])
//	7   - check start - edge sensitive  (ready on read)
//	8   - Disable Bitslip
//	9   - reset BitSlip
//	10  - calibrate IODELAY2
//`
//		Array registers:
//	0 - summ mask
// 1 - trigger mask
// 2 - selftrigger mask
// 3 - summ send threshold (for the summ of 16 channels)
// 4 - trigger send threshold - master trigger zero suppression
// 5 - self trigger threshold
// 6 - 64-channels sum trigger threshold
// 7 - selftrigger prescale
// 8 - master trigger window begin
// 9 - window length
// 10 - self trigger window begin
//	11 - common pedestal
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

`include "fpga_chan.vh"
//		Wires and registers
	wire CLK125;
	wire [191:0] D_s;					// data from ADC
	wire [63:0]  gtp_data_i;		// data to GTP
	wire [3:0]   gtp_comma_i;
	wire [63:0]  gtp_data_o; 		// data ftom GTP
	wire [3:0]   gtp_comma_o;		
	wire [31:0]  CSR;
	wire [255:0]  par_array;
	wire [255:0]  d2arb;
	wire [191:0]  d2sum;
	wire [255:0]  adc_ped;
	wire [255:0]  adc_err;
	wire [15:0]  req2arb;
	wire [15:0]  ack4arb;
	reg  [15:0]  trigger;
	wire sum_trig;
	wire seq_ready;
	wire seq_reset;
	wire seq_enable;
	wire del_cal;
	wire del_rst;
	wire del_ce;
	wire [63:0] bs_cnt;

//		WB-bus
	wire wb_clk;
	reg wb_rst;
`include "wb_intercon.vh"
	always @ (posedge wb_clk) wb_rst <= ICX[5];

	assign ACNTR = 4'bzzzz;
	assign TP = 0;

//		GTP communication module
	
	gtprcv4 # (.WB_DIVIDE(5), .WB_MULTIPLY(5)) UGTP (
		.rxpin		({RX3, RX2, RX1, RX0}),	// input data pins
		.txpin		({TX3, TX2, TX1, TX0}),	// output data pins
		.clkpin		(RCLK),						// input clock pins - tile0 package pins A10/B10
		.clkout		(CLK125),					// output 125 MHz clock
		.clkwb   	(wb_clk),					// output clock for wishbone
		.data_o		(gtp_data_o),				// data received
		.charisk_o	(gtp_comma_o),				// 
		.data_i     (gtp_data_i),				//	data send
		.charisk_i  (gtp_comma_i),
		.locked  	()
    );

//		SPI to ADCs
	wire [3:0] empty_spi_cs;
	xspi_master  #(
		.CLK_DIV (49),
		.CLK_POL (1'b0)
	) adc_spi (
		.wb_rst    	(wb_rst),
		.wb_clk   	(wb_clk),
		.wb_we   	(wb_m2s_adc_spi_we),
		.wb_dat_i	(wb_m2s_adc_spi_dat[15:0]),
		.wb_dat_o	(wb_s2m_adc_spi_dat[15:0]),
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

//		I2C to clock chip
	wire I2CCLK_o;
	wire I2CCLK_en;
	wire I2CDAT_o;
	wire I2CDAT_en;
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

//		SPI from the master Xilinx - master on the WB-bus
	spi_wbmaster spi_master(
		.CLK 			(wb_clk),
		.SPICLK 		(ICX[4]),
		.SPIDAT		(ICX[3]),
		.SPIFR		(ICX[2]),
		.STADDR		(CHN),
		.wb_adr_o 	(wb_m2s_spi_master_adr[14:2]),
		.wb_dat_o  	(wb_m2s_spi_master_dat[15:0]),
		.wb_sel_o  	(wb_m2s_spi_master_sel),
		.wb_we_o		(wb_m2s_spi_master_we),
		.wb_cyc_o  	(wb_m2s_spi_master_cyc),
		.wb_stb_o   (wb_m2s_spi_master_stb),
		.wb_cti_o   (wb_m2s_spi_master_cti),
		.wb_bte_o   (wb_m2s_spi_master_bte),
		.wb_dat_i   (wb_s2m_spi_master_dat[15:0]),
		.wb_ack_i   (wb_s2m_spi_master_ack),
		.wb_err_i   (wb_s2m_spi_master_err),
		.wb_rty_i   (wb_s2m_spi_master_rty)
   );

	assign wb_m2s_spi_master_adr[1:0] = 2'b00;
	assign wb_m2s_spi_master_adr[31:15] = 17'h00000;
	assign wb_m2s_spi_master_dat[31:16] = 16'h0000;
	assign ICX[15:4] = 12'hZZZ;
	assign ICX[2:0] = 3'bZZZ;

//		CSR
	inoutreg reg_csr (
		.wb_clk    (wb_clk), 
		.wb_adr    (wb_m2s_reg_csr_adr[2]), 
		.wb_dat_i  (wb_m2s_reg_csr_dat), 
		.wb_dat_o  (wb_s2m_reg_csr_dat),
		.wb_we     (wb_m2s_reg_csr_we),
		.wb_stb    (wb_m2s_reg_csr_stb),
		.wb_cyc    (wb_m2s_reg_csr_cyc), 
		.wb_ack    (wb_s2m_reg_csr_ack), 
		.reg_i	  ({CSR[31:8], seq_ready, CSR[6:0]}),
		.reg_o	  (CSR)
	);
	assign wb_s2m_reg_csr_err = 0;
	assign wb_s2m_reg_csr_rty = 0;
	
//		register array
	parreg16 #(.ADRBITS(4)) reg_array (
		.wb_clk    (wb_clk), 
		.wb_adr    (wb_m2s_reg_array_adr[5:2]), 
		.wb_dat_i  (wb_m2s_reg_array_dat[15:0]), 
		.wb_dat_o  (wb_s2m_reg_array_dat[15:0]),
		.wb_we     (wb_m2s_reg_array_we),
		.wb_stb    (wb_m2s_reg_array_stb),
		.wb_cyc    (wb_m2s_reg_array_cyc), 
		.wb_ack    (wb_s2m_reg_array_ack), 
		.reg_o	  (par_array)
	);
	assign wb_s2m_reg_array_err = 0;
	assign wb_s2m_reg_array_rty = 0;
	assign wb_s2m_reg_array_dat[31:16] = 16'h0000;

//		input array for pedestals
	inpreg16 #(.ADRBITS(4)) ped_array (
		.wb_clk    (wb_clk), 
		.wb_adr    (wb_m2s_ped_array_adr[5:2]), 
		.wb_dat_i  (wb_m2s_ped_array_dat[15:0]), 
		.wb_dat_o  (wb_s2m_ped_array_dat[15:0]),
		.wb_we     (wb_m2s_ped_array_we),
		.wb_stb    (wb_m2s_ped_array_stb),
		.wb_cyc    (wb_m2s_ped_array_cyc), 
		.wb_ack    (wb_s2m_ped_array_ack), 
		.reg_i	  (adc_ped)
	);
	assign wb_s2m_ped_array_err = 0;
	assign wb_s2m_ped_array_rty = 0;
	assign wb_s2m_ped_array_dat[31:16] = 16'h0000;

//		input array for ADC error counters
	inpreg16 #(.ADRBITS(4)) err_array (
		.wb_clk    (wb_clk), 
		.wb_adr    (wb_m2s_err_array_adr[5:2]), 
		.wb_dat_i  (wb_m2s_err_array_dat[15:0]), 
		.wb_dat_o  (wb_s2m_err_array_dat[15:0]),
		.wb_we     (wb_m2s_err_array_we),
		.wb_stb    (wb_m2s_err_array_stb),
		.wb_cyc    (wb_m2s_err_array_cyc), 
		.wb_ack    (wb_s2m_err_array_ack), 
		.reg_i	  (adc_err)
	);
	assign wb_s2m_err_array_err = 0;
	assign wb_s2m_err_array_rty = 0;
	assign wb_s2m_err_array_dat[31:16] = 16'h0000;

//		input array for Bitslip counters
	inpreg16 #(.ADRBITS(2)) bs_array (
		.wb_clk    (wb_clk), 
		.wb_adr    (wb_m2s_bs_array_adr[3:2]), 
		.wb_dat_i  (wb_m2s_bs_array_dat[15:0]), 
		.wb_dat_o  (wb_s2m_bs_array_dat[15:0]),
		.wb_we     (wb_m2s_bs_array_we),
		.wb_stb    (wb_m2s_bs_array_stb),
		.wb_cyc    (wb_m2s_bs_array_cyc), 
		.wb_ack    (wb_s2m_bs_array_ack), 
		.reg_i	  (bs_cnt)
	);
	assign wb_s2m_bs_array_err = 0;
	assign wb_s2m_bs_array_rty = 0;
	assign wb_s2m_bs_array_dat[31:16] = 16'h0000;

//		control IODELAYS
	iodelaypulse UDEL(
		.clk(CLK125),
		.reset(CSR[9]),
		.pulse(CSR[8]),
		.cal(CSR[10]),
		.del_ce(del_ce),
		.del_rst(del_rst),
		.del_cal(del_cal)
	);
	
//	ADC receivers
	adc4rcv DINA_rcv (
		.CLK    	(CLK125),	// global clock
		.CLKIN  	(ACA),		// input clock from ADC (375 MHz)
		.DIN    	(ADA),		// Input data from ADC
		.FR	  	(AFA),		// Input frame from ADC 
		.DOUT		(D_s[47:0]),	// output data (CLK clocked)
		.BSENABLE (!CSR[8]),
		.del_ce	(del_ce),
		.del_rst	(CSR[9]),
		.del_cal	(del_cal),
		.bs_cnt  (bs_cnt[15:0]),
		.bs_reset(seq_reset),
		.bs_cntenb(seq_enable),
		.debug  	()
	);
	adc4rcv DINB_rcv (
		.CLK    	(CLK125),	// global clock
		.CLKIN  	(ACB),		// input clock from ADC (375 MHz)
		.DIN    	(ADB),		// Input data from ADC
		.FR	   (AFB),		// Input frame from ADC 
		.DOUT		(D_s[95:48]),// output data (CLK clocked)
		.BSENABLE (!CSR[8]),
		.del_ce	(del_ce),
		.del_rst	(CSR[9]),
		.del_cal	(del_cal),
		.bs_cnt  (bs_cnt[31:16]),
		.bs_reset(seq_reset),
		.bs_cntenb(seq_enable),
		.debug  	()
   );
	adc4rcv DINC_rcv (
		.CLK    	(CLK125),	// global clock
		.CLKIN  	(ACC),		// input clock from ADC (375 MHz)
		.DIN    	(ADC),		// Input data from ADC
		.FR	   (AFC),		// Input frame from ADC 
		.DOUT		(D_s[143:96]),	// output data (CLK clocked)
		.BSENABLE (!CSR[8]),
		.del_ce	(del_ce),
		.del_rst	(CSR[9]),
		.del_cal	(del_cal),
		.bs_cnt  (bs_cnt[47:32]),
		.bs_reset(seq_reset),
		.bs_cntenb(seq_enable),
		.debug  	()
   );
	adc4rcv DIND_rcv (
		.CLK    	(CLK125),	// global clock
		.CLKIN  	(ACD),		// input clock from ADC (375 MHz)
		.DIN    	(ADD),		// Input data from ADC
		.FR	   (AFD),		// Input frame from ADC 
		.DOUT		(D_s[191:144]),	// output data (CLK clocked)
		.BSENABLE (!CSR[8]),
		.del_ce	(del_ce),
		.del_rst	(CSR[9]),
		.del_cal	(del_cal),
		.bs_cnt  (bs_cnt[63:48]),
		.bs_reset(seq_reset),
		.bs_cntenb(seq_enable),
		.debug  	()	 
   );

//		channel processing
	genvar i;
	generate
		for (i=0; i<16; i = i + 1) begin: UPRC1
		prc1chan UCHAN (
			.clk(CLK125), 
			.data(D_s[12*i+11:12*i]), 
			.d2sum(d2sum[12*i+11:12*i]), 
			.ped(adc_ped[16*i+11:16*i]), 
			.zthr(par_array[PAR_ZTHR*16+15:PAR_ZTHR*16]), 
			.sthr(par_array[PAR_STTHR*16+15:PAR_STTHR*16]), 
			.cped(par_array[PAR_CPED*16+15:PAR_CPED*16]),
			.prescale(par_array[PAR_STPRC*16+15:PAR_STPRC*16]), 
			.winbeg(par_array[PAR_WINBEG*16+15:PAR_WINBEG*16]), 
			.swinbeg(par_array[PAR_SWINBEG*16+15:PAR_SWINBEG*16]), 
			.winlen(par_array[PAR_WINLEN*16+15:PAR_WINLEN*16]), 
			.trigger(trigger), 
			.dout(d2arb[16*i+15:16*i]), 
			.num((CHN << 4) + i), 
			.req(req2arb[i]), 
			.ack(ack4arb[i]), 
			.smask(par_array[PAR_SMASK*16+i]), 
			.tmask(par_array[PAR_TMASK*16+i]), 
			.stmask(par_array[PAR_STMASK*16+i])
		);
		assign adc_ped[16*i+15:16*i+12] = 0;
		adccheck UCHECK (
			.data(D_s[12*i+11:12*i]),	// ADC data received
			.clk(CLK125),					// ADC clock
			.cnt(adc_err[16*i+15:16*i]),	//	Error counter
			.count(seq_enable),			// Count errors enable
			.reset(seq_reset),			// Reset error counter
			.type(CSR[3:0])	
		);
		end
	endgenerate
	
//		get master trigger
	always @ (posedge CLK125) begin
		if (gtp_comma_o[0]) begin
			trigger <= 0;
		end else begin
			trigger <= gtp_data_o[15:0];
		end
	end

//		Pattern check sequencer
	checkseq USEQ(
		.clk(CLK125),		// system clock
		.start(CSR[7]),	// start
		.cntmax(CSR[6:4]),
		.reset(seq_reset),
		.enable(seq_enable),
		.ready(seq_ready)
	);

//		arbitter
	arbitter UARB(
		.clk(CLK125),
		.data(d2arb),
		.dout(gtp_data_i[15:0]),
		.kchar(gtp_comma_i[0]),
		.trigger(sum_trig),
		.req(req2arb),
		.ack(ack4arb)
   );

//		Summa and trigger
	wire [15:0] sum2gtp;
	wire comma2gtp;
	assign gtp_data_i[63:16] = {sum2gtp, sum2gtp, sum2gtp};
	assign gtp_comma_i[3:1] = {comma2gtp, comma2gtp, comma2gtp};
	sumcalc #(.XDELAY(7)) USCALC (
		.clk(CLK125),						// master clock
		.data(d2sum),						// input data
		.sumdata(gtp_data_o[63:16]),	// sums from 3 other xilinxes
		.xcomma(gtp_comma_o[3:1]),		// commas from other xilinxes
		.sumres(sum2gtp),					// 16-channel sum
		.sumcomma(comma2gtp),			// comma / data
		.s16thr(par_array[PAR_STHR*16+15:PAR_STHR*16]),		// 16-channel sum threshold
		.s64thr(par_array[PAR_MTTHR*16+15:PAR_MTTHR*16]),	// 64-channel sum threshold
		.trigout(sum_trig)				// 64-channel trigger
   );

endmodule
