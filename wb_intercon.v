// THIS FILE IS AUTOGENERATED BY wb_intercon_gen
// ANY MANUAL CHANGES WILL BE LOST
module wb_intercon
   (input         wb_clk_i,
    input         wb_rst_i,
    input  [31:0] wb_spi_master_adr_i,
    input  [31:0] wb_spi_master_dat_i,
    input   [3:0] wb_spi_master_sel_i,
    input         wb_spi_master_we_i,
    input         wb_spi_master_cyc_i,
    input         wb_spi_master_stb_i,
    input   [2:0] wb_spi_master_cti_i,
    input   [1:0] wb_spi_master_bte_i,
    output [31:0] wb_spi_master_dat_o,
    output        wb_spi_master_ack_o,
    output        wb_spi_master_err_o,
    output        wb_spi_master_rty_o,
    output [31:0] wb_adc_spi_adr_o,
    output [31:0] wb_adc_spi_dat_o,
    output  [3:0] wb_adc_spi_sel_o,
    output        wb_adc_spi_we_o,
    output        wb_adc_spi_cyc_o,
    output        wb_adc_spi_stb_o,
    output  [2:0] wb_adc_spi_cti_o,
    output  [1:0] wb_adc_spi_bte_o,
    input  [31:0] wb_adc_spi_dat_i,
    input         wb_adc_spi_ack_i,
    input         wb_adc_spi_err_i,
    input         wb_adc_spi_rty_i,
    output [31:0] wb_i2c_clk_adr_o,
    output  [7:0] wb_i2c_clk_dat_o,
    output  [3:0] wb_i2c_clk_sel_o,
    output        wb_i2c_clk_we_o,
    output        wb_i2c_clk_cyc_o,
    output        wb_i2c_clk_stb_o,
    output  [2:0] wb_i2c_clk_cti_o,
    output  [1:0] wb_i2c_clk_bte_o,
    input   [7:0] wb_i2c_clk_dat_i,
    input         wb_i2c_clk_ack_i,
    input         wb_i2c_clk_err_i,
    input         wb_i2c_clk_rty_i,
    output [31:0] wb_reg_csr_adr_o,
    output [31:0] wb_reg_csr_dat_o,
    output  [3:0] wb_reg_csr_sel_o,
    output        wb_reg_csr_we_o,
    output        wb_reg_csr_cyc_o,
    output        wb_reg_csr_stb_o,
    output  [2:0] wb_reg_csr_cti_o,
    output  [1:0] wb_reg_csr_bte_o,
    input  [31:0] wb_reg_csr_dat_i,
    input         wb_reg_csr_ack_i,
    input         wb_reg_csr_err_i,
    input         wb_reg_csr_rty_i);

wire [31:0] wb_m2s_resize_i2c_clk_adr;
wire [31:0] wb_m2s_resize_i2c_clk_dat;
wire  [3:0] wb_m2s_resize_i2c_clk_sel;
wire        wb_m2s_resize_i2c_clk_we;
wire        wb_m2s_resize_i2c_clk_cyc;
wire        wb_m2s_resize_i2c_clk_stb;
wire  [2:0] wb_m2s_resize_i2c_clk_cti;
wire  [1:0] wb_m2s_resize_i2c_clk_bte;
wire [31:0] wb_s2m_resize_i2c_clk_dat;
wire        wb_s2m_resize_i2c_clk_ack;
wire        wb_s2m_resize_i2c_clk_err;
wire        wb_s2m_resize_i2c_clk_rty;

wb_mux
  #(.num_slaves (3),
    .MATCH_ADDR ({32'h00000040, 32'h00000000, 32'h00000008}),
    .MATCH_MASK ({32'hffffffe0, 32'hfffffff8, 32'hfffffff8}))
 wb_mux_spi_master
   (.wb_clk_i  (wb_clk_i),
    .wb_rst_i  (wb_rst_i),
    .wbm_adr_i (wb_spi_master_adr_i),
    .wbm_dat_i (wb_spi_master_dat_i),
    .wbm_sel_i (wb_spi_master_sel_i),
    .wbm_we_i  (wb_spi_master_we_i),
    .wbm_cyc_i (wb_spi_master_cyc_i),
    .wbm_stb_i (wb_spi_master_stb_i),
    .wbm_cti_i (wb_spi_master_cti_i),
    .wbm_bte_i (wb_spi_master_bte_i),
    .wbm_dat_o (wb_spi_master_dat_o),
    .wbm_ack_o (wb_spi_master_ack_o),
    .wbm_err_o (wb_spi_master_err_o),
    .wbm_rty_o (wb_spi_master_rty_o),
    .wbs_adr_o ({wb_m2s_resize_i2c_clk_adr, wb_adc_spi_adr_o, wb_reg_csr_adr_o}),
    .wbs_dat_o ({wb_m2s_resize_i2c_clk_dat, wb_adc_spi_dat_o, wb_reg_csr_dat_o}),
    .wbs_sel_o ({wb_m2s_resize_i2c_clk_sel, wb_adc_spi_sel_o, wb_reg_csr_sel_o}),
    .wbs_we_o  ({wb_m2s_resize_i2c_clk_we, wb_adc_spi_we_o, wb_reg_csr_we_o}),
    .wbs_cyc_o ({wb_m2s_resize_i2c_clk_cyc, wb_adc_spi_cyc_o, wb_reg_csr_cyc_o}),
    .wbs_stb_o ({wb_m2s_resize_i2c_clk_stb, wb_adc_spi_stb_o, wb_reg_csr_stb_o}),
    .wbs_cti_o ({wb_m2s_resize_i2c_clk_cti, wb_adc_spi_cti_o, wb_reg_csr_cti_o}),
    .wbs_bte_o ({wb_m2s_resize_i2c_clk_bte, wb_adc_spi_bte_o, wb_reg_csr_bte_o}),
    .wbs_dat_i ({wb_s2m_resize_i2c_clk_dat, wb_adc_spi_dat_i, wb_reg_csr_dat_i}),
    .wbs_ack_i ({wb_s2m_resize_i2c_clk_ack, wb_adc_spi_ack_i, wb_reg_csr_ack_i}),
    .wbs_err_i ({wb_s2m_resize_i2c_clk_err, wb_adc_spi_err_i, wb_reg_csr_err_i}),
    .wbs_rty_i ({wb_s2m_resize_i2c_clk_rty, wb_adc_spi_rty_i, wb_reg_csr_rty_i}));

wb_data_resize
  #(.aw  (32),
    .mdw (32),
    .sdw (8))
 wb_data_resize_i2c_clk
   (.wbm_adr_i (wb_m2s_resize_i2c_clk_adr),
    .wbm_dat_i (wb_m2s_resize_i2c_clk_dat),
    .wbm_sel_i (wb_m2s_resize_i2c_clk_sel),
    .wbm_we_i  (wb_m2s_resize_i2c_clk_we),
    .wbm_cyc_i (wb_m2s_resize_i2c_clk_cyc),
    .wbm_stb_i (wb_m2s_resize_i2c_clk_stb),
    .wbm_cti_i (wb_m2s_resize_i2c_clk_cti),
    .wbm_bte_i (wb_m2s_resize_i2c_clk_bte),
    .wbm_dat_o (wb_s2m_resize_i2c_clk_dat),
    .wbm_ack_o (wb_s2m_resize_i2c_clk_ack),
    .wbm_err_o (wb_s2m_resize_i2c_clk_err),
    .wbm_rty_o (wb_s2m_resize_i2c_clk_rty),
    .wbs_adr_o (wb_i2c_clk_adr_o),
    .wbs_dat_o (wb_i2c_clk_dat_o),
    .wbs_we_o  (wb_i2c_clk_we_o),
    .wbs_cyc_o (wb_i2c_clk_cyc_o),
    .wbs_stb_o (wb_i2c_clk_stb_o),
    .wbs_cti_o (wb_i2c_clk_cti_o),
    .wbs_bte_o (wb_i2c_clk_bte_o),
    .wbs_dat_i (wb_i2c_clk_dat_i),
    .wbs_ack_i (wb_i2c_clk_ack_i),
    .wbs_err_i (wb_i2c_clk_err_i),
    .wbs_rty_i (wb_i2c_clk_rty_i));

endmodule
