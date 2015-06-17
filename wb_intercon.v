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
    output        wb_spi_master_stall_o,
    output        wb_spi_master_rty_o,
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
    input         wb_reg_csr_stall_i,
    input         wb_reg_csr_rty_i,
    output [31:0] wb_reg_ver_adr_o,
    output [31:0] wb_reg_ver_dat_o,
    output  [3:0] wb_reg_ver_sel_o,
    output        wb_reg_ver_we_o,
    output        wb_reg_ver_cyc_o,
    output        wb_reg_ver_stb_o,
    output  [2:0] wb_reg_ver_cti_o,
    output  [1:0] wb_reg_ver_bte_o,
    input  [31:0] wb_reg_ver_dat_i,
    input         wb_reg_ver_ack_i,
    input         wb_reg_ver_err_i,
    input         wb_reg_ver_stall_i,
    input         wb_reg_ver_rty_i,
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
    input         wb_adc_spi_stall_i,
    input         wb_adc_spi_rty_i,
    output [31:0] wb_i2c_clk_adr_o,
    output [31:0] wb_i2c_clk_dat_o,
    output  [3:0] wb_i2c_clk_sel_o,
    output        wb_i2c_clk_we_o,
    output        wb_i2c_clk_cyc_o,
    output        wb_i2c_clk_stb_o,
    output  [2:0] wb_i2c_clk_cti_o,
    output  [1:0] wb_i2c_clk_bte_o,
    input  [31:0] wb_i2c_clk_dat_i,
    input         wb_i2c_clk_ack_i,
    input         wb_i2c_clk_err_i,
    input         wb_i2c_clk_stall_i,
    input         wb_i2c_clk_rty_i,
    output [31:0] wb_coef_array_adr_o,
    output [31:0] wb_coef_array_dat_o,
    output  [3:0] wb_coef_array_sel_o,
    output        wb_coef_array_we_o,
    output        wb_coef_array_cyc_o,
    output        wb_coef_array_stb_o,
    output  [2:0] wb_coef_array_cti_o,
    output  [1:0] wb_coef_array_bte_o,
    input  [31:0] wb_coef_array_dat_i,
    input         wb_coef_array_ack_i,
    input         wb_coef_array_err_i,
    input         wb_coef_array_stall_i,
    input         wb_coef_array_rty_i,
    output [31:0] wb_reg_array_adr_o,
    output [31:0] wb_reg_array_dat_o,
    output  [3:0] wb_reg_array_sel_o,
    output        wb_reg_array_we_o,
    output        wb_reg_array_cyc_o,
    output        wb_reg_array_stb_o,
    output  [2:0] wb_reg_array_cti_o,
    output  [1:0] wb_reg_array_bte_o,
    input  [31:0] wb_reg_array_dat_i,
    input         wb_reg_array_ack_i,
    input         wb_reg_array_err_i,
    input         wb_reg_array_stall_i,
    input         wb_reg_array_rty_i,
    output [31:0] wb_ped_array_adr_o,
    output [31:0] wb_ped_array_dat_o,
    output  [3:0] wb_ped_array_sel_o,
    output        wb_ped_array_we_o,
    output        wb_ped_array_cyc_o,
    output        wb_ped_array_stb_o,
    output  [2:0] wb_ped_array_cti_o,
    output  [1:0] wb_ped_array_bte_o,
    input  [31:0] wb_ped_array_dat_i,
    input         wb_ped_array_ack_i,
    input         wb_ped_array_err_i,
    input         wb_ped_array_stall_i,
    input         wb_ped_array_rty_i,
    output [31:0] wb_adc_rcv_adr_o,
    output [31:0] wb_adc_rcv_dat_o,
    output  [3:0] wb_adc_rcv_sel_o,
    output        wb_adc_rcv_we_o,
    output        wb_adc_rcv_cyc_o,
    output        wb_adc_rcv_stb_o,
    output  [2:0] wb_adc_rcv_cti_o,
    output  [1:0] wb_adc_rcv_bte_o,
    input  [31:0] wb_adc_rcv_dat_i,
    input         wb_adc_rcv_ack_i,
    input         wb_adc_rcv_err_i,
    input         wb_adc_rcv_stall_i,
    input         wb_adc_rcv_rty_i);

wb_mux
  #(.num_slaves (7),
    .MATCH_ADDR ({32'h00000000, 32'h00000008, 32'h00000010, 32'h00000020, 32'h00000080, 32'h000000c0, 32'h00000100}),
    .MATCH_MASK ({32'hfffffff8, 32'hfffffff8, 32'hfffffff8, 32'hffffffe0, 32'hffffffc0, 32'hffffffc0, 32'hffffff00}))
 wb_mux_spi_master
   (.wb_clk_i    (wb_clk_i),
    .wb_rst_i    (wb_rst_i),
    .wbm_adr_i   (wb_spi_master_adr_i),
    .wbm_dat_i   (wb_spi_master_dat_i),
    .wbm_sel_i   (wb_spi_master_sel_i),
    .wbm_we_i    (wb_spi_master_we_i),
    .wbm_cyc_i   (wb_spi_master_cyc_i),
    .wbm_stb_i   (wb_spi_master_stb_i),
    .wbm_cti_i   (wb_spi_master_cti_i),
    .wbm_bte_i   (wb_spi_master_bte_i),
    .wbm_dat_o   (wb_spi_master_dat_o),
    .wbm_ack_o   (wb_spi_master_ack_o),
    .wbm_err_o   (wb_spi_master_err_o),
    .wbm_stall_o (wb_spi_master_stall_o),
    .wbm_rty_o   (wb_spi_master_rty_o),
    .wbs_adr_o   ({wb_reg_csr_adr_o, wb_reg_ver_adr_o, wb_adc_spi_adr_o, wb_i2c_clk_adr_o, wb_reg_array_adr_o, wb_ped_array_adr_o, wb_adc_rcv_adr_o}),
    .wbs_dat_o   ({wb_reg_csr_dat_o, wb_reg_ver_dat_o, wb_adc_spi_dat_o, wb_i2c_clk_dat_o, wb_reg_array_dat_o, wb_ped_array_dat_o, wb_adc_rcv_dat_o}),
    .wbs_sel_o   ({wb_reg_csr_sel_o, wb_reg_ver_sel_o, wb_adc_spi_sel_o, wb_i2c_clk_sel_o, wb_reg_array_sel_o, wb_ped_array_sel_o, wb_adc_rcv_sel_o}),
    .wbs_we_o    ({wb_reg_csr_we_o, wb_reg_ver_we_o, wb_adc_spi_we_o, wb_i2c_clk_we_o, wb_reg_array_we_o, wb_ped_array_we_o, wb_adc_rcv_we_o}),
    .wbs_cyc_o   ({wb_reg_csr_cyc_o, wb_reg_ver_cyc_o, wb_adc_spi_cyc_o, wb_i2c_clk_cyc_o, wb_reg_array_cyc_o, wb_ped_array_cyc_o, wb_adc_rcv_cyc_o}),
    .wbs_stb_o   ({wb_reg_csr_stb_o, wb_reg_ver_stb_o, wb_adc_spi_stb_o, wb_i2c_clk_stb_o, wb_reg_array_stb_o, wb_ped_array_stb_o, wb_adc_rcv_stb_o}),
    .wbs_cti_o   ({wb_reg_csr_cti_o, wb_reg_ver_cti_o, wb_adc_spi_cti_o, wb_i2c_clk_cti_o, wb_reg_array_cti_o, wb_ped_array_cti_o, wb_adc_rcv_cti_o}),
    .wbs_bte_o   ({wb_reg_csr_bte_o, wb_reg_ver_bte_o, wb_adc_spi_bte_o, wb_i2c_clk_bte_o, wb_reg_array_bte_o, wb_ped_array_bte_o, wb_adc_rcv_bte_o}),
    .wbs_dat_i   ({wb_reg_csr_dat_i, wb_reg_ver_dat_i, wb_adc_spi_dat_i, wb_i2c_clk_dat_i, wb_reg_array_dat_i, wb_ped_array_dat_i, wb_adc_rcv_dat_i}),
    .wbs_ack_i   ({wb_reg_csr_ack_i, wb_reg_ver_ack_i, wb_adc_spi_ack_i, wb_i2c_clk_ack_i, wb_reg_array_ack_i, wb_ped_array_ack_i, wb_adc_rcv_ack_i}),
    .wbs_err_i   ({wb_reg_csr_err_i, wb_reg_ver_err_i, wb_adc_spi_err_i, wb_i2c_clk_err_i, wb_reg_array_err_i, wb_ped_array_err_i, wb_adc_rcv_err_i}),
    .wbs_stall_i ({wb_reg_csr_stall_i, wb_reg_ver_stall_i, wb_adc_spi_stall_i, wb_i2c_clk_stall_i, wb_reg_array_stall_i, wb_ped_array_stall_i, wb_adc_rcv_stall_i}),
    .wbs_rty_i   ({wb_reg_csr_rty_i, wb_reg_ver_rty_i, wb_adc_spi_rty_i, wb_i2c_clk_rty_i, wb_reg_array_rty_i, wb_ped_array_rty_i, wb_adc_rcv_rty_i}));

endmodule
