module rct_soc_top (
    // inout vdda1,	// User area 1 3.3V supply
    // inout vdda2,	// User area 2 3.3V supply
    // inout vssa1,	// User area 1 analog ground
    // inout vssa2,	// User area 2 analog ground
    inout vccd1,	// User area 1 1.8V supply
    // inout vccd2,	// User area 2 1.8v supply
    inout vssd1,	// User area 1 digital ground
    // inout vssd2,	// User area 2 digital ground
	input wire wb_clk_i,
	input wire user_clock2,
	input wire wb_rst_i,
	input wire wbs_cyc_i,
	input wire wbs_stb_i,
	input wire [31:0] wbs_adr_i,
	input wire wbs_we_i,
	input wire [31:0] wbs_dat_i,
	input wire [3:0] wbs_sel_i,
	output wire [31:0] wbs_dat_o,
	output wire wbs_ack_o,
	inout [28:0] analog_io,
	input wire [127:0] la_data_in,
	output wire [127:0] la_data_out,
	input wire [127:0] la_oenb,
	input wire [37:0] io_in,
	output wire [37:0] io_out,
	output wire [37:0] io_oeb,
	output wire [2:0] user_irq
);

	parameter WB_WIDTH = 32;
	localparam rct_cfg_RCT_WB_ADR_W = 32;
	localparam rct_cfg_RCT_WB_DAT_W = 32;
	localparam rct_cfg_RCT_WB_SEL_W = 4;
	wire jtag_tck;
	wire jtag_tms;
	wire jtag_tdi;
	wire jtag_tdo;
	wire jtag_tdo_oe;
	wire jtag_trst_n;
	wire test_clk;
	wire test_din;
	wire test_dout;
	wire test_doen;
	wire test_intr;
	wire [31:0] gpio_out;
	wire [31:0] gpio_oen;
	wire [31:0] gpio_in;
	wire gpio_intr;
	wire ti_clk_o;
	wire ti_clk_oen;
	wire [31:0] ti_dat_o;
	wire [31:0] ti_dat_oen;
	wire [31:0] ti_dat_i;
	wire [2:0] boot_mode;
	wire [1:0] ext_int;
	wire [1:0] cfg_ti_mode;
	wire [WB_WIDTH - 1:0] cfg_gpio_afr0;
	wire [5:0] cfg_gpio_afr1;
	wire [31:0] cfg_boot_pc;
	wire [31:0] cfg_rst_pc;
	wire [8:0] cfg_ti_clk_div;
	wire [7:0] cfg_rst_ctrl;
	wire wb_riscv_dcache_cyc_i;
	wire wb_riscv_dcache_stb_i;
	wire [WB_WIDTH - 1:0] wb_riscv_dcache_adr_i;
	wire wb_riscv_dcache_we_i;
	wire [WB_WIDTH - 1:0] wb_riscv_dcache_dat_i;
	wire [3:0] wb_riscv_dcache_sel_i;
	wire [WB_WIDTH - 1:0] wb_riscv_dcache_dat_o;
	wire wb_riscv_dcache_ack_o;
	wire wb_riscv_dcache_err_o;
	wire wb_riscv_icache_cyc_i;
	wire wb_riscv_icache_stb_i;
	wire [WB_WIDTH - 1:0] wb_riscv_icache_adr_i;
	wire wb_riscv_icache_we_i;
	wire [WB_WIDTH - 1:0] wb_riscv_icache_dat_i;
	wire [3:0] wb_riscv_icache_sel_i;
	wire [WB_WIDTH - 1:0] wb_riscv_icache_dat_o;
	wire wb_riscv_icache_ack_o;
	wire wb_riscv_icache_err_o;
	wire wb_riscv_ptw_cyc_i;
	wire wb_riscv_ptw_stb_i;
	wire [WB_WIDTH - 1:0] wb_riscv_ptw_adr_i;
	wire wb_riscv_ptw_we_i;
	wire [WB_WIDTH - 1:0] wb_riscv_ptw_dat_i;
	wire [3:0] wb_riscv_ptw_sel_i;
	wire [WB_WIDTH - 1:0] wb_riscv_ptw_dat_o;
	wire wb_riscv_ptw_ack_o;
	wire wb_riscv_ptw_err_o;
	wire wb_riscv_slv_cyc_o;
	wire wb_riscv_slv_stb_o;
	wire [WB_WIDTH - 1:0] wb_riscv_slv_adr_o;
	wire wb_riscv_slv_we_o;
	wire [WB_WIDTH - 1:0] wb_riscv_slv_dat_o;
	wire [3:0] wb_riscv_slv_sel_o;
	wire [WB_WIDTH - 1:0] wb_riscv_slv_dat_i;
	wire wb_riscv_slv_ack_i;
	wire wb_riscv_slv_err_i;
	wire wb_ti_ma_stb_i;
	wire [31:0] wb_ti_ma_adr_i;
	wire wb_ti_ma_we_i;
	wire [31:0] wb_ti_ma_dat_i;
	wire [3:0] wb_ti_ma_sel_i;
	wire wb_ti_ma_cyc_i;
	wire [31:0] wb_ti_ma_dat_o;
	wire wb_ti_ma_ack_o;
	wire wb_ti_ma_err_o;
	wire wb_int_cyc_i;
	wire wb_int_stb_i;
	wire [WB_WIDTH - 1:0] wb_int_adr_i;
	wire wb_int_we_i;
	wire [WB_WIDTH - 1:0] wb_int_dat_i;
	wire [3:0] wb_int_sel_i;
	wire [WB_WIDTH - 1:0] wb_int_dat_o;
	wire wb_int_ack_o;
	wire wb_int_err_o;
	wire wb_rom_stb_o;
	wire [WB_WIDTH - 1:0] wb_rom_adr_o;
	wire wb_rom_we_o;
	wire [WB_WIDTH - 1:0] wb_rom_dat_o;
	wire [3:0] wb_rom_sel_o;
	wire [9:0] wb_rom_bl_o;
	wire wb_rom_bry_o;
	wire wb_rom_cyc_o;
	wire [WB_WIDTH - 1:0] wb_rom_dat_i;
	wire wb_rom_ack_i;
	wire wb_rom_lack_i;
	wire wb_rom_err_i;
	wire wb_gpio_stb_o;
	wire [7:0] wb_gpio_adr_o;
	wire wb_gpio_we_o;
	wire [WB_WIDTH - 1:0] wb_gpio_dat_o;
	wire [3:0] wb_gpio_sel_o;
	wire wb_gpio_cyc_o;
	wire [WB_WIDTH - 1:0] wb_gpio_dat_i;
	wire wb_gpio_ack_i;
	wire wb_gpio_err_i;
	wire wb_uart_stb_o;
	wire [8:0] wb_uart_adr_o;
	wire wb_uart_we_o;
	wire [31:0] wb_uart_dat_o;
	wire [3:0] wb_uart_sel_o;
	wire wb_uart_cyc_o;
	wire [31:0] wb_uart_dat_i;
	wire wb_uart_ack_i;
	wire wb_uart_err_i;
	wire wb_l2_stb_o;
	wire [31:0] wb_l2_adr_o;
	wire wb_l2_we_o;
	wire [31:0] wb_l2_dat_o;
	wire [3:0] wb_l2_sel_o;
	wire wb_l2_cyc_o;
	wire [31:0] wb_l2_dat_i;
	wire wb_l2_ack_i;
	wire wb_l2_err_i;
	wire wb_ti_stb_o;
	wire [31:0] wb_ti_adr_o;
	wire wb_ti_we_o;
	wire [31:0] wb_ti_dat_o;
	wire [3:0] wb_ti_sel_o;
	wire wb_ti_cyc_o;
	wire [31:0] wb_ti_dat_i;
	wire wb_ti_ack_i;
	wire wb_ti_err_i;
	wire wb_rct_core_stb_o;
	wire [31:0] wb_rct_core_adr_o;
	wire wb_rct_core_we_o;
	wire [31:0] wb_rct_core_dat_o;
	wire [3:0] wb_rct_core_sel_o;
	wire wb_rct_core_cyc_o;
	wire [31:0] wb_rct_core_dat_i;
	wire wb_rct_core_ack_i;
	wire wb_rct_core_err_i;
	wire rom_mem_if_req_valid;
	wire rom_mem_if_req_ready;
	localparam rct_cfg_RCT_MEM_ADDR_W = 32;
	localparam rct_cfg_RCT_MEM_DATA_W = 32;
	localparam rct_cfg_RCT_MEM_MASK_W = 4;
	localparam rct_cfg_CPUNOC_TID_RID_SIZE = 4;
	localparam rct_cfg_CPUNOC_TID_SRCID_SIZE = 4;
	localparam rct_cfg_CPUNOC_TID_TID_SIZE = 8;
	wire [86:0] rom_mem_if_req;
	wire rom_mem_if_resp_valid;
	wire rom_mem_if_resp_ready;
	wire [50:0] rom_mem_if_resp;
	wire l2_mem_if_req_valid;
	wire l2_mem_if_req_ready;
	wire [86:0] l2_mem_if_req;
	wire l2_mem_if_resp_valid;
	wire l2_mem_if_resp_ready;
	wire [50:0] l2_mem_if_resp;
	wire ndm_rstn;
	wire dm_rstn;
	wire core_rstn;
	wire ndm_rst_req;
	wire rom_rstn;
	wire l2_sram_rstn;
	wire rom_arstn;
	wire [1:0] uart_arstn;
	wire gpio_arstn;
	wire ti_ma_arstn;
	wire ti_arstn;
	wire l2_sram_arstn;
	wire wb_int_rst_n;
	wire [31:0] cfg_clk_ctrl1;
	wire [3:0] cfg_cska_wi;
	wire [3:0] cfg_cska_wh;
	wire [3:0] cfg_cska_riscv;
	wire [3:0] cfg_cska_uart;
	wire [3:0] cfg_cska_rom;
	wire [3:0] cfg_cska_gpio;
	wire [3:0] cfg_cska_ti;
	wire [3:0] cfg_cska_l2;
	wire wb_wh_clk;
	wire wb_wi_clk;
	wire wb_riscv_clk;
	wire wb_uart_clk;
	wire wb_rom_clk;
	wire wb_gpio_clk;
	wire wb_ti_clk;
	wire wb_l2_clk;
	wire dbg_clk_mon;
	wire [1:0] uart_txd;
	wire [1:0] uart_rxd;
	wire uartm_rxd;
	wire uartm_txd;
	wb_host wb_host_u(
		.wbd_int_rst_n(wb_int_rst_n),
		.wbm_rst_i(wb_rst_i),
		.wbm_clk_i(wb_clk_i),
		.wbm_cyc_i(wbs_cyc_i),
		.wbm_stb_i(wbs_stb_i),
		.wbm_adr_i(wbs_adr_i),
		.wbm_we_i(wbs_we_i),
		.wbm_dat_i(wbs_dat_i),
		.wbm_sel_i(wbs_sel_i),
		.wbm_dat_o(wbs_dat_o),
		.wbm_ack_o(wbs_ack_o),
		.wbm_err_o(),
		.wbs_clk_i(wb_clk_i),
		.wbs_cyc_o(wb_int_cyc_i),
		.wbs_stb_o(wb_int_stb_i),
		.wbs_adr_o(wb_int_adr_i),
		.wbs_we_o(wb_int_we_i),
		.wbs_dat_o(wb_int_dat_i),
		.wbs_sel_o(wb_int_sel_i),
		.wbs_dat_i(wb_int_dat_o),
		.wbs_ack_i(wb_int_ack_o),
		.wbs_err_i(wb_int_err_o),
		.cfg_clk_ctrl1(cfg_clk_ctrl1),
		.la_data_in(la_data_in[17:0]),
		.uartm_rxd(uartm_rxd),
		.uartm_txd(uartm_txd),
		.dbg_clk_mon(dbg_clk_mon)
	);
	rct_reset rct_reset_u(
		.clk_i(wb_clk_i),
		.rstn_i(wb_int_rst_n),
		.rst_ctrl(cfg_rst_ctrl),
		.ndm_rst_req(ndm_rst_req),
		.ndm_rstn(ndm_rstn),
		.dm_rstn(dm_rstn),
		.core_rstn(core_rstn),
		.rom_arstn(rom_arstn),
		.uart_arstn(uart_arstn),
		.gpio_arstn(gpio_arstn),
		.ti_ma_arstn(ti_ma_arstn),
		.ti_arstn(ti_arstn),
		.l2_sram_arstn(l2_sram_arstn)
	);
	rct_top_wb rct_top_wb_u(
		.clk(wb_clk_i),
		.ndm_rstn(ndm_rstn),
		.dm_rstn(dm_rstn),
		.core_rstn(core_rstn),
		.ndm_rst_req(ndm_rst_req),
		.cfg_boot_pc({{32 {1'b0}}, cfg_rst_pc}),
		.irq(ext_int),
		.tck(jtag_tck),
		.tms(jtag_tms),
		.trst_n(jtag_trst_n),
		.tdi(jtag_tdi),
		.tdo(jtag_tdo),
		.tdo_oe(jtag_tdo_oe),
		.wb_icache_cyc_o(wb_riscv_icache_cyc_i),
		.wb_icache_stb_o(wb_riscv_icache_stb_i),
		.wb_icache_adr_o(wb_riscv_icache_adr_i),
		.wb_icache_we_o(wb_riscv_icache_we_i),
		.wb_icache_dat_o(wb_riscv_icache_dat_i),
		.wb_icache_sel_o(wb_riscv_icache_sel_i),
		.wb_icache_dat_i(wb_riscv_icache_dat_o),
		.wb_icache_ack_i(wb_riscv_icache_ack_o),
		.wb_icache_err_i(wb_riscv_icache_err_o),
		.wb_dcache_cyc_o(wb_riscv_dcache_cyc_i),
		.wb_dcache_stb_o(wb_riscv_dcache_stb_i),
		.wb_dcache_adr_o(wb_riscv_dcache_adr_i),
		.wb_dcache_we_o(wb_riscv_dcache_we_i),
		.wb_dcache_dat_o(wb_riscv_dcache_dat_i),
		.wb_dcache_sel_o(wb_riscv_dcache_sel_i),
		.wb_dcache_dat_i(wb_riscv_dcache_dat_o),
		.wb_dcache_ack_i(wb_riscv_dcache_ack_o),
		.wb_dcache_err_i(wb_riscv_dcache_err_o),
		.wb_ptw_cyc_o(wb_riscv_ptw_cyc_i),
		.wb_ptw_stb_o(wb_riscv_ptw_stb_i),
		.wb_ptw_adr_o(wb_riscv_ptw_adr_i),
		.wb_ptw_we_o(wb_riscv_ptw_we_i),
		.wb_ptw_dat_o(wb_riscv_ptw_dat_i),
		.wb_ptw_sel_o(wb_riscv_ptw_sel_i),
		.wb_ptw_dat_i(wb_riscv_ptw_dat_o),
		.wb_ptw_ack_i(wb_riscv_ptw_ack_o),
		.wb_ptw_err_i(wb_riscv_ptw_err_o),
		.wb_slv_cyc_i(wb_riscv_slv_cyc_o),
		.wb_slv_stb_i(wb_riscv_slv_stb_o),
		.wb_slv_adr_i(wb_riscv_slv_adr_o),
		.wb_slv_we_i(wb_riscv_slv_we_o),
		.wb_slv_dat_i(wb_riscv_slv_dat_o),
		.wb_slv_sel_i(wb_riscv_slv_sel_o),
		.wb_slv_dat_o(wb_riscv_slv_dat_i),
		.wb_slv_ack_o(wb_riscv_slv_ack_i),
		.wb_slv_err_o(wb_riscv_slv_err_i),
		.boot_mode(boot_mode),
		.cfg_ti_mode(cfg_ti_mode),
		.cfg_gpio_afr0(cfg_gpio_afr0),
		.cfg_gpio_afr1(cfg_gpio_afr1),
		.cfg_ti_clk_div(cfg_ti_clk_div),
		.cfg_rst_pc(cfg_rst_pc),
		.cfg_rst_ctrl(cfg_rst_ctrl)
	);
	wb_interconnect wb_interconnect_u(
		.clk_i(wb_clk_i),
		.rst_n(wb_int_rst_n),
		.m0_wbd_dat_i(wb_int_dat_i),
		.m0_wbd_adr_i(wb_int_adr_i),
		.m0_wbd_sel_i(wb_int_sel_i),
		.m0_wbd_we_i(wb_int_we_i),
		.m0_wbd_cyc_i(wb_int_cyc_i),
		.m0_wbd_stb_i(wb_int_stb_i),
		.m0_wbd_dat_o(wb_int_dat_o),
		.m0_wbd_ack_o(wb_int_ack_o),
		.m0_wbd_err_o(wb_int_err_o),
		.m1_wbd_dat_i(wb_riscv_ptw_dat_i),
		.m1_wbd_adr_i(wb_riscv_ptw_adr_i),
		.m1_wbd_sel_i(wb_riscv_ptw_sel_i),
		.m1_wbd_bl_i(3'h1),
		.m1_wbd_bry_i(1'b1),
		.m1_wbd_we_i(wb_riscv_ptw_we_i),
		.m1_wbd_cyc_i(wb_riscv_ptw_cyc_i),
		.m1_wbd_stb_i(wb_riscv_ptw_stb_i),
		.m1_wbd_dat_o(wb_riscv_ptw_dat_o),
		.m1_wbd_ack_o(wb_riscv_ptw_ack_o),
		.m1_wbd_lack_o(),
		.m1_wbd_err_o(wb_riscv_ptw_err_o),
		.m2_wbd_dat_i(wb_riscv_dcache_dat_i),
		.m2_wbd_adr_i(wb_riscv_dcache_adr_i),
		.m2_wbd_sel_i(wb_riscv_dcache_sel_i),
		.m2_wbd_bl_i(10'h001),
		.m2_wbd_bry_i(1'b1),
		.m2_wbd_we_i(wb_riscv_dcache_we_i),
		.m2_wbd_cyc_i(wb_riscv_dcache_cyc_i),
		.m2_wbd_stb_i(wb_riscv_dcache_stb_i),
		.m2_wbd_dat_o(wb_riscv_dcache_dat_o),
		.m2_wbd_ack_o(wb_riscv_dcache_ack_o),
		.m2_wbd_lack_o(),
		.m2_wbd_err_o(wb_riscv_dcache_err_o),
		.m3_wbd_dat_i(wb_riscv_icache_dat_i),
		.m3_wbd_adr_i(wb_riscv_icache_adr_i),
		.m3_wbd_sel_i(wb_riscv_icache_sel_i),
		.m3_wbd_bl_i(10'h001),
		.m3_wbd_bry_i(1'b1),
		.m3_wbd_we_i(wb_riscv_icache_we_i),
		.m3_wbd_cyc_i(wb_riscv_icache_cyc_i),
		.m3_wbd_stb_i(wb_riscv_icache_stb_i),
		.m3_wbd_dat_o(wb_riscv_icache_dat_o),
		.m3_wbd_ack_o(wb_riscv_icache_ack_o),
		.m3_wbd_lack_o(),
		.m3_wbd_err_o(wb_riscv_icache_err_o),
		.m4_wbd_dat_i(wb_ti_ma_dat_i),
		.m4_wbd_adr_i(wb_ti_ma_adr_i),
		.m4_wbd_sel_i(wb_ti_ma_sel_i),
		.m4_wbd_bl_i(10'h001),
		.m4_wbd_bry_i(1'b1),
		.m4_wbd_we_i(wb_ti_ma_we_i),
		.m4_wbd_cyc_i(wb_ti_ma_cyc_i),
		.m4_wbd_stb_i(wb_ti_ma_stb_i),
		.m4_wbd_dat_o(wb_ti_ma_dat_o),
		.m4_wbd_ack_o(wb_ti_ma_ack_o),
		.m4_wbd_lack_o(),
		.m4_wbd_err_o(wb_ti_ma_err_o),
		.s0_wbd_dat_i(wb_rom_dat_i),
		.s0_wbd_ack_i(wb_rom_ack_i),
		.s0_wbd_lack_i(wb_rom_lack_i),
		.s0_wbd_dat_o(wb_rom_dat_o),
		.s0_wbd_adr_o(wb_rom_adr_o),
		.s0_wbd_bry_o(wb_rom_bry_o),
		.s0_wbd_bl_o(wb_rom_bl_o),
		.s0_wbd_sel_o(wb_rom_sel_o),
		.s0_wbd_we_o(wb_rom_we_o),
		.s0_wbd_cyc_o(wb_rom_cyc_o),
		.s0_wbd_stb_o(wb_rom_stb_o),
		.s1_wbd_dat_i(wb_uart_dat_i),
		.s1_wbd_ack_i(wb_uart_ack_i),
		.s1_wbd_dat_o(wb_uart_dat_o),
		.s1_wbd_adr_o(wb_uart_adr_o),
		.s1_wbd_sel_o(wb_uart_sel_o),
		.s1_wbd_we_o(wb_uart_we_o),
		.s1_wbd_cyc_o(wb_uart_cyc_o),
		.s1_wbd_stb_o(wb_uart_stb_o),
		.s2_wbd_dat_i(wb_gpio_dat_i),
		.s2_wbd_ack_i(wb_gpio_ack_i),
		.s2_wbd_dat_o(wb_gpio_dat_o),
		.s2_wbd_adr_o(wb_gpio_adr_o),
		.s2_wbd_sel_o(wb_gpio_sel_o),
		.s2_wbd_we_o(wb_gpio_we_o),
		.s2_wbd_cyc_o(wb_gpio_cyc_o),
		.s2_wbd_stb_o(wb_gpio_stb_o),
		.s3_wbd_dat_i(wb_ti_dat_i),
		.s3_wbd_ack_i(wb_ti_ack_i),
		.s3_wbd_dat_o(wb_ti_dat_o),
		.s3_wbd_adr_o(wb_ti_adr_o),
		.s3_wbd_sel_o(wb_ti_sel_o),
		.s3_wbd_we_o(wb_ti_we_o),
		.s3_wbd_cyc_o(wb_ti_cyc_o),
		.s3_wbd_stb_o(wb_ti_stb_o),
		.s4_wbd_dat_i(wb_l2_dat_i),
		.s4_wbd_ack_i(wb_l2_ack_i),
		.s4_wbd_dat_o(wb_l2_dat_o),
		.s4_wbd_adr_o(wb_l2_adr_o),
		.s4_wbd_sel_o(wb_l2_sel_o),
		.s4_wbd_we_o(wb_l2_we_o),
		.s4_wbd_cyc_o(wb_l2_cyc_o),
		.s4_wbd_stb_o(wb_l2_stb_o),
		.s5_wbd_dat_i(wb_riscv_slv_dat_i),
		.s5_wbd_ack_i(wb_riscv_slv_ack_i),
		.s5_wbd_dat_o(wb_riscv_slv_dat_o),
		.s5_wbd_adr_o(wb_riscv_slv_adr_o),
		.s5_wbd_sel_o(wb_riscv_slv_sel_o),
		.s5_wbd_we_o(wb_riscv_slv_we_o),
		.s5_wbd_cyc_o(wb_riscv_slv_cyc_o),
		.s5_wbd_stb_o(wb_riscv_slv_stb_o)
	);
	uart_top uart_u(
		.app_clk(wb_clk_i),
		.uart_rstn(uart_arstn),
		.reg_cs(wb_uart_stb_o),
		.reg_wr(wb_uart_we_o),
		.reg_addr(wb_uart_adr_o[8:0]),
		.reg_wdata(wb_uart_dat_o),
		.reg_be(wb_uart_sel_o),
		.reg_rdata(wb_uart_dat_i),
		.reg_ack(wb_uart_ack_i),
		.uart_rxd(uart_rxd),
		.uart_txd(uart_txd)
	);
	gpio gpio_u(
		.mclk(wb_clk_i),
		.h_reset_n(gpio_arstn),
		.reg_cs(wb_gpio_stb_o),
		.reg_wr(wb_gpio_we_o),
		.reg_addr(wb_gpio_adr_o),
		.reg_wdata(wb_gpio_dat_o),
		.reg_be(wb_gpio_sel_o),
		.reg_rdata(wb_gpio_dat_i),
		.reg_ack(wb_gpio_ack_i),
		.gpio_out(gpio_out),
		.gpio_oen(gpio_oen),
		.gpio_in(gpio_in),
		.gpio_intr(gpio_intr)
	);
	reset_sync rom_rstn_u(
		.scan_mode(1'b0),
		.dclk(wb_clk_i),
		.arst_n(rom_arstn),
		.srst_n(rom_rstn)
	);
	rct_w2m_bridge w2m_bridge_rom_u(
		.clk_i(wb_clk_i),
		.rstn_i(rom_rstn),
		.wb_stb_i(wb_rom_stb_o),
		.wb_addr_i(wb_rom_adr_o),
		.wb_we_i(wb_rom_we_o),
		.wb_data_i(wb_rom_dat_o),
		.wb_sel_i(wb_rom_sel_o),
		.wb_cyc_i(wb_rom_cyc_o),
		.wb_ack_o(wb_rom_ack_i),
		.wb_err_o(wb_rom_err_i),
		.wb_data_o(wb_rom_dat_i),
		.mem_if_req_valid(rom_mem_if_req_valid),
		.mem_if_req_ready(rom_mem_if_req_ready),
		.mem_if_req(rom_mem_if_req),
		.mem_if_resp_valid(rom_mem_if_resp_valid),
		.mem_if_resp_ready(rom_mem_if_resp_ready),
		.mem_if_resp(rom_mem_if_resp)
	);
	rct_boot_rom boot_rom_u(
		.req_valid(rom_mem_if_req_valid),
		.req(rom_mem_if_req),
		.req_ready(rom_mem_if_req_ready),
		.resp_valid(rom_mem_if_resp_valid),
		.resp(rom_mem_if_resp),
		.resp_ready(rom_mem_if_resp_ready),
		.clk(wb_clk_i),
		.rstn(rom_rstn)
	);
	reset_sync l2_sram_rstn_u(
		.scan_mode(1'b0),
		.dclk(wb_clk_i),
		.arst_n(l2_sram_arstn),
		.srst_n(l2_sram_rstn)
	);
	rct_w2m_bridge w2m_bridge_l2_u(
		.clk_i(wb_clk_i),
		.rstn_i(l2_sram_rstn),
		.wb_stb_i(wb_l2_stb_o),
		.wb_addr_i(wb_l2_adr_o),
		.wb_we_i(wb_l2_we_o),
		.wb_data_i(wb_l2_dat_o),
		.wb_sel_i(wb_l2_sel_o),
		.wb_cyc_i(wb_l2_cyc_o),
		.wb_ack_o(wb_l2_ack_i),
		.wb_err_o(wb_l2_err_i),
		.wb_data_o(wb_l2_dat_i),
		.mem_if_req_valid(l2_mem_if_req_valid),
		.mem_if_req_ready(l2_mem_if_req_ready),
		.mem_if_req(l2_mem_if_req),
		.mem_if_resp_valid(l2_mem_if_resp_valid),
		.mem_if_resp_ready(l2_mem_if_resp_ready),
		.mem_if_resp(l2_mem_if_resp)
	);
	rct_l2_sram l2_sram_u(
		.req_valid(l2_mem_if_req_valid),
		.req(l2_mem_if_req),
		.req_ready(l2_mem_if_req_ready),
		.resp_valid(l2_mem_if_resp_valid),
		.resp(l2_mem_if_resp),
		.resp_ready(l2_mem_if_resp_ready),
		.clk(wb_clk_i),
		.rstn(l2_sram_rstn)
	);
	rct_testio_wb rct_testio_wb_u(
		.clk_i(wb_clk_i),
		.rstn_i(ti_arstn),
		.wb_stb_i(wb_ti_stb_o),
		.wb_addr_i(wb_ti_adr_o),
		.wb_we_i(wb_ti_we_o),
		.wb_data_i(wb_ti_dat_o),
		.wb_sel_i(wb_ti_sel_o),
		.wb_cyc_i(wb_ti_cyc_o),
		.wb_ack_o(wb_ti_ack_i),
		.wb_err_o(wb_ti_err_i),
		.wb_data_o(wb_ti_dat_i),
		.ti_mod_i(cfg_ti_mode),
		.ti_clk_div_i(cfg_ti_clk_div),
		.ti_dat_i(ti_dat_i),
		.ti_clk_o(ti_clk_o),
		.ti_clk_oen(ti_clk_oen),
		.ti_dat_o(ti_dat_o),
		.ti_dat_oen(ti_dat_oen)
	);
	rct_testio_ma_wb rct_testio_ma_wb_u(
		.clk_i(wb_clk_i),
		.rstn_i(ti_ma_arstn),
		.test_intr(test_intr),
		.test_clk(test_clk),
		.test_din(test_din),
		.test_dout(test_dout),
		.test_doen(test_doen),
		.wb_stb_o(wb_ti_ma_stb_i),
		.wb_addr_o(wb_ti_ma_adr_i),
		.wb_we_o(wb_ti_ma_we_i),
		.wb_data_o(wb_ti_ma_dat_i),
		.wb_sel_o(wb_ti_ma_sel_i),
		.wb_cyc_o(wb_ti_ma_cyc_i),
		.wb_ack_i(wb_ti_ma_ack_o),
		.wb_err_i(wb_ti_ma_err_o),
		.wb_data_i(wb_ti_ma_dat_o)
	);
	rct_pinmux pinmux_u(
		.digital_io_out(io_out),
		.digital_io_oen(io_oeb),
		.digital_io_in(io_in),
		.gpio_out(gpio_out),
		.gpio_oen(gpio_oen),
		.gpio_in(gpio_in),
		.tck(jtag_tck),
		.tms(jtag_tms),
		.trst_n(jtag_trst_n),
		.tdi(jtag_tdi),
		.tdo(jtag_tdo),
		.tdo_oe(jtag_tdo_oe),
		.ti_ma_clk_i(test_clk),
		.ti_ma_dat_i(test_din),
		.ti_ma_dat_o(test_dout),
		.ti_ma_dat_oen(test_doen),
		.ti_clk_o(ti_clk_o),
		.ti_clk_oen(ti_clk_oen),
		.ti_dat_o(ti_dat_o),
		.ti_dat_oen(ti_dat_oen),
		.ti_dat_i(ti_dat_i),
		.uart_txd(uart_txd),
		.uart_rxd(uart_rxd),
		.uartm_txd(uartm_txd),
		.uartm_rxd(uartm_rxd),
		.boot_mode(boot_mode),
		.ext_int(ext_int),
		.cfg_pinmux_l(cfg_gpio_afr0),
		.cfg_pinmux_h(cfg_gpio_afr1),
		.dbg_clk_mon(dbg_clk_mon)
	);
endmodule
