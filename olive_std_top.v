// ===================================================================
// TITLE : PERIDOT Standard configuration - 'Olive'
//
//   DEGISN : S.OSAFUNE (J-7SYSTEM Works)
//   DATE   : 2015/05/15 -> 2015/05/23
//   UPDATE : 
//
// ===================================================================
// *******************************************************************
//   Copyright (C) 2015, J-7SYSTEM Works.  All rights Reserved.
//
// * This module is a free sourcecode and there is NO WARRANTY.
// * No restriction on use. You can use, modify and redistribute it
//   for personal, non-profit or commercial products UNDER YOUR
//   RESPONSIBILITY.
// * Redistributions of source code must retain the above copyright
//   notice.
// *******************************************************************


module olive_std_top(
	// clk and system reset
	input			CLOCK_50,
	input			RESET_N,

	// Interface: SCI Host communication
	input			SCI_SCLK,
	input			SCI_TXD,
	output			SCI_RXD,
	output			SCI_TXR_N,
	input			SCI_RXR_N,

	// Interface: SPI-Flash memory controller
	output			EPCS_CSO_N,
	output			DCLK_OUT,
	output			EPCS_ASDO,
	input			EPCS_DATA0,

	// Interface: SDRAM
	output			SDR_CLK,
	output			SDR_CKE,
	output			SDR_CS_N,
	output			SDR_RAS_N,
	output			SDR_CAS_N,
	output			SDR_WE_N,
	output [1:0]	SDR_BA,
	output [11:0]	SDR_A,
	inout  [15:0]	SDR_DQ,
	output [1:0]	SDR_DQM,

	// GPIO
	inout  [27:0]	D,

	// OnBoard LED
	output			START_LED
);


/* ===== 外部変更可能パラメータ ========== */



/* ----- 内部パラメータ ------------------ */


/* ※以降のパラメータ宣言は禁止※ */

/* ===== ノード宣言 ====================== */
				/* 内部は全て正論理リセットとする。ここで定義していないノードの使用は禁止 */
	wire			reset_sig = ~RESET_N;			// モジュール内部駆動非同期リセット 

				/* 内部は全て正エッジ駆動とする。ここで定義していないクロックノードの使用は禁止 */
	wire			clock_sig = CLOCK_50;			// モジュール内部駆動クロック 

	wire			qsys_reset_n_sig;
	wire			clock_core_sig;
	wire			clock_peri_sig;
	wire			cpureset_sig;
	reg  [1:0]		resetreq_reg;

	wire			pfc_clock_sig;
	wire			pfc_reset_sig;
	wire [3:0]		pfc_address_sig;
	wire [31:0]		pfc_readdata_sig;
	wire 			pfc_write_sig;
	wire [31:0]		pfc_writedata_sig;
	wire			pfc_bank0_write_sig;
	wire [31:0]		pfc_bank0_readdata_sig;
	wire			pfc_bank1_write_sig;
	wire [31:0]		pfc_bank1_readdata_sig;
	wire			pfc_bank2_write_sig;
	wire [31:0]		pfc_bank2_readdata_sig;
	wire			pfc_bank3_write_sig;
	wire [31:0]		pfc_bank3_readdata_sig;

	wire [7:0]		pfc_bank0_din_sig, pfc_bank0_pin_through_sig;
	wire [5:0]		pfc_bank0_aux_in_sig;
	wire [7:0]		pfc_bank1_din_sig, pfc_bank1_pin_through_sig;
	wire [5:0]		pfc_bank1_aux_in_sig;
	wire [7:0]		pfc_bank2_din_sig, pfc_bank2_pin_through_sig;
	wire [5:0]		pfc_bank2_aux_in_sig;
	wire [7:0]		pfc_bank3_din_sig, pfc_bank3_pin_through_sig;
	wire [5:0]		pfc_bank3_aux_in_sig;

	wire			uart0_rxd_sig, uart0_txd_sig, uart0_rts_n_sig, uart0_cts_n_sig;
	wire			uart1_rxd_sig, uart1_txd_sig, uart1_rts_n_sig, uart1_cts_n_sig;
	wire			i2c_scl_in_sig,i2c_scl_oe_sig, i2c_sda_in_sig,i2c_sda_oe_sig;
	wire			spi_ss_n_sig, spi_sclk_sig, spi_mosi_sig, ss_miso_sig;
	wire [7:0]		servo_sig, analog_sig;


/* ※以降のwire、reg宣言は禁止※ */

/* ===== テスト記述 ============== */



/* ===== モジュール構造記述 ============== */

	///// PLLとQsysコアのインスタンス /////

	syspll
	u0 (
		.areset		(reset_sig),
		.inclk0		(clock_sig),
		.c0			(SDR_CLK),
		.c1			(clock_core_sig),
		.c2			(clock_peri_sig),
		.locked		(qsys_reset_n_sig)
	);


	olive_std_core
	u1 (
        .reset_reset_n   (qsys_reset_n_sig),		//    reset.reset_n
        .clk_100m_clk    (clock_core_sig),			// clk_100m.clk
        .clk_25m_clk     (clock_peri_sig),			//  clk_25m.clk

        .nios2_cpu_resetrequest (resetreq_reg[1]),	//    nios2.cpu_resetrequest <-- NiosIIのバストランザクションを完了させるリセット要求 
        .nios2_cpu_resettaken   (),					//         .cpu_resettaken

        .sci_sclk        (SCI_SCLK),				//      sci.sclk
        .sci_txd         (SCI_TXD),					//         .txd
        .sci_txr_n       (SCI_TXR_N),				//         .txr_n
        .sci_rxd         (SCI_RXD),					//         .rxd
        .sci_rxr_n       (SCI_RXR_N),				//         .rxr_n

        .sdr_addr        (SDR_A),					//      sdr.addr
        .sdr_ba          (SDR_BA),					//         .ba
        .sdr_cs_n        (SDR_CS_N),				//         .cs_n
        .sdr_ras_n       (SDR_RAS_N),				//         .ras_n
        .sdr_cas_n       (SDR_CAS_N),				//         .cas_n
        .sdr_we_n        (SDR_WE_N),				//         .we_n
        .sdr_dq          (SDR_DQ),					//         .dq
        .sdr_dqm         (SDR_DQM),					//         .dqm
        .sdr_cke         (SDR_CKE),					//         .cke

        .swi_cpureset    (cpureset_sig),			//      swi.cpureset
        .swi_led         (START_LED),				//         .led
        .swi_cso_n       (EPCS_CSO_N),				//         .cso_n
        .swi_dclk        (DCLK_OUT),				//         .dclk
        .swi_asdo        (EPCS_ASDO),				//         .asdo
        .swi_data0       (EPCS_DATA0),				//         .data0

        .pfcif_pfc_clk   (pfc_clock_sig), 			//    pfcif.pfc_clk
        .pfcif_pfc_reset (pfc_reset_sig),			//         .pfc_reset
        .pfcif_addrss    (pfc_address_sig),			//         .addrss
        .pfcif_readdata  (pfc_readdata_sig),		//         .readdata
        .pfcif_write     (pfc_write_sig),			//         .write
        .pfcif_writedata (pfc_writedata_sig),		//         .writedata

        .uart0_rxd       (uart0_rxd_sig),			//    uart0.rxd
        .uart0_txd       (uart0_txd_sig),			//         .txd
        .uart0_cts_n     (uart0_cts_n_sig),			//         .cts_n
        .uart0_rts_n     (uart0_rts_n_sig),			//         .rts_n

        .uart1_rxd       (uart1_rxd_sig),			//    uart1.rxd
        .uart1_txd       (uart1_txd_sig),			//         .txd
        .uart1_cts_n     (uart1_cts_n_sig),			//         .cts_n
        .uart1_rts_n     (uart1_rts_n_sig),			//         .rts_n

        .i2c_scl         (i2c_scl_in_sig),			//      i2c.scl
        .i2c_scl_oe      (i2c_scl_oe_sig),			//         .scl_oe
        .i2c_sda         (i2c_sda_in_sig),			//         .sda
        .i2c_sda_oe      (i2c_sda_oe_sig),			//         .sda_oe

        .spi_ss_n        (spi_ss_n_sig),			//      spi.ss_n
        .spi_sclk        (spi_sclk_sig),			//         .sclk
        .spi_mosi        (spi_mosi_sig),			//         .mosi
        .spi_miso        (spi_miso_sig),			//         .miso

        .servo_pwm       (servo_sig),				//    servo.pwm
        .servo_dsm       (analog_sig)				//         .dsm
    );


	// cpu_resetrequest信号の同期化 

	always @(posedge clock_core_sig or negedge qsys_reset_n_sig) begin
		if (!qsys_reset_n_sig) begin
			resetreq_reg <= 2'b00;
		end
		else begin
			resetreq_reg <= {resetreq_reg[0], cpureset_sig};
		end
	end



	///// ピンファンクションコントローラ接続 /////

	assign pfc_readdata_sig =
			(pfc_address_sig[3:2] == 2'd0)? pfc_bank0_readdata_sig :
			(pfc_address_sig[3:2] == 2'd1)? pfc_bank1_readdata_sig :
			(pfc_address_sig[3:2] == 2'd2)? pfc_bank2_readdata_sig :
			(pfc_address_sig[3:2] == 2'd3)? pfc_bank3_readdata_sig :
			{32{1'bx}};

	assign pfc_bank0_write_sig = (pfc_address_sig[3:2] == 2'd0)? pfc_write_sig : 1'b0;
	assign pfc_bank1_write_sig = (pfc_address_sig[3:2] == 2'd1)? pfc_write_sig : 1'b0;
	assign pfc_bank2_write_sig = (pfc_address_sig[3:2] == 2'd2)? pfc_write_sig : 1'b0;
	assign pfc_bank3_write_sig = (pfc_address_sig[3:2] == 2'd3)? pfc_write_sig : 1'b0;



	///// ピンファンクションコントローラのインスタンス /////

	// BANK0: D7 - D0

	assign uart1_rxd_sig   = pfc_bank0_din_sig[7];
	assign uart0_rxd_sig   = pfc_bank0_din_sig[6];
	assign uart1_cts_n_sig = pfc_bank0_din_sig[3];
	assign uart0_cts_n_sig = pfc_bank0_din_sig[2];

	peridot_pfc #(
		.PIN_WIDTH			( 8 ),				// output port width :1-8
		.DEFAULT_PINREGS	( 32'h00000000 ),	// init pinreg value
		.DEFAULT_FUNCREGS	( 32'h00000000 )	// init funcreg value
	)
	u_pfc0 (
		.coe_pin			(D[7:0]),

		.csi_clk			(pfc_clock_sig),
		.rsi_reset			(pfc_reset_sig),
		.avs_address		(pfc_address_sig[1:0]),
		.avs_read			(1'b1),
		.avs_readdata		(pfc_bank0_readdata_sig),
		.avs_write			(pfc_bank0_write_sig),
		.avs_writedata		(pfc_writedata_sig),

		.coe_pin_through	(pfc_bank0_pin_through_sig),
		.coe_pin_aux_in		(pfc_bank1_pin_through_sig[5:0]),	// aux input D13-D8

		.coe_function_din	(pfc_bank0_din_sig),
		.coe_function_dout	({
				uart1_txd_sig,		// func7 dout
				uart0_txd_sig,		// func6 dout
				1'b0,				// func5 dout
				1'b0,				// func4 dout
				uart1_rts_n_sig,	// func3 dout
				uart0_rts_n_sig,	// func2 dout
				1'bx,				// func1 dout
				1'bx				// func0 dout
			}),
		.coe_function_oe	({
				1'b1,				// func7 oe
				1'b1,				// func6 oe
				i2c_sda_oe_sig,		// func5 oe
				i2c_scl_oe_sig,		// func4 oe
				1'b1,				// func3 oe
				1'b1,				// func2 oe
				1'b0,				// func1 oe
				1'b0				// func0 oe
			}),

		.coe_function_aux0	(servo_sig),
		.coe_function_aux1	({8{1'bx}}),
		.coe_function_aux2	({8{1'bx}}),
		.coe_function_aux3	({8{1'bx}})
	);


	// BANK1: D15 - D8

	assign i2c_sda_in_sig  = pfc_bank1_din_sig[5];
	assign i2c_scl_in_sig  = pfc_bank1_din_sig[4];
	assign spi_miso_sig    = pfc_bank1_din_sig [0];

	peridot_pfc #(
		.PIN_WIDTH			( 8 ),				// output port width :1-8
		.DEFAULT_PINREGS	( 32'h00000000 ),	// init pinreg value
		.DEFAULT_FUNCREGS	( 32'h00000000 )	// init funcreg value
	)
	u_pfc1 (
		.coe_pin			(D[15:8]),

		.csi_clk			(pfc_clock_sig),
		.rsi_reset			(pfc_reset_sig),
		.avs_address		(pfc_address_sig[1:0]),
		.avs_read			(1'b1),
		.avs_readdata		(pfc_bank1_readdata_sig),
		.avs_write			(pfc_bank1_write_sig),
		.avs_writedata		(pfc_writedata_sig),

		.coe_pin_through	(pfc_bank1_pin_through_sig),
		.coe_pin_aux_in		(pfc_bank0_pin_through_sig[7:2]),	// aux input D7-D2

		.coe_function_din	(pfc_bank1_din_sig),
		.coe_function_dout	({
				uart1_txd_sig,	// func7 dout
				uart0_txd_sig,	// func6 dout
				1'b0,			// func5 dout
				1'b0,			// func4 dout
				spi_ss_n_sig,	// func3 dout
				spi_sclk_sig,	// func2 dout
				spi_mosi_sig,	// func1 dout
				1'bx			// func0 dout
			}),
		.coe_function_oe	({
				1'b1,			// func7 oe
				1'b1,			// func6 oe
				i2c_sda_oe_sig,	// func5 oe
				i2c_scl_oe_sig,	// func4 oe
				1'b1,			// func3 oe
				1'b1,			// func2 oe
				1'b1,			// func1 oe
				1'b0			// func0 oe
			}),

		.coe_function_aux0	(servo_sig),
		.coe_function_aux1	({8{1'bx}}),
		.coe_function_aux2	({8{1'bx}}),
		.coe_function_aux3	({8{1'bx}})
	);


	// BANK2: D21 - D16

	peridot_pfc #(
		.PIN_WIDTH			( 6 ),				// output port width :1-8
		.DEFAULT_PINREGS	( 32'h00000000 ),	// init pinreg value
		.DEFAULT_FUNCREGS	( 32'h00000000 )	// init funcreg value
	)
	u_pfc2 (
		.coe_pin			(D[21:16]),

		.csi_clk			(pfc_clock_sig),
		.rsi_reset			(pfc_reset_sig),
		.avs_address		(pfc_address_sig[1:0]),
		.avs_read			(1'b1),
		.avs_readdata		(pfc_bank2_readdata_sig),
		.avs_write			(pfc_bank2_write_sig),
		.avs_writedata		(pfc_writedata_sig),

		.coe_pin_through	(),
		.coe_pin_aux_in		({6{1'bx}}),

		.coe_function_din	(),
		.coe_function_dout	({8{1'bx}}),
		.coe_function_oe	({8{1'b0}}),

		.coe_function_aux0	({{2{1'bx}}, servo_sig[5:0]}),
		.coe_function_aux1	({{2{1'bx}}, analog_sig[5:0]}),
		.coe_function_aux2	({8{1'bx}}),
		.coe_function_aux3	({8{1'bx}})
	);


	// BANK3: D27 - D22

	peridot_pfc #(
		.PIN_WIDTH			( 6 ),				// output port width :1-8
		.DEFAULT_PINREGS	( 32'h00000000 ),	// init pinreg value
		.DEFAULT_FUNCREGS	( 32'h00000000 )	// init funcreg value
	)
	u_pfc3 (
		.coe_pin			(D[27:22]),

		.csi_clk			(pfc_clock_sig),
		.rsi_reset			(pfc_reset_sig),
		.avs_address		(pfc_address_sig[1:0]),
		.avs_read			(1'b1),
		.avs_readdata		(pfc_bank3_readdata_sig),
		.avs_write			(pfc_bank3_write_sig),
		.avs_writedata		(pfc_writedata_sig),

		.coe_pin_through	(),
		.coe_pin_aux_in		({6{1'bx}}),

		.coe_function_din	(),
		.coe_function_dout	({8{1'bx}}),
		.coe_function_oe	({8{1'b0}}),

		.coe_function_aux0	({8{1'bx}}),
		.coe_function_aux1	({8{1'bx}}),
		.coe_function_aux2	({8{1'bx}}),
		.coe_function_aux3	({8{1'bx}})
	);


endmodule

