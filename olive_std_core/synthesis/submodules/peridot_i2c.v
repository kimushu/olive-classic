// ===================================================================
// TITLE : PERIDOT / I2C master
//
//   DEGISN : S.OSAFUNE (J-7SYSTEM Works)
//   DATE   : 2015/05/21 -> 2015/05/22
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

// reg00(+0)  bit15:irqena(RW), bit12:sta(WO), bit11:stp(WO), bit10:rd_nwr(WO), bit9:start(W)/ready(R), bit8:nack(RW), bit7-0:txdata(W)/rxdata(R)
// reg01(+4)  bit15:devrst(RW), bit9-0:clkdiv(RW)

module peridot_i2c (
	// Interface: clk
	input			csi_clk,
	input			rsi_reset,

	// Interface: Avalon-MM slave
	input  [0:0]	avs_address,
	input			avs_read,			// read  0-setup,1-wait,0-hold
	output [31:0]	avs_readdata,
	input			avs_write,			// write 0-setup,0-wait,0-hold
	input  [31:0]	avs_writedata,

	// Interface: Avalon-MM Interrupt sender
	output			ins_irq,

	// External Interface
	output			i2c_scl_oe,
	output			i2c_sda_oe,
	input			i2c_scl,
	input			i2c_sda
//	inout			i2c_scl,	// test
//	inout			i2c_sda		// test
);


/* ===== �O���ύX�\�p�����[�^ ========== */



/* ----- �����p�����[�^ ------------------ */

	localparam	STATE_IDLE		= 5'd0,
				STATE_INIT		= 5'd1,
				STATE_SC_1		= 5'd2,
				STATE_SC_2		= 5'd3,
				STATE_SC_3		= 5'd4,
				STATE_SC_4		= 5'd5,
				STATE_SC_5		= 5'd6,
				STATE_BIT_ENTRY	= 5'd7,
				STATE_BIT_1		= 5'd8,
				STATE_BIT_2		= 5'd9,
				STATE_BIT_3		= 5'd10,
				STATE_BIT_4		= 5'd11,
				STATE_PC_1		= 5'd13,
				STATE_PC_2		= 5'd14,
				STATE_PC_3		= 5'd15,
				STATE_DONE		= 5'd31;

	localparam	STATE_IO_IDLE	= 5'd0,
				STATE_IO_SETSCL	= 5'd1,
				STATE_IO_SETSDA	= 5'd2,
				STATE_IO_WAIT	= 5'd3,
				STATE_IO_DONE	= 5'd31;


/* ���ȍ~�̃p�����[�^�錾�͋֎~�� */

/* ===== �m�[�h�錾 ====================== */
				/* �����͑S�Đ��_�����Z�b�g�Ƃ���B�����Œ�`���Ă��Ȃ��m�[�h�̎g�p�͋֎~ */
	wire			reset_sig = rsi_reset;			// ���W���[�������쓮�񓯊����Z�b�g 

				/* �����͑S�Đ��G�b�W�쓮�Ƃ���B�����Œ�`���Ă��Ȃ��N���b�N�m�[�h�̎g�p�͋֎~ */
	wire			clock_sig = csi_clk;			// ���W���[�������쓮�N���b�N 

	wire			begintransaction_sig;
	reg				irqena_reg;
	reg				sendstp_reg;
	reg				i2crst_reg;
	reg  [9:0]		divref_reg;

	reg  [4:0]		state_reg;
	reg				ready_reg;
	reg  [8:0]		txbyte_reg, rxbyte_reg;
	reg  [3:0]		bitcount;
	reg				setsclreq_reg, setsdareq_reg, pindata_reg;

	reg  [4:0]		state_io_reg;
	reg  [9:0]		divcount;
	wire			stateio_ack_sig;
	reg				scl_oe_reg, sda_oe_reg;
	reg  [1:0]		i2c_scl_in_reg, i2c_sda_in_reg;
	wire			scl_sig, sda_sig;


/* ���ȍ~��wire�Areg�錾�͋֎~�� */

/* ===== �e�X�g�L�q ============== */

//	assign i2c_scl = (i2c_scl_oe)? 1'b0 : 1'bz;
//	assign i2c_sda = (i2c_sda_oe)? 1'b0 : 1'bz;


/* ===== ���W���[���\���L�q ============== */

	///// Avalon-MM�C���^�[�t�F�[�X /////

	assign ins_irq = (irqena_reg)? ready_reg : 1'b0;

	assign avs_readdata =
			(avs_address == 1'd0)? {16'b0, irqena_reg, 5'b0, ready_reg, rxbyte_reg[0], rxbyte_reg[8:1]}:
			(avs_address == 1'd1)? {16'b0, i2crst_reg, 5'b0, divref_reg} :
			{32{1'bx}};

	assign begintransaction_sig = (avs_write && avs_address == 1'd0 && avs_writedata[9]);

	always @(posedge clock_sig or posedge reset_sig) begin
		if (reset_sig) begin
			i2crst_reg <= 1'b1;
			irqena_reg <= 1'b0;
			divref_reg <= 10'h3ff;
		end
		else begin

			// ���Z�b�g���W�X�^�̓ǂݏ��� 
			if (avs_write && avs_address == 1'd1) begin
				i2crst_reg <= avs_writedata[15];
			end

			// �X�e�[�^�X���W�X�^�̓ǂݏ��� 
			if (i2crst_reg) begin
				irqena_reg <= 1'b0;
			end
			else begin
				if (avs_write && ready_reg) begin
					case (avs_address)
					1'd0 : begin
						irqena_reg  <= avs_writedata[15];
					end
					1'd1 : begin
						divref_reg  <= avs_writedata[9:0];
					end
					endcase
				end
			end

		end
	end



	///// I2C����M���� /////

	always @(posedge clock_sig or posedge reset_sig) begin
		if (reset_sig) begin
			state_reg  <= STATE_INIT;
			ready_reg  <= 1'b0;
			setsclreq_reg <= 1'b0;
			setsdareq_reg <= 1'b0;
		end
		else begin
			if (i2crst_reg) begin
				state_reg  <= STATE_INIT;
				ready_reg  <= 1'b0;
				setsclreq_reg <= 1'b0;
				setsdareq_reg <= 1'b0;
			end
			else begin

				case (state_reg)

				// ������ 

				STATE_INIT : begin
					if (sda_sig == 1'b1 && scl_sig == 1'b1) begin
						state_reg <= STATE_IDLE;
						ready_reg <= 1'b1;
					end
				end


				// �g�����U�N�V�����J�n��t 

				STATE_IDLE : begin
					if (begintransaction_sig) begin
						ready_reg <= 1'b0;

						if (avs_writedata[12]) begin
							state_reg <= STATE_SC_1;
						end
						else begin
							state_reg <= STATE_BIT_ENTRY;
						end

						sendstp_reg <= avs_writedata[11];

						if (avs_writedata[10]) begin	// read
							txbyte_reg <= {8'hff, avs_writedata[8]};
						end
						else begin						// write
							txbyte_reg <= {avs_writedata[7:0], 1'b1};
						end
					end
				end


				// START�R���f�B�V�������s (SDA = 'H'�̏�ԂŊJ�n����)

				STATE_SC_1 : begin				// SCL = 'H'
					state_reg <= STATE_SC_2;
					setsclreq_reg <= 1'b1;
					pindata_reg   <= 1'b1;
				end
				STATE_SC_2 : begin				// SDA = 'L'
					if (stateio_ack_sig) begin
						state_reg <= STATE_SC_3;
						setsclreq_reg <= 1'b0;
						setsdareq_reg <= 1'b1;
						pindata_reg   <= 1'b0;
					end
				end
				STATE_SC_3 : begin				// tSU(STA) wait
					if (stateio_ack_sig) begin
						state_reg <= STATE_SC_4;
					end
				end
				STATE_SC_4 : begin				// SCL = 'L'
					if (stateio_ack_sig) begin
						state_reg <= STATE_SC_5;
						setsclreq_reg <= 1'b1;
						setsdareq_reg <= 1'b0;
						pindata_reg   <= 1'b0;
					end
				end
				STATE_SC_5 : begin
					if (stateio_ack_sig) begin
						state_reg <= STATE_BIT_ENTRY;
						setsclreq_reg <= 1'b0;
					end
				end


				// �r�b�g�ǂݏ��� (SCL = 'L'�̏�ԂŊJ�n����) 

				STATE_BIT_ENTRY : begin				// data set 
					state_reg <= STATE_BIT_1;
					setsdareq_reg <= 1'b1;
					pindata_reg   <= txbyte_reg[8];

					bitcount <= 1'd0;
				end
				STATE_BIT_1 : begin					// SCL = 'H'
					if (stateio_ack_sig) begin
						state_reg <= STATE_BIT_2;
						setsclreq_reg <= 1'b1;
						setsdareq_reg <= 1'b0;
						pindata_reg   <= 1'b1;
					end
				end
				STATE_BIT_2 : begin					// data latch & shift
					if (stateio_ack_sig) begin
						state_reg <= STATE_BIT_3;

						txbyte_reg <= {txbyte_reg[7:0], 1'b0};
						rxbyte_reg <= {rxbyte_reg[7:0], sda_sig};
					end
				end
				STATE_BIT_3 : begin					// SCL = 'L'
					if (stateio_ack_sig) begin
						state_reg <= STATE_BIT_4;
						pindata_reg <= 1'b0;
					end
				end
				STATE_BIT_4 : begin					// next data set
					if (stateio_ack_sig) begin
						setsclreq_reg <= 1'b0;
						setsdareq_reg <= 1'b1;

						if (bitcount == 8) begin
							if (sendstp_reg) begin		// SDA = 'L' (STOP condition)
								state_reg <= STATE_PC_1;
								pindata_reg <= 1'b0;
							end
							else begin					// SDA = 'H' (ACK release)
								state_reg <= STATE_DONE;
								pindata_reg <= 1'b1;
							end
						end
						else begin						// SDA = next bit (bit transfer)
							state_reg <= STATE_BIT_1;
							pindata_reg <= txbyte_reg[8];
						end

						bitcount <= bitcount + 1'd1;
					end
				end


				// STOP�R���f�B�V�������s 

				STATE_PC_1 : begin					// SCL = 'H'
					if (stateio_ack_sig) begin
						state_reg <= STATE_PC_2;
						setsclreq_reg <= 1'b1;
						setsdareq_reg <= 1'b0;
						pindata_reg   <= 1'b1;
					end
				end
				STATE_PC_2 : begin					// tH(STO) wait
					if (stateio_ack_sig) begin
						state_reg <= STATE_PC_3;
					end
				end
				STATE_PC_3 : begin					// SDA = 'H'
					if (stateio_ack_sig) begin
						state_reg <= STATE_DONE;
						setsclreq_reg <= 1'b0;
						setsdareq_reg <= 1'b1;
						pindata_reg   <= 1'b1;
					end
				end


				// �g�����U�N�V�����I�� 

				STATE_DONE : begin
					if (stateio_ack_sig) begin
						state_reg <= STATE_IDLE;
						setsclreq_reg <= 1'b0;
						setsdareq_reg <= 1'b0;
						ready_reg <= 1'b1;
					end
				end

				endcase
			end
		end
	end



	///// I2C�M�����o�� /////

	assign i2c_scl_oe = scl_oe_reg;
	assign i2c_sda_oe = sda_oe_reg;
	assign scl_sig = i2c_scl_in_reg[1];
	assign sda_sig = i2c_sda_in_reg[1];

	assign stateio_ack_sig = (state_io_reg == STATE_IO_DONE);

	always @(posedge clock_sig or posedge reset_sig) begin
		if (reset_sig) begin
			i2c_scl_in_reg <= 2'b00;
			i2c_sda_in_reg <= 2'b00;

			state_io_reg <= STATE_IO_IDLE;
			scl_oe_reg   <= 1'b0;
			sda_oe_reg   <= 1'b0;
		end
		else begin
			i2c_scl_in_reg <= {i2c_scl_in_reg[0], i2c_scl};
			i2c_sda_in_reg <= {i2c_sda_in_reg[0], i2c_sda};

			if (i2crst_reg) begin
				state_io_reg <= STATE_IO_IDLE;
				scl_oe_reg   <= 1'b0;
				sda_oe_reg   <= 1'b0;
			end
			else begin
				case (state_io_reg)

				STATE_IO_IDLE : begin
					if (setsclreq_reg) begin
						state_io_reg <= STATE_IO_SETSCL;
						scl_oe_reg   <= ~pindata_reg;
					end
					else if (setsdareq_reg) begin
						state_io_reg <= STATE_IO_SETSDA;
						sda_oe_reg   <= ~pindata_reg;
					end
				end

				// SCL set
				STATE_IO_SETSCL : begin
					if (scl_sig != scl_oe_reg) begin
						state_io_reg <= STATE_IO_WAIT;
						divcount     <= divref_reg;
					end
				end

				// SDA set
				STATE_IO_SETSDA : begin
					state_io_reg <= STATE_IO_WAIT;
					divcount     <= divref_reg;
				end

				// wait
				STATE_IO_WAIT : begin
					if (divcount == 0) begin
						state_io_reg <= STATE_IO_DONE;
					end
					else begin
						divcount <= divcount - 1'd1;
					end
				end

				STATE_IO_DONE : begin
					state_io_reg <= STATE_IO_IDLE;
				end

				endcase
			end
		end
	end



endmodule