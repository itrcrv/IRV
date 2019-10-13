`timescale 1ns/1ps
module rv_exec #
	(
	parameter           DB_RMW      = 0,     
	parameter           DIVIDER_EN    = 1,     
	parameter           MUL_ARCH      = "dsp", 
	parameter           USE_RISCV     = 1,     
	parameter           ENABLE_TIMER  = 1,     
	parameter           TIMER_XLEN    = 32     
	)
	(clk_i
	, rst_i
	, cmd_loadop3_i
	, cmd_signed_i
	, cmd_dbus_i
	, cmd_dbus_store_i
	, cmd_dbus_byte_i
	, cmd_dbus_hword_i
	, cmd_addsub_i
	, cmd_mul_i
	, cmd_div_i
	, cmd_div_mod_i
	, cmd_cmp_i
	, cmd_jump_i
	, cmd_negate_op2_i
	, cmd_and_i
	, cmd_xor_i
	, cmd_shift_i
	, cmd_shift_right_i
	, cmd_mul_high_i
	, cmd_signed_b_i
	, cmd_slt_i
	, cmd_csr_i
	, csr_x0_i
	, csr_op_i
	, cmd_trap_i
	, cmd_tret_i
	, trap_cause_i
	, interrupt_i
	, epc_i
	, ext_irq_in
	, riscv_interrupt_exec_o
	, epc_o
	, tvec_o
	, sstep_o
	, jump_type_i
	, op1_i
	, op2_i
	, op3_i
	, dst_i
	, sp_waddr_o
	, sp_we_o
	, sp_wdata_o
	, displacement_i
	, valid_i
	, ready_o
	, dbus_cyc_o
	, dbus_stb_o
	, dbus_we_o
	, dbus_sel_o
	, dbus_ack_i
	, dbus_adr_o
	, dbus_dat_o
	, dbus_dat_i
	, jump_valid_o
	, jump_dst_o
	, jump_ready_i
	, interrupt_return_o
	, wait_o
	);
	
	output riscv_interrupt_exec_o; 
	output[31:2] epc_o; 
	output[31:2] tvec_o; 
	output sstep_o; 
	output[7:0] sp_waddr_o; 
	output sp_we_o; 
	output[31:0] sp_wdata_o; 
	output ready_o; 
	output dbus_cyc_o; 
	output dbus_stb_o; 
	output dbus_we_o; 
	output[3:0] dbus_sel_o; 
	output[31:2] dbus_adr_o; 
	output[31:0] dbus_dat_o; 
	output jump_valid_o; 
	output[29:0] jump_dst_o; 
	output interrupt_return_o;
	output  wait_o;	
	
	input clk_i; 
	input rst_i; 
	input cmd_loadop3_i; 
	input cmd_signed_i; 
	input cmd_dbus_i; 
	input cmd_dbus_store_i; 
	input cmd_dbus_byte_i; 
	input cmd_dbus_hword_i; 
	input cmd_addsub_i; 
	input cmd_mul_i; 
	input cmd_div_i; 
	input cmd_div_mod_i; 
	input cmd_cmp_i; 
	input cmd_jump_i; 
	input cmd_negate_op2_i; 
	input cmd_and_i; 
	input cmd_xor_i; 
	input cmd_shift_i; 
	input cmd_shift_right_i; 
	input cmd_mul_high_i; 
	input cmd_signed_b_i; 
	input cmd_slt_i; 
	input cmd_csr_i; 
	input csr_x0_i; 
	input[1:0] csr_op_i; 
	input cmd_trap_i; 
	input cmd_tret_i; 
	input[3:0] trap_cause_i; 
	input interrupt_i; 
	input[31:2] epc_i; 
	input[7:0] ext_irq_in; 
	input[3:0] jump_type_i; 
	input[31:0] op1_i; 
	input[31:0] op2_i; 
	input[31:0] op3_i; 
	input[7:0] dst_i; 
	input[11:0] displacement_i; 
	input valid_i; 
	input dbus_ack_i; 
	input[31:0] dbus_dat_i; 
	input jump_ready_i; 
	
	
	wire busy; 
	wire can_execute; 
	
	wire[31:0] alu_result; 
	wire alu_we; 
	wire alu_busy; 
	wire alu_cmp_eq; 
	wire alu_cmp_ug; 
	wire alu_cmp_sg; 
	
	wire loadop3_we; 
	
	reg jump_condition; 
	reg jump_valid='b0; 
	reg[29:0] jump_dst; 
	reg[29:0] jump_dst_r; 
	reg[2:0] cond_reg; 
	
	reg slt_we =0; 
	wire slt_ce; 
	reg[31:0] slt_result=0; 
	reg slt_busy=0; 
	
	wire csr_we; 
	wire csr_ce; 
	wire csr_exception; 
	wire csr_busy; 
	wire[31:0] csr_result; 
	wire mtrap_strobe; 
	wire[3:0] trap_cause; 
	wire csr_tret_exec; 
	
	wire[31:2] mepc; 
	wire[31:2] mtvec; 
	wire mie; 
	
	reg[31:0] adr_reg; 
	reg store_reg; 
	
	wire ex_exception; 
	reg ex_exception_r=0; 
	reg[31:2] epc_reg; 
	wire[31:2] epc_mux; 
	
	reg[31:0] target_address; 
	
	
	wire[31:0] dbus_result; 
	wire dbus_busy; 
	wire dbus_we; 
	wire dbus_misalign; 
	wire s_dbus_cyc_o; 
	wire s_local_cyc_o; 
	wire s_dbus_stb_o; 
	wire s_dbus_we_o; 
	wire[3:0] s_dbus_sel_o; 
	wire s_dbus_ack_i; 
	wire s_local_ack; 
	wire[31:2] s_dbus_adr_o; 
	wire[31:0] s_dbus_dat_o; 
	
	
	wire timer_irq; 
	
	wire[31:0] result_mux; 
	wire result_valid; 
	wire[7:0] result_regaddr; 
	
	reg[7:0] dst_reg; 
	
	reg interrupt_return=0; 
	wire [31:0] alu_we_vect= (alu_we)? 32'hffffffff: 32'b0;
	wire [31:0] csr_we_vect= (csr_we)? 32'hffffffff: 32'b0;
	wire [31:0] slt_we_vect= (slt_we)? 32'hffffffff: 32'b0;
	wire [31:0] dbus_we_vect= (dbus_we)? 32'hffffffff: 32'b0;
	wire [31:0] loadop3_we_vect= (loadop3_we)? 32'hffffffff: 32'b0;
	
	
	
	assign ex_exception = dbus_misalign | csr_exception ;
	
	assign tvec_o = mtvec ;
	assign epc_o = mepc ;
	
	assign busy = alu_busy | dbus_busy | slt_busy | csr_busy | ex_exception | ex_exception_r ;
	assign ready_o = ~busy ;
	assign can_execute = valid_i & ~busy ;
		
	
	rv_alu #
		( .DIVIDER_EN(DIVIDER_EN)
		, .MUL_ARCH  (MUL_ARCH)
		)
		rv_alu_inst
		( .clk_i(clk_i)
		, .rst_i(rst_i)
		, .valid_i(can_execute)
		, .cmd_signed_i(cmd_signed_i)
		, .cmd_addsub_i(cmd_addsub_i)
		, .cmd_mul_i(cmd_mul_i)
		, .cmd_div_i(cmd_div_i)
		, .cmd_div_mod_i(cmd_div_mod_i)
		, .cmd_cmp_i(cmd_cmp_i)
		, .cmd_negate_op2_i(cmd_negate_op2_i)
		, .cmd_and_i(cmd_and_i)
		, .cmd_xor_i(cmd_xor_i)
		, .cmd_shift_i(cmd_shift_i)
		, .cmd_shift_right_i(cmd_shift_right_i)
		, .cmd_mul_high_i(cmd_mul_high_i)
		, .cmd_signed_b_i(cmd_signed_b_i)
		, .op1_i(op1_i)
		, .op2_i(op2_i)
		, .result_o(alu_result)
		, .cmp_eq_o(alu_cmp_eq)
		, .cmp_ug_o(alu_cmp_ug)
		, .cmp_sg_o(alu_cmp_sg)
		, .we_o(alu_we)
		, .busy_o(alu_busy)
		, .wait_o(wait_o)
		);
	
	
	
	
	assign loadop3_we = can_execute & cmd_loadop3_i ;
	
	always @(*)
		begin 
			reg c; 
			case (cond_reg)
				3'b000 :
					begin
						c = alu_cmp_eq; 
					end
				3'b001 :
					begin
						c = ~alu_cmp_eq; 
					end
				3'b100 :
					begin
						c = ~alu_cmp_eq & ~alu_cmp_sg; 
					end
				3'b101 :
					begin
						c = alu_cmp_eq | alu_cmp_sg; 
					end
				3'b110 :
					begin
						c = ~alu_cmp_eq & ~alu_cmp_ug; 
					end
				3'b111 :
					begin
						c = alu_cmp_eq | alu_cmp_ug; 
					end
				default :
					begin
						c = 1'bx; 
					end
			endcase 
			slt_result[0] <= c ; 
			if (cmd_cmp_i == 1'b0)
				begin
					jump_condition <= 1'b1 ; 
				end
			else
				begin
					jump_condition <= c ; 
				end 
		end 



	assign slt_ce = cmd_slt_i & can_execute ;
	
	always @(posedge clk_i)
		begin : slt_ctrl

			if (cmd_cmp_i == 1'b1)
				begin
					cond_reg <= jump_type_i[2:0] ; 
				end 

			slt_we <= slt_ce ; 
			if (slt_we == 1'b1 | rst_i == 1'b1)
				begin
					slt_busy <= 1'b0 ; 
				end
			else if (slt_ce == 1'b1)
				begin
					slt_busy <= 1'b1 ; 
				end  
		end 
	


	always @(op1_i or displacement_i)
		begin : xhdl_12
			reg[11:0] d; 
	
	
			target_address <= op1_i +  {{20{displacement_i[11]}},  displacement_i};
		end 
	
	
	always @(target_address or mtvec or mepc or cmd_tret_i or cmd_trap_i or 
		ex_exception)
		begin
			if (cmd_tret_i == 1'b1)
				begin
					jump_dst <= mepc ; 
				end
			else if (cmd_trap_i == 1'b1 | ex_exception == 1'b1)
				begin
					jump_dst <= mtvec ; 
				end
			else
				begin
					jump_dst <= target_address[31:2] ; 
				end 
		end 
	
	always @(posedge clk_i)
		begin
			if (rst_i == 1'b1)
				begin
					jump_valid <= 1'b0 ; 
					interrupt_return <= 1'b0 ; 
					jump_dst_r <= {30{1'bx}} ; 
					ex_exception_r <= 1'b0 ; 
				end
			else
				begin
					if (jump_valid == 1'b0)
						begin
							jump_dst_r <= jump_dst ; 
							ex_exception_r <= ex_exception ; 
							if ((can_execute == 1'b1 & cmd_jump_i == 1'b1 & jump_condition == 1'b1) | ex_exception == 1'b1)
								begin
									jump_valid <= 1'b1 ; 
								end 
						end
					else if (jump_ready_i == 1'b1)
						begin
							jump_valid <= 1'b0 ; 
							ex_exception_r <= 1'b0 ; 
							interrupt_return <= 1'b0 ; 
						end 
				end  
		end 
	assign jump_valid_o = jump_valid | (can_execute & cmd_jump_i & jump_condition) ;
	assign jump_dst_o = (jump_valid == 1'b1) ? jump_dst_r : jump_dst ;
	assign interrupt_return_o = interrupt_return ;
	
	
	assign s_dbus_ack_i = (s_dbus_cyc_o) ? dbus_ack_i : 1'b0 ;
	assign dbus_adr_o = s_dbus_adr_o ;
	assign dbus_cyc_o = s_dbus_cyc_o ;
	assign dbus_stb_o = s_dbus_stb_o & s_dbus_cyc_o ;
	assign dbus_we_o = s_dbus_we_o ;
	assign dbus_dat_o = s_dbus_dat_o ;
	assign dbus_sel_o = s_dbus_sel_o ;
	
	
	
	rv_db #
		(
		.ENABLE_LOCALMAP(USE_RISCV),
		.DB_RMW       (DB_RMW)
		)
		rv_dbus_inst
		(. clk_i(clk_i)
		, .rst_i(rst_i)
		, .valid_i(can_execute)
		, .cmd_dbus_i(cmd_dbus_i)
		, .cmd_dbus_store_i(cmd_dbus_store_i)
		, .cmd_dbus_byte_i(cmd_dbus_byte_i)
		, .cmd_dbus_hword_i(cmd_dbus_hword_i)
		, .cmd_signed_i(cmd_signed_i)
		, .addr_i(target_address)
		, .wdata_i(op2_i)
		, .rdata_o(dbus_result)
		, .busy_o(dbus_busy)
		, .we_o(dbus_we)
		, .misalign_o(dbus_misalign)
		, .dbus_cyc_o(s_dbus_cyc_o)
		, .dbus_stb_o(s_dbus_stb_o)
		, .dbus_we_o(s_dbus_we_o)
		, .dbus_sel_o(s_dbus_sel_o)
		, .dbus_ack_i(s_dbus_ack_i)
		, .dbus_adr_o(s_dbus_adr_o)
		, .dbus_dat_o(s_dbus_dat_o)
		, .dbus_dat_i(dbus_dat_i)
	
	); 
	

	assign csr_ce = cmd_csr_i & can_execute ;
	assign mtrap_strobe = (cmd_trap_i & can_execute) | ex_exception ;
	assign trap_cause = (dbus_misalign == 1'b1 & store_reg == 1'b0) ? 'h4 : (dbus_misalign == 1'b1 & store_reg == 1'b1) ? 'h6 : (csr_exception == 1'b1) ? 'h2 : trap_cause_i ;
	
	always @(posedge clk_i)
		begin
			if (can_execute == 1'b1)
				begin
					epc_reg <= epc_i ; 
					adr_reg <= target_address ; 
					store_reg <= cmd_dbus_store_i ; 
				end  
		end
	
	
	assign epc_mux = (ex_exception == 1'b1) ? epc_reg : epc_i ;
	assign csr_tret_exec = cmd_tret_i & can_execute ;
	
	
	
	
	rv_ctrl_unit #
		(
		.DIVIDER_EN(DIVIDER_EN),
		.MUL_ARCH  (MUL_ARCH)
		)
		rv_ctrl_inst
		(. op1_i(op1_i)
		, .wdata_o(csr_result)
		, .we_o(csr_we)
		, .csr_exception(csr_exception)
		, .csr_adr(displacement_i)
		, .ce_i(csr_ce)
		, .busy_o(csr_busy)
		, .csr_x0_i(csr_x0_i)
		, .csr_op_i(csr_op_i)
		, .clk_i(clk_i)
		, .rst_i(rst_i)
		, .mtvec_o(mtvec)
		, .mepc_o(mepc)
		, .sstep_o(sstep_o)
		, .mcause_i(trap_cause)
		, .mepc_i(epc_mux)
		, .mtrap_strobe_i(mtrap_strobe)
		, .adr_i(adr_reg)
		, .cmd_tret_i(csr_tret_exec)
		
		, .interrupt_ack_i(interrupt_i)
		, .ext_irq_in(ext_irq_in)
		, .interrupt_exec_o(riscv_interrupt_exec_o)
		); 
	



	
	
	assign result_mux   = (alu_result & alu_we_vect) |(op3_i & loadop3_we_vect) |(dbus_result & dbus_we_vect) |(csr_result & csr_we_vect) |(slt_result & slt_we_vect) ;
	assign result_valid = (                 alu_we   |              loadop3_we  |                    dbus_we  |                   csr_we  |                   slt_we) ;
	

	always @(posedge clk_i)
		if (can_execute == 1'b1)
			dst_reg <= dst_i ; 
	
	assign result_regaddr = (can_execute == 1'b1) ? dst_i : dst_reg ;
	assign sp_we_o = result_valid ;
	assign sp_waddr_o = result_regaddr ;
	assign sp_wdata_o = result_mux ;
	
endmodule
