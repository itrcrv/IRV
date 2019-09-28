`timescale 1ns/1ps

module riscv_decode_k
	( clk_i
	, rst_i
	, word_i
	, next_ip_i
	, valid_i
	, jump_valid_i
	, ready_o
	, fencei_o
	, interrupt_valid_i
	, interrupt_vector_i
	, interrupt_ready_o
	, sp_raddr1_o
	, sp_rdata1_i
	, sp_raddr2_o
	, sp_rdata2_i
	, displacement_o
	, ready_i
	, valid_o
	, cmd_loadop3_o
	, cmd_signed_o
	, cmd_dbus_o
	, cmd_dbus_store_o
	, cmd_dbus_byte_o
	, cmd_dbus_hword_o
	, cmd_addsub_o
	, cmd_mul_o
	, cmd_div_o
	, cmd_div_mod_o
	, cmd_cmp_o
	, cmd_jump_o
	, cmd_negate_op2_o
	, cmd_and_o
	, cmd_xor_o
	, cmd_shift_o
	, cmd_shift_right_o
	, cmd_mul_high_o
	, cmd_signed_b_o
	, cmd_slt_o
	, cmd_csr_o
	, csr_x0_o
	, csr_op_o
	, cmd_trap_o
	, cmd_tret_o
	, trap_cause_o
	, interrupt_o
	, epc_o
	, epc_i
	, tvec_i
	, sstep_i
	, jump_type_o
	, op1_o
	, op2_o
	, op3_o
	, dst_o
	);
	
	// included from package riscv_decodeutil
	input clk_i; 
	input rst_i; 
	input[31:0] word_i; // actual instruction to decode
	input[29:0] next_ip_i; // ip (PC) of next instruction
	input valid_i; // input valid
	input jump_valid_i; 
	output ready_o; // decode stage ready to decode next instruction
	output reg fencei_o; // FENCE.I Instruction 
	
	input interrupt_valid_i; 
	input[2:0] interrupt_vector_i; 
	output interrupt_ready_o; 
	output[7:0] sp_raddr1_o; 
	input[31:0] sp_rdata1_i; 
	output[7:0] sp_raddr2_o;
	input[31:0] sp_rdata2_i; 
	output reg [11:0] displacement_o; //TH Pass Load/Store displacement to execute stage
	input ready_i; // ready signal from execute stage
	output valid_o; // output status valid
	output reg cmd_loadop3_o; 
	output reg cmd_signed_o; 
	output reg cmd_dbus_o; 
	output reg cmd_dbus_store_o; 
	output reg cmd_dbus_byte_o; 
	output reg cmd_dbus_hword_o; // TH
	output reg cmd_addsub_o; 
	output reg cmd_mul_o; 
	output reg cmd_div_o; 
	output reg cmd_div_mod_o; 
	output reg cmd_cmp_o; 
	output reg cmd_jump_o; 
	output reg cmd_negate_op2_o; 
	output reg cmd_and_o; 
	output reg cmd_xor_o; 
	output reg cmd_shift_o; 
	output reg cmd_shift_right_o; 
	output reg cmd_mul_high_o; // TH: Multiplier bits
	output reg cmd_signed_b_o; // Multiplier operand b signed
	output reg cmd_slt_o; // TH: RISC-V SLT/SLTU command
	output reg cmd_csr_o; 
	output reg csr_x0_o; // should be set when rs field is x0
	output reg [1:0] csr_op_o; // lower bits of funct3
	output reg cmd_trap_o; // TH: Execute trap
	output reg cmd_tret_o; // TH: Execute trap returen
	output reg [3:0] trap_cause_o; // TH: Trap/Interrupt cause
	output reg interrupt_o; // Trap is interrupt
	output reg [31:2] epc_o; 
	input      [31:2] epc_i; 
	input      [31:2] tvec_i; 
	input sstep_i; 
	output reg [3:0] jump_type_o; 
	output reg [31:0] op1_o; 
	output reg [31:0] op2_o; 
	output reg [31:0] op3_o; 
	output     [7:0] dst_o; 
	
	// RISCV instruction fields
	wire [6:2] opcode; 
	wire [4:0] rd; 
	wire [4:0] rs1; 
	wire [4:0] rs2; 
	wire [2:0] funct3; 
	wire [6:0] funct7; 
	wire [29:0] current_ip; 
	// Signals related to pipeline control
	wire downstream_busy; 
	reg self_busy; 
	wire busy; 
	reg valid_out; 
	// Signals related to interrupt handling
	reg interrupt_ready; 
	// Signals related to RD operand decoding
	wire [7:0] rd1; 
	reg[7:0] rd1_reg; 
	wire [7:0] rd2; 
	reg[7:0] rd2_reg; 
	parameter[1:0] undef = 0; 
	parameter[1:0] Registered = 1; 
	parameter[1:0] imm = 2; 
	reg[1:0] rd1_select; 
	reg[31:0] rd1_direct; 
	reg[1:0] rd2_select; 
	reg[31:0] rd2_direct; 
	reg rd1_zero; // TH: Buffered zero address flags
	reg rd2_zero; // TH: Buffered zero address flags
	reg[7:0] dst_out; 
	wire [7:0] radr1_out; 
	wire [7:0] radr2_out; 
	reg trap_on_next; // '1' -> Raise a break trap after the next instruction
	reg trap_on_current; // '1' break trap now ...
	
	
	
	//????
	parameter[6:2] op_imm = 5'b00100; 
	parameter[6:2] op_op = 5'b01100; 
	parameter[6:2] op_jal = 5'b11011; 
	parameter[6:2] op_jalr = 5'b11001; 
	parameter[6:2] op_load = 5'b00000; 
	parameter[6:2] op_store = 5'b01000; 
	parameter[6:2] op_branch = 5'b11000; 
	parameter[6:2] op_lui = 5'b01101; 
	parameter[6:2] op_auipc = 5'b00101; 
	parameter[6:2] op_system = 5'b11100; 
	parameter[6:2] op_miscmem = 5'b00011; 
	parameter[3:0] rv_imm = 0; 
	parameter[3:0] rv_op = 1; 
	parameter[3:0] rv_jal = 2; 
	parameter[3:0] rv_jalr = 3; 
	parameter[3:0] rv_load = 4; 
	parameter[3:0] rv_store = 5; 
	parameter[3:0] rv_branch = 6; 
	parameter[3:0] rv_lui = 7; 
	parameter[3:0] rv_auipc = 8; 
	parameter[3:0] rv_system = 9; 
	parameter[3:0] rv_miscmem = 10; 
	parameter[3:0] rv_invalid = 11; 
	parameter[2:0] add = 3'b000; 
	parameter[2:0] slt = 3'b010; 
	parameter[2:0] sltu = 3'b011; 
	parameter[2:0] f_xor = 3'b100; 
	parameter[2:0] f_or = 3'b110; 
	parameter[2:0] f_and = 3'b111; 
	parameter[2:0] sl = 3'b001; 
	parameter[2:0] sr = 3'b101; 
	parameter[6:0] mulext = 7'b0000001; 
	parameter[2:0] mul = 3'b000; 
	parameter[2:0] mulh = 3'b001; 
	parameter[2:0] div = 3'b100; 
	parameter[2:0] divu = 3'b101; 
	parameter[2:0] f_rem = 3'b110; 
	parameter[2:0] remu = 3'b111; 
	
	
	// Decoder FSM state
	parameter[1:0] regular = 0; 
	parameter[1:0] continuecjmp = 1; 
	parameter[1:0] halt = 2; 
	reg[1:0] state; 
	// debug PC only to make debugging more comfortable
	// will be optimized away in synthesis
	wire [31:0] debug_pc; 
	
	function [32 - 1:0]get_sb_immediate;
		input[32 - 1:0] instr; 
		
		reg[32 - 1:0] temp; 
		reg[12:0] t2; 
		
		begin
			t2 = {instr[31], instr[7], instr[30:25], instr[11:8], 1'b0}; 
			temp = { {19{ t2[12]}} , t2};
			get_sb_immediate = temp; 
		end
	endfunction
	
	/*
	function [11:0]get_i_displacement;
	input[32 - 1:0] instr; 
	reg[11:0] t; 
	begin
	t = instr[31:20]; 
	get_i_displacement = t; 
	end
	endfunction
	
	function [31:0]get_uj_immediate;
	input[31:0] instr; 
	reg[20:0] t2; 
	
	begin
	t2 = ({instr[31], instr[19:12], instr[20], instr[30:21], 1'b0}); 
	get_uj_immediate = {12'b0, t2}; 
	end
	endfunction   */
	
	function [3:0]decode_op;
		input[6:2] opcode; 
		
		begin
			case (opcode)
				op_imm :
					begin
						decode_op = rv_imm; 
					end
				op_op :
					begin
						decode_op = rv_op; 
					end
				op_jal :
					begin
						decode_op = rv_jal; 
					end
				op_jalr :
					begin
						decode_op = rv_jalr; 
					end
				op_load :
					begin
						decode_op = rv_load; 
					end
				op_store :
					begin
						decode_op = rv_store; 
					end
				op_branch :
					begin
						decode_op = rv_branch; 
					end
				op_lui :
					begin
						decode_op = rv_lui; 
					end
				op_auipc :
					begin
						decode_op = rv_auipc; 
					end
				op_system :
					begin
						decode_op = rv_system; 
					end
				op_miscmem :
					begin
						decode_op = rv_miscmem; 
					end
				default :
					begin
						decode_op = rv_invalid; 
					end
			endcase 
		end
	endfunction
	
	initial
		begin
			self_busy <= 1'b0;
			valid_out <= 1'b0;
			interrupt_ready <= 1'b0;
			trap_on_next <= 1'b0;
			trap_on_current <= 1'b0;
			state <= regular;
		end
	
	// extract instruction fields
	assign opcode = word_i[6:2] ;
	assign rd = word_i[11:7] ;
	assign funct3 = word_i[14:12] ;
	assign rs1 = word_i[19:15] ;
	assign rs2 = word_i[24:20] ;
	assign funct7 = word_i[31:25] ;
	// decode Register addresses
	assign rd1 = {3'b000, rs1} ;
	assign rd2 = {3'b000, rs2} ;
	// Pipeline control
	assign downstream_busy = valid_out & ~ready_i ;
	assign busy = downstream_busy | self_busy ;
	// Instruction pointer  
	assign current_ip = next_ip_i - 1 ;
	assign debug_pc = {current_ip, 2'b00} ;
	// Control outputs
	assign valid_o = valid_out ;
	assign dst_o = dst_out ;
	assign ready_o = ~busy ;
	assign interrupt_ready_o = interrupt_ready ;
	
	always @(posedge clk_i)
		begin : xhdl_18
			reg [31:0] branch_target; 
			reg [31:0] u_immed; 
			reg [11:0] displacement; 
			reg t_valid; 
			reg trap; 
			reg not_implemented; 
			reg [3:0] optype; 
			if (rst_i == 1'b1)
				begin
					valid_out <= 1'b0 ; 
					self_busy <= 1'b0 ; 
					state <= regular ; 
					interrupt_ready <= 1'b0 ; 
					// all the following values are only initalized for simulation.
					cmd_loadop3_o <= 1'b0 ; 
					cmd_signed_o <= 1'b0 ; 
					cmd_dbus_o <= 1'b0 ; 
					cmd_dbus_store_o <= 1'b0 ; 
					cmd_dbus_byte_o <= 1'b0 ; 
					cmd_dbus_hword_o <= 1'b0 ; // TH
					cmd_addsub_o <= 1'b0 ; 
					cmd_negate_op2_o <= 1'b0 ; 
					cmd_mul_o <= 1'b0 ; 
					cmd_div_o <= 1'b0 ; 
					cmd_div_mod_o <= 1'b0 ; 
					cmd_cmp_o <= 1'b0 ; 
					cmd_jump_o <= 1'b0 ; 
					cmd_and_o <= 1'b0 ; 
					cmd_xor_o <= 1'b0 ; 
					cmd_shift_o <= 1'b0 ; 
					cmd_shift_right_o <= 1'b0 ; 
					rd1_select <= undef ; 
					rd1_direct <= {32'b0} ; 
					rd2_select <= undef ; 
					rd2_direct <= {32'b0} ; 
					op3_o <= {32'b0} ; 
					jump_type_o <= {4'b0} ; 
					dst_out <= {8'b0} ; 
					displacement = 12'b0; 
					cmd_mul_high_o <= 1'b0 ; 
					cmd_signed_b_o <= 1'b0 ; 
					cmd_slt_o <= 1'b0 ; 
					cmd_csr_o <= 1'b0 ; 
					cmd_trap_o <= 1'b0 ; 
					cmd_tret_o <= 1'b0 ; 
					interrupt_o <= 1'b0 ; 
					trap_on_next <= 1'b0 ; 
					trap_on_current <= 1'b0 ;
					epc_o <= 32'b0;
					trap_cause_o <= 4'h0 ;
					csr_x0_o <= 1'b0 ; 
				end
			else
				begin
					fencei_o <= 1'b0 ; // clear fencei_o always after one cycle
					if (jump_valid_i == 1'b1)
						begin
							// When exeuction stage exeuctes jump do nothing
							valid_out <= 1'b0 ; 
							self_busy <= 1'b0 ; 
							state <= regular ; 
						end
					else if (downstream_busy == 1'b0)
						begin
							case (state)
								regular :
									begin
										cmd_loadop3_o <= 1'b0 ; 
										cmd_signed_o <= 1'b0 ; 
										cmd_dbus_o <= 1'b0 ; 
										cmd_dbus_store_o <= 1'b0 ; 
										cmd_dbus_byte_o <= 1'b0 ; 
										cmd_dbus_hword_o <= 1'b0 ; // TH
										cmd_addsub_o <= 1'b0 ; 
										cmd_negate_op2_o <= 1'b0 ; 
										cmd_mul_o <= 1'b0 ; 
										cmd_div_o <= 1'b0 ; 
										cmd_div_mod_o <= 1'b0 ; 
										cmd_cmp_o <= 1'b0 ; 
										cmd_jump_o <= 1'b0 ; 
										cmd_and_o <= 1'b0 ; 
										cmd_xor_o <= 1'b0 ; 
										cmd_shift_o <= 1'b0 ; 
										cmd_shift_right_o <= 1'b0 ; 
										cmd_slt_o <= 1'b0 ; 
										cmd_csr_o <= 1'b0 ; 
										cmd_trap_o <= 1'b0 ; 
										cmd_tret_o <= 1'b0 ; 
										interrupt_o <= 1'b0 ; 
										jump_type_o <= 4'b0000 ; 
										dst_out <= {8{1'b0}} ; // defaults to register 0, which is never read
										displacement = {12'b0};
										t_valid = 1'b0; 
										not_implemented = 1'b0; 
										trap = 1'b0; 
										
										if (valid_i == 1'b1)
											begin
												// single step trap propagation pipeline             
												trap_on_current <= trap_on_next ; 
												trap_on_next <= 1'b0 ; 
												if (interrupt_valid_i == 1'b1)
													begin
														t_valid = 1'b1; 
														interrupt_o <= 1'b1 ; 
														//trap_cause_o <= X"4"; -- TODO: adapt cause based on interrupt source
														cmd_trap_o <= 1'b1 ; 
														cmd_jump_o <= 1'b1 ; 
														// Clear pending single steps in case of an interrupt
														trap_on_next <= 1'b0 ; 
														trap_on_current <= 1'b0 ; 
													end
												else if (trap_on_current == 1'b1)
													begin
														// execute Single step trap
														cmd_trap_o <= 1'b1 ; 
														cmd_jump_o <= 1'b1 ; 
														trap_cause_o <= 4'h3 ; 
														t_valid = 1'b1; 
													end
												else if (word_i[1:0] == 2'b11)
													begin
														// all RV32IM instructions have the lower bits set to 11                                   
														optype = decode_op(opcode); 
														case (optype)
															rv_imm, rv_op :
																begin
																	rd1_select <= Registered ; 
																	dst_out <= {3'b000, rd} ; 
																	if ((opcode[5]) == 1'b1)
																		begin
																			// OP_OP...
																			rd2_select <= Registered ; 
																		end
																	else
																		begin
																			//OP_IMM
																			rd2_direct <= {{20{word_i[31]}}, word_i[31:20]} ; // xfunx //rd2_direct <= std_logic_vector(get_i_immediate(word_i)) ;
																			rd2_select <= imm ; 
																		end 
																	if (funct7 == mulext & optype == rv_op)
																		begin
																			// M extension
																			if ((funct3[2]) == 1'b0)
																				begin
																					cmd_mul_o <= 1'b1 ; 
																					case (funct3[1:0])
																						2'b00 :
																							begin
																								// mul
																								cmd_mul_high_o <= 1'b0 ; 
																								cmd_signed_o <= 1'b0 ; 
																								cmd_signed_b_o <= 1'b0 ; 
																							end
																						2'b11 :
																							begin
																								// mulhu
																								cmd_mul_high_o <= 1'b1 ; 
																								cmd_signed_o <= 1'b0 ; 
																								cmd_signed_b_o <= 1'b0 ; 
																							end
																						2'b01 :
																							begin
																								// mulh (both operands signed)
																								cmd_mul_high_o <= 1'b1 ; 
																								cmd_signed_o <= 1'b1 ; 
																								cmd_signed_b_o <= 1'b1 ; 
																							end
																						2'b10 :
																							begin
																								// mulhsu (signed, unsigned)
																								cmd_mul_high_o <= 1'b1 ; 
																								cmd_signed_o <= 1'b1 ; 
																								cmd_signed_b_o <= 1'b0 ; 
																							end
																						default :
																							begin
																								not_implemented = 1'b1; 
																							end
																					endcase 
																				end
																			else
																				begin
																					cmd_div_o <= 1'b1 ; 
																					cmd_div_mod_o <= funct3[1] ; 
																					cmd_signed_o <= ~funct3[0] ; 
																				end 
																		end
																	else
																		begin
																			case (funct3)
																				add :
																					begin
																						cmd_addsub_o <= 1'b1 ; 
																						if ((opcode[5]) == 1'b1)
																							begin
																								cmd_negate_op2_o <= word_i[30] ; 
																							end 
																					end
																				f_and :
																					begin
																						cmd_and_o <= 1'b1 ; 
																					end
																				f_xor :
																					begin
																						cmd_xor_o <= 1'b1 ; 
																					end
																				f_or :
																					begin
																						cmd_and_o <= 1'b1 ; 
																						cmd_xor_o <= 1'b1 ; 
																					end
																				sl :
																					begin
																						cmd_shift_o <= 1'b1 ; 
																					end
																				sr :
																					begin
																						cmd_shift_o <= 1'b1 ; 
																						cmd_shift_right_o <= 1'b1 ; 
																						cmd_signed_o <= word_i[30] ; 
																					end
																				slt :
																					begin
																						cmd_cmp_o <= 1'b1 ; 
																						cmd_negate_op2_o <= 1'b1 ; // needed by ALU comparator to work correctly
																						cmd_slt_o <= 1'b1 ; 
																						jump_type_o <= 4'b0100 ; 
																					end
																				sltu :
																					begin
																						cmd_cmp_o <= 1'b1 ; 
																						cmd_negate_op2_o <= 1'b1 ; // needed by ALU comparator to work correctly
																						cmd_slt_o <= 1'b1 ; 
																						jump_type_o <= 4'b0110 ; 
																					end
																				default :
																					begin
																					end
																			endcase 
																		end 
																	t_valid = 1'b1; 
																end
															rv_jal :
																begin
																	rd1_select <= imm ; 
																	// xfunx //rd1_direct <= std_logic_vector(signed({current_ip, 2'b00}) + get_uj_immediate(word_i)) ; 
																	rd1_direct <= {current_ip, 2'b00} + ({12'b0, word_i[31], word_i[19:12], word_i[20], word_i[30:21], 1'b0}); 
																	cmd_jump_o <= 1'b1 ; 
																	cmd_loadop3_o <= 1'b1 ; 
																	op3_o <= {next_ip_i, 2'b00} ; 
																	dst_out <= {3'b000, rd} ; 
																	t_valid = 1'b1; 
																end
															rv_jalr :
																begin
																	rd1_select <= Registered ; 
																	cmd_jump_o <= 1'b1 ; 
																	cmd_loadop3_o <= 1'b1 ; 
																	op3_o <= {next_ip_i, 2'b00} ; 
																	dst_out <= {3'b000, rd} ; 
																	displacement = word_i[31:20]; // xfunx //displacement = get_i_displacement(word_i);
																	t_valid = 1'b1; 
																end
															rv_branch :
																begin
																	// xfunx //branch_target = std_logic_vector(signed({current_ip, 2'b00}) + get_sb_immediate(word_i)); 
																	branch_target = {current_ip, 2'b00} + get_sb_immediate(word_i); 
																	rd1_select <= Registered; 
																	rd2_select <= Registered; 
																	jump_type_o <= {1'b0, funct3} ; // "reuse" lxp jump_type for the funct3 field, see generated coding in lxp32_execute
																	cmd_cmp_o <= 1'b1 ; 
																	cmd_negate_op2_o <= 1'b1 ; // needed by ALU comparator to work correctly
																	t_valid = 1'b1; 
																	self_busy <= 1'b1 ; 
																	state <= continuecjmp ; 
																end
															rv_load :
																begin
																	rd1_select <= Registered; 
																	displacement = word_i[31:20]; // xfunx //displacement = get_i_displacement(word_i);
																	cmd_dbus_o <= 1'b1 ; 
																	dst_out <= {3'b000, rd} ; 
																	if (funct3[1:0] == 2'b00)
																		begin
																			// Byte access
																			cmd_dbus_byte_o <= 1'b1 ; 
																		end
																	else if (funct3[1:0] == 2'b01)
																		begin
																			//  16 BIT (H) access
																			cmd_dbus_hword_o <= 1'b1 ; 
																		end 
																	cmd_signed_o <= ~funct3[2] ; 
																	t_valid = 1'b1; 
																end
															rv_store :
																begin
																	rd1_select <= Registered;
																	displacement = {word_i[31:25], word_i[11:7] }; // xfunx //displacement = get_s_displacement(word_i); 
																	rd2_select <= Registered; 
																	cmd_dbus_o <= 1'b1 ; 
																	cmd_dbus_store_o <= 1'b1 ; 
																	if (funct3[1:0] == 2'b00)
																		begin
																			// Byte access
																			cmd_dbus_byte_o <= 1'b1 ; 
																		end
																	else if (funct3[1:0] == 2'b01)
																		begin
																			//  16 BIT (H) access
																			cmd_dbus_hword_o <= 1'b1 ; 
																		end // TODO: Implement 16 BIT (H) instructons
																	t_valid = 1'b1; 
																end
															rv_lui, rv_auipc :
																begin
																	// we will use the ALU to calculate the result
																	// this saves an adder and time
																	// xfunx //u_immed = get_u_immediate(word_i);
																	u_immed = {word_i[31:12], 12'b0};
																	
																	rd2_select <= imm ; 
																	rd2_direct <= u_immed ;  // xfunx //rd2_direct <= std_logic_vector(u_immed) ; 
																	rd1_select <= imm ; 
																	cmd_addsub_o <= 1'b1 ; 
																	if ((word_i[5]) == 1'b1)
																		begin
																			// LUI
																			rd1_direct <= {32{1'b0}} ; 
																		end
																	else
																		begin
																			rd1_direct <= {current_ip, 2'b00} ; // xfunx //rd1_direct <= {std_logic_vector(current_ip), 2'b00} ; 
																		end 
																	dst_out <= {3'b000, rd} ; 
																	t_valid = 1'b1; 
																end
															rv_system :
																begin
																	if (funct3 == 3'b000)
																		begin
																			// ECALL EBREAK
																			cmd_jump_o <= 1'b1 ; 
																			interrupt_o <= 1'b0 ; 
																			case (word_i[21:20])
																				2'b01 :
																					begin
																						// EBREAK
																						trap_cause_o <= 4'h3 ; 
																						trap = 1'b1; 
																						t_valid = 1'b1; 
																					end
																				2'b00 :
																					begin
																						// ECALL
																						trap_cause_o <= 4'hB ; 
																						trap = 1'b1; 
																						t_valid = 1'b1; 
																					end
																				2'b10 :
																					begin
																						// XRET
																						cmd_tret_o <= 1'b1 ; 
																						t_valid = 1'b1; 
																						if (sstep_i == 1'b1)
																							begin
																								trap_on_next <= 1'b1 ; 
																							end 
																					end
																				default :
																					begin
																					end
																				// nothing...
																			endcase 
																			cmd_trap_o <= trap ; 
																		end
																	else
																		begin
																			cmd_csr_o <= 1'b1 ; 
																			csr_op_o <= funct3[1:0] ; 
																			if (rs1 == 5'b00000)
																				begin
																					csr_x0_o <= 1'b1 ; 
																				end
																			else
																				begin
																					csr_x0_o <= 1'b0 ; 
																				end 
																			displacement = word_i[31:20]; // CSR address
																			if ((funct3[2]) == 1'b1)
																				begin
																					rd1_select <= imm ; 
																					rd1_direct <= {{29{word_i[19]}}, word_i[19:15]} ;// xfunx // rd1_direct <= std_logic_vector(resize(unsigned(word_i[19:15]), 32));
																				end
																			else
																				begin
																					rd1_select <= Registered; 
																				end 
																			dst_out <= {3'b000, rd} ; 
																			t_valid = 1'b1; 
																		end 
																end
															rv_miscmem :
																begin
																	case (funct3)
																		3'b000 :
																			begin
																				// FENCE: currently like a NOP
																				t_valid = 1'b1; 
																			end
																		3'b001 :
																			begin
																				// FENCE.I
																				// we just jump to next_ip, this will effectivly flush the pipeline and the prefetch buffer
																				rd1_select <= imm ; 
																				// xfunx //rd1_direct <= std_logic_vector(signed({next_ip_i, 2'b00})) ; 
																				rd1_direct <= {next_ip_i, 2'b00};
																				cmd_jump_o <= 1'b1 ; 
																				fencei_o <= 1'b1 ; 
																				t_valid = 1'b1; 
																			end
																		default :
																			begin
																				not_implemented = 1'b1; 
																			end
																	endcase 
																end
															rv_invalid :
																begin
																	not_implemented = 1'b1; 
																end
														endcase 
													end
												else
													begin
														not_implemented = 1'b1; 
													end 
												if (t_valid == 1'b0 | not_implemented == 1'b1)
													begin
														// illegal opcode
														cmd_jump_o <= 1'b1 ; 
														interrupt_o <= 1'b0 ; 
														trap_cause_o <= 'h2 ; 
														cmd_trap_o <= 1'b1 ; 
														valid_out <= 1'b1 ; 
													end
												else
													begin
														valid_out <= t_valid ; 
													end 
												// the epc_o register is always set
												// In case of an exception downstream the pipeline this register can be copied
												// to the CSR register.
												epc_o <= current_ip;  // xfunx //epc_o <= std_logic_vector(current_ip) ;
											end // if valid_i='1'
									end
								continuecjmp :
									begin
										rd1_select <= imm ; 
										rd1_direct <= branch_target ; 
										valid_out <= 1'b1 ; 
										cmd_jump_o <= 1'b1 ; 
										self_busy <= 1'b0 ; 
										state <= regular ; 
									end
								halt :
									begin
										if (interrupt_valid_i == 1'b1)
											begin
												self_busy <= 1'b0 ; 
												state <= regular ; 
											end 
									end
							endcase 
						end 
				end 
			displacement_o <= displacement ;  
		end 
	
	// Operand handling
	always @(posedge clk_i)
		begin
			if (busy == 1'b0)
				begin
					rd1_reg <= rd1 ; 
					if (rd1[4:0] == 5'b00000)
						begin
							rd1_zero <= 1'b1 ; 
						end
					else
						begin
							rd1_zero <= 1'b0 ; 
						end 
					rd2_reg <= rd2 ; 
					if (rd2[4:0] == 5'b00000)
						begin
							rd2_zero <= 1'b1 ; 
						end
					else
						begin
							rd2_zero <= 1'b0 ; 
						end 
				end  
		end 
	assign radr1_out = (busy == 1'b1) ? rd1_reg : rd1 ;
	assign sp_raddr1_o = radr1_out ;
	assign radr2_out = (busy == 1'b1) ? rd2_reg : rd2 ;
	assign sp_raddr2_o = radr2_out ;
	
	//Operand 1 multiplexer
	always @(rd1_direct or rd1_select or sp_rdata1_i or rd1_zero)
		begin : xhdl_25
			reg[31:0] rdata; 
			if (rd1_select == imm)
				begin
					op1_o <= rd1_direct ; 
				end
			else
				begin
					if (rd1_zero == 1'b1)
						begin
							op1_o <= 'h00000000 ; 
						end
					else
						begin
							op1_o <= sp_rdata1_i ; 
						end 
				end 
		end 
	
	//operand 2 multiplexer
	always @(rd2_direct or rd2_select or sp_rdata2_i or rd2_zero)
		begin
			if (rd2_select == imm)
				begin
					op2_o <= rd2_direct ; 
				end
			else
				begin
					if (rd2_zero == 1'b1)
						begin
							op2_o <= 'h00000000 ; 
						end
					else
						begin
							op2_o <= sp_rdata2_i ; 
						end 
				end 
		end 
endmodule
