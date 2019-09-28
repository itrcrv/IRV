//////////////////////////////////////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////////////////////////////////////
//
//-------------------------------------------------------------------
// Arithmetic logic unit
//
// Part of the LXP32 CPU
//
// Copyright (c) 2016 by Alex I. Kuznetsov
//
// Performs arithmetic and logic operations.
// TH: Added Support for high word of multipler result 
// Attention: Only works with mul_dsp architecture currently !!!!
//-------------------------------------------------------------------
`timescale 1ns/1ps
module rv_alu #
	(
	parameter           DIVIDER_EN    = 0,     /*                   */
	parameter           MUL_ARCH      = "dsp" /* DSP / Multi Stage */
	)
	( clk_i, rst_i
	, valid_i, cmd_signed_i, cmd_addsub_i, cmd_mul_i
	, cmd_div_i, cmd_div_mod_i, cmd_cmp_i, cmd_negate_op2_i
	, cmd_and_i, cmd_xor_i, cmd_shift_i, cmd_shift_right_i
	, cmd_mul_high_i, cmd_signed_b_i, op1_i, op2_i, result_o
	, cmp_eq_o, cmp_ug_o, cmp_sg_o, we_o, busy_o, wait_o
	);
	
	input clk_i; 
	input rst_i; 
	input valid_i; 
	input cmd_signed_i; 
	input cmd_addsub_i; 
	input cmd_mul_i; 
	input cmd_div_i; 
	input cmd_div_mod_i; 
	input cmd_cmp_i; 
	input cmd_negate_op2_i; 
	input cmd_and_i; 
	input cmd_xor_i; 
	input cmd_shift_i; 
	input cmd_shift_right_i; 
	input cmd_mul_high_i; 
	input cmd_signed_b_i; 
	input[31:0] op1_i; 
	input[31:0] op2_i; 
	
	output[31:0] result_o;
	output reg cmp_eq_o; 
	output cmp_ug_o; 
	output cmp_sg_o; 
	output we_o; 
	output busy_o; 
	output wait_o; 
	
	
	wire[31:0] addend1; 
	wire[31:0] addend2; 
	wire[32:0] adder_result; 
	wire adder_we; 
	reg cmp_carry; 
	reg cmp_s1; 
	reg cmp_s2; 
	wire[31:0] logic_result; 
	wire logic_we; 
	wire[31:0] mul_result; 
	wire[31:0] mul_result_high; 
	wire[31:0] mul_result_low; 
	wire mul_ce; 
	wire mul_we; 
	reg mul_high; 
	wire[31:0] div_result; 
	wire div_ce; 
	wire div_we; 
	wire[31:0] shift_result; 
	wire shift_ce; 
	wire shift_we; 
	wire result_we; 
	reg busy= 1'b0; 
	wire[31:0] cmd_and_vect; 
	wire[31:0] cmd_xor_vect; 
	wire[31:0] adder_we_vect; 
	wire[31:0] logic_we_vect; 
	wire[31:0] mul_we_vect; 
	wire[31:0] div_we_vect; 
	wire[31:0] shift_we_vect; 
	
	
	assign addend1 = op1_i ;
	assign addend2 = (cmd_negate_op2_i == 1'b0) ? op2_i: ((~op2_i) +1'b1);
	assign adder_result = ({1'b0, addend1}) + ({1'b0, addend2}) ;  //Farahany edithed
	assign adder_we = cmd_addsub_i & valid_i ;
	
	always @(posedge clk_i)
		if (rst_i)
			begin
				cmp_eq_o     <= 1'b0 ;
				cmp_carry    <= 1'b0 ;
				cmp_s1       <= 1'b0 ; 
				cmp_s2       <= 1'b0 ;
			end
		else if (valid_i & cmd_cmp_i)
			begin
				if (op1_i == op2_i)
					cmp_eq_o <= 1'b1 ; 
				else
					cmp_eq_o <= 1'b0 ; 
				cmp_carry    <= adder_result[32] ; 
				cmp_s1       <= op1_i[31] ; 
				cmp_s2       <= op2_i[31] ; 
			end  
	
	assign cmp_ug_o     = cmp_carry & ~cmp_eq_o ;
	assign cmp_sg_o     = ((cmp_s1 & cmp_s2 & cmp_carry) | (~cmp_s1 & ~cmp_s2 & cmp_carry) | (~cmp_s1 & cmp_s2)) & ~cmp_eq_o ;
	assign cmd_and_vect = (cmd_and_i) ? {32'hffffffff} : {32'b0} ;
	assign cmd_xor_vect = (cmd_xor_i) ? {32'hffffffff} : {32'b0} ;
	assign logic_result = (op1_i & op2_i & cmd_and_vect) | ((op1_i ^ op2_i) & cmd_xor_vect) ;
	assign logic_we     = (cmd_and_i | cmd_xor_i) & valid_i ;
	assign mul_ce       =  cmd_mul_i & valid_i ;
	
	always @(posedge clk_i)
		begin : mul_reg
			if (mul_ce )
				mul_high <= cmd_mul_high_i ; 
		end 
	
	
	rv_mul_dsp #(.pipelined(1'b0))
		rv_mul_inst
		( .clk_i(clk_i)
		, .rst_i(rst_i)
		, .ce_i(mul_ce)
		, .op1_i(op1_i)
		, .op2_i(op2_i)
		, .ce_o(mul_we)
		, .result_o(mul_result_low)
		, .result_high_o(mul_result_high)
		); 
	
	assign mul_result = (mul_high ) ? mul_result_high : mul_result_low ;
	assign div_ce = cmd_div_i & valid_i;
	assign div_we = div_ce ;
	assign div_result = 32'b0;
	assign shift_ce = cmd_shift_i & valid_i ;
	
	
	rv_shifter
		rv_shifter_inst
		( .clk_i(clk_i)
		, .rst_i(rst_i)
		, .ce_i(shift_ce)
		, .d_i(op1_i)
		, .s_i(op2_i[4:0])
		, .right_i(cmd_shift_right_i)
		, .sig_i(cmd_signed_i)
		, .ce_o(shift_we)
		, .d_o(shift_result)
		); 
	
	
	assign adder_we_vect = (adder_we ) ? {32'hffffffff} : {32'b0} ;
	assign logic_we_vect = (logic_we ) ? {32'hffffffff} : {32'b0} ;
	assign mul_we_vect = (mul_we ) ? {32'hffffffff} : {32'b0} ;
	assign div_we_vect = (div_we ) ? {32'hffffffff} : {32'b0} ;
	assign shift_we_vect = (shift_we ) ? {32'hffffffff} : {32'b0} ;
	// mux
	assign result_o = (adder_result[31:0] & adder_we_vect) | (logic_result & logic_we_vect) | 
		(mul_result & mul_we_vect) | (div_result & div_we_vect) | (shift_result & shift_we_vect) ;
	
	assign result_we = adder_we | logic_we | mul_we | div_we | shift_we ;
	assign we_o = result_we ;
	
	always @(posedge clk_i)
		if (rst_i | result_we)
			busy <= 1'b0 ; 
		else if (shift_ce | mul_ce | div_ce)
			busy <= 1'b1 ;
	
	assign busy_o = busy ;
	assign wait_o = shift_ce | mul_ce | div_ce ;
endmodule
