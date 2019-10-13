`timescale 1ns/1ps
module rv_mul_dsp 
	( clk_i
	, rst_i
	, ce_i
	, op1_i
	, op2_i
	, ce_o
	, result_o
	, result_high_o
	);
	
	parameter pipelined  = 1;
	input clk_i; 
	input rst_i; 
	input ce_i; 
	input[31:0] op1_i; 
	input[31:0] op2_i; 
	output ce_o; 
	wire ce_o;
	output[31:0] result_o; 
	wire[31:0] result_o;
	output[31:0] result_high_o; 
	wire[31:0] result_high_o;
	
	reg[31:0] pp00; 
	reg[31:0] pp01; 
	reg[31:0] pp10; 
	reg[31:0] pp11; 
	wire[31:0] product; 
	wire[31:0] product_h; 
	reg ceo =1'b0; 
	wire ce1; 
	
	always @(posedge clk_i)
		begin
			pp00 <= ((op1_i[15:0]) * (op2_i[15:0])) ; 
			pp01 <= ((op1_i[15:0]) * (op2_i[31:16])) ; 
			pp10 <= ((op1_i[31:16]) * (op2_i[15:0])) ; 
			pp11 <= ((op1_i[31:16]) * (op2_i[31:16])) ;  
		end 
	assign product[31:16] = (pp00[31:16]) + (pp01[15:0]) + (pp10[15:0]) ;
	assign product[15:0] = (pp00[15:0]) ;
	assign product_h[15:0] = (pp01[31:16]) + (pp10[31:16]) + (pp11[15:0]) ;
	assign product_h[31:16] = (pp11[31:16]) ;
	assign result_o = (product) ;
	assign result_high_o = (product_h) ;
	
	always @(posedge clk_i)
		if (rst_i == 1'b1)
			ceo <= 1'b0 ; 
		else
			ceo <= ce_i ;
			
	assign ce_o = ceo ;
endmodule
