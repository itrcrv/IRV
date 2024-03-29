`timescale 1ns/1ps
module rv_shifter
	( clk_i
	, rst_i
	, ce_i
	, d_i
	, s_i
	, right_i
	, sig_i
	, ce_o
	, d_o
	);
	
	input         clk_i; 
	input         rst_i; 
	input         ce_i; 
	input  [31:0] d_i; 
	input  [4:0]  s_i; 
	input         right_i; 
	input         sig_i; 
	output        ce_o; 
	output[31:0]  d_o; 
	wire[31:0]    datax;
	
	wire[31:0] data_shifted; 
	wire fill; 
	wire[3:0] fill_v; 
	wire[31:0] cascades0; 
	wire[31:0] cascades1; 
	wire[31:0] cascades2; 
	wire[31:0] cascades3; 
	wire[31:0] cascades4; 
	
	reg[31:0] stage2_data; 
	reg[4:0] stage2_s; 
	reg stage2_fill; 
	wire[15:0] stage2_fill_v; 
	reg stage2_right; 
	reg ceo=1'b0; 
	
	
	function [31:0] ReverseBits32;
		input [31:0] Byte;
		integer i;
		begin
			for (i = 0; i < 32; i = i + 1)
				ReverseBits32[31-i] = Byte[i];
		end
	endfunction
	
	assign datax = (right_i)? ReverseBits32(d_i): d_i;
	assign fill = sig_i & datax[0] ;
	assign fill_v = {4{fill}} ;
	assign cascades0 = (s_i[0]) ? {datax[30:0], fill_v[0]} : datax ;
	assign cascades1 = (s_i[1]) ? {cascades0[29:0], fill_v[1:0]} : cascades0 ;
	assign cascades2 = (s_i[2]) ? {cascades1[27:0], fill_v[3:0]} : cascades1 ;
	
	always @(posedge clk_i)
		if (rst_i)
			begin
				ceo <= 1'b0 ; 
				stage2_data <= {32{1'bx}} ; 
				stage2_s <= {5{1'bx}} ; 
				stage2_fill <= 1'bx ; 
				stage2_right <= 1'bx ; 
			end
		else
			begin
				ceo <= ce_i ; 
				stage2_data <= cascades2; 
				stage2_s <= s_i ; 
				stage2_fill <= fill ; 
				stage2_right <= right_i ; 
			end  
	
	assign stage2_fill_v = {16{stage2_fill}} ;
	assign cascades3 = (stage2_s[3]) ? {stage2_data[23:0], stage2_fill_v[7:0]} : stage2_data ;
	assign cascades4 = (stage2_s[4]) ? {cascades3[15:0], stage2_fill_v[15:0]} : cascades3 ;
	
	assign data_shifted = (stage2_right == 1'b0) ? cascades4 : (ReverseBits32(cascades4));
	assign d_o = data_shifted ;
	assign ce_o = ceo ;
endmodule
