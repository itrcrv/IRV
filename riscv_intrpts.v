`timescale 1ns/1ps

module riscv_intrpts 
	( mie
	, ir_in
	, ir_out
	, interrupt_exec_o
	, interrupt_ack_i
	, mcause_o
	, ext_irq_in
	, timer_irq_in
	, software_irq_in
	, l_irq_in
	, clk_i
	, rst_i
	);
	
	parameter NUM_LOCALINTERUPTS  = 8;
	
	output [18:0] ir_out; 
	output interrupt_exec_o; 
	output reg [4:0] mcause_o; 
	
	input interrupt_ack_i;
	input mie; 
	input [18:0] ir_in;
	input ext_irq_in; 
	input timer_irq_in; 
	input software_irq_in; 
	input [NUM_LOCALINTERUPTS - 1:0] l_irq_in; 
	input clk_i; 
	input rst_i; 
	
	reg interrupt_exec = 1'b0; 
	reg [18:0] irq_pending = 19'b0;
	// 0 -> msip, msie
	// 1 -> mtip, mtie
	// 2 -> meip, meie
	// 18-3-> lip, lie
	
	parameter MCYCLE_EN =1'b1;
	parameter [18:0] c_pending_init = 18'b0;
	parameter IRQ_CODE_MSOFTWARE = 3;
	parameter IRQ_CODE_MTIMER = 7;
	parameter IRQ_CODE_MEXTERNAL = 11;
	parameter IRQ_CODE_LOCAL_BASE = 16;
	
	function [4:0] IrqPosition;
		input [7:0] Irq;
		integer i;
		begin
			IrqPosition = 5'b0;
			for (i = 7; i >= 0; i = i - 1)
				begin :lop
					if (Irq[i])
						begin
							IrqPosition = i;
							disable lop;
						end
				end
		end
	endfunction
	
	assign ir_out = irq_pending ;
	
	always @(posedge clk_i)
		begin
			if (rst_i == 1'b1)
				begin
					irq_pending <= c_pending_init ; 
				end
			else
				begin
					irq_pending[1] <= timer_irq_in ; 
					irq_pending[0] <= software_irq_in ; 
					irq_pending[2] <= ext_irq_in ; 
					irq_pending[18:3] <= {8'b0, l_irq_in} ; 
				end  
		end 
	assign interrupt_exec_o = interrupt_exec ;
	
	wire [15:0] irq_req = ir_in[18:3] & irq_pending [18:3];
	
	always @(posedge clk_i)
		begin : xhdl_5
			reg found;
			if (rst_i == 1'b1)
				interrupt_exec <= 1'b0 ; 
			else
				begin
					found = 0; 
					if (mie == 1'b1 & interrupt_exec == 1'b0)
						begin
							if (irq_req)
								begin
									interrupt_exec <= 1'b1 ; 
									found = 1;
									mcause_o = 	IrqPosition (irq_req);
									/*
									begin : xhdl_4
									for(i = NUM_LOCALINTERUPTS - 1; i >= 0; i = i - 1)
									if ((ir_in[i]) & (irq_pending[i]))
									begin
									interrupt_exec <= 1'b1 ; 
									mcause_o <= irq_cause(i + 16) ; 
									found = 1; 
									disable xhdl_4; 
									end 
									end
									*/
								end
							if (~found)
								begin
									if (ir_in[2] == 1'b1 & irq_pending[2] == 1'b1) //mei
										begin
											interrupt_exec <= 1'b1 ; 
											mcause_o <= IRQ_CODE_MEXTERNAL;
										end
									else if (ir_in[0] == 1'b1 & irq_pending[0] == 1'b1) //msi
										begin
											interrupt_exec <= 1'b1 ; 
											mcause_o <= IRQ_CODE_MSOFTWARE ; 
										end
									else if (ir_in[1] == 1'b1 & irq_pending[1] == 1'b1) //mti
										begin
											interrupt_exec <= 1'b1 ; 
											mcause_o <= IRQ_CODE_MTIMER ; 
										end 
								end 
						end
					else
						if (interrupt_exec == 1'b1 & interrupt_ack_i == 1'b1)
							interrupt_exec <= 1'b0 ; 
				end  
		end 
endmodule
