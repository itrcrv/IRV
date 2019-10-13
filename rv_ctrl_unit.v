`timescale 1ns/1ps
module rv_ctrl_unit
	#(
	parameter           DIVIDER_EN    = 1,     
	parameter           MUL_ARCH      = "dsp"  
	)
	( clk_i
	, rst_i
	, op1_i
	, wdata_o
	, we_o
	, csr_exception
	, csr_adr
	, ce_i
	, busy_o
	, csr_x0_i
	, csr_op_i
	, mtvec_o
	, mepc_o
	, sstep_o
	, mcause_i
	, mepc_i
	, adr_i
	, mtrap_strobe_i
	, cmd_tret_i
	, ext_irq_in
	
	, interrupt_exec_o
	, interrupt_ack_i
	);
	
	
	parameter [3:0]	m_stdprefix = 4'h3;
	parameter [3:0] m_nonstdprefix =4'h7;
	parameter [3:0] m_roprefix =4'hf; 
	
	
	parameter [7:0] status = 8'h00; 
	parameter [7:0] isa    = 8'h01;
	parameter [7:0] edeleg = 8'h02;
	parameter [7:0] ideleg = 8'h03;
	parameter [7:0] a_ie   = 8'h04;
	parameter [7:0] tvec =   8'h05;
	
	
	parameter [7:0] vendorid =  8'h11;
	parameter [7:0] marchid =  8'h12;
	parameter [7:0] impid   =  8'h13;
	parameter [7:0] hartid  =  8'h14;
	
	
	parameter [7:0] scratch =   8'h40;
	parameter [7:0] epc=        8'h41;
	parameter [7:0] cause =     8'h42;
	parameter [7:0] badaddr =   8'h43;
	parameter [7:0] a_ip =      8'h44;
	
	
	parameter [11:0] a_mcycle  = 12'HB00;
	parameter [11:0] a_mcycleh = 12'HB80;
	parameter [11:0] m_bonfire_csr = 12'H7C0;
	parameter [31:0] impvers = 32'h00010010;
	parameter MCYCLE_EN =1'b1;
	
	
	input  [31:0] op1_i; 
	output reg [31:0] wdata_o; 
	output we_o; 
	output csr_exception; 
	input  [11:0] csr_adr; 
	input  ce_i; 
	output busy_o; 
	input  csr_x0_i; 
	input  [01:0] csr_op_i; 
	
	
	output [31:2] mtvec_o; 
	output [31:2] mepc_o; 
	output sstep_o; 
	
	
	input  [3:0] mcause_i; 
	input  [31:2] mepc_i; 
	input  [31:0] adr_i; 
	input  mtrap_strobe_i; 
	input  cmd_tret_i; 
	
	
	input  [07:0] ext_irq_in; 
	
	output interrupt_exec_o; 
	input  interrupt_ack_i; 
	
	input  clk_i; 
	input  rst_i; 
	
	wire  [31:0] csr_in; 
	reg   [31:0] csr_out; 
	wire  [07:0] csr_offset; 
	wire  [31:0] csr_t1; 
	wire  busy; 
	reg   we=0; 
	reg   exception=0; 
	reg   [31:2] mtvec=0; 
	reg   [31:0] mscratch=0; 
	reg   [31:2] mepc=0; 
	reg   [04:0] mcause=0; 
	reg   [04:0] mcause_31=0; 
	reg   [31:0] mbadaddr=0; 
	reg   mie=0; 
	reg   mpie=0; 
	reg   [18:0] irq_enable= 19'b0; 
	wire  [18:0] irq_pending= 19'b0; 
	
	
	
	function [31:0] get_mie;
		input [18:0] ir;
		reg [31:0] mie = 32'b0;
		begin
			mie[3] = ir[0];
			mie[7] = ir[1];
			mie[11] = ir[2];
			mie[31:16] = ir[18:3];
			get_mie = mie;
		end
	endfunction
	
	function [31:0] cause_csr;
		input is_irq;
		input [4:0] cause;
		reg [31:0] csr = 32'b0;
		begin
			csr[4:0]    =  cause;
			csr[31]     =  is_irq;
			cause_csr   =  csr;
		end
	endfunction
	
	wire  [04:0] mcause_irq; 
	reg   [01:0] m_bonfire=2'b0;  
	wire  [63:0] mcycle;
	reg   [63:0] mcycl_count;
	
	assign we_o = we ;
	assign busy_o = we | exception ;
	assign csr_exception = exception ;
	assign mtvec_o = mtvec ;
	assign mepc_o = mepc ;
	assign sstep_o = m_bonfire[1] ;
	assign csr_offset = csr_adr[7:0] ;
	assign csr_t1 = (csr_offset == status) ? {24'b0000-0000-0000-0000-0001-1000, mpie, 3'b0, mie, 3'b0} : (csr_offset == isa) ? 
		32'b0100-0000-0000-0000-0001-0001-0000-0000 : (csr_offset == tvec) ? {mtvec, 2'b00} : (csr_offset == scratch) ?
		mscratch : (csr_offset == cause) ? cause_csr(mcause_31, mcause) : (csr_offset == badaddr) ?
		mbadaddr : (csr_offset == epc) ? {mepc, 2'b00} : (csr_offset == impid) ? impvers : (csr_offset == a_ie) 
		? get_mie(irq_enable) : (csr_offset == a_ip) ? get_mie(irq_pending) : {32{1'b0}} ; 
	assign csr_in = ((csr_adr[11:8] == m_stdprefix) | (csr_adr[11:8] == m_roprefix)) ? csr_t1 : (csr_adr == a_mcycle) ?	mcycle[31:0]
		: (csr_adr == a_mcycleh) ? mcycle[63:32] : (csr_adr == m_bonfire_csr) ? {31'b0, m_bonfire[0]} : {32'bx} ;
	
	
	
	always @(posedge clk_i)
		begin
			if (rst_i)
				mcycl_count = 0;
			else
				mcycl_count = mcycl_count +1'b1;
		end		
	assign mcycle =  mcycl_count;
	
	
	
	always @(op1_i or csr_in or csr_op_i)
		begin : csr_alu
			case (csr_op_i)
				2'b01 :	csr_out <= op1_i ; 
				2'b10 :	csr_out <= csr_in | op1_i ; 
				2'b11 :	csr_out <= csr_in & (~op1_i) ; 
				default : csr_out <= {32{1'bx}} ; 
			endcase 
		end 
	
	riscv_intrpts #()
		irq_unit
		( .mie(mie)
		, .ir_in(irq_enable)
		, .ir_out(irq_pending)
		, .interrupt_exec_o(interrupt_exec_o)
		, .interrupt_ack_i(interrupt_ack_i)
		, .mcause_o(mcause_irq)
		, .ext_irq_in(ext_irq_in[0])
		, .l_irq_in(ext_irq_in[7:1])
		, .timer_irq_in(timer_irq_in)
		, .software_irq_in(1'b0)
		, .clk_i(clk_i)
		, .rst_i(rst_i)
		); 
	
	always @(posedge clk_i)
		begin : xhdl_11
			reg l_exception; 
			if (rst_i == 1'b1)
				begin
					exception  <= 1'b0 ; 
					mtvec      <= 30'b0 ; 
					mepc       <= 30'b0 ; 
					mcause     <= 32'b0 ; 
					mcause_31  <= 1'b0 ; 
					we         <= 1'b0 ; 
					mie        <= 1'b0 ; 
					mpie       <= 1'b0 ; 
					irq_enable <= 19'b0 ; 
					m_bonfire  <= 2'b0 ; 
				end
			else
				begin
					if (exception == 1'b1)
						exception <= 1'b0 ; 
					
					if (we == 1'b1)
						we <= 1'b0 ; 
					
					if (mtrap_strobe_i == 1'b1)
						begin
							if (interrupt_ack_i == 1'b1)
								begin
									mcause_31 <= 1'b1 ; 
									mcause <= mcause_irq ; 
								end
							else
								begin
									mcause_31 <= 1'b0 ; 
									mcause <= {1'b0, mcause_i} ; 
									case (mcause_i)
										'h4, 'h6, 'h0 :
											mbadaddr <= adr_i ; 
										
										'h2, 'h3 :
											mbadaddr <= {mepc_i, 2'b00} ; 
										
										default : begin end
									endcase 
								end 
							mepc <= mepc_i ; 
							mpie <= mie ; 
							mie <= 1'b0 ; 
						end
					else if (cmd_tret_i == 1'b1)
						mie <= mpie ; 
					
					if (ce_i == 1'b1)
						begin
							if (csr_adr[11:8] == m_stdprefix)
								begin
									l_exception = 1'b0; 
									case (csr_adr[7:0])
										status :
											begin
												mie <= csr_out[3] ; 
												mpie <= csr_out[7] ; 
											end
										isa     : begin end
										tvec    : mtvec <= csr_out[31:2];
										scratch : mscratch <= csr_out; 
										epc     : mepc <= csr_out[31:2];
										cause   :
											begin
												mcause <= csr_out[4:0] ; 
												mcause_31 <= csr_out[31] ; 
											end
										badaddr : mbadaddr <= csr_out;
										a_ie    : irq_enable = {csr_out[31:16], csr_out[11], csr_out[7], csr_out[3]};
										
										
										edeleg, ideleg, a_ip : begin end
										default : l_exception = 1'b1;
									endcase 
								end
							else if (MCYCLE_EN & (csr_adr == a_mcycle | csr_adr == a_mcycleh))
								l_exception = 1'b0;
							else if (csr_adr[11:8] == m_roprefix)
								case (csr_adr[7:0])
									vendorid, marchid, impid, hartid :
										l_exception = 1'b0; 
									default :
										l_exception = 1'b1; 
								endcase 
							
							else if (csr_adr == m_bonfire_csr)
								begin
									l_exception = 1'b0; 
	
									m_bonfire[1] <= csr_out[0];
								end
							else
								l_exception = 1'b1; 
							
							if (l_exception == 1'b0)
								begin
									wdata_o <= csr_in ; 
									we <= 1'b1 ; 
								end
							else
								wdata_o <= {32{1'b0}} ; 
							
							exception <= l_exception ; 
						end 
				end  
		end 
endmodule
