`timescale 1ns/1ps
module fetch_rv #
	( parameter[29:0] START_ADDR  = 30'b0
	, parameter USE_RISCV  = 1
	)
	( clk_i
	, rst_i
	, lli_re_o
	, lli_adr_o
	, lli_dat_i
	, lli_busy_i
	, lli_cc_invalidate_o
	, word_o
	, next_ip_o
	, valid_o
	, ready_i
	, fence_i_i
	, jump_valid_i
	, jump_dst_i
	, jump_ready_o
	);
	
	input clk_i; 
	input rst_i; 
	output lli_re_o; 
	output[29:0] lli_adr_o; 
	input[31:0] lli_dat_i; 
	input lli_busy_i; 
	output lli_cc_invalidate_o; 
	output[31:0] word_o; 
	output[29:0] next_ip_o; 
	output reg valid_o; 
	input ready_i; 
	input fence_i_i; 
	input jump_valid_i; 
	input[29:0] jump_dst_i; 
	output jump_ready_o; 
	
	reg[29:0] fetch_addr; 
	reg[29:0] last_addr; 
	wire next_word; 
	reg suppress_re; 
	wire re; 
	reg requested; 
	wire fifo_rst; 
	wire fifo_we; 
	wire[61:0] fifo_din; 
	//signal fifo_re: std_logic;
	reg[61:0] fifo_dout; 
	//signal fifo_empty: std_logic;
	//signal fifo_full: std_logic;
	reg jr; 
	
	initial
		begin
			//<<x-hdl>> cannot initialize wire type
			// init <= 1'b1;
			
			//<<x-hdl>> cannot initialize wire type
			// init_cnt <= {ñ{1'b0}};
			
			fetch_addr <= START_ADDR;
			suppress_re <= 1'b0;
			requested <= 1'b0;
			jr <= 1'b0;
		end
	
	// init state machine (to initialize all registers)
	// all cpu registers are expected to be zero-initialized after reset.
	// since these registers are implemented as a ram block, we perform
	// the initialization sequentially by generating "mov rn, 0" instructions
	// for each n from 0 to 255.
	//
	// with sram-based fpgas, flip-flops and ram blocks have deterministic
	// state after configuration. on these technologies the cpu can operate
	// without reset and the initialization procedure described above is not
	// needed. however, the initialization is still performed as usual when
	// external reset signal is asserted.
	//valid_o <= not fifo_empty;	
	// fetch state machine
	
	always @(posedge clk_i)
		last_addr = fetch_addr; 
	
	
	always @(posedge clk_i)
		begin
			if (rst_i)
				begin
					fetch_addr <= START_ADDR ; 
					requested <= 1'b0 ; 
					jr <= 1'b0 ; 
					suppress_re <= 1'b0 ; 
				end
			else
				begin
					jr <= 1'b0 ; 
					// suppress lli request if jump signal is active but will not be processed
					// in this cycle. helps to reduce jump latency with high-latency lli slaves.
					// note: gating "re" with "jump_valid_i and not jr" asynchronously would
					// reduce jump latency even more, but we really want to avoid too large
					// clock-to-out on lli outputs.
					suppress_re <= jump_valid_i & ~jr & ~next_word ; 
					if (lli_busy_i == 1'b0)
						begin
							requested <= re & ~(jump_valid_i & ~jr) ; 
						end 
					if (next_word == 1'b1)
						begin
							if (jump_valid_i == 1'b1 & jr == 1'b0)
								begin
									fetch_addr <= jump_dst_i ; 
									jr <= 1'b1 ; 
								end
							else
								begin
									fetch_addr <= fetch_addr + 1'b1 ; 
								end 
						end 
				end  
		end 
	//next_word <= (fifo_empty or ready_i) and not lli_busy_i and init;
	assign next_word = (ready_i) & ~lli_busy_i ;
	//re <= (fifo_empty or ready_i) and init and not suppress_re;
	assign re = (ready_i) & ~suppress_re ;
	assign lli_re_o = re ;
	assign lli_adr_o = (ready_i)?fetch_addr: last_addr;
	assign jump_ready_o = jr ;
	assign lli_cc_invalidate_o = fence_i_i ;
	// small instruction buffer
	assign fifo_rst = rst_i | (jump_valid_i & ~jr) ;
	assign fifo_we = requested & ~lli_busy_i ;
	assign fifo_din = (ready_i)? {fetch_addr, lli_dat_i} : {last_addr, lli_dat_i} ;
	
	
	always @(posedge clk_i)
		begin
			if (rst_i)
				begin
					valid_o <= 1'b0 ; 
					fifo_dout <= {621'b0} ; 
				end
			else
				begin
					if (ready_i) begin
							valid_o <= 1'b1 ; 
							fifo_dout <= fifo_din ;
					end
				//else valid_o <= 1'b0 ; 
					
				end  
		end 
	assign next_ip_o = fifo_dout[61:32] ;
	assign word_o = fifo_dout[31:0]  ;
	
endmodule
