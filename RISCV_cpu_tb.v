`timescale 1ns/1ps

module RISCV_cpu_tb;
	
	reg clk_i = 1;
	reg rst_i;
	wire [29:0] lli_adr_o;
	wire [31:0] lli_dat_i;  
	wire [3 :0] dbus_sel_o;
	wire [7 :0] irq_i;
	wire [31:0] dbus_dat_o;
	wire [31:0] dbus_dat_i;
	wire [29:0] dbus_adr_o;
	//wire dbus_ack_i;
	wire dbus_we_o;
	wire dbus_ack_i;
	wire dbus_cyc_o, dbus_stb_o;
	
	initial
		begin
			data_mem.mem[8]=32'h12345678;
		end
	
	always @(clk_i) clk_i <= #5 ~clk_i;
	
	initial begin rst_i=1; #16; rst_i=0; end
	
	//initial begin 			#0;	$readmemh("cache.hex", uut.inst_mem.mem_cache); 		end//*/
	
	//	lxp32_cpu 
	//		//#(	.MUL_ARCH(MUL_ARCH), .REG_RAM_STYLE(REG_RAM_STYLE), .TIMER_XLEN(TIMER_XLEN))
	//		cpu0(
	//		.clk_i(clk_i)
	//		,.rst_i(rst_i)
	//		,.lli_re_o()
	//		,.lli_adr_o()
	//		,.lli_dat_i(lli_dat_i)
	//		,.lli_busy_i(lli_busy_i)
	//		,.lli_cc_invalidate_o()
	//		,.dbus_cyc_o()
	//		,.dbus_stb_o()
	//		,.dbus_we_o()
	//		,.dbus_sel_o()
	//		,.dbus_ack_i(dbus_ack_i)
	//		,.dbus_adr_o()
	//		,.dbus_dat_o()
	//		,.dbus_dat_i(dbus_dat_i)
	//		,.irq_i(irq_i)
	//		);
	
	rv_cpu 
		//#(	.MUL_ARCH(MUL_ARCH), .REG_RAM_STYLE(REG_RAM_STYLE), .TIMER_XLEN(TIMER_XLEN))
		cput(
		.clk_i(clk_i)
		,.rst_i(rst_i)
		,.lli_re_o(lli_re_o)
		,.lli_adr_o(lli_adr_o)
		,.lli_dat_i(lli_dat_i)
		,.lli_busy_i(lli_busy_i)
		,.lli_cc_invalidate_o(lli_cc_invalidate_o)
		,.dbus_cyc_o(dbus_cyc_o)
		,.dbus_stb_o(dbus_stb_o)
		,.dbus_we_o(dbus_we_o)
		,.dbus_sel_o(dbus_sel_o)
		,.dbus_ack_i(dbus_ack_i)
		,.dbus_adr_o(dbus_adr_o)
		,.dbus_dat_o(dbus_dat_o)
		,.dbus_dat_i(dbus_dat_i)
		,.irq_i(irq_i)
		);
	
	im_new inst_mem
		(
		.clk(clk_i)
		,.addr({lli_adr_o, 2'b0})
		,.data(lli_dat_i)
		,.extend( extend )  //extend_mode ????
		,.datain_RC(64'b0)
		,.wr_RC(1'b0)
		,.readyn(lli_busy_i)
		);
	
	dm_org data_mem
		( .clk(clk_i)
		, .addr(dbus_adr_o[6:0])
		, .wr(dbus_we_o)
		, .dbus_cyc_o(dbus_cyc_o)
		, .dbus_stb_o(dbus_stb_o)
		, .sel(dbus_sel_o)
		, .wdata(dbus_dat_o )
		, .rdata(dbus_dat_i)
		, .done(dbus_ack_i)
		);
	
endmodule