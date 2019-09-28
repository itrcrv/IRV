`timescale 1ns/1ps
module rv_cpu 
  #(
  parameter           DBUS_RMW      = 0,     /* expression        */
  parameter           DIVIDER_EN    = 1,     /*                   */
  parameter           MUL_ARCH      = "dsp", /* DSP / Multi Stage */
  parameter  [29:0]   START_ADDR    = 30'b0, /* expression        */
  parameter           REG_RAM_STYLE = 1,     /* 1:BRAM 0:SRAM     */
  parameter           ENABLE_TIMER  = 1,     /* 1:ENBL 0:DISBL    */
  parameter           TIMER_XLEN    = 32     /* 1:ENBL 0:DISBL    */
  )
  (
  clk_i,
  rst_i,
  lli_re_o,
  lli_adr_o,
  lli_dat_i, 
  lli_busy_i,
  lli_cc_invalidate_o,
  dbus_cyc_o,
  dbus_stb_o,
  dbus_we_o,
  dbus_sel_o,
  dbus_ack_i, 
  dbus_adr_o, 
  dbus_dat_o, 
  dbus_dat_i,
  irq_i
  );
  
  //  parameter DBUS_RMW  = 0;
  //  parameter DIVIDER_EN  = 1;
  //  parameter MUL_ARCH  = "dsp";
  //  parameter[29:0] START_ADDR  = 30'b0;
  //  parameter USE_RISCV  = 1;
  //  parameter REG_RAM_STYLE  = "block";
  //  parameter ENABLE_TIMER  = 1;
  //  parameter TIMER_XLEN  = 32;
  input clk_i; 
  input rst_i; 
  output lli_re_o; 
  output [29:0] lli_adr_o; 
  input  [31:0] lli_dat_i; 
  input lli_busy_i; 
  output lli_cc_invalidate_o; 
  output dbus_cyc_o; 
  
  output dbus_stb_o; 
  output dbus_we_o; 
  output [3:0]  dbus_sel_o; 
  input dbus_ack_i; 
  output [31:2] dbus_adr_o; 
  output [31:0] dbus_dat_o; 
  input  [31:0] dbus_dat_i; 
  input  [7:0]  irq_i; 
  
  wire [31:0] fetch_word; 
  wire [29:0] fetch_next_ip; 
  wire fetch_valid; 
  wire fetch_jump_ready; 
  wire fetch_fence_i; 
  wire decode_ready; 
  wire decode_valid; 
  wire decode_cmd_loadop3; 
  wire decode_cmd_signed; 
  wire decode_cmd_dbus; 
  wire decode_cmd_dbus_store; 
  wire decode_cmd_dbus_byte; 
  wire decode_cmd_dbus_hword; 
  wire decode_cmd_addsub; 
  wire decode_cmd_mul; 
  wire decode_cmd_div; 
  wire decode_cmd_div_mod; 
  wire decode_cmd_cmp; 
  wire decode_cmd_jump; 
  wire decode_cmd_negate_op2; 
  wire decode_cmd_and; 
  wire decode_cmd_xor; 
  wire decode_cmd_shift; 
  wire decode_cmd_shift_right; 
  wire decode_cmd_mul_high; // TH
  wire decode_cmd_signed_b; // TH
  wire decode_cmd_slt; // TH
  wire decode_cmd_csr; // TH
  wire decode_cmd_trap; // TH: Execute trap
  wire decode_cmd_tret; // TH: return from trap
  wire [3:0] decode_jump_type; 
  wire [31:0] decode_op1; 
  wire [31:0] decode_op2; 
  wire [31:0] decode_op3; 
  wire [7:0] decode_dst; 
  wire decode_csr_x0_o; // should be set when rs field is x0
  wire [1:0] decode_csr_op_o; // lower bits of funct3
  wire execute_ready; 
  wire execute_jump_valid; 
  wire [29:0] execute_jump_dst; 
  wire [7:0] sp_raddr1; 
  wire [31:0] sp_rdata1; 
  wire [7:0] sp_raddr2; 
  wire [31:0] sp_rdata2; 
  wire [7:0] sp_waddr; 
  wire sp_we; 
  wire [31:0] sp_wdata; 
  wire[11:0] displacement; 
  wire interrupt_valid; 
  wire[2:0] interrupt_vector; 
  wire interrupt_ready; 
  wire interrupt_return; 
  wire[3:0] decode_trap_cause; // TH: Trap/Interrupt cause
  wire decode_interrupt; // Trap is interrupt 
  wire[31:2] decode_epc; 
  wire[31:2] ex_epc; 
  wire[31:2] ex_tvec; 
  wire sstep; 
  wire wait_s;
  
  
  fetch_rv #(
    .START_ADDR (START_ADDR),
    .USE_RISCV  (1)
    )
    fetch_inst
    (
    .  clk_i(clk_i)
    , .rst_i(rst_i)
    // to memory
    , .lli_re_o(lli_re_o)
    , .lli_adr_o(lli_adr_o)
    , .lli_dat_i(lli_dat_i)
    , .lli_busy_i(lli_busy_i)
    , .lli_cc_invalidate_o(lli_cc_invalidate_o)
    // to next stage of pipeline
    , .word_o(fetch_word)
    , .next_ip_o(fetch_next_ip)
    , .valid_o(fetch_valid)
    , .ready_i(decode_ready)
    , .fence_i_i(fetch_fence_i)
    // to exec stage
    , .jump_valid_i(execute_jump_valid)
    , .jump_dst_i(execute_jump_dst)
    , .jump_ready_o(fetch_jump_ready)
    ); 
  
  /*
  riscv_decode decode_inst
  riscv_decode_r decode_inst_r
  (
  .  clk_i(clk_i)
  , .rst_i(rst_i)
  , .word_i(fetch_word)
  , .next_ip_i(fetch_next_ip)
  , .valid_i(fetch_valid)
  , .jump_valid_i(execute_jump_valid)
  , .ready_o()
  , .fencei_o()
  , .interrupt_valid_i(interrupt_valid)
  , .interrupt_vector_i(interrupt_vector)
  , .interrupt_ready_o()
  , .sp_raddr1_o()
  , .sp_rdata1_i(sp_rdata1)
  , .sp_raddr2_o()
  , .sp_rdata2_i(sp_rdata2)
  , .displacement_o()
  , .ready_i(execute_ready)
  , .valid_o()
  , .cmd_loadop3_o()
  , .cmd_signed_o()
  , .cmd_dbus_o()
  , .cmd_dbus_store_o()
  , .cmd_dbus_byte_o()
  , .cmd_dbus_hword_o()
  , .cmd_addsub_o()
  , .cmd_mul_o()
  , .cmd_div_o()
  , .cmd_div_mod_o()
  , .cmd_cmp_o()
  , .cmd_jump_o()
  , .cmd_negate_op2_o()
  , .cmd_and_o()
  , .cmd_xor_o()
  , .cmd_shift_o()
  , .cmd_shift_right_o()
  , .cmd_mul_high_o()
  , .cmd_signed_b_o()
  , .cmd_slt_o()
  , .cmd_csr_o()
  , .csr_x0_o()
  , .csr_op_o()
  , .cmd_trap_o()
  , .cmd_tret_o()
  , .trap_cause_o()
  , .interrupt_o()
  , .epc_o()
  , .epc_i(ex_epc)
  , .tvec_i(ex_tvec)
  , .sstep_i(sstep)
  , .jump_type_o()
  , .op1_o()
  , .op2_o()
  , .op3_o()
  , .dst_o()
  ); 
  */
  
  riscv_decode_k decode_inst_k
    (
    .  clk_i(clk_i)
    , .rst_i(rst_i)
    , .word_i(fetch_word)
    , .next_ip_i(fetch_next_ip)
    , .valid_i(fetch_valid)
    , .jump_valid_i(execute_jump_valid)
    , .ready_o(decode_ready)
    , .fencei_o(fetch_fence_i)
    , .interrupt_valid_i(interrupt_valid)
    , .interrupt_vector_i(interrupt_vector)
    , .interrupt_ready_o(interrupt_ready)
    , .sp_raddr1_o(sp_raddr1)
    , .sp_rdata1_i(sp_rdata1)
    , .sp_raddr2_o(sp_raddr2)
    , .sp_rdata2_i(sp_rdata2)
    , .displacement_o(displacement)
    , .ready_i(execute_ready)
    , .valid_o(decode_valid)
    , .cmd_loadop3_o(decode_cmd_loadop3)
    , .cmd_signed_o(decode_cmd_signed)
    , .cmd_dbus_o(decode_cmd_dbus)
    , .cmd_dbus_store_o(decode_cmd_dbus_store)
    , .cmd_dbus_byte_o(decode_cmd_dbus_byte)
    , .cmd_dbus_hword_o(decode_cmd_dbus_hword)
    , .cmd_addsub_o(decode_cmd_addsub)
    , .cmd_mul_o(decode_cmd_mul)
    , .cmd_div_o(decode_cmd_div)
    , .cmd_div_mod_o(decode_cmd_div_mod)
    , .cmd_cmp_o(decode_cmd_cmp)
    , .cmd_jump_o(decode_cmd_jump)
    , .cmd_negate_op2_o(decode_cmd_negate_op2)
    , .cmd_and_o(decode_cmd_and)
    , .cmd_xor_o(decode_cmd_xor)
    , .cmd_shift_o(decode_cmd_shift)
    , .cmd_shift_right_o(decode_cmd_shift_right)
    , .cmd_mul_high_o(decode_cmd_mul_high)
    , .cmd_signed_b_o(decode_cmd_signed_b)
    , .cmd_slt_o(decode_cmd_slt)
    , .cmd_csr_o(decode_cmd_csr)
    , .csr_x0_o(decode_csr_x0_o)
    , .csr_op_o(decode_csr_op_o)
    , .cmd_trap_o(decode_cmd_trap)
    , .cmd_tret_o(decode_cmd_tret)
    , .trap_cause_o(decode_trap_cause)
    , .interrupt_o(decode_interrupt)
    , .epc_o(decode_epc)
    , .epc_i(ex_epc)
    , .tvec_i(ex_tvec)
    , .sstep_i(sstep)
    , .jump_type_o(decode_jump_type)
    , .op1_o(decode_op1)
    , .op2_o(decode_op2)
    , .op3_o(decode_op3)
    , .dst_o(decode_dst)
    ); 		   
  
  // TH
  //TH
  // TH
  // TH
  // for RISC-V only...
  // for RISC-V: Interrupt are handle by the control unit, the signal below will be asserted by the CU
  // to the decode to execute an interrupt  
  
  
  reg [31:0] exec_next_ip, exec_word ;
  reg exec_ready;
  
  always @ (posedge clk_i)
    begin
      exec_next_ip <= fetch_next_ip;
      exec_word <= fetch_word;
      exec_ready <= decode_ready;
    end
  
  rv_exec
    #(
    .DBUS_RMW     (DBUS_RMW),
    .DIVIDER_EN   (DIVIDER_EN),
    .MUL_ARCH     (MUL_ARCH),
    .USE_RISCV    (1),
    .ENABLE_TIMER (ENABLE_TIMER),
    .TIMER_XLEN   (TIMER_XLEN)
    ) 
    execute_inst
    (
    .  clk_i(clk_i)
    , .rst_i(rst_i)
    , .cmd_loadop3_i(decode_cmd_loadop3)
    , .cmd_signed_i(decode_cmd_signed)
    , .cmd_dbus_i(decode_cmd_dbus)
    , .cmd_dbus_store_i(decode_cmd_dbus_store)
    , .cmd_dbus_byte_i(decode_cmd_dbus_byte)
    , .cmd_dbus_hword_i(decode_cmd_dbus_hword)
    , .cmd_addsub_i(decode_cmd_addsub)
    , .cmd_mul_i(decode_cmd_mul)
    , .cmd_div_i(decode_cmd_div)
    , .cmd_div_mod_i(decode_cmd_div_mod)
    , .cmd_cmp_i(decode_cmd_cmp)
    , .cmd_jump_i(decode_cmd_jump)
    , .cmd_negate_op2_i(decode_cmd_negate_op2)
    , .cmd_and_i(decode_cmd_and)
    , .cmd_xor_i(decode_cmd_xor)
    , .cmd_shift_i(decode_cmd_shift)
    , .cmd_shift_right_i(decode_cmd_shift_right)
    , .cmd_mul_high_i(decode_cmd_mul_high)
    , .cmd_signed_b_i(decode_cmd_signed_b)
    , .cmd_slt_i(decode_cmd_slt)
    , .cmd_csr_i(decode_cmd_csr)
    , .csr_op_i(decode_csr_op_o)
    , .csr_x0_i(decode_csr_x0_o)
    , .cmd_trap_i(decode_cmd_trap)
    , .cmd_tret_i(decode_cmd_tret)
    , .trap_cause_i(decode_trap_cause)
    , .interrupt_i(decode_interrupt)
    , .epc_i(decode_epc)
    , .epc_o(ex_epc)
    , .tvec_o(ex_tvec)
    , .sstep_o(sstep)
    , .jump_type_i(decode_jump_type)
    , .op1_i(decode_op1)
    , .op2_i(decode_op2)
    , .op3_i(decode_op3)
    , .dst_i(decode_dst)
    , .sp_waddr_o(sp_waddr)
    , .sp_we_o(sp_we)
    , .sp_wdata_o(sp_wdata)
    , .displacement_i(displacement)
    , .valid_i(decode_valid)
    , .ready_o(execute_ready)
    , .dbus_cyc_o(dbus_cyc_o)
    , .dbus_stb_o(dbus_stb_o)
    , .dbus_we_o(dbus_we_o)
    , .dbus_sel_o(dbus_sel_o)
    , .dbus_ack_i(dbus_ack_i)
    , .dbus_adr_o(dbus_adr_o)
    , .dbus_dat_o(dbus_dat_o)
    , .dbus_dat_i(dbus_dat_i)
    , .jump_valid_o(execute_jump_valid)
    , .jump_dst_o(execute_jump_dst)
    , .jump_ready_i(fetch_jump_ready)
    , .interrupt_return_o(interrupt_return)
    , .ext_irq_in(irq_i)
    , .riscv_interrupt_exec_o(interrupt_valid)
	, .wait_o(wait_s)
    ); 
  
  
  /*
  riscv_regfile #(
  .REG_RAM_STYLE (REG_RAM_STYLE)
  ) 
  RF
  (
  .  clk_i(clk_i)
  , .raddr1_i(sp_raddr1)
  , .rdata1_o()
  , .raddr2_i(sp_raddr2)
  , .rdata2_o()
  , .waddr_i(sp_waddr)
  , .we_i(sp_we)
  , .wdata_i(sp_wdata)
  );
  */	
  
  reg [31:0] reg_next_ip, reg_word ;
  reg reg_ready;
  
  always @ (posedge clk_i)
    begin
      reg_next_ip <= exec_next_ip;
      reg_word <= exec_word;
      reg_ready <= exec_ready;
    end
  
  wire [4:0] wrreg = sp_waddr;
  reg_bank regfile
    (.rst_i(rst_i)
    ,.clk_i(clk_i)
    ,.read1(sp_raddr1)
    ,.read2(sp_raddr2)
    ,.read3(5'b0)
    ,.read4(5'b0)
    ,.data1(sp_rdata1)
    ,.data2(sp_rdata2)
    ,.data3()
    ,.data4()
    ,.regwrite(sp_we)
    ,.wrreg(wrreg)
    ,.wrdata(sp_wdata)
	,.wait_i(wait_s)
    );
  
  
endmodule
