`timescale 1ns/1ps
module riscv_db
  (input		   	clk_i
  ,input				rst_i
  
  ,input				cmd_dbus_i
  ,input				cmd_dbus_store_i
  ,input				cmd_dbus_byte_i
  ,input				valid_i
  ,input				dbus_ack_i

  
  ,input		[31:0]	addr_i
  ,input		[31:0]	data_exec_i
  ,input		[31:0]	data_mem_i
  
  ,output reg	[31:2]	addr_mem_o
  ,output reg	[31:0]	dat_mem_o
  ,output reg	[31:0]	dat_reg_o
  ,output reg     	mem_we	
  );
  
  always @ (posedge clk_i)
    if (rst_i)
      begin
        addr_mem_o    <= 30'b0;
        dat_mem_o     <= 32'b0;
        mem_we        <= 1'b0;
      end
    else
      if (valid_i & cmd_dbus_i)
        begin
          addr_mem_o  <= addr_i[31:2];
          dat_mem_o   <= data_mem_i;
          mem_we      <= cmd_dbus_store_i;
        end
      else if (dbus_ack_i)
        mem_we <= 1'b0;
endmodule