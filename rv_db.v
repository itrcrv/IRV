`timescale 1ns/1ps
module rv_db
	#(
	parameter           DB_RMW           = 0,     
	parameter           ENABLE_LOCALMAP    = 1,     
	parameter[31:16]    LOCAL_PREFIX       = 'hFFFF
	)
	( clk_i
	, rst_i
	, valid_i
	, cmd_dbus_i
	, cmd_dbus_store_i
	, cmd_dbus_byte_i
	, cmd_dbus_hword_i
	, cmd_signed_i
	, addr_i
	, wdata_i
	, rdata_o
	, we_o
	, busy_o
	, misalign_o
	, dbus_cyc_o
	, dbus_stb_o
	, dbus_we_o
	, dbus_sel_o
	, dbus_ack_i
	, dbus_adr_o
	, dbus_dat_o
	, dbus_dat_i
	
	);
	
	input clk_i; 
	input rst_i; 
	input valid_i; 
	input cmd_dbus_i; 
	input cmd_dbus_store_i; 
	input cmd_dbus_byte_i; 
	input cmd_dbus_hword_i; 
	input cmd_signed_i; 
	input[31:0] addr_i; 
	input[31:0] wdata_i; 
	output[31:0] rdata_o; 
	reg[31:0] rdata_o;
	output we_o;
	output busy_o; 
	output misalign_o; 
	output dbus_cyc_o; 
	output dbus_stb_o; 
	output dbus_we_o; 
	output[3:0] dbus_sel_o; 
	input dbus_ack_i; 
	output[31:2] dbus_adr_o; 
	reg[31:2] dbus_adr_o;
	output[31:0] dbus_dat_o; 
	reg[31:0] dbus_dat_o;
	input[31:0] dbus_dat_i; 
	
	reg strobe = 1'b0;
	reg we_out = 1'b0;
	reg we; 
	reg byte_mode; 
	reg hword_mode; 
	reg[3:0] sel; 
	reg sig; 
	
	
	reg[1:0] adr_reg; 
	
	reg local_cyc = 1'b0; 
	reg dbus_cyc = 1'b0; 
	reg[31:0] dbus_rdata; 
	wire[7:0] selected_byte; 
	reg misalign; 
	
	
	
	
	function [31:0]dbus_align;
		input[1:0] adr_i; 
		input[31:0] db_i; 
		
		reg[31:0] db_o; 
		reg[15:0] low16; 
		
		begin
			low16 = db_i[15:0]; 
			case (adr_i)
				2'b00 : begin
						db_o = db_i; 
					end
				2'b01 : begin
						db_o = {8'h00, low16, 8'h00}; 
					end
				2'b10 : begin
						db_o = {low16, 16'h0000}; 
					end
				2'b11 : begin
						db_o = {low16[7:0], 24'h000000}; 
					end
				default : begin
						db_o = {32{1'bx}}; 
					end
			endcase 
			dbus_align = db_o; 
		end
	endfunction
	
	always @(posedge clk_i)
		begin 
			reg misalign_t; 
			
			
			if (rst_i == 1'b1)
				begin
					we_out <= 1'b0 ; 
					strobe <= 1'b0 ; 
					sig <= 1'b0 ; 
					byte_mode <= 1'b0 ; 
					sel <= 4'b0;
					we <= 1'b0 ; 
			
					dbus_adr_o <= 30'b0 ; 
					dbus_dat_o <= 32'b0 ; 
					misalign <= 1'b0 ; 
					local_cyc <= 1'b0 ; 
					dbus_cyc <= 1'b0 ;
					hword_mode <= 1'b0 ; 
				end
			else
				begin
					we_out <= 1'b0 ; 
					misalign_t = 1'b0; 
					if (strobe == 1'b0)
						begin
							if (valid_i & cmd_dbus_i)
								begin
									
									sig <= cmd_signed_i ; 
									dbus_adr_o <= addr_i[31:2] ; 
									adr_reg <= addr_i[1:0] ; 
									dbus_dat_o <= dbus_align(addr_i[1:0], wdata_i) ; 
									


									
									if (cmd_dbus_byte_i)
										begin
											byte_mode <= 1'b1 ; 
											hword_mode <= 1'b0 ; 
											case (addr_i[1:0])
												default : begin
														sel <= 4'b0001 ; 
													end
												2'b01 : begin
														sel <= 4'b0010 ; 
													end
												2'b10 : begin
														sel <= 4'b0100 ; 
													end
												2'b11 : begin
														sel <= 4'b1000 ; 
													end
											endcase 
										end
									else if (cmd_dbus_hword_i)
										begin
											byte_mode <= 1'b0 ; 
											hword_mode <= 1'b1 ;
											
											if (~(addr_i[1:0] != 2'b11)) $display("Misaligned half word granular access on data bus (warning)"); 
											
											case (addr_i[1:0])
												default : begin
														sel <= 4'b0011 ; 
													end
												2'b01 : begin
														sel <= 4'b0110 ; 
													end
												2'b10 : begin
														sel <= 4'b1100 ; 
													end
												2'b11 : begin
														sel <= 4'b0000 ; 
														misalign_t = 1'b1; 
													end
											endcase 
										end
									else
										begin 
											byte_mode <= 1'b0 ; 
											hword_mode <= 1'b0 ;
											
										
											if (addr_i[1:0] == 2'b00)
												sel <= 4'b1111 ; 
											else
												begin
													sel <= 4'b0000 ; 
													misalign_t = 1'b1; 
												end 
											
											if (~(addr_i[1:0] == 2'b00)) $display("Misaligned word-granular access on data bus (warning)"); 
											
										end 
									we <= cmd_dbus_store_i ; 
								
							
									if (misalign_t == 1'b0)
										begin
											strobe <= 1'b1 ; 
											dbus_cyc <= 1'b1 ; 
										end 
								end 
						end
					else
						begin
							if (dbus_ack_i)
								begin
									
									strobe         <= 1'b0 ; 
									local_cyc      <= 1'b0 ; 
									dbus_cyc       <= 1'b0 ; 
									if (!we)
										we_out <= 1'b1 ; 
									 
								end 
						end 
					misalign <= misalign_t ; 
				end  
		end 
	assign dbus_cyc_o = dbus_cyc ;
	assign local_cyc_o = local_cyc ;
	
	always @(posedge clk_i)
		begin
			dbus_rdata <= dbus_dat_i ;  
		end 
	
	assign dbus_sel_o = sel;
	assign dbus_stb_o = strobe ;
	assign dbus_we_o  = we ;



	always @(dbus_rdata or sel or byte_mode or hword_mode or sig or adr_reg)
		begin : rdata_mux
			reg[7:0] byte8; 
			reg[15:0] hword; 
			if (byte_mode == 1'b1)
				begin
					case (adr_reg)
						2'b00 : begin
								byte8 = dbus_rdata[7:0]; 
							end
						2'b01 : begin
								byte8 = dbus_rdata[15:8]; 
							end
						2'b10 : begin
								byte8 = dbus_rdata[23:16]; 
							end
						2'b11 : begin
								byte8 = dbus_rdata[31:24]; 
							end
						default : begin
								byte8 = {8{1'bx}}; 
							end
					endcase 
					if ((sig == 1'b0) | (byte8[7] == 1'b0) )
						rdata_o <= {24'h000000, byte8} ; 
					else
						rdata_o <= {24'hFFFFFF, byte8} ; 
				end
			else if (hword_mode == 1'b1)
				begin
					case (adr_reg)
						2'b00 : begin
								hword = dbus_rdata[15:0]; 
							end
						2'b01 :begin
								hword = dbus_rdata[23:8]; 
							end
						2'b10 : begin
								hword = dbus_rdata[31:16]; 
							end
						default : begin
								hword = {16{1'bx}}; 
							end
					endcase 
					if (sig == 1'b0 | (hword[15] == 1'b0))
						rdata_o <= {16'h0000, hword};
					else
						rdata_o <= {16'hFFFF, hword} ;
				end
			else
				begin
					rdata_o <= dbus_rdata ; 
				end 
		end 
	assign we_o = we_out ;
	assign busy_o = strobe | we_out ;
	assign misalign_o = misalign ;
endmodule
