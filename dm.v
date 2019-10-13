`timescale 1ns/1ps
module dm_org(
	input                   clk
	,input      	[6:0]   addr
	,input 	                wr
	,input                  dbus_cyc_o
	,input                  dbus_stb_o
	,input          [3:0]   sel
	,input        	[31:0]  wdata
	,output reg    	[31:0]  rdata
	,output reg             done
	);
	
	reg [31:0] mem [0:127];  
	
	
	wire rd = (!wr) & dbus_stb_o & dbus_stb_o;
	wire we = (wr) & dbus_stb_o & dbus_stb_o;
	
	
	always @(posedge clk)
		begin
			done <= 1'b0;
			if (we)
				begin
					if (sel ==4'b1111)
						mem[addr] <= wdata;
					else if (sel ==4'b1000 )
						mem[addr][31:24] <= wdata[31:24];
					else if (sel ==4'b0100 )
						mem[addr][23:16] <= wdata[23:16];
					else if (sel ==4'b0010 )
						mem[addr][15:08] <= wdata[15:08];
					else if (sel ==4'b0001 )
						mem[addr][07:00] <= wdata[07:00];
					else if (sel ==4'b0011 )
						mem[addr][15:00] <= wdata[15:00];
					else if (sel ==4'b0110 )
						mem[addr][23:08] <= wdata[23:08];
					else if (sel ==4'b1100 )
						mem[addr][31:16] <= wdata[31:16];
					
						
					
					done      <= 1'b1;
				end
			else
				if (rd)
					begin
						rdata     <= mem[addr];
						done      <= 1'b1;
					end
		end
	
	
	
endmodule