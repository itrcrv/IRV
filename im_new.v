`timescale 1ns/1ps
module im_new (datain_RC, wr_RC, addr, data, clk, extend, readyn );
	parameter NMEM                                  = 512;    
	parameter Address_width                         = 32;    
	parameter RC_DATA_width                         = 32;    
	parameter Index                                 = 6;    
	parameter Offset                                = 2;    
	parameter Tag                                   = Address_width-Index-Offset ; 
	parameter Cache_cell_Width                      = 64 + Tag +1 ; 
	parameter Depth                                 = 2**Index;
	
	input                                           wr_RC;
	input   [Address_width-1:0]                     addr;
	input   [Address_width+RC_DATA_width-1:0]       datain_RC;
	output  [RC_DATA_width-1:0]                     data;
	output                                          readyn;
	input                                           clk;
	output                                          extend;
	
	
	wire    [Address_width-3:0]                     addr_d4 = addr[Address_width-1:2];
	wire                                            extend1;
	wire    [Index-1:0]                             addr_index  =  addr[7:2];  //[5:0]
	wire    [Tag-1:0]                               addr_tag    =  addr[31:8];
	wire    [Tag-1:0]                               addr_3;   //[23:0]
	wire    [RC_DATA_width-1:0]                     data_inst32;
	reg     [Cache_cell_Width-1:0]                  mem_cache   [0:Depth-1];
	//RC region
	wire    [Address_width+RC_DATA_width:0]         data_RC1;
	reg     [Address_width+RC_DATA_width:0]         data_RC2;
	reg     wr_RC_internal;
	wire    [Cache_cell_Width-1:0]                  data_instRC; //tag+ 64bit inst 
	wire    [31:0]                                  addr_RC     = data_RC2[95:64];
	wire    [23:0]                                  tag_RC      = addr_RC[31:8];
	wire    [7:0]                                   index_RC    = addr_RC[7:2];
	
	//  initial begin
	//      #10;
	//      mem_cache[8]        = 89'h100000025004000F120FFF9;
	//    end
	
	assign data_RC1             =  {wr_RC,datain_RC};
	always @ (posedge clk)
		begin
			data_RC2                <=  data_RC1;
		end
	
	always @ (*)
		begin
			wr_RC_internal          <= 1'b0;
			if (data_RC2[Address_width+RC_DATA_width]==1'b1)
				if ( !(data_RC1[Address_width+RC_DATA_width]) )
					wr_RC_internal      <= 1'b1;
		end
	
	always @ (posedge clk)
		begin
			if (wr_RC_internal)
				mem_cache[index_RC]   <=  {1'b1, tag_RC, data_RC2[63:0]};
		end  
	
	wire  addr_valid            = (addr_3==addr_tag)?1'b1:1'b0;
	assign  data_inst32         = {32'b0, ROM(addr[31:2])};  
	//assign extend               = (data_instRC [Cache_cell_Width-1]) && (addr_valid);
	assign  extend1             = (data_instRC [Cache_cell_Width-1]) && (addr_valid);
	//assign  extend              = extend1;
	assign  extend              = 1'b0;
	assign  readyn              = 1'b0;
	
	assign  data                = (extend)? data_instRC[63:0]:data_inst32;
	assign  addr_3              = data_instRC[87:64];
	assign  data_instRC         = mem_cache[addr_index];
	
	function [31:0] ROM;
		input [31:0] addrs;
		begin
			case(addrs)
				00: ROM = 32'H00100093;  //addi	x1,x1,1
				01: ROM = 32'hfff74493;	 //not	x9,a4
				02: ROM = 32'Hfff80213;	 //addi	x4,x4,-1
				03: ROM = 32'H00100093;	 //addi	x1,x1,1
				04: ROM = 32'Hffe10113;  //addi	x2,x2,-2
				05: ROM = 32'H00215113;  //srli	a2,x2,0x2
				06: ROM = 32'HDEA00293;  //addi	x5,x10,1
				07: ROM = 32'H00168193;  //addi	x3,x3,1
				08: ROM = 32'H00150513;	 //addi	a0,a0,1
				09: ROM = 32'H00150513;	 //addi	a0,a0,1
				10: ROM = 32'H00150513;	 //addi	a0,a0,1
				11: ROM = 32'H00100093;	 //addi	x1,x1,1
				12: ROM = 32'H00150513;  //addi	a0,a0,1
				13: ROM = 32'H00161613;	 //slli	a2,a2,0x2 //00279793
				14: ROM = 32'H00050613;	 //mv	a2,a0
				15: ROM = 32'H00150513;	 //addi	a0,a0,1
				16: ROM = 32'H00150513;	 //--------------bne	x2,a2,28 <
				/*17: ROM = 32'H00a31aa3;  //sw   x10, 13(x6)
				//17: ROM = 32'H00150513;  
				18: ROM = 32'H00a32823;	 //sw   x10, 16(x6)
				19: ROM = 32'H00150513;
				20: ROM = 32'H02032403;  //lw   x6, 16(x8) 
				*/
				17: ROM = 32'H00a32823;  //sw   x10, 8(x6)
				//17: ROM = 32'H00150513;  
				18: ROM = 32'H00100093;
				19: ROM = 32'H00150513;
				20: ROM = 32'H02032403;  //lw   x6, 16(x8)
				//Start
				21: ROM = 32'H0000f093;  //andi x1,x1,0
				22: ROM = 32'H00017113;  //andi x2,x2,0
				23: ROM = 32'H0001f193;  //andi x3,x3,0
				24: ROM = 32'H00027213;  //andi x4,x4,0
				25: ROM = 32'H0002f293;  //andi x5,x5,0
				26: ROM = 32'Hdead0337;  //lui  x6,0xdead0000
				
				27: ROM = 32'Hbeef0437;  //lui  x8,0xbeef0000
				28: ROM = 32'H01035313;  //srli	x6,x6,0x10
				29: ROM = 32'H01045413;  //srli	x8,x8,0x10
				30: ROM = 32'H0004f493;  //andi x9,x9,0
				31: ROM = 32'H00057513;  //andi a0,a0,0
				32: ROM = 32'H00067613;  //andi a2,a2,0
				33: ROM = 32'H06058593;  //addi	a1,a1,60
				
				// 0007a783  lw	a5,0(a5)
				34: ROM = 32'H00612023;   //sw  rs6, 0(rs2)	 
				35: ROM = 32'H00630233;	  //add x4,x6,x6
				//36: ROM = 32'H0000f093;
				//37: ROM = 32'H0000f093;
				36: ROM = 32'H00434233;	  //xor x4,x4,x6
				37: ROM = 32'H00824333;	  //xor x6,x4,x8
				38: ROM = 32'Hfff58593;   //addi a1,a1,-1
				39: ROM = 32'H00410113;	  //addi x1,x1,4
				40: ROM = 32'Hfeb016e3;	  //bne	x0,a1,34
				41: ROM = 32'H0000f093;
				42: ROM = 32'H0000f093;
				43: ROM = 32'H0000f093;
				44: ROM = 32'H0000f093;
				45: ROM = 32'H0000f093;
				46: ROM = 32'H00110113;
				47: ROM = 32'H00110113;
				//  
				
				
				//32: ROM = 32'h00000000;
				default: ROM = 32'h0;
			endcase
		end
	endfunction
	
endmodule