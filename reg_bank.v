`timescale 1ns/1ps
module reg_bank
	(input				rst_i
	,input				clk_i
	,input		[4:0]	read1, read2, read3, read4
	,output reg	[31:0]	data1, data2, data3, data4
	,input				regwrite
	,input		[4:0]	wrreg
	,input		[31:0]	wrdata
	,input  			wait_i
	);
	
	reg 		[31:0]  r [0:31];  // 32-bit memory with 32 entries
	reg 		[31:0]  data_1, data_2, data_3, data_4;
	
	always @(read1 or wrreg or regwrite or wrdata ) begin
			if (read1 == 5'd0)
				data_1 = 32'd0;
			else if ((read1 == wrreg) && regwrite)
				data_1 = wrdata;
			else
				data_1 = r[read1];
		end
	always @(read2 or wrreg or regwrite or wrdata ) begin
			if (read2 == 5'd0)
				data_2 = 32'd0;
			else if ((read2 == wrreg) && regwrite)
				data_2 = wrdata;
			else
				data_2 = r[read2];
		end
	
	//  always @(read3 or wrreg or regwrite or wrdata ) begin
	//      if (read3 == 5'd0)
	//        data_3 = 32'd0;
	//      else if ((read3 == wrreg) && regwrite)
	//        data_3 = wrdata;
	//      else
	//        data_3 = r[read3][31:0];
	//    end
	//  
	//  always @(read4 or wrreg or regwrite or wrdata ) begin
	//      if (read4 == 5'd0)
	//        data_4 = 32'd0;
	//      else if ((read4 == wrreg) && regwrite)
	//        data_4 = wrdata;
	//      else
	//        data_4 = r[read4][31:0];
	//    end
	
	always @(posedge rst_i or posedge clk_i)
		if (rst_i)
			begin
				r[0]<= 32'b0;
				r[1]<= 32'b0;r[2]<= 32'b0;r[3]<= 32'b0;r[4]<= 32'b0;r[5]<= 32'b0;r[6]<= 32'b0;r[7]<= 32'b0;r[8]<= 32'b0;r[9]<= 32'b0;
				r[10]<= 32'b0;r[11]<= 32'b0;r[12]<= 32'b0;r[13]<= 32'b0;r[14]<= 32'b0;r[15]<= 32'b0;r[16]<= 32'b0;r[17]<= 32'b0;r[18]<= 32'b0;
				r[19]<= 32'b0;r[20]<= 32'b0;r[21]<= 32'b0;r[22]<= 32'b0;r[23]<= 32'b0;r[24]<= 32'b0;r[25]<= 32'b0;r[26]<= 32'b0;r[27]<= 32'b0;
				r[28]<= 32'b0;r[29]<= 32'b0;r[30]<= 32'b0;r[31]<= 32'b0;
			end
		else if (regwrite && wrreg != 5'd0)
			r[wrreg] <= wrdata;// write a non $zero register
	
	always @(posedge rst_i or posedge clk_i)
		if (rst_i)
			begin
				data1 <= 32'b0;
				data2 <= 32'b0;
				data3 <= 32'b0;
				data4 <= 32'b0;
			end
		else if (!wait_i)
			begin
				data1 <= data_1;
				data2 <= data_2;
				data3 <= data_3;
				data4 <= data_4;
			end
	
endmodule