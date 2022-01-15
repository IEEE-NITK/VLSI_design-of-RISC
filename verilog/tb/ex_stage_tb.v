module tb;
    parameter word_size = 32;
    parameter reg_size = 5;
    parameter fun_size = 6;
    
    wire zero;
    wire [word_size-1:0] AddResult, alu_result;
    wire [word_size-1:0] WriteData;
    wire [2:0] mem_control_signals_out;
    wire [1:0] wb_control_signals_out;
    wire [reg_size -1:0] destination_reg;
    
    reg [3:0] ex_control_signals;
    reg [reg_size-1:0] rt, rd;
    reg [word_size-1:0] PC_in;
    reg [word_size-1:0] ReadData1, ReadData2, sign_extended;
    reg [2:0] mem_control_signals;
    reg [1:0] wb_control_signals;
    
    parameter rt_in = 6'b010000; 
    parameter rd_in = 6'b010100;
    parameter pc_in = 32'h01;
    
    EXStage uut (WriteData, alu_result, zero, destination_reg, mem_control_signals_out, wb_control_signals_out, PC_in, ReadData1, ReadData2, sign_extended, rt, rd, AddResult, ex_control_signals, mem_control_signals, wb_control_signals);
	
    initial begin
    #1
    #10 PC_in = pc_in;                       //add
        rt = rt_in; 
	rd = rd_in; 
	ex_control_signals = 4'b1010; 
	ReadData1 = 32'h06; 
	ReadData2 = 32'h06; 
	mem_control_signals = 3'b111; 
	wb_control_signals = 2'b11;
	sign_extended = 32'b100000;

    #10 PC_in = pc_in;                      //subtract
	rt = rt_in; 
	rd = rd_in; 
	ex_control_signals = 4'b0010; 
	ReadData1 = 32'h06; 
	ReadData2 = 32'h06; 
	mem_control_signals = 3'b111;
	wb_control_signals = 2'b11; 
	sign_extended = 32'b100010;
    
    #10 PC_in = pc_in;                      //set less than
	rt = rt_in; 
	rd = rd_in; 
	ex_control_signals = 4'b1010; 
	ReadData1 = 32'h06; 
	ReadData2 = 32'h07; 
	mem_control_signals = 3'b111; 
	wb_control_signals = 2'b11; 
	sign_extended = 32'b101010;
    
    #10 $finish;
   end
  
endmodule