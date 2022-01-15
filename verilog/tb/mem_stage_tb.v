module tb;

    parameter word_size = 32;
    parameter reg_size = 5;

    wire [word_size-1:0] ReadData;
    wire [word_size-1:0] AluResult_out;
    wire [reg_size-1:0] destination_reg_out;
    wire [1:0] wb_control_signals_out;
    
    reg [2:0] mem_control_signals;
    reg [1:0] wb_control_signals;
    reg [word_size-1:0] AddResult;
    reg [word_size-1:0] WriteData;
    reg [reg_size-1:0] destination_reg;
    reg [word_size-1:0] Address;
    reg zero;
    reg clk;

 
    MEMStage uut (Address, WriteData, AddResult, ReadData, mem_control_signals, wb_control_signals, zero, wb_control_signals_out,clk,AluResult_out,destination_reg,destination_reg_out);
    initial
    begin
    clk=0;
    forever #5 clk=~clk;
    end

    initial begin
    #10 Address=32'h16;
        AddResult=32'h10;
    	WriteData=32'h15;
    	mem_control_signals=3'b001;
    	wb_control_signals=2'b11;
    	zero=1'b0;
    	destination_reg = 5'h02;
    
    #10 Address=32'h16;
    	AddResult=32'h10;
    	WriteData=32'h15;
    	mem_control_signals=3'b110;
    	wb_control_signals=2'b11;
    	zero=1'b1;
    	destination_reg = 5'h03;
    
    #10 Address=32'h18;
    	AddResult=32'h11;
    	WriteData=32'h16;
    	mem_control_signals=3'b000;
    	wb_control_signals=2'b11;
    	zero=1'b1;
    	destination_reg = 5'h04;
    
    #10 Address=32'h18;
    	AddResult=32'h11;
    	WriteData=32'h16;
    	mem_control_signals=3'b110;
    	wb_control_signals=2'b11;
    	zero=1'b0;
    	destination_reg = 5'h05;
    end

endmodule
