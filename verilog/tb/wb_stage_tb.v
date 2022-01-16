module tb;
    parameter word_size = 32;
    parameter reg_size = 5;
    
    wire [word_size-1:0] WriteData;
    wire [reg_size-1:0] WriteReg;
    wire RegWrite;

    reg [1:0] wb_control_signals;
    reg [word_size-1:0] ReadData;
    reg [reg_size-1:0] destination_reg;
    reg [word_size-1:0] AluResult;
    
    WBStage uut (WriteReg, WriteData, RegWrite, ReadData, AluResult, destination_reg, wb_control_signals);
	
    initial begin
    #10 wb_control_signals = 2'b00; ReadData = 32'h16; destination_reg = 5'h02; AluResult = 32'h014;
    #10 wb_control_signals = 2'b01; ReadData = 32'h16; destination_reg = 5'h02; AluResult = 32'h014; //Write data
    #10 wb_control_signals = 2'b10; ReadData = 32'h16; destination_reg = 5'h02; AluResult = 32'h014; // Write to Register
    #10 $finish;
   end
  
endmodule
