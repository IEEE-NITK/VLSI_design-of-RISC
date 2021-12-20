// Code your testbench here
// or browse Examples

module tb;
  parameter word_size = 32;
  parameter reg_size = 5;
  parameter RegTotal = 32;
  
  reg [reg_size-1: 0] ReadReg1, ReadReg2, WriteReg;
  reg [word_size-1: 0] WriteData;
  reg RegWrite, rst;
  reg clk;
  wire [word_size-1: 0] ReadData1, ReadData2;
  
  Register_Bank R (ReadData1, ReadData2, ReadReg1, ReadReg2, WriteReg, WriteData, RegWrite, clk, rst);
  
  always begin
    	#5 clk = ~clk;
  end

  
  initial begin
    clk = 0;
    rst = 1;
    
    #3
    WriteData = 32'h25;   // write data to reg3
    RegWrite = 1;
    WriteReg = 3;
    ReadReg1 = 2;
    ReadReg2 = 6;
    
    #10
    WriteData = 32'h15;  // write data to reg6
    RegWrite = 1;
    WriteReg = 6;
    ReadReg1 = 3;
    ReadReg2 = 6;
    
    #10
    WriteData = 32'h15;  // read data from reg3 and reg6
    RegWrite = 0;
    WriteReg = 6;
    ReadReg1 = 3;
    ReadReg2 = 6;
    
    #10
    WriteData = 32'h75;  // write data to reg6
    RegWrite = 1;
    WriteReg = 6;
    ReadReg1 = 3;
    ReadReg2 = 6;
    
     #10
    WriteData = 32'h15;  // read data from reg8 and reg12
    RegWrite = 0;
    WriteReg = 6;
    ReadReg1 = 8;
    ReadReg2 = 12;
    
     #10
    WriteData = 32'h15;  // read data from reg3 and reg12
    RegWrite = 0;
    WriteReg = 6;
    ReadReg1 = 3;
    ReadReg2 = 12;
    
    #5 
    $finish;
  end

endmodule
    