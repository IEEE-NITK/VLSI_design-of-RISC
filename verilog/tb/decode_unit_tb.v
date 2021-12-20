// Code your testbench here
// or browse Examples

module tb;
  parameter word_size = 32;
  parameter reg_size = 5;
  parameter opcode_size = 6;
  
  wire [3:0] ex_control_signals;
  wire [2:0] mem_control_signals;
  wire [1:0] wb_control_signals;
  wire [word_size-1: 0] PC_out, ReadData1, ReadData2, sign_extend_out;
  wire [reg_size-1:0] rt, rd;
  
  reg [word_size-1: 0] PC_in, instruction, WriteData;
  reg RegWrite, clk, rst;
  reg [reg_size-1:0] WriteReg;
  
  decode_unit decode (ex_control_signals, mem_control_signals, wb_control_signals, PC_out, ReadData1, ReadData2, sign_extend_out, rt, rd, instruction, PC_in, WriteReg, WriteData, RegWrite, clk, rst);
  
  always begin
    	#5 clk = ~clk;
  end
  
  initial begin
    clk = 0;
    rst = 1;
    
    
    WriteData = 32'h25;   // write data to registers
    RegWrite = 1;
    WriteReg = 3;
    
    #10
    WriteData = 32'h15;
    RegWrite = 1;
    WriteReg = 6;
    
    #10
    WriteData = 32'h75;
    RegWrite = 1;
    WriteReg = 6;
    
    #10
    WriteData = 32'h16;
    RegWrite = 1;
    WriteReg = 8;
    
    #10 
    PC_in = 32'h15;    // Read data
    instruction = {6'b000000, 5'b01000, 5'b00110, 16'hB5A6};
    RegWrite = 0;
    
    #10
    PC_in = 32'h45;  // Write data
    instruction = {6'b000000, 5'b00011, 5'b00110, 16'hC526};
    RegWrite = 1;
    WriteReg = 6;
    WriteData = 32'h62;
    
    #15 
    $finish;
  end
endmodule
  