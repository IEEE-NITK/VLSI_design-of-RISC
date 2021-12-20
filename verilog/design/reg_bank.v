// Code your design here


module Register_Bank (ReadData1, ReadData2, ReadReg1, ReadReg2, WriteReg, WriteData, RegWrite, clk, rst);
  parameter word_size = 32;
  parameter reg_size = 5;
  parameter RegTotal = 32;
  
  // input output declaration
  input [reg_size-1: 0] ReadReg1, ReadReg2, WriteReg;
  input [word_size-1: 0] WriteData;
  input RegWrite, clk, rst;
  output reg [word_size-1: 0] ReadData1, ReadData2;
  
  // temp wires
  reg [RegTotal-1:0] LoadRegVector;
  wire [word_size-1: 0] RegDataOut [31: 0];
  
  genvar i;
  
  // instantiate register 32 times
  generate
    for (i = 0; i < RegTotal; i = i + 1) begin
      Register_Unit regs (.data_out(RegDataOut[i]), .data_in(WriteData), .load(LoadRegVector[i]), .clk(clk), .rst(rst));
    end
  endgenerate
  
  always @(*) begin 
    ReadData1 = RegDataOut[ReadReg1];
    ReadData2 = RegDataOut[ReadReg2];
  
    if (RegWrite) LoadRegVector = (1 << WriteReg);    // load the particular register to which data is to be loaded
    else LoadRegVector = 0;
  end
  
  initial begin
    LoadRegVector = 0;
  end
  
endmodule


module Register_Unit (data_out, data_in, load, clk, rst);
  parameter word_size = 32;
  
  output [word_size-1: 0] data_out;
  input [word_size-1: 0] data_in;
  input load;
  input clk, rst;
  reg [word_size-1: 0] data_out;

  always @ (posedge clk or negedge rst) begin
    if (rst == 0) data_out <= 0; 
  	else if (load) data_out <= data_in;
  end
endmodule
