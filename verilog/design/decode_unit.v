// Code your design here

module decode_unit (ex_control_signals, mem_control_signals, wb_control_signals, PC_out, ReadData1, ReadData2, sign_extend_out, rt, rd, instruction, PC_in, WriteReg, WriteData, RegWrite, clk, rst);
  parameter word_size = 32;
  parameter reg_size = 5;
  parameter opcode_size = 6;
  
  // output declaration
  output [3:0] ex_control_signals;
  output [2:0] mem_control_signals;
  output [1:0] wb_control_signals;
  output [word_size-1: 0] PC_out, ReadData1, ReadData2, sign_extend_out;
  output [reg_size-1:0] rt, rd;
  
  // inputs declaration
  input [word_size-1: 0] PC_in, instruction, WriteData;
  input RegWrite, clk, rst;
  input [reg_size-1:0] WriteReg;
  
  // temp variables
  wire [opcode_size-1:0] opcode;
  wire [reg_size-1:0] rs, rt, rd;
  wire [15:0] address;
  
  // assign outputs or temp variables
  assign rs = instruction[25:21];
  assign rt = instruction[20:16];
  assign rd = instruction[15:11];
  assign PC_out = PC_in;
  assign opcode = instruction[31:26];
  assign address = instruction[15:0];
  
  // instantiate modules
  control_unit C (ex_control_signals, mem_control_signals, wb_control_signals, opcode);
  Register_Bank regs (ReadData1, ReadData2, rs, rt, WriteReg, WriteData, RegWrite, clk, rst);
  sign_extend S (.out(sign_extend_out), .in(address));
	
endmodule


module control_unit (ex_control_signals, mem_control_signals, wb_control_signals, opcode);
  parameter opcode_size = 6;
  
  input [opcode_size-1:0] opcode;
  output [3:0] ex_control_signals;
  output [2:0] mem_control_signals;
  output [1:0] wb_control_signals;
  
  wire RegDst, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp1, ALUOp0;
  wire Rformat, lw, sw, beq;
  wire op2_neg, op3_neg, op4_neg;

  assign op2_neg = ~opcode[2];
  assign op3_neg = ~opcode[3];
  assign op4_neg = ~opcode[4];

  nor a1 (Rformat, opcode[0], opcode[1], opcode[2], opcode[3], opcode[4], opcode[5]);
  and a2 (lw, opcode[0], opcode[1], op2_neg, op3_neg, op4_neg, opcode[5]);
  and a3 (sw, opcode[0], opcode[1], op2_neg, opcode[3], op4_neg, opcode[5]);
  nor a4 (beq, opcode[0], opcode[1], op2_neg, opcode[3], opcode[4], opcode[5]);

  
  assign RegDst = Rformat;
  assign ALUSrc = (lw | sw);
  assign MemtoReg = lw;
  assign RegWrite = (Rformat | lw);
  assign MemRead = lw;
  assign MemWrite = sw;
  assign Branch = beq;
  assign ALUOp1 = Rformat;
  assign ALUOp0 = beq;
  
  assign ex_control_signals = {RegDst, ALUSrc, ALUOp1, ALUOp0};
  assign mem_control_signals = {MemRead, MemWrite, Branch};
  assign wb_control_signals = {RegWrite, MemtoReg};
endmodule


module Register_Bank (ReadData1, ReadData2, ReadReg1, ReadReg2, WriteReg, WriteData, RegWrite, clk, rst);
  parameter word_size = 32;
  parameter reg_size = 5;
  parameter RegTotal = 32;
  
  input [reg_size-1: 0] ReadReg1, ReadReg2, WriteReg;
  input [word_size-1: 0] WriteData;
  input RegWrite, clk, rst;
  output reg [word_size-1: 0] ReadData1, ReadData2;
  
  reg [RegTotal-1:0] LoadRegVector;
  wire [word_size-1: 0] RegDataOut [RegTotal-1: 0];
  
  genvar i;
  
  generate
    for (i = 0; i < RegTotal; i = i + 1) begin
      Register_Unit regs (.data_out(RegDataOut[i]), .data_in(WriteData), .load(LoadRegVector[i]), .clk(clk), .rst(rst));
    end
  endgenerate
  
  always @(*) begin 
    ReadData1 = RegDataOut[ReadReg1];
    ReadData2 = RegDataOut[ReadReg2];
  
    if (RegWrite) LoadRegVector = (1 << WriteReg);   //load signal to the register
    else LoadRegVector = 0;
  end
  
  initial begin
    LoadRegVector = 0;
  end
  
endmodule


module sign_extend (out, in);
  parameter word_size = 32;
  
  input [(word_size/2)-1: 0] in;
  output wire [word_size-1: 0] out;
  
  assign out[(word_size/2)-1: 0] = in;
  assign out[word_size-1: word_size/2] = {(word_size/2){in[(word_size/2)-1]}};
  
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