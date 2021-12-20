// Code your design here

module control_unit (ex_control_signals, mem_control_signals, wb_control_signals, opcode);
  parameter opcode_size = 6;
  
  // input output port declaration
  input [opcode_size-1:0] opcode;
  output [3:0] ex_control_signals;
  output [2:0] mem_control_signals;
  output [1:0] wb_control_signals;
  
  // temp wires
  wire RegDst, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, ALUOp1, ALUOp0;
  wire Rformat, lw, sw, beq;
  wire op2_neg, op3_neg, op4_neg;

  // negation of opcode bits
  assign op2_neg = ~opcode[2];
  assign op3_neg = ~opcode[3];
  assign op4_neg = ~opcode[4];

  // control unit logic
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
  
  // group it wrt the stage in which signals are used
  assign ex_control_signals = {RegDst, ALUSrc, ALUOp1, ALUOp0};
  assign mem_control_signals = {MemRead, MemWrite, Branch};
  assign wb_control_signals = {RegWrite, MemtoReg};
endmodule

