module EXStage (WriteData, alu_result, zero, destination_reg, mem_control_signals_out, wb_control_signals_out, PC_in, ReadData1, ReadData2, sign_extended, rt, rd, AddResult, ex_control_signals, mem_control_signals, wb_control_signals);
  parameter word_size = 32;
  parameter reg_size = 5;
  parameter fun_size = 6;
    
  output zero;
  output [word_size-1:0] AddResult, alu_result;
  output [word_size-1:0] WriteData;
  output [reg_size-1:0] destination_reg;
  output [2:0] mem_control_signals_out;
  output [1:0] wb_control_signals_out;
   
    
  input [3:0] ex_control_signals;
  input [reg_size-1:0] rt, rd;
  input [word_size-1:0] PC_in;
  input [word_size-1:0] ReadData1, ReadData2, sign_extended;
  input [2:0] mem_control_signals;
  input [1:0] wb_control_signals;
    
  wire [word_size-1:0] mux_output;
  wire [fun_size-1:0] Functfield;
  wire [word_size-1:0] shift_left_out;
  wire [3:0] aluCTRL;
  wire RegDst, ALUSrc, ALUOp1, ALUOp0,Cout;
  wire [1:0] ALUOp;
    
  assign {RegDst, ALUSrc, ALUOp1, ALUOp0} = ex_control_signals;
  assign Functfield = sign_extended [5:0];
  assign ALUOp = {ALUOp1,ALUOp0};
  assign WriteData = ReadData2;
  assign mem_control_signals_out = mem_control_signals;
  assign wb_control_signals_out = wb_control_signals;
   
  //instantiate modules 
  mux_2 MUX1 (mux_output, ReadData2, sign_extended, ALUSrc);
  defparam MUX1.word_size = 32;
  
  mux_2 MUX2 (destination_reg, rt, rd, RegDst);
  defparam MUX2.word_size = 5;
  
  ALU_control alc (aluCTRL, ALUOp, Functfield);
  Shiftleft sl2 (.dout(shift_left_out), .din(sign_extended));
  carry_select_32bit Add (AddResult, Cout, PC_in, shift_left_out, 1'b0);
  alu  ALU (alu_result, zero, ReadData1, mux_output, aluCTRL);
    
endmodule

module Shiftleft(dout, din);
  parameter word_size = 32;
    
  output [word_size-1:0] dout;
  input [word_size-1:0] din;  
    
  assign dout = din << 2;  
    
endmodule 

module ALU_control (aluCTRL, ALUOp, Functfield);
  parameter fun_size = 6;
    
  output [3:0] aluCTRL;
    
  input [1:0] ALUOp;
  input [fun_size-1:0] Functfield;
    
    
  wire t1, t2;
    
  or (t1, Functfield[0], Functfield[3]);
  and (aluCTRL[0], ALUOp[1], t1);
  and (t2, Functfield[1], ALUOp[1]);
  or (aluCTRL[2], ALUOp[0], t2);
  or (aluCTRL[1], ~ALUOp[1], ~Functfield[2]);
    
  assign aluCTRL[3]= 1'b0;
  
endmodule

module alu (alu_result, zero, data1, data2, aluCTRL);
  parameter word_size = 32;
  
  // output input declaration
  output zero;
  output reg [word_size-1:0] alu_result;
  
  input [word_size-1:0] data1, data2;
  input [3:0] aluCTRL;
  
  
  // temporary variables 
  reg [word_size-1:0] b;    // For intermediate variable containing ~data2 for subtraction 
  reg [word_size-1:0] out;  // variable to hold output
  wire [word_size-1:0] out_addsub;  // variable to hold out of carry select adder
  
  reg cin;
  wire cout;
  
  
  carry_select_32bit CSA (out_addsub, cout, data1, b, cin);
  
  assign zero = ~|alu_result;  // zero flag
  
  parameter ADD = 4'b0010;
  parameter SUB = 4'b0110;
  parameter AND = 4'b0000;
  parameter  OR = 4'b0001;
  parameter SLT = 4'b0111;
  parameter NOR = 4'b1100;
	
  always @(aluCTRL or data1 or data2) begin
    
	case (aluCTRL)
      
      ADD: begin
        b = data2;
        cin = 1'b0;
      end
      

      SUB: begin
        b = ~data2;
        cin = 1'b1;
      end
      

      AND: out = data1 & data2;
      
      OR: out = data1 | data2;
      
      NOR: out = ~(data1 | data2);
      
      SLT: begin
        if(data1 < data2) out = 1'b1;
        else out = 1'b0;
      end

	endcase
  end
  
  always @(*) begin
    if (aluCTRL == ADD | aluCTRL == SUB) alu_result = out_addsub;
    else alu_result = out;
  end
  
endmodule



module carry_select_32bit (out, cout, a, b, cin);
  parameter word_size = 32;
  
  output [word_size-1:0] out;
  output cout;
  
  input [word_size-1:0] a, b;
  input cin;
  
  wire c4, c8, c12, c16, c20, c24, c28;
  
  ripple_carry_4bit unit0 (out[3:0], c4, a[3:0], b[3:0], cin);
  adder_4bit unit1 (out[7:4], c8, a[7:4], b[7:4], c4);
  adder_4bit unit2 (out[11:8], c12, a[11:8], b[11:8], c8);
  adder_4bit unit3 (out[15:12], c16, a[15:12], b[15:12], c12);
  adder_4bit unit4 (out[19:16], c20, a[19:16], b[19:16], c16);
  adder_4bit unit5 (out[23:20], c24, a[23:20], b[23:20], c20);
  adder_4bit unit6 (out[27:24], c28, a[27:24], b[27:24], c24);
  adder_4bit unit7 (out[31:28], cout, a[31:28], b[31:28], c28);
  
endmodule



module adder_4bit (out, cout, a, b, cin);
  
  output [3:0] out;
  output cout;
  
  input [3:0] a, b;
  input cin;
  
  wire [3:0] out0, out1;
  wire cout_0, cout_1;
  
  ripple_carry_4bit RCA0 (out0, cout_0, a, b, 1'b0);
  ripple_carry_4bit RCA1 (out1, cout_1, a, b, 1'b1);
  
  mux_2 mbit0 (out[0], out0[0], out1[0], cin);
  mux_2 mbit1 (out[1], out0[1], out1[1], cin);
  mux_2 mbit2 (out[2], out0[2], out1[2], cin);
  mux_2 mbit3 (out[3], out0[3], out1[3], cin);
  mux_2 carry (cout, cout_0, cout_1, cin);
  
endmodule



module ripple_carry_4bit (out, cout, a, b, cin);
  
  output [3:0] out;
  output cout;
  
  input [3:0] a, b;
  input cin;
  
  wire c1, c2, c3;
  
  full_adder F0 (out[0], c1, a[0], b[0], cin);
  full_adder F1 (out[1], c2, a[1], b[1], c1);
  full_adder F2 (out[2], c3, a[2], b[2], c2);
  full_adder F3 (out[3], cout, a[3], b[3], c3);
  
endmodule



module full_adder (out, cout, a, b, cin);  
  
  output reg out, cout;
  
  input a, b, cin;
 
  always @ (*) begin
    out = a ^ b ^ cin;
    cout = (a & b) | (b & cin) | (a & cin);
  end
  
endmodule

  

module mux_2 (mux_out, data_a, data_b, sel);
  parameter word_size = 1;
  
  output reg [word_size-1: 0] mux_out;
  input [word_size-1: 0] data_a, data_b;
  input sel;

  always @ (*) begin
  	mux_out = (sel == 0) ? data_a: data_b; 
  end
  
endmodule