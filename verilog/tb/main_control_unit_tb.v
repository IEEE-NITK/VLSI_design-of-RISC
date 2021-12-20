// Code your testbench here
// or browse Examples

module tb;
  
  reg [5:0] opcode;
  wire [3:0] ex_control_signals;
  wire [2:0] mem_control_signals;
  wire [1:0] wb_control_signals;
  
  control_unit C0 (ex_control_signals, mem_control_signals, wb_control_signals, opcode);
  
  initial begin
    opcode = 0;            // R-format
    #5 opcode = 6'b100011; // lw
    #5 opcode = 6'b101011; // sw
    #5 opcode = 6'b000100; // beq
    #5 opcode = 6'b000110; // none
    #5 $finish;
  end
  
endmodule