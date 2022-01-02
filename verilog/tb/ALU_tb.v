// Code your testbench here
// or browse Examples
module tb;
  parameter word_size = 32;
  
  wire [word_size-1:0] out;
  wire zero;
  
  reg [word_size-1:0] a, b;
  reg [3:0] aluCTRL;
  
  alu A0 (out, zero, a, b, aluCTRL);
  
  parameter ADD = 4'b0010;
  parameter SUB = 4'b0110;
  parameter AND = 4'b0000;
  parameter OR = 4'b0001;
  parameter SLT = 4'b0111;
  parameter NOR = 4'b1100;
  
  initial begin
    #5;
    aluCTRL = ADD;
    a = 32'd69;
    b = 32'd69;
    
    #5;
    aluCTRL = SUB;
    
    #5;
    aluCTRL = AND;
    
    #5;
    aluCTRL = OR;
    
    #5;
    aluCTRL = SLT;
    
    #5;
    aluCTRL = NOR;
    
    #5
    $finish;
    
  end
  
  initial begin
  	$dumpfile("dump.vcd"); 
    $dumpvars;
  end
  
endmodule