// Code your testbench here
// or browse Examples

module tb;
  parameter word_size = 32;
  
  wire [word_size-1:0] instruction,  PC_next_normal;
  reg [word_size-1:0] PC_next_branch;
  reg PCSrc, start;
  reg clk, rst;
  
  fetch_unit F0 (instruction, PC_next_normal, PC_next_branch, PCSrc, start, clk, rst);
  
  always begin
    	#5 clk = ~clk;
  end
  
  initial begin
    clk = 0;
    rst = 1;
    start = 1;
    
    F0.instruction_memory.Data_Memory[0] = 8'h2;
    F0.instruction_memory.Data_Memory[1] = 8'h4;
    F0.instruction_memory.Data_Memory[2] = 8'h5;
    F0.instruction_memory.Data_Memory[3] = 8'h6;
    
    F0.instruction_memory.Data_Memory[4] = 8'h1;
    F0.instruction_memory.Data_Memory[5] = 8'h8;
    F0.instruction_memory.Data_Memory[6] = 8'h9;
    F0.instruction_memory.Data_Memory[7] = 8'h10;
    
    F0.instruction_memory.Data_Memory[8] = 8'h13;
    F0.instruction_memory.Data_Memory[9] = 8'h32;
    F0.instruction_memory.Data_Memory[10] = 8'h56;
    F0.instruction_memory.Data_Memory[11] = 8'h69;
        
    F0.start_address = 0;
    
    #1
    rst = 1;
    PCSrc = 0;
    PC_next_branch = 32'd678;
    
    #10
    start = 0;
    PC_next_branch = 32'd698;
    
    #10
    PC_next_branch = 32'h420;
    PCSrc = 0;
    
    #10
    PCSrc = 1;
    
    #10
    $finish;
    
  end
  
  initial begin
  	$dumpfile("dump.vcd"); 
    $dumpvars;
  end
  
endmodule
  