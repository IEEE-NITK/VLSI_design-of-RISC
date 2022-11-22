module tb();
  
  reg clk, rst, start;

  always begin
    #5 clk = ~clk;
  end
  
  pipelined_RISC P1 (clk, rst, start);
  
  initial begin
    clk = 0;
    rst = 1;
    start = 1;
    P1.F.start_address = 0;
    
    P1.F.instruction_memory.Data[0] = 8'h0;
    P1.F.instruction_memory.Data[1] = 8'h0;
    P1.F.instruction_memory.Data[2] = 8'h3;
    P1.F.instruction_memory.Data[3] = 8'h8C;
    
    P1.M.data_memory.Data[0] = 8'hA;
    P1.M.data_memory.Data[1] = 8'h0;
    P1.M.data_memory.Data[2] = 8'h0;
    P1.M.data_memory.Data[3] = 8'h0;
    
    
    P1.F.instruction_memory.Data[4] = 8'h4;
    P1.F.instruction_memory.Data[5] = 8'h0;
    P1.F.instruction_memory.Data[6] = 8'h4;
    P1.F.instruction_memory.Data[7] = 8'h8C;
    
    P1.M.data_memory.Data[4] = 8'h9;
    P1.M.data_memory.Data[5] = 8'h0;
    P1.M.data_memory.Data[6] = 8'h0;
    P1.M.data_memory.Data[7] = 8'h0;
    
    P1.F.instruction_memory.Data[8] = 8'h20;
    P1.F.instruction_memory.Data[9] = 8'h28;
    P1.F.instruction_memory.Data[10] = 8'h83;
    P1.F.instruction_memory.Data[11] = 8'h0;
    
    #1
    rst = 1;
    
    #10
    start = 0;
    
    #70
    $finish;
    
  end
  
  initial begin
  	$dumpfile("dump.vcd"); 
    $dumpvars;
  end
    
endmodule
