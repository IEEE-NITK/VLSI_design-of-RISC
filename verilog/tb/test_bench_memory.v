// Code your testbench here

module tb;
  
  wire [31:0] read_data;
  reg [31:0] address;
  reg [31:0] write_data;
  reg MemWrite,MemRead,clk;
 
  
  memory_unit C0 (address, write_data, read_data, MemWrite, MemRead, clk);

  initial begin
	clk=0;
	forever #5 clk=~clk;
  end



  initial begin
    #1  
    #10 address=10'b00001010; write_data=32'h000001;MemWrite=1'b1; MemRead=1'b0; //write data
    #10  MemRead= 1'b1; MemWrite=1'b0; //read data
    #10 MemWrite=1'b0; MemRead= 1'b1;
    #10 $finish;
  end
  
  initial begin
  	$dumpfile("dump.vcd"); 
    $dumpvars;
  end

endmodule