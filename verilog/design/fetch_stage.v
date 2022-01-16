// Code your design here
module fetch_unit (instruction, PC_next_normal, PC_next_branch, PCSrc, start, clk, rst);
  parameter word_size = 32;
  
  // input output declaration
  output [word_size-1:0] instruction;
  inout [word_size-1:0] PC_next_normal;
  input [word_size-1:0] PC_next_branch;
  input PCSrc, start;
  input clk, rst;
  
  // temporary variables
  wire [word_size-1: 0] mux_out, inst_address, reg_out;
  wire cout;
  reg [word_size-1: 0] start_address;
  
  // modules instantiation
  mux_2 mux_fetch (mux_out, PC_next_normal, PC_next_branch, PCSrc);
  defparam mux_fetch.word_size = 32;
  
  Register_Unit PC (reg_out, mux_out, 1'b1, clk, rst);
  
  mux_2 mux_start (inst_address, reg_out, start_address, start);
  defparam mux_start.word_size = 32;
  
  carry_select_32bit CSA_fetch (PC_next_normal, cout, inst_address, 32'd4, 1'b0);
  
  memory_unit instruction_memory (instruction, inst_address, 32'd0, 1'b0, 1'b1, clk);

  
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


module memory_unit (read_data, address, write_data, MemWrite, MemRead, clk);

  //input output declaration
  output reg [31:0] read_data;

  input [31:0] address;
  input MemWrite, MemRead;
  input [31:0] write_data;
  input clk;
  
  //memory declaration
  reg [7:0] Data_Memory [2**16-1:0];

  //memory unit logic
  always@(*) begin

    if (MemRead==1'b1) begin 
      read_data[7:0]   = Data_Memory[address];
      read_data[15:8]  = Data_Memory[address+1];
      read_data[23:16] = Data_Memory[address+2];
      read_data[31:24] = Data_Memory[address+3]; 	
    end
    
    if (clk && MemWrite==1'b1) begin
      Data_Memory[address]   = write_data[7:0];
      Data_Memory[address+1] = write_data[15:8];
      Data_Memory[address+2] = write_data[23:16];
      Data_Memory[address+3] = write_data[31:24];  	
    end

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

