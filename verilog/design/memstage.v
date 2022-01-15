module MEMStage (Address, WriteData, AddResult, ReadData, mem_control_signals, wb_control_signals, zero, wb_control_signals_out, clk, AluResult_out, destination_reg, destination_reg_out);
    parameter word_size = 32;
    parameter reg_size = 5;

    output [word_size-1:0] ReadData;
    output [word_size-1:0] AluResult_out;
    output [reg_size-1:0] destination_reg_out;
    output [1:0] wb_control_signals_out;
    
    input [2:0] mem_control_signals;
    input [1:0] wb_control_signals;
    input [word_size-1:0] AddResult;
    input [word_size-1:0] WriteData;
    input [reg_size-1:0] destination_reg;
    input [word_size-1:0] Address;
    input zero;
    input clk;
    
    wire MemRead, MemWrite, Branch;
    wire Branch_out;
    
    
    assign {MemRead, MemWrite, Branch} = mem_control_signals;
    assign wb_control_signals_out = wb_control_signals;
    assign AluResult_out = Address;
    assign destination_reg_out = destination_reg;
    
    and (Branch_out, Branch, zero);

    //instantiate module
    memory_unit MEM (ReadData, Address, WriteData,  MemWrite, MemRead, clk);
    

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
