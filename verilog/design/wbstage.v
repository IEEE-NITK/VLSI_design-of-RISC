module WBStage (wb_control_signals, ReadData, AluResult, WriteReg, destination_reg, WriteData);
    parameter word_size = 32;
    parameter reg_size = 5;
    
    output [word_size-1:0] WriteData;
    output [reg_size-1:0] WriteReg;

    input [1:0] wb_control_signals;
    input [word_size-1:0] ReadData;
    input [reg_size-1:0] destination_reg;
    input [word_size-1:0] AluResult;

    wire RegWrite, MemtoReg;
    assign {RegWrite, MemtoReg} = wb_control_signals;
    assign WriteReg = destination_reg;
    
    //instantiate module
    mux_2 MUX3(WriteData, ReadData, AluResult, MemtoReg);
    defparam MUX3.word_size = 32;
    
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
