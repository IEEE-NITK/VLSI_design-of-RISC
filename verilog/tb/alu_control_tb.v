module tb;
	parameter fun_size = 6;
	
	wire [3:0] aluCTRL;

	reg [1:0] ALUOp;
	reg [fun_size-1:0] Functfield;
	
	ALU_control uut (ALUOp, Functfield, aluCTRL);
	
	initial
    begin
    #1
    #10 ALUOp = 2'b00; Functfield = 6'b100000; //lw or sw
    #10 ALUOp = 2'b01; Functfield = 6'b100000; //branch equal
    #10 ALUOp = 2'b10; Functfield = 6'b100000; // R-type 
    #10 ALUOp = 2'b10; Functfield = 6'b100010;
    #10 ALUOp = 2'b10; Functfield = 6'b100100;
    #10 ALUOp = 2'b10; Functfield = 6'b100101;
    #10 ALUOp = 2'b10; Functfield = 6'b101010;
    #10 $finish;
   end

endmodule
