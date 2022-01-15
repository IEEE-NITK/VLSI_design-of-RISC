module ALU_control (ALUOp, Functfield, aluCTRL);
    parameter fun_size = 6;
    
    output [3:0] aluCTRL;
    
    input [1:0] ALUOp;
    input [fun_size-1:0] Functfield;
    
    wire t1,t2;
    
    or (t1, Functfield[0], Functfield[3]);
    and (aluCTRL[0], ALUOp[1], t1);
    and (t2, Functfield[1], ALUOp[1]);
    or (aluCTRL[2], ALUOp[0], t2);
    or (aluCTRL[1], ~ALUOp[1], ~Functfield[2]);
    
    assign aluCTRL[3]= 1'b0;
   
endmodule
