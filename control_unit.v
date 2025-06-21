module control_unit(
    input [2:0] opcode,
    output reg RegWrite,
    output reg ALUSrc,
    output reg MemtoReg,
    output reg MemRead,
    output reg MemWrite,
    output reg [1:0] ALUOp,
    output reg Jump
);
    always @(*) begin
        case(opcode)
            3'b000: begin // ADD
                {RegWrite, ALUSrc, MemtoReg, MemRead, MemWrite, ALUOp, Jump} = 
                {1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 2'b00, 1'b0};
            end
            3'b001: begin // SUB
                {RegWrite, ALUSrc, MemtoReg, MemRead, MemWrite, ALUOp, Jump} = 
                {1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 2'b01, 1'b0};
            end
            3'b010: begin // LOAD
                {RegWrite, ALUSrc, MemtoReg, MemRead, MemWrite, ALUOp, Jump} = 
                {1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 2'b00, 1'b0};
            end
            3'b011: begin // STORE
                {RegWrite, ALUSrc, MemtoReg, MemRead, MemWrite, ALUOp, Jump} = 
                {1'b0, 1'b1, 1'bx, 1'b0, 1'b1, 2'b00, 1'b0};
            end
            3'b100: begin // JUMP
                {RegWrite, ALUSrc, MemtoReg, MemRead, MemWrite, ALUOp, Jump} = 
                {1'b0, 1'bx, 1'bx, 1'b0, 1'b0, 2'bxx, 1'b1};
            end
            default: begin // Default to ADD
                {RegWrite, ALUSrc, MemtoReg, MemRead, MemWrite, ALUOp, Jump} = 
                {1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 2'b00, 1'b0};
            end
        endcase
    end
endmodule
