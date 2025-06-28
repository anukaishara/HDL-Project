module control_unit (
    input wire [3:0] opcode,
    output wire mem_to_reg,
    output wire reg_write,
    output wire alu_src,
    output wire [1:0] alu_op,
    output wire branch,
    output wire jump,
    output wire data_write
);
    
    // Opcode definitions
    localparam OP_ADD  = 4'b0000;
    localparam OP_SUB  = 4'b0001;
    localparam OP_LW   = 4'b0010;  // LOAD
    localparam OP_SW   = 4'b0011;  // STORE
    localparam OP_BEQ  = 4'b0100;  // Branch if equal
    localparam OP_JMP  = 4'b0101;  // JUMP
    
    // Control signals
    assign reg_write = (opcode == OP_ADD) | (opcode == OP_SUB) | (opcode == OP_LW);
    assign alu_src = (opcode == OP_LW) | (opcode == OP_SW);
    assign mem_to_reg = (opcode == OP_LW);
    assign data_write = (opcode == OP_SW);
    assign branch = (opcode == OP_BEQ);
    assign jump = (opcode == OP_JMP);
    
    // ALU operation
    assign alu_op[1] = (opcode == OP_SUB);
    assign alu_op[0] = (opcode == OP_BEQ);
    
endmodule