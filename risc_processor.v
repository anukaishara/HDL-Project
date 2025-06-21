`include "instruction_memory.v"
`include "control_unit.v"
`include "data_memory.v"
`include "reg_file.v"
`include "alu.v"

module risc_processor(
    input clk,
    input reset
);
    // Program Counter
    reg [7:0] PC;
    
    // Instruction Memory Interface
    wire [15:0] instruction;
    
    // Control Signals
    wire RegWrite, ALUSrc, MemtoReg, MemRead, MemWrite, Jump;
    wire [1:0] ALUOp;
    
    // Datapath Signals
    wire [2:0] rd, rs1, rs2;
    wire [7:0] imm_sign_ext, alu_result, mem_read_data, write_data;
    
    // Instruction Fetch
    instruction_memory imem(.addr(PC), .instruction(instruction));
    
    // Control Unit
    control_unit ctrl(
        .opcode(instruction[15:13]),
        .RegWrite(RegWrite),
        .ALUSrc(ALUSrc),
        .MemtoReg(MemtoReg),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .ALUOp(ALUOp),
        .Jump(Jump)
    );
    
    // Register File
    assign rd = instruction[12:10];
    assign rs1 = instruction[9:7];
    assign rs2 = instruction[6:4];
    
    reg_file regfile(
        .clk(clk),
        .write_enable(RegWrite),
        .read_addr1(rs1),
        .read_addr2(rs2),
        .write_addr(rd),
        .write_data(write_data),
        .read_data1(read_data1),
        .read_data2(read_data2)
    );
    
    // Sign Extension (7-bit → 8-bit)
    assign imm_sign_ext = {{1{instruction[6]}}, instruction[6:0]};
    
    // ALU Input Mux
    wire [7:0] alu_input2 = ALUSrc ? imm_sign_ext : read_data2;
    
    // ALU
    alu alu_unit(
        .a(read_data1),
        .b(alu_input2),
        .op(ALUOp),
        .result(alu_result)
    );
    
    // Data Memory
    data_memory dmem(
        .clk(clk),
        .addr(alu_result),
        .write_data(read_data2),
        .mem_write(MemWrite),
        .mem_read(MemRead),
        .read_data(mem_read_data)
    );
    
    // Writeback Mux
    assign write_data = MemtoReg ? mem_read_data : alu_result;
    
    // Next PC Logic
    wire [7:0] PC_next = Jump ? instruction[12:5] : (PC + 2);
    always @(posedge clk or posedge reset) begin
        if (reset) PC <= 8'b0;
        else PC <= PC_next;
    end
endmodule
