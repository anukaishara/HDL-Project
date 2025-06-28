`include "core/control_unit.v"
`include "core/program_counter.v"
`include "core/register_file.v"
`include "core/alu.v"

module risc_core (
    input wire clk,
    input wire reset,
    
    // Instruction memory interface
    output wire [7:0] instr_addr,
    input wire [15:0] instr_data,
    
    // Data memory interface
    output wire [7:0] data_addr,
    output wire data_write,
    output wire [7:0] data_out,
    input wire [7:0] data_in
);

    // Internal signals
    wire [3:0] opcode;
    wire [1:0] rs, rt, rd;
    wire [7:0] immediate;
    wire [7:0] reg_data1, reg_data2;
    wire [7:0] alu_result;
    wire zero_flag;
    wire mem_to_reg;
    wire reg_write;
    wire alu_src;
    wire [1:0] alu_op;
    wire branch;
    wire jump;
    
    // Instruction fields
    assign opcode = instr_data[15:12];
    assign rs = instr_data[11:10];
    assign rt = instr_data[9:8];
    assign rd = instr_data[7:6];
    assign immediate = instr_data[7:0];
    
    // Instantiate control unit
    control_unit ctrl (
        .opcode(opcode),
        .mem_to_reg(mem_to_reg),
        .reg_write(reg_write),
        .alu_src(alu_src),
        .alu_op(alu_op),
        .branch(branch),
        .jump(jump),
        .data_write(data_write)
    );
    
    // Instantiate register file
    register_file reg_file (
        .clk(clk),
        .reset(reset),
        .reg_write(reg_write),
        .read_reg1(rs),
        .read_reg2(rt),
        .write_reg(rd),
        .write_data(mem_to_reg ? data_in : alu_result),
        .read_data1(reg_data1),
        .read_data2(reg_data2)
    );
    
    // Instantiate ALU
    alu alu_unit (
        .a(reg_data1),
        .b(alu_src ? immediate : reg_data2),
        .alu_op(alu_op),
        .result(alu_result),
        .zero(zero_flag)
    );
    
    // Instantiate program counter
    program_counter pc (
        .clk(clk),
        .reset(reset),
        .branch(branch & zero_flag),
        .jump(jump),
        .immediate(immediate),
        .pc(instr_addr)
    );
    
    // Data memory interface
    assign data_addr = alu_result;
    assign data_out = reg_data2;
    
endmodule