`include "memory/data_memory.v"
`include "memory/instr_memory.v"
`include "risc_core.v"


module risc_system (
    input wire clk,
    input wire reset
);

    // Processor-memory interfaces
    wire [7:0] instr_addr;
    wire [15:0] instr_data;
    wire [7:0] data_addr;
    wire data_write;
    wire [7:0] data_out;
    wire [7:0] data_in;
    
    // Instantiate the processor core
    risc_core processor (
        .clk(clk),
        .reset(reset),
        .instr_addr(instr_addr),
        .instr_data(instr_data),
        .data_addr(data_addr),
        .data_write(data_write),
        .data_out(data_out),
        .data_in(data_in)
    );
    
    // Instantiate instruction memory
    instr_memory imem (
        .addr(instr_addr),
        .instr_out(instr_data)
    );
    
    // Instantiate data memory
    data_memory dmem (
        .clk(clk),
        .reset(reset),
        .addr(data_addr),
        .write_en(data_write),
        .data_in(data_out),
        .data_out(data_in)
    );
    
endmodule