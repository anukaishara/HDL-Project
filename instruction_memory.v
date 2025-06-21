// Instruction Memory (256 bytes)
module instruction_memory(
    input [7:0] addr,
    output [15:0] instruction
);
    reg [7:0] mem [0:255];
    assign instruction = {mem[addr], mem[addr+1]}; // 16-bit fetch
endmodule

// Data Memory (256 bytes)
module data_memory(
    input clk,
    input [7:0] addr,
    input [7:0] write_data,
    input mem_write,
    input mem_read,
    output reg [7:0] read_data
);
    reg [7:0] mem [0:255];
    
    always @(*) begin
        if (mem_read) read_data = mem[addr];
    end
    
    always @(posedge clk) begin
        if (mem_write) mem[addr] <= write_data;
    end
endmodule
