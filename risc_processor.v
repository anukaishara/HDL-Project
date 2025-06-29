// RISC Processor Module: A simple 8-bit RISC processor with basic instructions
// This processor supports a small set of instructions: ADD, SUB, LOAD, STORE, and JUMP.
// It includes a program counter, instruction memory, data memory, ALU, and control unit.


module risc_processor (
    input clk,                    // Clock signal
    input reset,                  // Reset signal (active high)
    input [7:0] external_data_in, // External data input
    output [7:0] data_out         // Data output 
);

    // Internal wires and registers

    // Program counter: current address, next address, and address + 1
    wire [7:0] pc_current;  // Current instruction address
    wire [7:0] pc_next;     // Next instruction address
    wire [7:0] pc_plus1;    // Current address + 1 (for sequential execution)


    // Instruction fetched from memory
    wire [15:0] instruction;


    // Instruction fields
    wire [2:0] opcode;      // 3-bit opcode (ADD, SUB, LOAD, STORE, JUMP)
    wire [1:0] rd;          // Destination/target register (2 bits, 4 registers)
    wire [1:0] rs1;         // Source register 1
    wire [1:0] rs2;         // Source register 2
    wire [7:0] immediate;   // Immediate value or offset

    // Control signals
    wire reg_write;         // Enable register writeback
    wire mem_read;          // Enable memory read
    wire mem_write;         // Enable memory write
    wire alu_src;           // Select ALU input (register or immediate)
    wire alu_op;            // ALU operation (ADD or SUB)
    wire pc_src;            // Select next PC (PC+1 or jump target)

    // Data wires
    wire [7:0] read_data1;  // Data read from register 1
    wire [7:0] read_data2;  // Data read from register 2
    wire [7:0] alu_b;       // ALU input B (from register or immediate)
    wire [7:0] alu_result;  // ALU output
    wire [7:0] mem_read_data; // Data read from memory (including I/O)
    wire [7:0] write_data_reg; // Data to write to register

    // Program Counter module: tracks current instruction address
    pc pc_inst (
        .clk(clk),          // Clock input
        .reset(reset),      // Reset input
        .next_pc(pc_next),  // Next address to jump to
        .current_pc(pc_current) // Current address output
    );

    // Calculate next sequential address
    assign pc_plus1 = pc_current + 1;

    // Determine next PC: jump target (from instruction) or PC+1
    assign pc_next = (pc_src) ? {1'b0, instruction[6:0]} : pc_plus1;

    // Instruction Memory (ROM): stores program instructions
    instruction_memory imem (
        .address(pc_current),   // Address to read from
        .instruction(instruction) // Instruction output
    );

    // Instruction parsing: extract fields from instruction
    assign opcode = instruction[15:13]; // Opcode: bits 15-13
    assign rd = instruction[12:11];     // Destination register: bits 12-11
    assign rs1 = instruction[10:9];     // Source register 1: bits 10-9
    assign rs2 = instruction[8:7];      // Source register 2: bits 8-7
    assign immediate = instruction[7:0]; // Immediate/offset: bits 7-0

    // Control Unit: decodes opcode and generates control signals
    control_unit ctrl (
        .opcode(opcode),      // Opcode input
        .reg_write(reg_write), // Register write enable
        .mem_read(mem_read),   // Memory read enable
        .mem_write(mem_write), // Memory write enable
        .alu_src(alu_src),     // ALU input select
        .alu_op(alu_op),       // ALU operation select
        .pc_src(pc_src)        // PC source select
    );

    // Register File: stores 4 general-purpose 8-bit registers
    register_file rf (
        .reset(reset),         // Reset signal
        .clk(clk),             // Clock input
        .reg_write(reg_write), // Write enable
        .read_reg1(rs1),       // Read register 1 address
        .read_reg2(rs2),       // Read register 2 address
        .write_reg(rd),        // Write register address
        .write_data(write_data_reg), // Data to write
        .read_data1(read_data1),     // Data from register 1
        .read_data2(read_data2)      // Data from register 2
    );

    // ALU input multiplexer: selects between register and immediate
    assign alu_b = (alu_src) ? immediate : read_data2;

    // ALU: performs ADD or SUB
    alu alu_inst (
        .a(read_data1),        // Input A (from register)
        .b(alu_b),             // Input B (from mux)
        .op(alu_op),           // Operation (ADD or SUB)
        .result(alu_result)    // ALU result
    );

    // Data Memory (RAM with I/O): stores data and handles I/O
    data_memory dmem (
        .clk(clk),             // Clock input
        .mem_read(mem_read),   // Read enable
        .mem_write(mem_write), // Write enable
        .address(alu_result),  // Address to read/write
        .write_data(read_data2), // Data to write
        .read_data(mem_read_data), // Data read from memory/I/O
        .external_data_in(external_data_in) // External data input
    );

    // Writeback multiplexer: selects between ALU result and memory data
    assign write_data_reg = (opcode == 3'b010) ? mem_read_data : alu_result;

    // Optional: data output for external devices (e.g., store to output port)
    // If writing to address 0xFF, output the data
    assign data_out = (mem_write && alu_result == 8'hFF) ? read_data2 : 8'h00;

endmodule






// Program Counter module: tracks current instruction address
module pc (
    input clk,
    input reset,
    input [7:0] next_pc,
    output reg [7:0] current_pc
);
    always @(posedge clk or posedge reset) begin
        if (reset) current_pc <= 0;
        else current_pc <= next_pc;
    end
endmodule






// Instruction Memory (ROM): stores program instructions
module instruction_memory (
    input [7:0] address,
    output [15:0] instruction
);
    reg [15:0] rom [0:255];
    integer i;
    initial begin
        // CORRECTED INSTRUCTIONS:
        rom[0] = 16'h4810; // LOAD R1, 0x10(R0)   [010_01_00_00_00010000]
        rom[1] = 16'h1200; // ADD R2, R1, R0      [000_10_01_00_00000000]
        rom[2] = 16'h7020; // STORE R2, 0x20(R0)  [011_10_00_00_00100000]
        rom[3] = 16'h8005; // JUMP 0x05           [100_00_00_00_00000101]
        for (i=4; i<256; i=i+1) rom[i] = 16'h0000;
    end
    assign instruction = rom[address];
endmodule






// Control Unit: decodes opcode and generates control signals
module control_unit (
    input [2:0] opcode,
    output reg reg_write,
    output reg mem_read,
    output reg mem_write,
    output reg alu_src,
    output reg alu_op,
    output reg pc_src
);
    always @(*) begin
        // DEFAULT ALL SIGNALS TO 0
        {reg_write, mem_read, mem_write, alu_src, alu_op, pc_src} = 0;
        
        case (opcode)
            3'b000: begin // ADD
                reg_write = 1;
                alu_op   = 0;
            end
            3'b001: begin // SUB
                reg_write = 1;
                alu_op   = 1;
            end
            3'b010: begin // LOAD
                reg_write = 1;
                mem_read  = 1;
                alu_src   = 1;  // Use immediate
                alu_op    = 0;  // ADD for address calc
            end
            3'b011: begin // STORE
                mem_write = 1;
                alu_src   = 1;  // Use immediate
                alu_op    = 0;  // ADD for address calc
            end
            3'b100: begin // JUMP
                pc_src = 1;
            end
            default: begin // Default case
                reg_write = 0;
                mem_read  = 0;
                mem_write = 0;
                alu_src   = 0;
                alu_op    = 0;
                pc_src    = 0;
            end
        endcase
    end
endmodule






// Register File: stores 4 general-purpose 8-bit registers
module register_file (
    input reset,
    input clk,                // Clock signal
    input reg_write,          // Write enable
    input [1:0] read_reg1,    // Read register 1 address
    input [1:0] read_reg2,    // Read register 2 address
    input [1:0] write_reg,    // Write register address
    input [7:0] write_data,   // Data to write
    output reg [7:0] read_data1, // Data from register 1
    output reg [7:0] read_data2  // Data from register 2
);
    reg [7:0] regs [0:3];     // 4x8-bit registers

    initial begin // Initialize registers to 0
        // Set all registers to 0 at startup
    regs[0] = 0; regs[1] = 0; regs[2] = 0; regs[3] = 0;
    end


    // Write to register on clock edge if write is enabled
    always @(posedge clk or posedge reset) begin
        if (reset) begin
        regs[0] <= 0; regs[1] <= 0; regs[2] <= 0; regs[3] <= 0;
        end else if (reg_write) begin
        regs[write_reg] <= write_data;
        end
    end


    // Read from registers (combinational)
    always @(*) begin
        read_data1 = regs[read_reg1];
        read_data2 = regs[read_reg2];
    end
endmodule





// ALU: performs ADD or SUB
module alu (
    input [7:0] a,           // Input A
    input [7:0] b,           // Input B
    input op,                // Operation (0: ADD, 1: SUB)
    output reg [7:0] result  // Result output
);
    always @(*) begin
        case (op)
            0: result = a + b; // ADD operation
            1: result = a - b; // SUB operation
            default: result = 8'h00; // Default to 0
        endcase
    end
endmodule





// Data Memory (RAM with I/O): stores data and handles I/O
module data_memory (
    input clk,                // Clock signal
    input mem_read,           // Read enable
    input mem_write,          // Write enable
    input [7:0] address,      // Address to read/write
    input [7:0] write_data,   // Data to write
    output reg [7:0] read_data, // Data read from memory/I/O
    input [7:0] external_data_in // External data input
);
    reg [7:0] ram [0:255];    // 256x8 RAM for data

    // Write to RAM on clock edge if write is enabled and not I/O address
    always @(posedge clk) begin
        if (mem_write && address != 8'hFF)
            ram[address] <= write_data;
    end

    // Read from RAM or external input (combinational)
    always @(*) begin
        if (mem_read) begin
            if (address == 8'hFF) // If address is I/O, read external input
                read_data = external_data_in;
            else                  // Otherwise, read from RAM
                read_data = ram[address];
        end else
            read_data = 8'h00;    // Default output if not reading
    end
endmodule