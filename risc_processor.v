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
    wire [1:0]alu_op;            // ALU operation (ADD or SUB)
    wire pc_src;            // Select next PC (PC+1 or jump target)
    wire mem_to_reg;        // Select data source for register write (ALU or memory)

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
    assign pc_next = (pc_src) ? immediate : pc_plus1;

    // Instruction Memory (ROM): stores program instructions
    instruction_memory imem (
        .clk(clk),             // Clock input
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
        .pc_src(pc_src),        // PC source select
        .mem_to_reg(mem_to_reg) // Memory to register select
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
    assign write_data_reg = (mem_to_reg) ? mem_read_data : alu_result;

    // Optional: data output for external devices (e.g., store to output port)
    // If writing to address 0xFF, output the data
    assign data_out = (mem_write && alu_result == 8'hFF) ? read_data2 : 8'h00;

endmodule






// Program Counter module: tracks current instruction address
module pc (
    input clk,           // Clock input
    input reset,         // Reset input
    input [7:0] next_pc,    // Next address to jump to
    output reg [7:0] current_pc // Current instruction address
);
    always @(posedge clk or posedge reset) begin
        if (reset) current_pc <= 0; // Reset to address 0
        else current_pc <= next_pc; // Update to next address
    end
endmodule





// Instruction Memory: stores program instructions in ROM
module instruction_memory (
    inout clk,
    input [7:0] address, // Address to read instruction from
    output reg [15:0] instruction // Instruction output
);
    parameter MEM_INIT_FILE = ""; // Memory initialization file (hex format)
    reg [15:0] rom [0:255]; // ROM memory
    
    initial begin
        if (MEM_INIT_FILE != "") begin 
            // Initialize ROM from file
            $readmemh(MEM_INIT_FILE, rom); 
        end else begin
            // Default program
            rom[0] = 16'h4810; // LOAD R1, 0x10(R0)   ; R1 = MEM[0x10]
            rom[1] = 16'h1200; // ADD R2, R1, R0      ; R2 = R1 + R0
            rom[2] = 16'h7020; // STORE R2, 0x20(R0)  ; MEM[0x20] = R2
            rom[3] = 16'h8005; // JUMP 0x05           ; Jump to instruction 5
            rom[5] = 16'h48FF; // LOAD R1, 0xFF(R0)   ; R1 = external_data_in
            rom[6] = 16'h1D80; // SUB R3, R1, R2      ; R3 = R1 - R2

            for (integer i=7; i<256; i++) rom[i] = 16'h0000;
        end
    end
    
    always @(posedge clk) begin
        instruction = rom[address]; // Read instruction from ROM
    end
endmodule





// Control Unit
module control_unit (
    input [2:0] opcode,
    output reg reg_write,
    output reg mem_read,
    output reg mem_write,
    output reg alu_src,
    output reg [1:0] alu_op,
    output reg pc_src,
    output reg mem_to_reg
);
    always @(*) begin
        // Default
        {reg_write, mem_read, mem_write, alu_src, alu_op, pc_src, mem_to_reg} = 9'b0;

        case (opcode)
            3'b000: begin // ADD
                reg_write = 1;
                alu_op = 2'b00;
            end
            3'b001: begin // SUB
                reg_write = 1;
                alu_op = 2'b01;
            end
            3'b010: begin // LOAD
                reg_write = 1;
                mem_read = 1;
                alu_src = 1;
                alu_op = 2'b00;
                mem_to_reg = 1;
            end
            3'b011: begin // STORE
                mem_write = 1;
                alu_src = 1;
                alu_op = 2'b00;
            end
            3'b100: begin // JUMP
                pc_src = 1;
            end
            3'b111: begin
            end
            default: ; // NOP or undefined
        endcase
    end
endmodule






// Register File: stores 4 general-purpose 8-bit registers
module register_file (
    input reset,
    input clk,
    input reg_write,
    input [1:0] read_reg1,
    input [1:0] read_reg2,
    input [1:0] write_reg,
    input [7:0] write_data,
    output reg [7:0] read_data1,
    output reg [7:0] read_data2
);
    reg [7:0] regs [0:3];
    
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            regs[0] <= 8'h00;
            regs[1] <= 8'h00;
            regs[2] <= 8'h00;
            regs[3] <= 8'h00;
        end
        else if (reg_write) begin
            regs[write_reg] <= write_data;
        end
    end
    
    always @(*) begin
        read_data1 = regs[read_reg1];
        read_data2 = regs[read_reg2];
    end
endmodule






// ALU: performs ADD or SUB
module alu (
    input [7:0] a,
    input [7:0] b,
    input [1:0] op,              // Now 2-bit ALU op
    output reg [7:0] result
);
    always @(*) begin
        case (op)
            2'b00: result = a + b;  // ADD
            2'b01: result = a - b;  // SUB
            default: result = 8'h00;
        endcase
    end
endmodule





// Data Memory (RAM with I/O): stores data and handles I/O
module data_memory (
    input clk,
    input mem_read,
    input mem_write,
    input [7:0] address,
    input [7:0] write_data,
    output reg [7:0] read_data,
    input [7:0] external_data_in
);
    reg [7:0] ram [0:255];
    
    always @(posedge clk) begin
        if (mem_write && address != 8'hFF)
            ram[address] <= write_data;
            
        if (mem_read) begin
            if (address == 8'hFF)
                read_data <= external_data_in;
            else
                read_data <= ram[address];
        end
    end
endmodule