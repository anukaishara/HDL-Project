module instr_memory (
    input wire [7:0] addr,
    output reg [15:0] instr_out
);

    // 256-word instruction memory (16-bit instructions)
    reg [15:0] memory[0:255];
    
    // Initialize with sample program
    initial begin
        // Initialize all to NOP (ADD r0, r0, r0)
        for (integer i = 0; i < 256; i = i + 1)
            memory[i] = 16'b0000_00_00_00_00;
            
        // Sample program - sum numbers from 1 to 10
        memory[0] = 16'b0010_00_01_00001000; // LW r1, 8(r0)   - Load 10 (count) from mem[8]
        memory[1] = 16'b0010_00_10_00001001; // LW r2, 9(r0)   - Load 1 (increment) from mem[9]
        memory[2] = 16'b0010_00_11_00001010; // LW r3, 10(r0)  - Load 0 (initial sum) from mem[10]
        memory[3] = 16'b0000_11_01_11_00;    // ADD r3, r3, r1 - sum = sum + count
        memory[4] = 16'b0001_01_10_01_00;    // SUB r1, r1, r2 - count = count - 1
        memory[5] = 16'b0100_01_00_11111100; // BEQ r1, r0, -4 - Loop if count != 0
        memory[6] = 16'b0011_00_11_00001011; // SW r3, 11(r0)  - Store result to mem[11]
        memory[7] = 16'b0101_00_00000000;    // JMP 0          - Restart program
        
        // Data section
        memory[8] = 16'h000A;  // 10 (count)
        memory[9] = 16'h0001;  // 1 (increment)
        memory[10] = 16'h0000; // 0 (initial sum)
        memory[11] = 16'h0000; // Result will be stored here
    end
    
    // Asynchronous read
    always @(*) begin
        instr_out = memory[addr];
    end
    
endmodule