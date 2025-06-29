`include "risc_processor.v"
`timescale 1ns / 1ps

module tb_risc_processor();
    reg clk;
    reg reset;
    reg [7:0] external_data_in;
    wire [7:0] data_out;

    // Instantiate the processor
    risc_processor uut (
        .clk(clk),
        .reset(reset),
        .external_data_in(external_data_in),
        .data_out(data_out)
    );

    // Clock generation (100 MHz)
    always #5 clk = ~clk;

    // Hierarchical monitoring
    wire [7:0]  pc_current       = uut.pc_current;
    wire [15:0] instruction      = uut.instruction;
    wire [2:0]  opcode           = uut.opcode;
    wire [7:0]  regs_0           = uut.rf.regs[0];
    wire [7:0]  regs_1           = uut.rf.regs[1];
    wire [7:0]  regs_2           = uut.rf.regs[2];
    wire [7:0]  regs_3           = uut.rf.regs[3];
    wire        pc_src           = uut.pc_src;

    // VCD dump for GTKWave
    initial begin
        $dumpfile("risc_processor_tb.vcd");
        $dumpvars(0, tb_risc_processor);
    end

    // Test sequence
    initial begin
        // Initialize signals
        clk = 0;
        reset = 1;
        external_data_in = 0;

        // Preload data memory
        uut.dmem.ram[8'h10] = 8'hA5;
        uut.dmem.ram[8'h20] = 8'h00;

        // Reset sequence
        #10 reset = 0;
        #10 reset = 1;
        #20 reset = 0;

        $display("--- [TEST 1: RESET] ---");
        $display("PC after reset: %h (expected: 00)", pc_current);
        $display("Registers after reset: R0=%h, R1=%h, R2=%h, R3=%h", 
                 regs_0, regs_1, regs_2, regs_3);
        #10;

        // Test 2: LOAD instruction
        $display("\n--- [TEST 2: LOAD] ---");
        #20;
        $display("After LOAD: R1 = %h (expected: A5)", regs_1);
        if (regs_1 !== 8'hA5) $error("LOAD test failed!");

        // Test 3: ADD instruction
        $display("\n--- [TEST 3: ADD] ---");
        #20;
        $display("After ADD: R2 = %h (expected: A5)", regs_2);
        if (regs_2 !== 8'hA5) $error("ADD test failed!");

        // Test 4: STORE instruction (FIXED)
        $display("\n--- [TEST 4: STORE] ---");
        #20;
        @(posedge clk); // WAIT FOR CLOCK EDGE
        #5;  // Small delay after clock edge
        $display("Data at 0x20: %h (expected: A5)", uut.dmem.ram[8'h20]);
        if (uut.dmem.ram[8'h20] !== 8'hA5) $error("STORE test failed!");

        // Test 5: JUMP instruction (FIXED)
        $display("\n--- [TEST 5: JUMP] ---");
        #20;
        $display("PC after JUMP: %h (expected: 05)", pc_current);
        $display("pc_src = %b, instruction = %h", pc_src, instruction);
        if (pc_current !== 8'h05) $error("JUMP test failed!");

        // Test 6: I/O Read (FIXED)
        $display("\n--- [TEST 6: I/O READ] ---");
        external_data_in = 8'hF0;
        uut.imem.rom[5] = 16'h48FF; // LOAD R1, 0xFF(R0) [010_01_00_00_11111111]
        #20;
        $display("R1 after I/O read: %h (expected: F0)", regs_1);
        if (regs_1 !== 8'hF0) $error("I/O read test failed!");

        // Test 7: SUB instruction (FIXED)
        $display("\n--- [TEST 7: SUB] ---");
        uut.imem.rom[6] = 16'h1D80; // SUB R3, R1, R2 [001_11_01_10_00000000]
        #20;
        $display("After SUB: R3 = %h (expected: 4B)", regs_3);
        if (regs_3 !== 8'h4B) $error("SUB test failed!");

        // Test 8: Reset recovery
        $display("\n--- [TEST 8: RESET RECOVERY] ---");
        #10 reset = 1;
        #20 reset = 0;
        $display("PC after reset: %h (expected: 00)", pc_current);
        if (pc_current !== 8'h00) $error("Reset recovery failed!");

        $display("\nAll tests completed!");
        $finish;
    end

    // Real-time monitoring
    always @(posedge clk) begin
        $display("[%t] CLK: PC=%h, OPCODE=%b, R1=%h, R2=%h", 
                 $time, pc_current, opcode, regs_1, regs_2);
    end
endmodule
