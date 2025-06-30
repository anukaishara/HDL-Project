`timescale 1ns / 1ps
`include "risc_processor.v"

module tb_risc_processor();

    reg clk;
    reg reset;
    reg [7:0] external_data_in;
    wire [7:0] data_out;

    // Instantiate DUT
    risc_processor uut (
        .clk(clk),
        .reset(reset),  // Active-low reset
        .external_data_in(external_data_in),
        .data_out(data_out)
    );

    // Clock generation (10ns period)
    always #5 clk = ~clk;

    // Monitoring internal state
    wire [7:0]  pc_current       = uut.pc_inst.current_pc;
    wire [15:0] instruction      = uut.instruction;
    wire [2:0]  opcode           = uut.opcode;
    wire [7:0]  regs_0           = uut.rf.regs[0];
    wire [7:0]  regs_1           = uut.rf.regs[1];
    wire [7:0]  regs_2           = uut.rf.regs[2];
    wire [7:0]  regs_3           = uut.rf.regs[3];
    wire        pc_src           = uut.pc_src;
    wire        mem_write        = uut.mem_write;
    wire [7:0]  alu_result       = uut.alu_result;
    wire [7:0]  read_data2       = uut.read_data2;

    initial begin
        $dumpfile("risc_processor_tb.vcd");
        $dumpvars(0, tb_risc_processor);
    end

    initial begin
        clk = 0;
        reset = 1;  // Active-low reset: keep high for normal op
        external_data_in = 0;

        // Preload RAM contents manually
        uut.dmem.ram[8'h10] = 8'hA5;
        uut.dmem.ram[8'h20] = 8'h00;

        // Program Instructions (manually load into ROM)
        uut.imem.rom[0] = 16'h4810; // LOAD R1, 0x10(R0)
        uut.imem.rom[1] = 16'h1200; // ADD R2, R1, R0
        uut.imem.rom[2] = 16'h7020; // STORE R2, 0x20(R0)
        uut.imem.rom[3] = 16'h8005; // JUMP 0x05
        uut.imem.rom[5] = 16'h48FF; // LOAD R1, 0xFF(R0)
        uut.imem.rom[6] = 16'h1D80; // SUB R3, R1, R2

        // Reset pulse (active-low)
        #5 reset = 0;
        #10 reset = 1;

        $display("--- [TEST 1: RESET] ---");
        @(posedge clk); #1;
        $display("PC after reset: %h (expected: 00)", pc_current);
        $display("Registers: R0=%h, R1=%h, R2=%h, R3=%h", regs_0, regs_1, regs_2, regs_3);


        $display("\n--- [TEST 2: LOAD] ---");
        @(posedge clk); #1;
        $display("After LOAD: R1 = %h (expected: A5)", regs_1);
        if (regs_1 !== 8'hA5) begin
            $display("ERROR: LOAD test failed!");
            $stop;
        end


        $display("\n--- [TEST 3: ADD] ---");
        @(posedge clk); #1;
        $display("After ADD: R2 = %h (expected: A5)", regs_2);
        if (regs_2 !== 8'hA5) begin
            $display("ERROR: ADD test failed!");
            $stop;
        end


        $display("\n--- [TEST 4: STORE] ---");
        @(posedge clk); #1;
        $display("mem_write=%b, address=%h, data=%h", mem_write, alu_result, read_data2);
        $display("Data at 0x20: %h (expected: A5)", uut.dmem.ram[8'h20]);
        if (uut.dmem.ram[8'h20] !== 8'hA5) begin
            $display("ERROR: STORE test failed!");
            $stop;
        end


        $display("\n--- [TEST 5: JUMP] ---");
        @(posedge clk); #1;
        $display("Instruction=%h, Opcode=%b, pc_src=%b", instruction, opcode, pc_src);
        $display("PC after JUMP: %h (expected: 05)", pc_current);
        if (pc_current !== 8'h05) begin
            $display("ERROR: JUMP test failed!");
            $stop;
        end


        $display("\n--- [TEST 6: I/O READ] ---");
        external_data_in = 8'hF0;
        @(posedge clk); #1;
        $display("R1 after I/O read: %h (expected: F0)", regs_1);
        if (regs_1 !== 8'hF0) begin
            $display("ERROR: I/O read test failed!");
            $stop;
        end


        $display("\n--- [TEST 7: SUB] ---");
        @(posedge clk); #1;
        $display("After SUB: R3 = %h (expected: 4B)", regs_3);
        if (regs_3 !== 8'h4B) begin
            $display("ERROR: SUB test failed!");
            $stop;
        end


        $display("\n--- [TEST 8: RESET RECOVERY] ---");
        #10 reset = 0;
        #10 reset = 1;
        @(posedge clk); #1;
        $display("PC after reset: %h (expected: 00)", pc_current);
        if (pc_current !== 8'h00) begin
            $display("ERROR: Reset failed!");
            $stop;
        end


        $display("\nAll tests completed successfully.");
        $finish;
    end

    // Trace CPU on every clock
    always @(posedge clk) begin
        $display("[%0t] CLK: PC=%h, INST=%h, OP=%b, R1=%h, R2=%h", 
                 $time, pc_current, instruction, opcode, regs_1, regs_2);
    end

endmodule
