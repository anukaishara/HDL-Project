`timescale 1ns / 1ps
`include "risc_sysstem.v"

module system_tb();
    reg clk;
    reg reset;
    wire [7:0] debug_pc;
    wire [7:0] debug_reg0, debug_reg1, debug_reg2, debug_reg3;
    
    // Instantiate complete system
    risc_system uut (
        .clk(clk),
        .reset(reset),
        .debug_pc(debug_pc),
        .debug_reg0(debug_reg0),
        .debug_reg1(debug_reg1),
        .debug_reg2(debug_reg2),
        .debug_reg3(debug_reg3)
    );
    
    // Clock generation (50 MHz)
    always #10 clk = ~clk;
    
    // Test procedure
    initial begin
        // Initialize
        clk = 0;
        reset = 1;
        
        // Create VCD dump for waveform viewing
        $dumpfile("system_wave.vcd");
        $dumpvars(0, system_tb);
        
        // Release reset after 3 clocks
        #30 reset = 0;
        
        // Run until program completes (adjust as needed)
        #1000;
        
        // Display final state
        $display("\nFinal System State:");
        $display("PC = %h", debug_pc);
        $display("Registers: R0=%h R1=%h R2=%h R3=%h",
            debug_reg0, debug_reg1, debug_reg2, debug_reg3);
        $display("Memory[10] = %h", uut.dmem.memory[10]);
        
        $finish;
    end
    
    // Instruction execution monitor
    always @(posedge clk) begin
        if (!reset) begin
            $display("[%t] PC=%h Instr=%h", 
                $time, debug_pc, uut.imem.memory[debug_pc]);
        end
    end
endmodule