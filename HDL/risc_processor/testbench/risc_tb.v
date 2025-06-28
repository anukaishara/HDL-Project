`timescale 1ns / 1ps
`include "../rtl/risc_core.v"

module core_tb();
    reg clk;
    reg reset;
    wire [7:0] pc;
    wire [15:0] instr;
    wire [7:0] mem_addr;
    wire mem_write;
    wire [7:0] mem_data_out;
    reg [7:0] mem_data_in;
    
    // Instantiate the core
    risc_core uut (
        .clk(clk),
        .reset(reset),
        .instr_addr(pc),
        .instr_data(instr),
        .data_addr(mem_addr),
        .data_write(mem_write),
        .data_out(mem_data_out),
        .data_in(mem_data_in)
    );
    
    // Clock generation (50 MHz)
    always #10 clk = ~clk;
    
    // Instruction memory mockup
    reg [15:0] fake_mem [0:255];
    assign instr = fake_mem[pc];
    
    // Test procedure
    initial begin
        // Initialize signals
        clk = 0;
        reset = 1;
        mem_data_in = 0;
        
        // Load test program into fake memory
        fake_mem[0] = 16'b0010_00_01_00001000; // LW R1, 8(R0)
        fake_mem[1] = 16'b0010_00_10_00001001; // LW R2, 9(R0)
        fake_mem[2] = 16'b0000_01_10_11_00;    // ADD R3, R1, R2
        fake_mem[3] = 16'b0011_00_11_00001010;  // SW R3, 10(R0)
        
        // Release reset after 3 clocks
        #30 reset = 0;
        
        // Simulate memory reads
        #40 mem_data_in = 8'h0A; // Response to LW R1
        #20 mem_data_in = 8'h01; // Response to LW R2
        
        // Run for 10 instructions
        #200;
        
        // Display results
        $display("Test completed:");
        $display("Final PC = %h", pc);
        $display("R3 value = %h", uut.register_file[3]);
        
        $finish;
    end
    
    // Monitor register changes
    always @(posedge clk) begin
        $display("[%t] PC=%h Instr=%h R1=%h R2=%h R3=%h", 
            $time, pc, instr, 
            uut.register_file[1], 
            uut.register_file[2], 
            uut.register_file[3]);
    end
endmodule