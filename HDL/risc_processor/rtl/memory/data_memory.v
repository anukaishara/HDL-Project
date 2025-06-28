module data_memory (
    input wire clk,
    input wire reset,
    input wire [7:0] addr,
    input wire write_en,
    input wire [7:0] data_in,
    output wire [7:0] data_out
);

    // 256-byte data memory (8-bit address space)
    reg [7:0] memory[0:255];
    
    // Initialize memory to zeros
    integer i;
    initial begin
        for (i = 0; i < 256; i = i + 1)
            memory[i] = 8'b0;
    end
    
    // Synchronous write, asynchronous read
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 256; i = i + 1)
                memory[i] <= 8'b0;
        end
        else if (write_en) begin
            memory[addr] <= data_in;
        end
    end
    
    assign data_out = memory[addr];
    
endmodule