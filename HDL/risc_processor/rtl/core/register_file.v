module register_file (
    input wire clk,
    input wire reset,
    input wire reg_write,
    input wire [1:0] read_reg1,
    input wire [1:0] read_reg2,
    input wire [1:0] write_reg,
    input wire [7:0] write_data,
    output wire [7:0] read_data1,
    output wire [7:0] read_data2
);
    
    reg [7:0] registers[0:3];
    
    // Initialize registers to zero
    integer i;
    initial begin
        for (i = 0; i < 4; i = i + 1)
            registers[i] = 8'b0;
    end
    
    // Read operations (asynchronous)
    assign read_data1 = registers[read_reg1];
    assign read_data2 = registers[read_reg2];
    
    // Write operation (synchronous)
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 4; i = i + 1)
                registers[i] <= 8'b0;
        end
        else if (reg_write) begin
            registers[write_reg] <= write_data;
        end
    end
    
endmodule