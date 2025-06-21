module reg_file(
    input clk,
    input write_enable,
    input [2:0] read_addr1,
    input [2:0] read_addr2,
    input [2:0] write_addr,
    input [7:0] write_data,
    output [7:0] read_data1,
    output [7:0] read_data2
);
    reg [7:0] registers [0:7];
    
    // Initialize $0 to 0
    initial registers[0] = 8'b0;
    
    assign read_data1 = (read_addr1 == 0) ? 8'b0 : registers[read_addr1];
    assign read_data2 = (read_addr2 == 0) ? 8'b0 : registers[read_addr2];
    
    always @(posedge clk) begin
        if (write_enable && write_addr != 0) begin
            registers[write_addr] <= write_data;
        end
    end
endmodule
