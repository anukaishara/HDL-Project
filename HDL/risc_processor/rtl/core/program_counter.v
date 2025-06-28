module program_counter (
    input wire clk,
    input wire reset,
    input wire branch,
    input wire jump,
    input wire [7:0] immediate,
    output reg [7:0] pc
);
    
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 8'b0;
        end
        else if (jump) begin
            pc <= immediate;
        end
        else if (branch) begin
            pc <= pc + 1 + immediate;
        end
        else begin
            pc <= pc + 1;
        end
    end
    
endmodule