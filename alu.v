module alu(
    input [7:0] a,
    input [7:0] b,
    input [1:0] op,
    output reg [7:0] result
);
    always @(*) begin
        case(op)
            2'b00: result = a + b;  // ADD
            2'b01: result = a - b;  // SUB
            default: result = a + b; // Default to ADD
        endcase
    end
endmodule
