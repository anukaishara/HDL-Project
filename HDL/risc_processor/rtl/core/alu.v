module alu (
    input wire [7:0] a,
    input wire [7:0] b,
    input wire [1:0] alu_op,
    output reg [7:0] result,
    output wire zero
);
    
    always @(*) begin
        case (alu_op)
            2'b00: result = a + b;    // ADD
            2'b01: result = a - b;    // SUB
            2'b10: result = a & b;    // AND (not in requirements but useful)
            2'b11: result = a | b;    // OR (not in requirements but useful)
        endcase
    end
    
    assign zero = (result == 8'b0);
    
endmodule