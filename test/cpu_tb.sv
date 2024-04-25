module cpu_tb();
    reg clk;
    reg [1:0] enable;
    reg [63:0] sum;
    reg ready;
    reg [63:0] a;
    reg [63:0] b;

    cpu my_cpu(
        .clk(clk),
        .enabler(enable),
        .a(a),
        .b(b),
        .sum(sum),
        .ready(ready)
    );

    initial begin
        clk = 1'b0;
        forever begin
            # 5 clk = ~clk;
        end
    end

    initial begin
        enable = 2'b00;
        a <= 64'b0011111111110001100110011001100110011001100110011001100110011010;
        b <= 64'b0011111111100000110011001100110011001100110011001100110011001101;
        //a <= 64'b0011111111110000000000000000000000000000000000000000000000000000;
        //b <= 64'b0100000000000000000000000000000000000000000000000000000000000000;
        //a <= 64'b1100000000000000000000000000000000000000000000000000000000000000;
        //b <= 64'b0100000000001000000000000000000000000000000000000000000000000000;
        #60;
        if (ready == 1) begin
            $display("sum:", $bitstoreal(sum));
            $display("sum: %b ", sum);
        end
        $finish;
    end
endmodule