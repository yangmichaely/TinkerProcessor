`ifndef eval_bench
`define eval_bench

`include "mod/cpu.sv"
`include "macros/reset_and_load_macro.sv"

module evaluator();

    // create clk and reset top signals
    logic clk;
    logic reset;

    // cpu module
    logic halt; 
    logic in_signal;
    logic [63:0] in_data;
    logic out_signal;
    logic [63:0] out_data;

    initial begin
        $dumpfile("./vcd/cpu.vcd");
        $dumpvars(0, cpu_inst);
    end

    cpu cpu_inst (
        .clk(clk),
        .reset(reset),
        .halt(halt),
        .in_signal(in_signal),
        .in_data(in_data),
        .out_signal(out_signal),
        .out_data(out_data)
    );

    // clock generator
    always #5 clk = ~clk;

    initial begin
        // ======================================================
        // reset the processor and load the tko file into the memory
        `RESET_AND_LOAD_FILE("test-suite/tko/dividebyzerotest.tko", clk, reset, cpu_inst.memory.bytes);
    
        // display the memory contents
        // for (int i = 0; i < 40; i++) begin
        //     $display("Memory[%d]: %d", i, cpu_inst.memory.bytes[i]);
        // end
        while(halt != 1) begin
            if(out_signal == 1) begin
                $display("Output: %d", out_data);
            end
            #10;
        end  
        $finish;      
    end

endmodule

`endif