`ifndef cpu_module
`define cpu_module

`include "mod/ram.sv"

/*
* CPU module
*
* This module is the CPU of the system. It is responsible for wiring together the different components of the system.
*
* @input clk: The clock signal
* @input reset: The reset signal
* @output halt: The halt signal
* @output in_signal: The input present signal. Set to 1 when reading from the input port
* @input in_data: The input data
* @output out_signal: The output present signal. Set to 1 when writing to the output port
* @output out_data: The output data
*/

module cpu (
    input clk,
    input reset,
    output logic halt,
    output logic error,
    output logic in_signal,
    input logic[63:0] in_data,
    output logic out_signal,
    output logic[63:0] out_data
);
    //initialize stuff
    localparam init = 3'b000;
    localparam fetch = 3'b001;
    localparam decode = 3'b010;
    localparam alu = 3'b011;
    localparam read_write = 3'b100;
    reg [2:0] state = init;
    reg [63:0] pc;

    //fetch stuff
    reg fetch_ready;

    //decode stuff
    reg [4:0] opcode;
    reg [4:0] rd_num;
    reg [4:0] rs_num;
    reg [4:0] rt_num;
    reg [63:0] rd_val;
    reg [63:0] rs_val;
    reg [63:0] rt_val;
    reg [11:0] imm;
    reg decode_ready;
    reg read_or_write;

    //register file
    reg [63:0] reg_file[31:0];
    reg read_write_ready;

    //alu stuff
    reg alu_ready;
    reg [63:0] ans;
    real float_ans;
    real float_rs;
    real float_rt;

    // RAM
    logic [63:0] r_addr;
    logic [63:0] rw_addr;
    logic [63:0] rw_data_in;
    logic rw_write_en;
    wire [31:0] r_data_out;
    wire r_error;
    wire [63:0] rw_data_out;
    wire rw_error;
    
    ram memory (
        .clk(clk),
        .reset(reset),
        .r_addr(r_addr),
        .rw_addr(rw_addr),
        .rw_data_in(rw_data_in),
        .rw_write_en(rw_write_en),
        .r_data_out(r_data_out),
        .r_error(r_error),
        .rw_data_out(rw_data_out),
        .rw_error(rw_error)
    );

    always @(posedge clk) begin
        if (reset) begin
            error <= 0;
            state <= init;
        end
        //TODO: double check error checking
        if(r_error != 0 || rw_error != 0) begin
            error <= 1;
            halt <= 1;
        end
        case(state)
            init: begin
                pc <= 0;
                state <= fetch;
                fetch_ready <= 0;
                decode_ready <= 0;
                alu_ready <= 0;
                read_write_ready <= 1;
                rw_write_en <= 0;
                for(int i = 0; i < 31; i = i + 1) begin
                    reg_file[i] <= 0;
                end
                reg_file[31] <= 1024 * 512;
                state <= fetch;
            end
            fetch: begin
                if(read_write_ready == 1 && fetch_ready == 0) begin
                    read_write_ready <= 0;
                    r_addr <= pc;
                    fetch_ready <= 1;
                    state <= decode;
                end
            end
            decode: begin                
                else if(fetch_ready == 1 && decode_ready == 0) begin
                    fetch_ready <= 0;
                    opcode <= r_data_out[31:27];
                    if(opcode == 5'b11111) begin
                        error <= 0;
                        halt <= 1;
                    end
                    else begin
                        rd_num <= r_data_out[26:22];
                        rs_num <= r_data_out[21:17];
                        rt_num <= r_data_out[16:12];
                        imm <= r_data_out[11:0];
                        decode_ready <= 1;
                        read_or_write <= 0;
                        state <= read_write;
                    end
                end
            end
            alu: begin
                if(read_write_ready == 1 && alu_ready == 0) begin
                    else begin
                        read_write_ready = 0;
                        case(opcode)
                            0: begin
                                ans <= rs_val + rt_val;
                                pc <= pc + 4;
                            end
                            1: begin
                                ans <= rd_val + imm;
                                pc <= pc + 4;
                            end
                            2: begin
                                ans <= rs_val - rt_val;
                                pc <= pc + 4;
                            end
                            3: begin
                                ans <= rd_val - imm;
                                pc <= pc + 4;
                            end
                            4: begin
                                ans <= rs_val * rt_val;
                                pc <= pc + 4;
                            end
                            5: begin
                                ans <= rs_val / rt_val;
                                pc <= pc + 4;
                            end
                            6: begin
                                ans <= rs_val & rt_val;
                                pc <= pc + 4;
                            end
                            7: begin
                                ans <= rs_val | rt_val;
                                pc <= pc + 4;
                            end
                            8: begin
                                ans <= rs_val ^ rt_val;
                                pc <= pc + 4;
                            end
                            9: begin
                                ans <= ~rs_val;
                                pc <= pc + 4;
                            end
                            10: begin
                                ans <= rs_val >> rt_val;
                                pc <= pc + 4;
                            end
                            11: begin
                                ans <= rd_val >> imm;
                                pc <= pc + 4;
                            end
                            12: begin
                                ans <= rs_val << rt_val;
                                pc <= pc + 4;
                            end
                            13: begin
                                ans <= rs_val << imm;
                                pc <= pc + 4;
                            end
                            14: begin
                                pc <= rd_val;
                            end
                            15: begin
                                pc <= pc + rd_val;
                            end
                            16: begin
                                pc <= pc + imm;
                            end
                            17: begin
                                if(rs_val == 0) begin
                                    pc <= pc + 4;
                                end
                                else begin
                                    pc <= rd_val;
                                end
                            end
                            18: begin
                                rw_addr <= reg_file[31] - 8;
                                rw_data_in <= pc + 4;
                                rw_write_en <= 1;
                                pc <= rd_val;
                            end
                            19: begin
                                rw_addr <= reg_file[31] - 8;
                                pc = rw_data_out;
                            end
                            20: begin
                                if(rs_val <= rt_val) begin
                                    pc <= pc + 4;
                                end
                                else begin
                                    pc <= rd_val;
                                end
                            end
                            21: begin
                                rw_addr <= rs_val + imm;
                                pc <= pc + 4;
                            end
                            22: begin
                                ans <= rs_val;
                                pc <= pc + 4;
                            end
                            23: begin
                                ans[11:0] <= imm;
                                pc <= pc + 4;
                            end
                            24: begin
                                rw_addr <= rd_val + imm;
                                rw_data_in <= rs_val;
                                rw_write_en <= 1;
                                pc <= pc + 4;
                            end
                            25: begin
                                float_ans = float_rs + float_rt;
                                ans <= $realtobits(float_ans);
                                pc <= pc + 4;
                            end
                            26: begin
                                float_ans = float_rs - float_rt;
                                ans <= $realtobits(float_ans);
                                pc <= pc + 4;
                            end
                            27: begin
                                float_ans = float_rs * float_rt;
                                ans <= $realtobits(float_ans);
                                pc <= pc + 4;
                            end
                            28: begin
                                float_ans = float_rs / float_rt;
                                ans <= $realtobits(float_ans);
                                pc <= pc + 4;
                            end
                            29: begin
                                //TODO: check in function
                                if(rs_val == 0) begin
                                    ans <= in_data;
                                    in_signal <= 1;
                                end
                                pc <= pc + 4;
                            end
                            30: begin
                                //TODO: check out function
                                if(rd_val == 1) begin
                                    out_data <= reg_file[rs_val];
                                    out_signal <= 1;
                                end
                                pc <= pc + 4;
                            end
                            default: begin
                                error <= 1;
                                halt <= 1;
                            end
                        endcase
                        alu_ready <= 1;
                        read_or_write <= 1;
                        state <= read_write;
                    end
                end
            end
            read_write: begin
                else if(read_or_write == 1 && alu_ready == 1 && read_write_ready == 0) begin
                    if(opcode == 21) begin
                        ans <= rw_data_out;
                    end
                    rw_write_en <= 0;
                    reg_file[rd_num] <= ans;
                    alu_ready <= 0;
                    read_write_ready <= 1;
                    state <= fetch;
                end
                else if(read_or_write == 0 && decode_ready == 1 && read_write_ready == 0) begin
                    rd_val <= reg_file[rd_num];
                    rs_val <= reg_file[rs_num];
                    rt_val <= reg_file[rt_num];
                    ans <= rd_val;
                    float_rs <= $bitstoreal(rs_val);
                    float_rt <= $bitstoreal(rt_val);
                    decode_ready <= 0;
                    read_write_ready <= 1;
                    state <= alu;
                end
            end
        endcase
    end
endmodule
`endif

// module f_add_sub(
//     input wire [63:0] a,
//     input wire [63:0] b,
//     input wire clk,
//     input wire [1:0] enabler,
//     output wire [63:0] sum,
//     output wire ready
// );
//     localparam init = 2'b00;
//     localparam shift = 2'b01;
//     localparam calc = 2'b10;
//     localparam norm = 2'b11;

//     reg [52:0] a_mantissa = 0;
//     reg [10:0] a_exponent = 0;
//     reg a_sign = 0;
//     reg [52:0] b_mantissa = 0;
//     reg [10:0] b_exponent = 0;
//     reg b_sign = 0;
//     reg readyBit = 0;
//     reg [63:0] ans = 0;
//     reg [1:0] state = init;
//     wire [10:0] diff;
//     wire[63:0] new_b;
//     reg[10:0] exp = 0;
//     reg[52:0] ans_mantissa = 0;

//     assign sum = ans;
//     assign ready = readyBit;
//     assign diff = (a_exponent > b_exponent) ? a_exponent - b_exponent : b_exponent - a_exponent;
//     assign new_b = (enabler == 2'b01) ? -b : b;

//     always @(posedge clk) begin
//         if((enabler == 2'b00 || enabler == 2'b01) && readyBit == 0) begin
//             case(state)
//                 init: begin
//                     a_mantissa <= {1'b1, a[51:0]};
//                     a_exponent <= a[62:52];
//                     a_sign <= a[63:63];

//                     b_mantissa <= {1'b1, new_b[51:0]};
//                     b_exponent <= new_b[62:52];
//                     b_sign <= new_b[63:63];
//                     state <= shift;
//                 end
//                 shift: begin
//                     if(a_exponent > b_exponent) begin
//                         b_mantissa <= b_mantissa >> diff;
//                         exp <= a_exponent;
//                     end
//                     else begin
//                         a_mantissa <= a_mantissa >> diff;
//                         exp <= b_exponent;
//                     end
//                     state <= calc;
//                 end
//                 calc: begin
//                     if(a_sign == b_sign) begin
//                         ans_mantissa <= a_mantissa + b_mantissa;
//                         ans[63] <= a_sign;
//                     end

//                     else begin
//                         if(a_exponent > b_exponent) begin
//                             ans_mantissa <= a_mantissa - b_mantissa;
//                             ans[63] <= a_sign;
//                         end
//                         else if (a_exponent < b_exponent) begin
//                             ans_mantissa <= b_mantissa - a_mantissa;
//                             ans[63] <= b_sign;
//                         end
//                         else begin
//                             if(a_mantissa > b_mantissa) begin
//                                 ans_mantissa <= a_mantissa - b_mantissa;
//                                 ans[63] <= a_sign;
//                             end
//                             else begin
//                                 ans_mantissa <= b_mantissa - a_mantissa;
//                                 ans[63] <= b_sign;
//                             end
//                         end
//                     end
//                     state <= norm;
//                 end
//                 norm: begin
//                     while(ans_mantissa[52] == 0 && ans_mantissa != 0) begin
//                         ans_mantissa = ans_mantissa << 1;
//                         exp = exp - 1;
//                     end
//                     ans[51:0] <= ans_mantissa[51:0];
//                     ans[62:52] <= exp;
//                     readyBit <= 1;
//                 end
//             endcase
//         end
//         else begin
//             readyBit <= 0;
//         end
//     end
// endmodule