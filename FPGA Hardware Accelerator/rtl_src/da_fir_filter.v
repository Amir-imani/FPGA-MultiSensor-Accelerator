// Module: da_fir_filter.v
// Description: Multiplierless 4-Tap FIR Filter using Distributed Arithmetic (DA)
// Author: Amirali Imanipanah


`timescale 1ns / 1ps

module da_fir_filter (
    input wire               clk,
    input wire               reset,
    input wire               enable,          
    input wire signed [15:0] x_in,            
    output reg signed [15:0] y_out,           
    output reg               valid_out
);

    reg signed [15:0] tap [0:3];
    integer i;


    reg signed [15:0] da_lut [0:15];

    initial begin
        da_lut[0]  = 16'sd0;     // 0000
        da_lut[1]  = 16'sd1024;  // 0001 -> c3
        da_lut[2]  = 16'sd3072;  // 0010 -> c2
        da_lut[3]  = 16'sd4096;  // 0011 -> c2 + c3
        da_lut[4]  = 16'sd3072;  // 0100 -> c1
        da_lut[5]  = 16'sd4096;  // 0101 -> c1 + c3
        da_lut[6]  = 16'sd6144;  // 0110 -> c1 + c2
        da_lut[7]  = 16'sd7168;  // 0111 -> c1 + c2 + c3
        da_lut[8]  = 16'sd1024;  // 1000 -> c0
        da_lut[9]  = 16'sd2048;  // 1001 -> c0 + c3
        da_lut[10] = 16'sd4096;  // 1010 -> c0 + c2
        da_lut[11] = 16'sd5120;  // 1011 -> c0 + c2 + c3
        da_lut[12] = 16'sd4096;  // 1100 -> c0 + c1
        da_lut[13] = 16'sd5120;  // 1101 -> c0 + c1 + c3
        da_lut[14] = 16'sd7168;  // 1110 -> c0 + c1 + c2
        da_lut[15] = 16'sd8192;  // 1111 -> c0 + c1 + c2 + c3 (Gain = 1.0)
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 4; i = i + 1) begin
                tap[i] <= 16'sd0;
            end
            y_out     <= 16'sd0;
            valid_out <= 1'b0;
        end else if (enable) begin
            tap[0] <= x_in;
            tap[1] <= tap[0];
            tap[2] <= tap[1];
            tap[3] <= tap[2];


            y_out <= (da_lut[{tap[0][15], tap[1][15], tap[2][15], tap[3][15]}] >>> 1) + 
                     (tap[0] >>> 3) + (tap[1] >>> 2) + (tap[2] >>> 2) + (tap[3] >>> 3);
                     
            valid_out <= 1'b1;
        end else begin
            valid_out <= 1'b0;
        end
    end

endmodule