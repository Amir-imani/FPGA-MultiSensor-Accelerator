// Module: adaptive_kalman_filter_1d.v
// Description: Hardware-Efficient Adaptive Tracking Filter using Dynamic Shifting
// Author: Amirali Imanipanah


`timescale 1ns / 1ps

module adaptive_kalman_filter_1d (
    input wire               clk,
    input wire               reset,
    input wire               enable,
    input wire signed [15:0] z_in,            
    output reg signed [15:0] x_est_out,       
    output reg signed [15:0] v_est_out,       
    output reg [1:0]         filter_mode_out, 
    output reg               valid_out
);

    localparam signed [15:0] THRESH_HIGH = 16'sd4096; 
    localparam signed [15:0] THRESH_LOW  = 16'sd1024; 

    reg signed [15:0] x_est;
    reg signed [15:0] v_est;

    wire signed [15:0] x_pred;
    wire signed [15:0] innovation;
    wire signed [15:0] abs_innovation;
    
    reg signed [15:0] alpha_update;
    reg signed [15:0] beta_update;
    reg [1:0]         current_mode;


    assign x_pred     = x_est + v_est;
    assign innovation = z_in - x_pred;
    
    assign abs_innovation = (innovation < 0) ? (-innovation) : innovation;


    always @(*) begin
        if (abs_innovation > THRESH_HIGH) begin
            //Fast Maneuver Mode
            alpha_update = innovation >>> 1; // Alpha = 1/2
            beta_update  = innovation >>> 2; // Beta  = 1/4
            current_mode = 2'd0;
        end else if (abs_innovation > THRESH_LOW) begin
            //Normal Tracking Mode
            alpha_update = innovation >>> 2; // Alpha = 1/4
            beta_update  = innovation >>> 4; // Beta  = 1/16
            current_mode = 2'd1;
        end else begin
            //Deep Noise Rejection Mode
            alpha_update = innovation >>> 3; // Alpha = 1/8
            beta_update  = innovation >>> 6; // Beta  = 1/64
            current_mode = 2'd2;
        end
    end


    always @(posedge clk or posedge reset) begin
        if (reset) begin
            x_est           <= 16'sd0;
            v_est           <= 16'sd0;
            x_est_out       <= 16'sd0;
            v_est_out       <= 16'sd0;
            filter_mode_out <= 2'd1;
            valid_out       <= 1'b0;
        end else if (enable) begin
            x_est           <= x_pred + alpha_update;
            v_est           <= v_est  + beta_update;
            
            x_est_out       <= x_pred + alpha_update;
            v_est_out       <= v_est  + beta_update;
            filter_mode_out <= current_mode;
            valid_out       <= 1'b1;
        end else begin
            valid_out       <= 1'b0;
        end
    end

endmodule