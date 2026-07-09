// Module: System_Top_Integrated.v
// Description: Unified Multi-Sensor Hardware Accelerator (LiDAR + IMU)
// Author: Amirali Imanipanah


`timescale 1ns / 1ps

module System_Top_Integrated (

    input wire               sys_clk,
    input wire               sys_reset,


    input wire               lidar_clk,
    input wire signed [15:0] lidar_r_in,        
    input wire signed [15:0] lidar_theta_in,    
    input wire               lidar_valid_in,
    output wire              lidar_ready_out,



    input wire signed [15:0] imu_accel_in,     
    input wire               imu_valid_in,


    output reg signed [15:0] clean_lidar_x,     
    output reg signed [15:0] clean_lidar_y,     
    output wire [1:0]        lidar_filter_mode, 
    output reg               lidar_data_valid,

    output reg signed [15:0] clean_imu_accel,   
    output reg               imu_data_valid
);


    wire signed [31:0] cdc_in  = {lidar_r_in, lidar_theta_in};
    wire signed [31:0] cdc_out;
    wire               cdc_valid;

    cdc_handshake #(.DATA_WIDTH(32)) u_lidar_cdc (
        .sensor_clk       (lidar_clk),
        .sensor_reset     (sys_reset),
        .sensor_data_in   (cdc_in),
        .sensor_data_valid(lidar_valid_in),
        .sensor_ready     (lidar_ready_out),
        .sys_clk          (sys_clk),
        .sys_reset        (sys_reset),
        .sys_data_out     (cdc_out),
        .sys_data_valid   (cdc_valid)
    );

    wire signed [15:0] sync_r     = cdc_out[31:16];
    wire signed [15:0] sync_theta = cdc_out[15:0];


    reg signed [15:0] r_pipe [0:13];
    reg               valid_pipe [0:13];
    integer i;

    always @(posedge sys_clk or posedge sys_reset) begin
        if (sys_reset) begin
            for (i = 0; i < 14; i = i + 1) begin
                r_pipe[i]     <= 16'sd0;
                valid_pipe[i] <= 1'b0;
            end
        end else begin
            r_pipe[0]     <= sync_r;
            valid_pipe[0] <= cdc_valid;
            for (i = 1; i < 14; i = i + 1) begin
                r_pipe[i]     <= r_pipe[i-1];
                valid_pipe[i] <= valid_pipe[i-1];
            end
        end
    end


    wire signed [15:0] cos_val, sin_val;
    

    CORDIC_HDL_Core_fixpt u_cordic (
        .clk      (sys_clk),
        .reset    (sys_reset),
        .theta_in (sync_theta),
        .x_out    (cos_val),
        .y_out    (sin_val)
    );

    (* use_dsp48 = "no", mult_style = "lut" *) wire signed [31:0] mult_x_lut = r_pipe[13] * cos_val;
    (* use_dsp48 = "no", mult_style = "lut" *) wire signed [31:0] mult_y_lut = r_pipe[13] * sin_val;


    reg signed [31:0] mult_x_stage1, mult_y_stage1;
    reg               mult_vld_stage1;

    reg signed [31:0] mult_x, mult_y;
    reg               mult_vld;

    always @(posedge sys_clk or posedge sys_reset) begin
        if (sys_reset) begin
            mult_x_stage1   <= 32'sd0; 
            mult_y_stage1   <= 32'sd0; 
            mult_vld_stage1 <= 1'b0;
            mult_x          <= 32'sd0; 
            mult_y          <= 32'sd0; 
            mult_vld        <= 1'b0;
        end else begin
            mult_x_stage1   <= mult_x_lut;
            mult_y_stage1   <= mult_y_lut;
            mult_vld_stage1 <= valid_pipe[13];

            mult_x          <= mult_x_stage1 >>> 13;
            mult_y          <= mult_y_stage1 >>> 13;
            mult_vld        <= mult_vld_stage1;
        end
    end

    wire signed [15:0] raw_x = mult_x[15:0];
    wire signed [15:0] raw_y = mult_y[15:0];

    wire signed [15:0] k_x_out, k_y_out;
    wire [1:0]         mode_x;
    wire               k_x_vld, k_y_vld;

    adaptive_kalman_filter_1d u_kalman_x (
        .clk            (sys_clk),
        .reset          (sys_reset),
        .enable         (mult_vld),
        .z_in           (raw_x),
        .x_est_out      (k_x_out),
        .v_est_out      (),
        .filter_mode_out(mode_x),
        .valid_out      (k_x_vld)
    );

    adaptive_kalman_filter_1d u_kalman_y (
        .clk            (sys_clk),
        .reset          (sys_reset),
        .enable         (mult_vld),
        .z_in           (raw_y),
        .x_est_out      (k_y_out),
        .v_est_out      (),
        .filter_mode_out(),
        .valid_out      (k_y_vld)
    );

    assign lidar_filter_mode = mode_x;


    wire signed [15:0] fir_out;
    wire               fir_vld;

    da_fir_filter u_imu_fir (
        .clk      (sys_clk),
        .reset    (sys_reset),
        .enable   (imu_valid_in),
        .x_in     (imu_accel_in),
        .y_out    (fir_out),
        .valid_out(fir_vld)
    );


    always @(posedge sys_clk or posedge sys_reset) begin
        if (sys_reset) begin
            clean_lidar_x    <= 16'sd0;
            clean_lidar_y    <= 16'sd0;
            lidar_data_valid <= 1'b0;
            clean_imu_accel  <= 16'sd0;
            imu_data_valid   <= 1'b0;
        end else begin
            clean_lidar_x    <= k_x_out;
            clean_lidar_y    <= k_y_out;
            lidar_data_valid <= k_x_vld & k_y_vld;
            clean_imu_accel  <= fir_out;
            imu_data_valid   <= fir_vld;
        end
    end

endmodule