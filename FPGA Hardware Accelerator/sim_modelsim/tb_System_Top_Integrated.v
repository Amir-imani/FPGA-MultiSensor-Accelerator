// Testbench for  Hardware Accelerator LiDAR + IMU
// Features: 
// 1. Asynchronous Clock Simulation (sys_clk @ 115MHz, lidar_clk @ 10MHz)
// 2. Toggle Handshake Verification
// 3. Noise Injection for Kalman & FIR Filter Testing


`timescale 1ns / 1ps
module tb_System_Top_Integrated;

    reg sys_clk;
    reg sys_reset;
    
    reg lidar_clk;
    reg signed [15:0] lidar_r_in;
    reg signed [15:0] lidar_theta_in;
    reg lidar_valid_in;
    
    reg signed [15:0] imu_accel_in;
    reg imu_valid_in;

    wire lidar_ready_out;
    wire signed [15:0] clean_lidar_x;
    wire signed [15:0] clean_lidar_y;
    wire [1:0] lidar_filter_mode;
    wire signed [15:0] clean_imu_accel;

    System_Top_Integrated uut (
        .sys_clk(sys_clk), 
        .sys_reset(sys_reset), 
        .lidar_clk(lidar_clk), 
        .lidar_r_in(lidar_r_in), 
        .lidar_theta_in(lidar_theta_in), 
        .lidar_valid_in(lidar_valid_in), 
        .lidar_ready_out(lidar_ready_out), 
        .clean_lidar_x(clean_lidar_x), 
        .clean_lidar_y(clean_lidar_y), 
        .lidar_filter_mode(lidar_filter_mode), 
        .imu_accel_in(imu_accel_in), 
        .imu_valid_in(imu_valid_in), 
        .clean_imu_accel(clean_imu_accel)
    );


    initial begin
        sys_clk = 0;
        forever #4.33 sys_clk = ~sys_clk;
    end

    initial begin
        lidar_clk = 0;
        forever #50 lidar_clk = ~lidar_clk;
    end

    initial begin
        sys_reset = 1;
        lidar_r_in = 0;
        lidar_theta_in = 0;
        lidar_valid_in = 0;
        imu_accel_in = 0;
        imu_valid_in = 0;

        #200;
        sys_reset = 0;
        #200;


        @(posedge lidar_clk);
        lidar_r_in     = 16'sd16384; 
        lidar_theta_in = 16'sd6434;
        lidar_valid_in = 1;         
        
        imu_accel_in = 16'sd1024;    
        imu_valid_in = 1;

        wait(lidar_ready_out == 1);
        @(posedge lidar_clk);
        lidar_valid_in = 0;          
        
        #800; 


        @(posedge lidar_clk);
        lidar_r_in     = 16'sd28672;
        lidar_theta_in = 16'sd6434;
        lidar_valid_in = 1;
        
        imu_accel_in = 16'sd4096;   
        
        wait(lidar_ready_out == 1);
        @(posedge lidar_clk);
        lidar_valid_in = 0;
        
        #800;


        @(posedge lidar_clk);
        lidar_r_in     = 16'sd16384; 
        lidar_theta_in = 16'sd6434;
        lidar_valid_in = 1;
        
        imu_accel_in = 16'sd1024;
        
        wait(lidar_ready_out == 1);
        @(posedge lidar_clk);
        lidar_valid_in = 0;

        #1500;
        
        $display("Simulation Completed Successfully.");
        $stop;
    end

endmodule