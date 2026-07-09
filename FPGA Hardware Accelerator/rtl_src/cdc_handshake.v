// Module: cdc_handshake.v
// Author: Amirali Imanipanah


`timescale 1ns / 1ps

module cdc_handshake #
(
    parameter DATA_WIDTH = 32
)
(
    input  wire                         sensor_clk,
    input  wire                         sensor_reset,

    input  wire signed [DATA_WIDTH-1:0] sensor_data_in,
    input  wire                         sensor_data_valid,

    output wire                         sensor_ready,

    input  wire                         sys_clk,
    input  wire                         sys_reset,

    output reg signed [DATA_WIDTH-1:0]  sys_data_out,
    output reg                          sys_data_valid
);



reg signed [DATA_WIDTH-1:0] data_buffer;



reg req_toggle;
reg ack_toggle;


reg req_sync1, req_sync2;
reg ack_sync1, ack_sync2;


assign sensor_ready = (req_toggle == ack_sync2);

always @(posedge sensor_clk or posedge sensor_reset)
begin

    if(sensor_reset)
    begin
        req_toggle  <= 1'b0;
        data_buffer <= 0;
    end
    else
    begin

        if(sensor_data_valid && sensor_ready)
        begin
            data_buffer <= sensor_data_in;
            req_toggle  <= ~req_toggle;
        end

    end

end



always @(posedge sys_clk or posedge sys_reset)
begin

    if(sys_reset)
    begin
        req_sync1 <= 0;
        req_sync2 <= 0;
    end
    else
    begin
        req_sync1 <= req_toggle;
        req_sync2 <= req_sync1;
    end

end



reg req_sync2_d;

always @(posedge sys_clk or posedge sys_reset)
begin

    if(sys_reset)
    begin

        req_sync2_d   <= 0;
        ack_toggle    <= 0;

        sys_data_out  <= 0;
        sys_data_valid<= 0;

    end
    else
    begin

        req_sync2_d <= req_sync2;

        if(req_sync2 != req_sync2_d)
        begin

            sys_data_out   <= data_buffer;
            sys_data_valid <= 1'b1;

            ack_toggle <= req_sync2;

        end
        else
        begin
            sys_data_valid <= 1'b0;
        end

    end

end



always @(posedge sensor_clk or posedge sensor_reset)
begin

    if(sensor_reset)
    begin
        ack_sync1 <= 0;
        ack_sync2 <= 0;
    end
    else
    begin
        ack_sync1 <= ack_toggle;
        ack_sync2 <= ack_sync1;
    end

end

endmodule