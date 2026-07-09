function [x_out, y_out] = CORDIC_HDL_Core(theta_in)

    x = fi(0.60725, 1, 16, 13);
    y = fi(0, 1, 16, 13);
    z = theta_in;
    
    atan_table = fi([0.785398, 0.463647, 0.244978, 0.124354, 0.062418, ...
                     0.031239, 0.015623, 0.007812, 0.003906, 0.001953, ...
                     0.000976, 0.000488, 0.000244], 1, 16, 13);
    
    coder.unroll(); 
    for i = 1:13
        x_shift = bitsra(x, i-1);
        y_shift = bitsra(y, i-1);
        
        if z >= 0
            x(:) = x - y_shift;
            y(:) = y + x_shift;
            z(:) = z - atan_table(i);
        else
            x(:) = x + y_shift;
            y(:) = y - x_shift;
            z(:) = z + atan_table(i);
        end
    end
    
    x_out = x;
    y_out = y;
end