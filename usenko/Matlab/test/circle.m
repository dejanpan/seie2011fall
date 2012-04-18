function [lp]=circle(center, step, radius)
    
    lp=[];
    angle_step = step / radius;
    
    for angle=0:angle_step:2*pi
        p_x = radius * cos(angle); 
        p_y = radius * sin(angle); 
        p = [p_x p_y 0];
        p = p+center;
        lp = [lp;  p];
    end    

end