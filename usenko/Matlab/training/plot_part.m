

function plot_part(points, pc)

    [r c] = size(points);
    
    %min_points = 35;
    min_points = 1;
    
    
    if ( r >= min_points )
        %step_r = 15;
        step_r = 1;
    else 
        step_r = 1;
    end;    
    
    %scatter3( points(1:step_r:r,1), points(1:step_r:r,2), points(1:step_r:r,3), 25, pc, 'filled' );        
    scatter3( points(1:step_r:r,1), points(1:step_r:r,2), points(1:step_r:r,3), 5, pc, 'filled' );        
    axis([0.5 3.5 -1.5 1.5 -0.10 1.5]); 

end    