

function plot_part_test(points, pc)

    [r c] = size(points);
    
    min_points = 35;
    
    if ( r >= min_points )
        step_r = 30;
        %step_r = 5;
    else 
        step_r = 1;
    end;    
    
    scatter3( points(1:step_r:r,1), points(1:step_r:r,2), points(1:step_r:r,3), 5, pc, 'filled' );        
    axis([-2 7 -5 5 -0.10 2.0]); 

end    
