


function c = centerBoundingBox(points)

    max_x = max( points(:, 1) );
    min_x =  min( points(:, 1) );
    
    max_y = max( points(:, 2) );
    min_y =  min( points(:, 2) );
    
    max_z = max( points(:, 3) );
    min_z =  min( points(:, 3) );


    inc_x = (max_x - min_x) / 2;
    inc_y = (max_y - min_y) / 2;
    inc_z = (max_z - min_z) / 2;
    
    c= [min_x+inc_x, min_y+inc_y, min_z+inc_z];
end