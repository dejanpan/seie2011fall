
%---------------------------------------
% Check points inside bounding box
%---------------------------------------
all_part_points=[];

[kk n_c] = size( final_scene_part_contributors(o).parts ); 
for p=1:n_c;
    p_id = final_scene_part_contributors(o).parts(p);
    
    [n_p l_p] = size(points);
    part_points_indexes = find(  points(:, l_p) == p_id);
    part_points = points( part_points_indexes, : );  

    all_part_points = [all_part_points; part_points];
end       


% check
inc_x = bbox(1);
inc_y = bbox(2);
limit1 = [point(1)-inc_x, point(2)-inc_y]; 
limit2 = [point(1)+inc_x, point(2)+inc_y];

[n_r kk] = size( all_part_points ); 
for p=1:n_r;
    p_point = all_part_points(p,1:2);
    if ( p_point(1) > limit1(1) & ...
         p_point(1) < limit2(1) & ...
         p_point(2) > limit1(2) & ...
         p_point(2) < limit2(2)   ) 
        
         inside_bbox(v+1) =  inside_bbox(v+1) + 1.0;
    end     
end    

inside_bbox(v+1) = inside_bbox(v+1) / n_r;


