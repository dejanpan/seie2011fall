%------------------------------------
% Plot 
%------------------------------------


class_colors= 'rgbcmyk';

% chair = 1 = r
% table = 2 = b

    
% look for points pertaining to this part
[n_p l_p] = size(points);
part_points_indexes = find(  points(:, l_p) == p_id);
part_points = points( part_points_indexes, : );  
    
% plot points
plot_part_test( part_points, class_colors(mod(p,7)+1) ) ;      
%hold on;

% find center
[n_p l_p] = size(part_centers);
ind = find( part_centers(:, l_p) == p_id) ;
p_center = part_centers(ind, 1:3);   
    
% plot center
%scatter3( p_center(1), p_center(2), p_center(3), 30, 'g','filled' );







