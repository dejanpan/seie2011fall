%------------------------------------
% Plot 
%------------------------------------


class_colors= 'grcbmykwgrcbmykwgrcbmykwgrcbmykwgrcbmykw';

% chair = 1 = r
% table = 2 = b

    

% look for points pertaining to this part
p_v = p_id (1);
p_p = p_id (2);
p_o = p_id (3);

[n_p l_p] = size(training_points);
part_points_indexes = find(  training_points(:, l_p-2)==p_v & training_points(:, l_p-1)==p_p & training_points(:, l_p)==p_o);
part_points = training_points( part_points_indexes, : );  
    
% plot points
plot_part_test( part_points, class_colors(p) ) ;      
hold on;

% find center
[n_p l_p] = size(training_points);
ind = find( training_part_centers(:, l_p-2) == p_v  &  training_part_centers(:, l_p-1) == p_p & training_part_centers(:, l_p) == p_o ) ;
p_center = training_part_centers(ind, 1:3);   
    
% plot center
scatter3( p_center(1), p_center(2), p_center(3), 30, 'g','filled' );







