%-----------------------------------
% Plot points in views
%-----------------------------------

function plot_points(p_features, p_points, part_centers)
    
    part_colors= colorcube( 20 );

    figure();
      
    fprintf('Plotting scene...\n');
    [n_f l_f] = size(p_features);        

    for i = 1:n_f                                    
        % view and part identifiers
        p = p_features(i, l_f);

        % look for points pertaining to this part
        [n_p l_p] = size(p_points);
        part_points_indexes = find( p_points(:, l_p)==p );
        part_points = p_points( part_points_indexes, : );    

        plot_part_test(part_points, part_colors(i+1,:))
        %plot_part_test(part_points, 'b')
        hold on;

        % plot boundingbox centers
        c_ind = find( part_centers(:, end)==p );
        p_center = part_centers(c_ind,:);
        scatter3( p_center(1,1), p_center(1,2), p_center(1,3), 50, 'k', 'filled');
        hold on;

    end    
    fprintf('DONE\n');
    pause;
    clf;
    
    close; 
end    





