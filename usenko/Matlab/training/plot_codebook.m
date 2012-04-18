%-----------------------------
% Plot codebook
%-----------------------------
part_colors= colorcube( 20 );

figure(); 

max_views = max(n_views);

[kk n_clusters] = size(final_clusters);
for c = 1:n_clusters  
    c
    
    % look for parts pertaining to this codebook
    c_parts = codebook(c).parts;
    
    [n_f l_f] = size(c_parts);
    for i = 1:n_f
        % view and part identifiers
        v = c_parts(i, l_f-2);
        p = c_parts(i, l_f-1);
        o = c_parts(i, l_f);

        % look for points pertaining to this part
        [n_p l_p] = size(points);
        part_points_indexes = find( points(:, l_p-2)==v & points(:, l_p-1)==p & points(:, l_p)==o);
        part_points = points( part_points_indexes, : );    

        % plot points
        subplot(n_objects, max_views, ( (o-1)*max_views)  + v+1 );
        hold on;
        plot_part( part_points, part_colors(p+1,:) )
    end   
    fprintf('...Done');
    
    pause;
    clf;
end    
close;
