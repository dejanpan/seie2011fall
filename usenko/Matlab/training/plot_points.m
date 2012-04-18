%-----------------------------------
% Plot points in views
%-----------------------------------

function plot_points(p_features, p_points, n_views, n_objects, view_centers)
    
    part_colors= colorcube( 20 );

    figure();
    
    for o=1:n_objects                
        for v=0:n_views(o)-1
            o
            v
            [n_f l_f] = size(p_features);        
            o_features_indexes = find( p_features(:, l_f-2)==v & p_features(:, l_f)==o);
            o_features = p_features(o_features_indexes, : );

            fprintf('Plotting views...\n');
            [n_f l_f] = size(o_features);        
            for i = 1:n_f                                    
                % view and part identifiers
                v = o_features(i, l_f-2);
                p = o_features(i, l_f-1);

                % look for points pertaining to this part
                [n_p l_p] = size(p_points);
                part_points_indexes = find( p_points(:, l_p-2)==v & p_points(:, l_p-1)==p & p_points(:, l_p)==o);
                part_points = p_points( part_points_indexes, : );    

                plot_part_training(part_points, part_colors(p+1,:))
                %plot_part_training(part_points, 'b')
                hold on;
            end    
            
%             % plot boundingbox centers
%              c_ind = find( view_centers(:, end-1)==v & view_centers(:, end)==o );
%              p_center = view_centers(c_ind,:);
%              scatter3( p_center(1,1), p_center(1,2), p_center(1,3), 50, 'k', 'filled');
%              hold on;

            fprintf('DONE\n');
            pause;
            clf;
        end % views
    end; %for o=1:n_objects                    
    close; 
end    





