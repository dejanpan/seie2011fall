

%check features

function check_features(p_features, p_points, n_views, n_objects)
    
    part_colors= colorcube( 70 );

    figure();
    hold on;

    for o=1:n_objects                
        
        [n_f l_f] = size(p_features);        
        o_features_indexes = find( p_features(:, l_f)==o);
        o_features = p_features(o_features_indexes, : );
        
        
        [n_f l_f] = size(o_features);        
        for i = 1:n_f                                    
            
            fprintf('Plotting part...\n');
            
            % view and part identifiers
            v = o_features(i, l_f-2)
            p = o_features(i, l_f-1)

            % look for points pertaining to this part
            [n_p l_p] = size(p_points);
            part_points_indexes = find( p_points(:, l_p-2)==v & p_points(:, l_p-1)==p & p_points(:, l_p)==o);
            part_points = p_points( part_points_indexes, 1:3 );    

            
            
            
            % plot points            
            plot_part(part_points, 'k')
            hold on;
            
            % plot eigenvectors
            % plot the centroid
            centroid = mean(part_points);
            scatter3(centroid(1), centroid(2), centroid(3), 40, 'r','filled');
            hold on;
            
            % plot eigenvectors
            eval0 = o_features(i, 6);
            evec0x = o_features(i, 7);
            evec0y = o_features(i, 8);            
            evec0z = o_features(i, 9);
            
            eval1 = o_features(i, 10);
            evec1x = o_features(i, 11);
            evec1y = o_features(i, 12);            
            evec1z = o_features(i, 13);
            
            eval2 = o_features(i, 14);
            evec2x = o_features(i, 15);
            evec2y = o_features(i, 16);                                    
            evec2z = o_features(i, 17);
            
            l0min= o_features(i, 25);
            l0max= o_features(i, 26);
            l1min= o_features(i, 27);
            l1max= o_features(i, 28);            
            l2min= o_features(i, 29);
            l2max= o_features(i, 30);
            
                                                            
            % biggest
            quiver3(centroid(1), centroid(2), centroid(3), evec2x, evec2y, evec2z, l2max, 'r');            
            hold on;
            quiver3(centroid(1), centroid(2), centroid(3), -evec2x, -evec2y, -evec2z, l2min, 'r');            
            hold on;
            
            quiver3(centroid(1), centroid(2), centroid(3), evec1x, evec1y, evec1z, l1max, 'b');            
            hold on;
            quiver3(centroid(1), centroid(2), centroid(3), -evec1x, -evec1y, -evec1z, l1min, 'b');            
            hold on;
            
            quiver3(centroid(1), centroid(2), centroid(3), evec0x, evec0y, evec0z, l0max, 'g');            
            hold on;
            if l0min==0
                l0min=l0max;
            end
            quiver3(centroid(1), centroid(2), centroid(3), -evec0x, -evec0y, -evec0z, l0min, 'g');            
            hold on;
            
            
            
            
            
            fprintf('DONE\n');
            pause;
            clf;    
        end    
        
    end; %for o=1:n_objects                
    
    close; 
end    
