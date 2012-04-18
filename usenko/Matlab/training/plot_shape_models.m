

%-----------------------------------
% Plot shape models
%-----------------------------------

part_colors= colorcube( 10 );

figure();
hold on;
    
for o=1:n_objects  
    
    for v=0:n_views(o)-1
    
        o
        v
        
        % get parts of object & view        
        [kk l_f] = size( filtered_features ); 
        indexes = find( filtered_features(:, l_f-2)==v &  filtered_features(:, l_f)  == o );
        o_parts = filtered_features( indexes,: );

        [n_p l_p] = size(o_parts);
        for p = 1:n_p
            
            % get part identifier
            p_v = o_parts(p, l_p-2);
            p_p = o_parts(p, l_p-1);
            p_o = o_parts(p, l_p);

            % look for points of this part
            [kk l_po] = size(points);
            indexes = find( points(:, l_po-2)==p_v & points(:, l_po-1)==p_p & points(:, l_po)==p_o);
            part_points = points( indexes, : );
            
            % plot parts 
             plot_part(part_points, part_colors(p+1,:) )
             hold on;
             
             % plot object center
             % space_relations = [space_relations; p_v p_p p_o p_center object_center p_rel];
             index = find ( space_relations(:,1) ==p_v &  space_relations(:,2) ==p_p & space_relations(:,3) ==p_o );
             rel = space_relations(index, : );
             scatter3( rel(7), rel(8), rel(9), 80, 'k', 'filled');
             hold on;
             
             % plot part center
             scatter3( rel(4), rel(5), rel(6), 20, 'r', 'filled');
             hold on;
             
            % plot relation vectors
            c_x =rel(4);
            c_y = rel(5);
            c_z = rel(6);
            vec = [rel(10)  rel(11)   rel(12)];
            
            
            quiver3(c_x, c_y, c_z, vec(1), vec(2), vec(3), 1, 'k');            
            hold on;
            
        end
        pause;
        clf;

    end % for v=0:n_views(o)-1
end; %for o=1:n_objects                

close; 

