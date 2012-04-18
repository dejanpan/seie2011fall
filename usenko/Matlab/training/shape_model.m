
%-----------------------------------
% Shape Model
%-----------------------------------

space_relations = [];

for o=1:n_objects
    for v=0:n_views(o)-1
    
        % points from object and view
         [n_p l_p] = size(points);
         indexes = find( points(:, l_p-2)==v & points(:, l_p)==o);
         object_points = points( indexes, : );

         % centroid
         mu=mean(object_points);
         object_center = mu(1:3); 

        %---------------------
        % part_relations
        %---------------------

        % look for parts pertaining to this object & view
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
             
             % part center
             p_center = [ o_parts(p, l_p-5), o_parts(p, l_p-4), o_parts(p, l_p-3) ];
             
             % part relation
             p_rel = object_center - p_center;
             space_relations = [space_relations; p_v p_p p_o p_center object_center p_rel];
        end
    end
end
