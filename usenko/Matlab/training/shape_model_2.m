
%-----------------------------------
% Shape Model
%-----------------------------------

space_relations = [];

training_part_centers=[];

for o=1:n_objects
    for v=0:n_views(o)-1
    
        % points from object and view
         [n_p l_p] = size(points);
         indexes = find( points(:, l_p-2)==v & points(:, l_p)==o);
         object_points = points( indexes, : );

         % center of bounding box
         o_ind = find( view_centers(:, 4)==v & view_centers(:, 5)==o );
         object_center = view_centers(o_ind,1:3); 

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
             
             % look for part center
             [kk l_po] = size(part_centers);
             index = find( part_centers(:, l_po-2)==p_v & part_centers(:, l_po-1)==p_p & part_centers(:, l_po)==p_o);
             p_center = part_centers( index, 1:3);
                          
             %p_center = [ o_parts(p, l_p-5), o_parts(p, l_p-4), o_parts(p, l_p-3) ];
                                       
             
             % part relation
             p_rel = object_center - p_center;
             space_relations = [space_relations; p_v p_p p_o p_center object_center p_rel];
        end
    end
end


