
%-----------------------------
% data
%-----------------------------
points = [];
view_centers= [];
part_centers = [];

for o=1:n_objects

    for v=0:n_views(1)-1            
        
        % getting the number of parts for the view
        filename = strcat(prefix_1{o}, int2str(v), suffix_1{o},  '.pcd');
        n_parts = getNParts(filename);    
        
        view_points = [];
        
        for p=0:n_parts -1            
            filename = strcat(prefix_1{o}, int2str(v), suffix_1{o} , suffix_2{o}, int2str(p),  '.pcd')
            segment = loadSegment(filename, v, p, o);
            points =[points; segment];
            view_points = [ view_points; segment];
            p_c = centerBoundingBox(segment);
            part_centers = [part_centers; p_c v p o];
        end    
        
        center_v = centerBoundingBox(view_points);
        view_centers = [view_centers; center_v v o];
        
    end
    
end;    









