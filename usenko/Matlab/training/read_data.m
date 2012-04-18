
%-----------------------------
% data
%-----------------------------
points = [];
part_centers = [];

for o=1:n_objects

    for v=0:n_views(1)-1            
        
        % getting the number of parts for the view
        filename = strcat(prefix_1{o}, int2str(v), suffix_1{o},  '.pcd');
        n_parts = getNParts(filename);    
        
        for p=0:n_parts -1            
            filename = strcat(prefix_1{o}, int2str(v), suffix_1{o} , suffix_2{o}, int2str(p),  '.pcd')
            segment = loadSegment(filename, v, p, o);
            points =[points; segment];
            part_centers = [part_centers; mean( segment(:, 1:3)) v p o];
        end    
        
    end
    
end;    

