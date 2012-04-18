
%-----------------------------
% data
%-----------------------------
points = [];
part_centers = [];

  
% getting the number of parts for the view
filename = strcat(prefix, int2str(current_view), '.pcd');
n_parts = getNParts(filename);    

for p=0:n_parts -1            
    filename = strcat(prefix, int2str(current_view), suffix, int2str(p),  '.pcd')
    segment = loadSegment_test(filename, p);
    points =[points; segment];
    
    p_c = centerBoundingBox(segment);
    
    part_centers = [part_centers; p_c  p];
end    
        

