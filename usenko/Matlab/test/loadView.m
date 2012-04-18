

%-----------------------------
% data
%-----------------------------

     
% getting the number of parts for the view
filename = strcat(training_prefix_1{o}, int2str(v), training_suffix_1{o},  '.pcd');
current_view_n_parts = getNParts(filename);    

current_view_points=[];
current_view_part_centers = [];
current_view_center = [];

view_points = [];

for p=0:current_view_n_parts -1            
    filename = strcat(training_prefix_1{o}, int2str(v), training_suffix_1{o}, training_suffix_2{o}, int2str(p),  '.pcd')
    segment = loadSegment_training(filename, v, p, o);
    current_view_points =[current_view_points; segment];

    p_c = centerBoundingBox(segment);
    current_view_part_centers = [current_view_part_centers; p_c, v, p, o];

    % for the bounding box
    view_points = [ view_points; segment];
end    

center_v = centerBoundingBox(view_points);
current_view_center = [current_view_center; center_v v o]; 
%current_view_view_bbox = boundingBox(view_points);










