

%---------------------------------------
% Check object
%---------------------------------------

        
% %
% % debug
% %
% figure();
% hold on;
% 
% [kk n_c] = size( final_scene_part_contributors(o).parts ); 
% for p=1:n_c;
%     p_id = final_scene_part_contributors(o).parts(p);
%     plot_scene_contributing_part;
% end       


point = hypo(1,3:4);        

inside_bbox = zeros(1, training_n_views(o));

for v = 0:training_n_views(o)-1
    % corresponding bbox
    [kk n_t] = size(training_bounding_boxes);
    indx = find( training_bounding_boxes(:, n_t) == o & training_bounding_boxes(:, n_t-1) == v );
    bbox = training_bounding_boxes(indx,:);

    % check points inside bounding box
    check_points_in_bbox;
    
    
%     %debug
%     plot_bbox( [point bbox], 'r' );       
end


% select best bounding box
[prob best_v]  = max( inside_bbox );


[kk n_t] = size(training_bounding_boxes);
indx = find( training_bounding_boxes(:, n_t) == o & training_bounding_boxes(:, n_t-1) == best_v -1 );
best_bbox = training_bounding_boxes(indx,:);

% %debug
plot_bbox( [point best_bbox], 'r' );

best_bboxes = [best_bboxes; point best_bbox];           


%
% load view
%
v = best_v -1;
loadView;


% debug
figure();
plot_part_test(best_view_points, 'b')













           