

%---------------------------------------
% Check object
%---------------------------------------

% contributing parts
c_parts = final_scene_part_contributors(o).parts;
h_center = hypo(3:4);

view_distances= zeros(training_n_views(o), 1);

for v=0:training_n_views(o)-1                        
        
%     % debug
%     loadView;
%     d_current_view_points = current_view_points;
%     d_current_view_points(:,1) = d_current_view_points(:,1) + d_vector(1);
%     d_current_view_points(:,2) = d_current_view_points(:,2) + d_vector(2);
%     plot_part_test(d_current_view_points, 'k');
% 
    
    %
    % look for current view center
    %
    [n_r n_c] = size( training_view_centers );
    index = find( training_view_centers(:, n_c-1) == v & training_view_centers(:, n_c) == o );
    c_v_center = training_view_centers(index, :);

    d_vector = h_center - c_v_center(1:2);

    
       
    %
    % look for current training part centers
    %    
    [n_r n_c] = size( training_part_centers );
    indexes = find( training_part_centers(:, n_c-2) == v &  training_part_centers(:, n_c) == o);
    current_view_part_centers = training_part_centers(indexes, :);
    
    
    [n_r n_c] = size(current_view_part_centers);
    final_distances = zeros(n_r, 1);
    
    for t_p=0:n_r-1
        idx = find( current_view_part_centers(:, n_c  ) == o & ... 
                    current_view_part_centers(:, n_c-1) == t_p & ... 
                    current_view_part_centers(:, n_c-2) == v );
        t_p_center = current_view_part_centers(idx, 1:3);
        t_p_center(1:2) = t_p_center(1:2) + d_vector;
        
        % debug
        % scatter3( t_p_center(1), t_p_center(2), t_p_center(3), 50, 'r', 'filled' ); 
       
        %
        % look for closest contributing center
        %         
        [c_r kk] = size(c_parts);        
        p_distances = zeros(c_r,1);
        for c_p=1:c_r
            p_id = c_parts(c_p,2);
            
            % find center
            [n_p l_p] = size(part_centers);
            ind = find( part_centers(:, l_p) == p_id) ;
            p_center = part_centers(ind, 1:3);   

            p_distances(c_p,1) = sqrEuclideanDistance(t_p_center, p_center);
        end                    
        
        final_distances(t_p+1) =  min(p_distances);
    end 
    
    % final view distance
    view_distances(v+1) =  min( final_distances );   
end


% best view
[best_dist best_v] = min(view_distances);

threshold_dist = center_thresholds(o); % 30 centimetros
if  best_dist < threshold_dist
    % correct hypothesis
    correct_hypo(n_hypo) = 1;
end    








           
