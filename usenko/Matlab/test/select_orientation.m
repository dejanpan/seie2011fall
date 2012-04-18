

%--------------------------------------------------------------------------------
% select_orientation
%
%
% Get parts contributing to the best hypotheses and select a possible
% orientation
%-------------------------------------------------------------------------------

majority_view = ones(n_training_objects, 1)* -1;   

for o=1:n_training_objects            
           
    % get first hypothesis
    hypo = best_hypotheses(o).list(1,:);         
        
    % get 2d cell
    row_2d  =  hypo(1);
    col_2d = hypo(2);
    % get parts contributing
    parts = space_2d(o).parts(row_2d, col_2d).list;

    [r_p c_p] = size(parts);
    % for each part
    p_views = [];
    for p=1:r_p
        p_id = parts(p, :);

        p_v = p_id (1);
        p_views = [p_views, p_v]; 
    end 
    
    % select the majority
    unique_views = unique(p_views);
    repetitions_views = histc(p_views, unique_views);
    majority_view(o) = max(repetitions_views);
end    




