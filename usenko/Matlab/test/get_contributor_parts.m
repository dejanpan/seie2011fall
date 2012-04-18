

%------------------------------------------------
% get_contributor_parts
%
%
% Get parts contributing to the best hypotheses
%-----------------------------------------------

for o=1:n_training_objects            
    best_hypo = best_hypotheses(o).list;         

    for n=1:n_best_hypotheses   
        
        figure();
        hold on;
        
        % get first hypothesis
        hypo = best_hypo(n,:);
        
        % get 2d cell
        row_2d  =  hypo(1);
        col_2d = hypo(2);
        % get parts contributing
        parts = space_2d(o).parts(row_2d, col_2d).list;
        
        [r_p c_p] = size(parts);
        % for each part
        for p=1:r_p
            p_id = parts(p, :);
            
            % debug
            plot_contributing_part;
                        
        end
        
    end    % end   for n=1:n_best_hypotheses   
end    




