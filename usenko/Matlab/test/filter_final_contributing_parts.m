
%-------------------------------------------
% Remove parts of hypotheses that did not survive
%-------------------------------------------


[n_r kk] = size( best_hypotheses(o).list );
if  n_r > 0
    [n_r kk] = size( final_scene_part_contributors(o).parts ); 
    to_delete = zeros(n_r,1);
    for p=1:n_r;
        hypo_id = final_scene_part_contributors(o).parts(p,3);  

        indexes = find( best_hypotheses(o).list(:,6) == hypo_id );
        if isempty(indexes) 
            % mark to delete
            to_delete(p,1) = 1;
        end   
    end   
    
    % delete
    indx = find( to_delete(:,1) == 1 );
    final_scene_part_contributors(o).parts(indx,:) = [];  
end       

        
        
