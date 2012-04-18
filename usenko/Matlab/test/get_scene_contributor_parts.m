

%------------------------------------------------
% get_scene_contributor_parts
%
%
% Get parts contributing to the best hypotheses
%-----------------------------------------------

    
scene_part_contributors = [];

% get 2d cell
row_2d = hypo(1);
col_2d = hypo(2);

% get parts contributing
parts = space_2d(o).parts(row_2d, col_2d).list;

[r_p c_p] = size(parts);
% for each part
for p=1:r_p
    p_id = parts(p, 2);
    vote_weight = parts(p, 1);
    scene_part_contributors = [scene_part_contributors; vote_weight p_id];
end;
%final_scene_part_contributors(o).parts = unique(scene_part_contributors);

all_final_scene_part_contributors(o).parts = scene_part_contributors;
unique_final_scene_part_contributors(o).parts = unique(scene_part_contributors(:,2)); 

[n_r kk] = size( unique_final_scene_part_contributors(o).parts ); 
for p=1:n_r;
    p_id = unique_final_scene_part_contributors(o).parts(p);  
    mask = all_final_scene_part_contributors(o).parts(:,2)==p_id;
    resul = sum(all_final_scene_part_contributors(o).parts(mask,:),1);
    weight=resul(1);
    
    % "n_hypo" contains the "id" of the current hypothesis in object(o). See
    % previous for loop in  run_test.m
    hypo_id = hypo(6);
    final_scene_part_contributors(o).parts=[final_scene_part_contributors(o).parts; weight p_id hypo_id];  
end            


    
%
% debug 
% figure();
% hold on;
% 
% [n_r kk] = size( unique_final_scene_part_contributors(o).parts ); 
% for p=1:n_r;
%     p_id = unique_final_scene_part_contributors(o).parts(p);  
%     plot_scene_contributing_part;
% end            





