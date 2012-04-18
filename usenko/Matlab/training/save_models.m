%-----------------------------------
% Save data models
%-----------------------------------




% feature vectors 
training_vectors = filtered_features;
save( strcat(dir_model, 'training_vectors'), 'training_vectors' );

% clustering
training_clustering = idx;
save( strcat(dir_model, 'training_clustering'), 'training_clustering');

%
training_nclusters = n_clusters;
save( strcat(dir_model, 'training_nclusters'), 'training_nclusters');

% 
training_space_relations = space_relations;
save( strcat(dir_model, 'training_space_relations'), 'training_space_relations');


training_codebook = codebook;
save( strcat(dir_model, 'training_codebook'), 'training_codebook');


% normalization factors
save( strcat(dir_model, 'training_max_features'), 'training_max_features');
save( strcat(dir_model, 'training_min_features'), 'training_min_features');


% part centers
training_part_centers = part_centers;
save( strcat(dir_model, 'training_part_centers'), 'training_part_centers');


% view centers
training_view_centers = view_centers;
save( strcat(dir_model, 'training_view_centers'), 'training_view_centers');


