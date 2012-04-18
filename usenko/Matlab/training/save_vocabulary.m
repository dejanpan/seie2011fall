

%-----------------------------------
% Save vocabulary
%-----------------------------------


% feature vectors 
training_vectors = filtered_features;
save('training_vectors', 'training_vectors');

% clustering
training_clustering = idx;
save('training_clustering', 'training_clustering');

%
training_nclusters = n_clusters;
save('training_nclusters', 'training_nclusters');

