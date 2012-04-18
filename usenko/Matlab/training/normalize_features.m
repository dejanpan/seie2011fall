
%-----------------------------
% Normalize features
%-----------------------------

% zscore
%n_selected_features = zscore(selected_features);


only_vectors = filtered_features(:, 1:end-6);

% max - min values
max_f = max( only_vectors );
min_f = min( only_vectors );


% keep for training
training_max_features = max_f;
training_min_features = min_f;


range_f = max_f -min_f;
[r c] = size( only_vectors );

n_vectors = ( only_vectors - ones(r,1)*min_f ) ./ ( ones(r,1)*(max_f-min_f ) );

%add identifiers + center
filtered2_features = [n_vectors filtered_features(:, end-5:end) ];
