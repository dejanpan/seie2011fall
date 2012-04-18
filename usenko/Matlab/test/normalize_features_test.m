
%-----------------------------
% Normalize features
%-----------------------------

% zscore
%n_selected_features = zscore(selected_features);


only_vectors = filtered_features(:, 1:end-4);

% max - min values


max_f = training_max_features;
min_f = training_min_features;


% filter values
[r c] = size( only_vectors );
for i=1:r
    for j=1:c
        if ( only_vectors(i, j) < min_f(j) ) 
            only_vectors(i, j) = min_f(j);
        end
        
        if ( only_vectors(i, j) > max_f(j) ) 
            only_vectors(i, j) = max_f(j);
        end
    end
end    


range_f = max_f -min_f;




n_vectors = ( only_vectors - ones(r,1)*min_f ) ./ ( ones(r,1)*(max_f-min_f ) );

%add identifiers + center
filtered2_features = [n_vectors filtered_features(:, end-3:end) ];
