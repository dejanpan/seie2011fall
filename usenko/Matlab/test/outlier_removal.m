

%
% Outlier removal
%

only_vectors = filtered_features(:, 1:end-6);

mu = mean(only_vectors);
sigma = std(only_vectors); 
[n, p] = size(only_vectors);
outliers = abs(only_vectors - mu(ones(n, 1),:)) > 3*sigma(ones(n, 1),:);

% rejected feature vectors
filtered2_features = filtered_features;
filtered2_features( any(outliers'), :)=[];

