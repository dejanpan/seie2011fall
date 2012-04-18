


%-----------------------------
% PCA
%-----------------------------
[coeff pca_features] = princomp(final_features);

scatter(pca_features(:,1), pca_features(:,2), 50, idx, 'filled' );

%scatter3(pca_features(:,1), pca_features(:,2), pca_features(:,3), 50, idx, 'filled' );