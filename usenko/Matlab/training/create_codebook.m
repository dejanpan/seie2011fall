

%-------------------------
% create codebooks
%-------------------------


for c=1:n_clusters
    
    % find parts of this cluster
    indexes = find ( idx== c );
    c_parts = filtered_features( indexes, : );
    c_features =  c_parts(:, 1:end-6);  
       
    % parts
    codebook(c).parts = c_parts;
    
    
    % part identifiers
    codebook(c).part_identifiers = c_parts(:, end-2:end);  % view, part, object
    
    % centers   
    codebook(c).center = k_centers(c,:);   
    
    
%     % centers   
%     mu = mean(c_features);
%     codebook(c).center = mu;
%     
%     %standar deviations
%     sigma = cov(c_features);
%     codebook(c).sigma = sigma;
    
    
    % all distances
    [n_p kk] = size(c_features);
    p_k_distances = k_distances(indexes, c);        
        
%     distances = zeros(n_p, 1);
%     for i=1:n_p
%         distances(i, 1) = sqrEuclideanDistance(c_features(i,:), codebook(c).center);
%     end
    codebook(c).distances = p_k_distances;
    codebook(c).max_dist = max(p_k_distances);
                   
    
    % space_relations
    % for each part look for the corresponding space relation    
    rel = [];
    [n_p l_p] = size(c_parts);
    for p = 1:n_p

        % get part identifier
        p_v = c_parts(p, l_p-2);
        p_p = c_parts(p, l_p-1);
        p_o = c_parts(p, l_p);

        % space_relations = [space_relations; p_v p_p p_o p_center object_center p_rel];
        index = find ( space_relations(:,1) == p_v &  space_relations(:,2) == p_p & space_relations(:,3) == p_o );
        rel = [rel; space_relations(index, : ) ];
    end
    
    codebook(c).space_relations = rel;
	
    
    
    
end
