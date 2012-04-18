

% centroids
fprintf('Classifying...');

[n_parts kk] = size(final_features);
classification = zeros(n_parts,1);

n_words = training_nclusters;

for p=1:n_parts    
    p
    part_vector = final_features(p, :) ;
    distances= zeros(n_words,1);

    for c=1:n_words
        mu = training_codebook(c).center;
        sigma = training_codebook(c).sigma;
        sigma = diag(diag(sigma));  % diagonal covariance matrix
        distances(c,1) =  mvnpdf( part_vector, mu, sigma );        
    end    
    

    %
    % vote in the 3D space
    %
    [mindist closest] = min(distances);
    
    % object votes
    relations = training_codebook(closest).space_relations;
     
    codebook_votes(p).votes = [];
    codebook_votes(p).objects = [];

    p_center = [ filtered_features(p, end-5:end-3) ];      
    [n_rel kk] = size(relations);
    for r=1:n_rel
        vector = [ relations(r,10) relations(r,11) relations(r,12) ];
        vote = p_center + vector;
        codebook_votes(p).votes = [codebook_votes(p).votes; vote];
        object = relations(r,3);
        codebook_votes(p).objects = [codebook_votes(p).objects; object];
    end
   
end    

fprintf('DONE');
