


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
        word_vector = training_codebook(c).center ;
        distances(c,1) = EuclideanDistance(part_vector, word_vector);        
    end    
    

    %
    % assign the majority class
    %
    [mindist closest] = min(distances);
    [n_entries kk] = size(training_codebook(closest).parts);    
    
    % object votes
    object_votes = zeros(1, n_training_objects);
    for n_c=1:n_entries
        c_part = training_codebook(closest).parts(n_c, :);
        % part object
        part_object = c_part(1,end);
        object_votes(part_object) = object_votes(part_object) +1;
    end    
    
    % assign most voted class
    [kk clas] = max( object_votes );
    classification(p, 1) = clas;
    
end    

fprintf('DONE');