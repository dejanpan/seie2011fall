
%------------------------
% classify_parts
%------------------------

% nearest_neighbour
% fprintf('Classifying...');
% 
% [n_parts kk] = size(final_features);
% classification = zeros(n_parts,1);
% 
% for p=1:n_parts    
%     p
%     part_vector = final_features(p, :) ;
%     
%     [n_t kk] = size(training_vectors);
%     distances= zeros(n_t, 1);
%     for c=1:n_t
%         t_vector = training_vectors(c, :) ;
%         word_vector = t_vector(1, 1:end-3);
%         d = sqrEuclideanDistance(part_vector, word_vector);
%         distances(c,1) = d;
%     end    
% 
%     % assign closest vector
%     [kk closest] = min(distances);
%     % 1=chair, 2=table
%     clas = training_vectors(closest, end)
%     classification(p, 1) = clas;
% end    
% 
% fprintf('DONE');




% centroids
% fprintf('Classifying...');
% 
% [n_parts kk] = size(final_features);
% classification = zeros(n_parts,1);
% 
% n_words = training_nclusters;
% 
% for p=1:n_parts    
%     p
%     part_vector = final_features(p, :) ;
%     distances= zeros(n_words,1);
% 
%     for c=1:n_words
%         word_vector = training_codebook(c).center ;
%         distances(c,1) = sqrEuclideanDistance(part_vector, word_vector);        
%     end    
% 
%     % assign closest word
%     [kk closest] = min(distances);
%     classification(p, 1) = closest;
% end    
% 
% fprintf('DONE');