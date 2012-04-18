
max_dist = realmax;
codebook_index = -1;

%
% activate best N codebooks
%
distances = zeros(n_words, 1);

for c=1:n_words
    word_vector = training_codebook(c).center;    
    max_dist = training_codebook(c).max_dist;    
    d = gaussianDistance(part_vector, word_vector, max_dist);
    %d = sqrEuclideanDistance(part_vector, word_vector);
    distances(c) = d;
end    

% order by distance
[codebook_dist, idx] = sort(distances, 'descend');

n_best_distances = 5;
for n_d=1:n_best_distances
    act_idx = idx(n_d);
    activations(act_idx) = distances(act_idx);    
end    
