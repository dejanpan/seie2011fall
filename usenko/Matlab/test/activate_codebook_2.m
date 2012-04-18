
max_dist = realmax;
codebook_index = -1;

%
% activate according to distance to centers
%

activation_threshold =  0.5;

for c=1:n_words
    word_vector = training_codebook(c).center;    
    max_distance = training_codebook(c).max_dist;
    d = gaussianDistance(part_vector, word_vector, max_distance);
    if d >= activation_threshold  
        activations(c) = d;        
    end
end    

