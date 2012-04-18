


max_dist = realmax;
codebook_index = -1;

%
% closest instance
%
% for c=1:n_words
%     word_parts = training_codebook(c).parts;
%     
%     [n_wp kk] = size(word_parts);
%     
%     for wp=1:n_wp
%         wp_vector = word_parts(wp);
%         d = sqrEuclideanDistance(part_vector, wp_vector);
%         if d <= max_dist
%             max_dist = d;
%             codebook_index = c;
%         end;
%     end    
% end    

% 
% activations(codebook_index) = 1;    

%
% closest center
%
for c=1:n_words
    word_vector = training_codebook(c).center ;    
    d = sqrEuclideanDistance(part_vector, word_vector);
    if d < max_dist  
        max_dist = d;
        codebook_index = c;
    end
end    
activations(codebook_index) = 1;    
