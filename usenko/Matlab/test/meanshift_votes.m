
%-------------------------
% meanshift of votes
%-------------------------

 

o_votes = v;

o_votes_t = o_votes(:, 1:3);

o_votes_t(:,3) = 0;

o_votes_t =  o_votes_t';

[clustCent, point2cluster,clustMembsCell] = MeanShiftCluster(o_votes_t, band);
numClust = length(clustMembsCell);


final_scores= zeros(1, numClust);
for k = 1:numClust
    myMembers = clustMembsCell{k};
    myClustCen = clustCent(:,k);

    % get score for each cluster
    scores = o_votes(myMembers, 4);
    final_scores(k) = sum(scores);        
end  


[value ind] = max(final_scores);


% % plot maximum score
myClustCen = clustCent(:,ind);
scatter3(myClustCen(1), myClustCen(2), myClustCen(3), 200, class_colors(o), 'filled');                


%
% plot scores bigger than threshold
%
% for k = 1:numClust
%     if ( final_scores(k) >= mean_shift_threshold(o) ) 
%        myClustCen = clustCent(:, k);
%        scatter3(myClustCen(1), myClustCen(2), myClustCen(3), 200, class_colors(o), 'filled');                            
%     end
% end  
    

