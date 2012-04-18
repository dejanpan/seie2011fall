


% %
% best Hypotheses
%
figure(h_main);
class_colors= 'rbgcmykw';
for o=1:n_training_objects       
    votes = space_2d(o).local_maxima;
	[n_best_hypotheses, kk] = size(best_hypotheses(o).list);
    
    for n=1:n_best_hypotheses          
        point = best_hypotheses(o).list(n,3:4);        
        weight = best_hypotheses(o).list(n,5);        
        marker_size = ceil(weight * 200);
        scatter3(point(1,1), point(1,2), 0, marker_size, class_colors(o), 'filled');        
    end
end    
