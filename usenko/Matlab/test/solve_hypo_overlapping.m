
%--------------------------------------
% Hypotheses which are too close
%--------------------------------------
[ n_best_hypotheses, kk ] = size( best_hypotheses(o).list );
distances= zeros( n_best_hypotheses, n_best_hypotheses );

for i=1:n_best_hypotheses
    for j=1:n_best_hypotheses
        p_1 = best_hypotheses(o).list(i, 3:4);
        p_2 = best_hypotheses(o).list(j, 3:4);
        distances(i, j) = sqrt( sqrEuclideanDistance(p_1, p_2) );        
    end
end    

threshold_distance = dist_final_hypo_thresholds(o);
survival = (1:n_best_hypotheses);

finish = false;
in = 1;
while finish == false
    c_hypo = survival(in);
    [kk n_s] = size(survival);
    for j=in+1:n_s
        entry = survival(j);
        if distances(c_hypo, entry) <  threshold_distance  
            survival(j) = -1;
        end
    end
    index = find( survival==-1 );
    survival(index) = [];        
    
    in = in + 1;
    [kk n_s] = size(survival);
    if in >= n_s
        finish = true;
    end
end    

best_hypotheses(o).list = best_hypotheses(o).list(survival, :); 
