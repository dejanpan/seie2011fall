%-----------------------------
%  K-MEANS 
%-----------------------------

[idx, k_centers, sumd, k_distances]= kmeans (final_features, n_clusters);     

final_clusters = (1:1:n_clusters);

% delete clusters with less than 3 parts





% m_min = 2;
% m_max = 60;
% m_step= 4;
% db=[];
% sil = [];
% j3=[];
% 
% index=0;
% m = m_min;
% while  m<= m_max
%     m
%     idx = kmeans (final_features, m);     
%     db = [ db; calculate_db(final_features, idx, m) ];    
%     %j3 = [ j3; calculate_j3(final_features,idx,m) ];
%     ss = silhouette(final_features, idx);
%     sil = [ sil;  mean( ss ) ] ;       
%     m = m + m_step;
% end;

 

%look for minimum in db
%[v i] = min(db);
%n_clusters = m_min + (i-1)*m_step;
%idx = kmeans(final_features, n_clusters);     

% figure(), 
% [pc, zscores, pcvars] = princomp(selected_features);
% %scatter3(final_features(:,1), final_features(:,2), final_features(:,3), 10, idx);
% scatter(zscores(:,1), zscores(:,2), 10, idx);
% t = strcat('min db index. m = ', int2str(m));
% title(t);


%look for maximum in sil
% [v i] = max(sil);
% n_clusters = m_min + (i-1)*m_step;
% idx = kmeans(final_features, n_clusters);     



