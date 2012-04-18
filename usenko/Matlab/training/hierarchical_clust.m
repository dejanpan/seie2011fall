
%-----------------------------
% Hierarchical clustering
%-----------------------------


% select clustering until we have a minimum of them with minimum number of
% parts

min_parts = 3; % minimum number of parts per cluster
n_final_clusters=6;
Y = pdist(final_features, 'euclid'); 
Z = linkage(Y,'single'); 

n_clusters = n_final_clusters;
while true
    final_clusters = [];
    m = n_clusters;
    fprintf( strcat('Hierarchical clustering. m: ', int2str(m)) );
    idx = cluster(Z, 'maxclust', m);
    j=m;
    for i=1:j
    [r c] = size( find(idx==i) );
        if r < min_parts
           m = m - 1;
        else
           final_clusters = [final_clusters, i];
        end
    end
    m
    if m < n_final_clusters
        n_clusters = n_clusters +1;
    else
        break;
    end
end    


%------------------------------------
% Filter out parts not in the final clustering
%------------------------------------
[kk n_clusters] = size(final_clusters);
to_keep = [];
for i=1:n_clusters    
    ind = find( idx== final_clusters(i) ); 
    to_keep= [to_keep; ind];
end    
    
final_features = final_features(to_keep,:);
idx = idx(to_keep);










% Y = pdist(final_features, 'euclid'); 
% Z = linkage(Y,'complete'); 
% %Z = linkage(Y,'single'); 
% c= cophenet(Z,Y);
% 
% H = dendrogram(Z);
% 
% 
% m_min = 2;
% m_max = 60;
% m_step= 4;
% db=[];
% sil = [];
% 
% m = m_min;
% while  m<= m_max
%     m
%     idx = cluster(Z, 'maxclust', m);
%     ss = silhouette(final_features, idx);
%     sil = [ sil;  mean( ss ) ] ;       
%     m = m + m_step;
% end;



