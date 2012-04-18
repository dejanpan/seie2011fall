%-----------------------------
% Load features
%-----------------------------
features = [];
feaure_ids=[]; 


% getting the number of parts for the view
filename = strcat(prefix, int2str(current_view), '.pcd')
n_parts = getNParts(filename);    

for p=0:n_parts-1
    fprintf('Loading features..');
    filename = strcat(prefix, int2str(current_view), suffix, int2str(p),  '.pf')                                    
    [f_v, f_ids]= loadFeatures(filename, p);
    fprintf('DONE\n');

    if isempty(f_v) == 0  % not empty
        features = [features; f_v];
        feature_ids = f_ids;
    end    
end
