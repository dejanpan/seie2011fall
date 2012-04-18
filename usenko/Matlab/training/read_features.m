%-----------------------------
% Load features
%-----------------------------
features = [];
feaure_ids=[]; 


for o=1:n_objects
    for v=0:n_views(o)-1        

       % getting the number of parts for the view
        filename = strcat(prefix_1{o}, int2str(v), suffix_1{o},  '.pcd');
        n_parts = getNParts(filename);    

        for p=0:n_parts-1
            
            fprintf('Loading features..');
            filename = strcat(prefix_1{o}, int2str(v), suffix_1{o}, suffix_2{o}, int2str(p),  '.pf')                                    
            [f_v, f_ids]= loadFeatures(filename, v, p, o);
            fprintf('DONE\n');
            
            if isempty(f_v) == 0  % not empty
                features = [features; f_v];
                feature_ids = f_ids;
            end    
       end
    end;
end;