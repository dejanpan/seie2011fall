


%
% Parameters
% 
minimum_points = 50;
n_clusters = 30;
dir_model = 'C:\OSCAR\projects\TUM\object_detection\experiments\Real_scene\table_chair_cabinet\for_RSS_paper\20_views\k_30\';    

%
% TRAINING DATA
%
n_objects = 3;

prefix_1 = {  ...
    'C:\OSCAR\projects\TUM\object_detection\experiments\Real_scene\table\2000x1000x740_01-centered_v';  ... 
    'C:\OSCAR\projects\TUM\object_detection\experiments\Real_scene\chair_aluminium\Aluminium_Group_EA_119_0000F152-centered_v';  ... 
    'C:\OSCAR\projects\TUM\object_detection\experiments\Real_scene\cabinet\Spatio_Sideboard_160_0000F1C5_squished-centered_v'; ...
};

suffix_1= { ...
    '', ...
    '', ...
    '', ...
    '', ...
};


suffix_2 = { ...
    '\part'; ...
    '\part'; ...
    '\part'; ...
    '\part'; ...
};

n_views = [ ...
    20; ...
    20; ...
    20; ...
];


% %



%-----------------------------
% Load points
%-----------------------------
read_data_training;

%-----------------------------
% Load features
%-----------------------------
read_features;


%-----------------------------------
% Plot points in different views
%-----------------------------------
%plot_points(features, points, n_views, n_objects, view_centers);

%-----------------------------
% Reject parts with:
%       - few points
%       - lines 
%-----------------------------
fprintf('filter_parts...\n');
filter_parts;
filtered_features=filtered1_features;


%-----------------------------
% Select features
%-----------------------------
fprintf('select_features...\n');
select_features;
filtered_features = selected_features;


%-----------------------------
% Outlier removal;
% NOT YET!
%-----------------------------
% outlier_removal;
% filtered_features=filtered2_features;


%-----------------------------
% Normalize features
%-----------------------------
fprintf('normalize_features...\n');
normalize_features;
filtered_features = filtered2_features; 

%plot_points(filtered_features, points, n_views, n_objects, view_centers);

final_features = filtered_features(:, 1:end-6);


%-----------------------------
% Save data for CVAP
%-----------------------------
%export_features_cvap;



%-----------------------------
% Autocorrelation
%-----------------------------
%[R P]= corrcoef(final_features);
%imagesc(abs(R));


%-----------------------------
% clustering
%-----------------------------
% 
% K-MEANS
fprintf('K-MEANS...\n');
kmeans_clus;



% -----------------------------------
% Plot segments according to clustering
% -----------------------------------
%plot_clustering;



%-----------------------------------
% Shape Model
%-----------------------------------
fprintf('Shape_model...\n');
shape_model_2;


%-----------------------------------
% Plot shape models
%-----------------------------------
%plot_shape_models;


%-----------------------------------
% codebook
%-----------------------------------
fprintf('create_codebook...\n');
create_codebook;

%plot_codebook;



%-----------------------------------
% Save models
%-----------------------------------
fprintf('save_models...\n');
save_models;













