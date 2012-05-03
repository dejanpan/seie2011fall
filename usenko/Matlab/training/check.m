






n_objects = 1;

prefix_1 = {  ...
    '/home/oscar/projects/TUM/object_detection/3d_data/zoltan/from_2_meters/test3_table/table1_v';  ...
};

suffix_1= { ...
    ''; ...
};

suffix_2 = { ...
    '/part'; ...
};

n_views = [ ...
    8; ...
];


%-----------------------------
% Load points
%-----------------------------
%read_data;

%-----------------------------
% Load features
%-----------------------------
%read_features;


%-----------------------------
% Check features
%-----------------------------
check_features(features, points, n_views, n_objects);